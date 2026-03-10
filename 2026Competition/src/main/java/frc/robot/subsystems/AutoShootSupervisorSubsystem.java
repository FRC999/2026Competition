package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.RobotContainer;
import frc.robot.lib.TurretHelpers;

/**
 * AutoShootSupervisorSubsystem
 *
 * "Superstructure" owner of the volley state machine.
 *
 * Why a subsystem owns state (instead of the command):
 * - Commands are interruptible; you still want the system to "know what it was
 * doing" for telemetry/debug.
 * - The driver can press the shoot button multiple times; we want deterministic
 * behavior.
 *
 * High-level goals:
 * - Aim turret at target at all times (if enabled).
 * - When the driver requests shooting, run a continuous volley:
 * keep shooter + hood + turret commanded; stage balls; fire as soon as gates
 * are satisfied; repeat until empty.
 * - If no shooting solution exists for the next ball, the volley pauses
 * (NO_SOLUTION) instead of firing blind.
 *
 * Performance architecture:
 * - This subsystem does the math once per 20 ms loop (50 Hz). The solver itself
 * is lightweight.
 * - Artillery table is loaded once at startup from /deploy (CSV).
 * - Robot acceleration is estimated from two consecutive velocity samples (no
 * CTRE acceleration signal needed).
 */
public class AutoShootSupervisorSubsystem extends SubsystemBase {

    public enum VolleyState {
    IDLE,
    ARMING,
    FIRING,
    RECOVERING,
    NO_SOLUTION
  }

  public enum SolutionValidity {
    VALID,
    TURRET_ONLY_INVALID,
    GLOBAL_INVALID
  }

  private TurretHelpers.ArtilleryTableIndexedByShooterRpmAndHoodAngle table;

  // Driver request flag (set by commands)
  private boolean shootRequested = false;

  // Teleop-only trench safety interlock
  private boolean trenchLockoutActive = false;
  private boolean wasInTrenchZone = false;

  private VolleyState state = VolleyState.IDLE;

  private SolutionValidity solutionValidity = SolutionValidity.GLOBAL_INVALID;
  private Constants.FieldTargets.AimTarget currentAimTarget = Constants.FieldTargets.AimTarget.HUB;
  private double rawDesiredTurretDeg = Double.NaN;
  private double hoodCompensationRad = 0.0;

  // Estimated field acceleration
  private double lastVelXField = 0.0;
  private double lastVelYField = 0.0;
  private double lastVelTs = -1.0;

  // Soft-limit flip suppression
  private boolean avoidingEdge = false;
  private double suppressShootUntilTs = 0.0;

  // Cached desired turret command each loop (deg in turret-forward frame)
  private double desiredTurretDeg = Double.NaN;

  // Cached solver output
  private TurretHelpers.Solution lastSolution = TurretHelpers.makeInvalidSolution();

  

  public AutoShootSupervisorSubsystem() {

    if (!EnabledSubsystems.supervisor) {
      return;
    }

    // Load artillery table once. If missing/empty, hasAnyData() will be false and
    // solver will return invalid.
    this.table = TurretHelpers.ArtilleryTableIndexedByShooterRpmAndHoodAngle
        .loadFromDeployCsv(Constants.OperatorConstants.ArtilleryTable.DEPLOY_CSV_PATH);
  }

  /**
   * Driver intent: true = attempt to run a volley; false = stop shooting
   * immediately.
   */
    public void setShootRequested(boolean requested) {
    if (requested && !shootRequested) {
      if (DriverStation.isTeleopEnabled()) {
        trenchLockoutActive = false;
      }

      avoidingEdge = false;
      suppressShootUntilTs = 0.0;
      solutionValidity = SolutionValidity.GLOBAL_INVALID;
      hoodCompensationRad = 0.0;
    }

    shootRequested = requested;

    if (!shootRequested) {
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
      RobotContainer.spindexerSubsystem.stop();
      RobotContainer.shooterSubsystem.stopFeederRelatedOutputs();
    }
  }

  public boolean isShootRequested() {
    return shootRequested;
  }

  public VolleyState getVolleyState() {
    return state;
  }

  public TurretHelpers.Solution getLastSolution() {
    return lastSolution;
  }


  // Shot mode: moving solver vs static presets (existing behavior)
  public enum ShotMode {
    MOVING_AUTO,
    STATIC_HUB_BASE,
    STATIC_TOWER_BASE
  }

  private ShotMode shotMode = ShotMode.MOVING_AUTO;

  public void setShotMode(ShotMode mode) {
    shotMode = mode;
  }

  public SolutionValidity getSolutionValidity() {
    return solutionValidity;
  }

  public Constants.FieldTargets.AimTarget getCurrentAimTarget() {
    return currentAimTarget;
  }
  public ShotMode getShotMode() {
    return shotMode;
  }

  public void setCalibrationActive(boolean active) {
    Constants.EnabledSubsystems.calibration = active;
  }

  public boolean isCalibrationActive() {
    return Constants.EnabledSubsystems.calibration;
  }

  @Override
  public void periodic() {

    if (!EnabledSubsystems.supervisor) {
      return;
    }

    final double now = Timer.getFPGATimestamp();
    if (isCalibrationActive()) {
      state = VolleyState.IDLE;
      publishTelemetry();
      return;
    }

    // ------------------------------------------------------------------
    // Teleop-only trench safety interlock (DO NOT affect autos)
    //
    // Behavior:
    // - If we ENTER a trench zone while shootRequested is true, lock out shooting
    // and force hood to neutral.
    // - Lockout clears only when the driver schedules shooting again (i.e. a fresh
    // rising edge of setShootRequested(true)), implemented in setShootRequested().
    // ------------------------------------------------------------------
    final boolean teleopEnabled = DriverStation.isTeleopEnabled();
    boolean effectiveShootRequested = shootRequested;

    if (teleopEnabled) {
      Pose2d pose = RobotContainer.driveSubsystem.getPose();
      boolean inTrenchZone = isInTrenchZoneAllianceAware(pose.getX(), pose.getY());
      boolean enteredTrenchZone = !wasInTrenchZone && inTrenchZone;

      if (!trenchLockoutActive && enteredTrenchZone && shootRequested) {
        trenchLockoutActive = true;
      }

      wasInTrenchZone = inTrenchZone;

      if (trenchLockoutActive) {
        effectiveShootRequested = false;

        // Force hood to neutral continuously while locked out so nothing can
        // re-command it upward during trench traversal.
        RobotContainer.hoodSubsystem.setTargetAngleRad(Constants.OperatorConstants.Hood.NEUTRAL_ANGLE_RAD);
      }
    }

    // --- 1) Compute target position ---
        // --- 1) Read drive state first; target selection depends on pose ---
    var driveState = RobotContainer.driveSubsystem.getState();
    var poseField = driveState.Pose;

    currentAimTarget = selectAimTargetForPose(poseField);
    Translation2d target2d = getAllianceAwareAimTarget(currentAimTarget);

    Translation3d target3d = new Translation3d(
        target2d.getX(),
        target2d.getY(),
        Constants.OperatorConstants.FieldGeometry.HUB_OPENING_CENTER_Z_METERS);

    // CTRE state.Speeds is robot-relative chassis speeds; convert to FIELD frame.
    double vxRobot = driveState.Speeds.vxMetersPerSecond;
    double vyRobot = driveState.Speeds.vyMetersPerSecond;
    Translation2d vField = new Translation2d(vxRobot, vyRobot).rotateBy(poseField.getRotation());
    double omega = driveState.Speeds.omegaRadiansPerSecond;

    Translation2d aField = estimateAccelerationField(now, vField);

    // --- 3) Solve shooting (or just aim) ---

    final boolean isStatic = (shotMode == ShotMode.STATIC_HUB_BASE) || (shotMode == ShotMode.STATIC_TOWER_BASE);

    if (!isStatic) {
      // MOVING: use your existing moving-shot solver (no behavior change)
      lastSolution = TurretHelpers
          .solveForShooterRpmAndHoodAngleCommandsWhileRobotIsMovingUsingMeasuredTableIndexedByRpmAndHood(
              poseField,
              vField,
              aField,
              omega,
              Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC,
              Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS,
              Constants.OperatorConstants.TurretGeometry.BALL_RELEASE_HEIGHT_METERS,
              target3d,
              table,
              Constants.OperatorConstants.ArtillerySolver.TOF_MIN_SEC,
              Constants.OperatorConstants.ArtillerySolver.TOF_MAX_SEC,
              Constants.OperatorConstants.ArtillerySolver.TOF_STEP_SEC,
              Constants.OperatorConstants.ArtillerySolver.GRAVITY_MPS2,
              Constants.OperatorConstants.ArtillerySolver.ANGLE_WEIGHT,
              Constants.OperatorConstants.ArtillerySolver.SPEED_WEIGHT);
    } else {
      // STATIC (failsafe): pose-only yaw-to-hub + hardwired RPM/hood presets (no
      // vision, no solver)

      // Field yaw from robot position -> hub center
      final double dx = target2d.getX() - poseField.getX();
      final double dy = target2d.getY() - poseField.getY();
      final double yawFieldRad = Math.atan2(dy, dx);

      final double shooterRpm = (shotMode == ShotMode.STATIC_TOWER_BASE)
          ? Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_RPM
          : Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_RPM;

      final double hoodAngleRad = Math.toRadians(
          (shotMode == ShotMode.STATIC_TOWER_BASE)
              ? Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_HOOD_DEG
              : Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_HOOD_DEG);

      // Create a "valid" solution object so the rest of the supervisor pipeline stays
      // unchanged.
      lastSolution = new TurretHelpers.Solution(
          true,
          0.0,
          yawFieldRad,
          Double.NaN,
          Double.NaN,
          new Translation3d(),
          shooterRpm,
          hoodAngleRad,
          Double.NaN,
          Double.NaN);
    }

        boolean ballisticValid = lastSolution.valid
        && Double.isFinite(lastSolution.shooterRpmCommand)
        && Double.isFinite(lastSolution.hoodCommandAngleRad)
        && Double.isFinite(lastSolution.yawFieldRad);

    final boolean isStaticForPredict = (shotMode == ShotMode.STATIC_HUB_BASE)
        || (shotMode == ShotMode.STATIC_TOWER_BASE);
    final double omegaForPredict = isStaticForPredict ? 0.0 : omega;

    rawDesiredTurretDeg = ballisticValid
        ? computeDesiredTurretDeg(
            poseField.getRotation().getRadians(),
            omegaForPredict,
            Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC,
            lastSolution.yawFieldRad,
            Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG)
        : Double.NaN;

    boolean turretZoneValid = ballisticValid && isTurretWithinLegalShootZone(rawDesiredTurretDeg);

    if (!ballisticValid) {
      solutionValidity = SolutionValidity.GLOBAL_INVALID;
    } else if (!turretZoneValid) {
      solutionValidity = SolutionValidity.TURRET_ONLY_INVALID;
    } else {
      solutionValidity = SolutionValidity.VALID;
    }

    desiredTurretDeg = Double.isFinite(rawDesiredTurretDeg)
        ? chooseSoftLimitedEquivalent(rawDesiredTurretDeg, now)
        : Double.NaN;

    boolean aimEnabled = Constants.OperatorConstants.AutoShoot.ALWAYS_AIM || effectiveShootRequested;
    if (aimEnabled && Double.isFinite(desiredTurretDeg)) {
      RobotContainer.turretSubsystem.goToAngleDeg(desiredTurretDeg);
    }

    // --- 4) Decide state machine ---

        boolean suppress = now < suppressShootUntilTs;

    boolean turretAimed = isTurretAimed(desiredTurretDeg);
    boolean shooterReady = RobotContainer.shooterSubsystem.isReadyToShoot();
    boolean ballAtThroat = RobotContainer.transferSubsystem.hasBallAtThroat();

    if (Constants.DebugTelemetrySubsystems.supervisor) {
      SmartDashboard.putString("AutoShoot/SolutionValidity", solutionValidity.toString());
      SmartDashboard.putBoolean("AutoShoot/TurretAimed", turretAimed);
      SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooterReady);
      SmartDashboard.putBoolean("AutoShoot/BallAtThroat", ballAtThroat);
      SmartDashboard.putBoolean("AutoShoot/Suppress", suppress);
      SmartDashboard.putBoolean("AutoShoot/TrenchLockoutActive", trenchLockoutActive);
      SmartDashboard.putString("AutoShoot/AimTarget", currentAimTarget.toString());
    }

    // If not requested (or lockout active), keep shooter off and hold transfer at blocked-stage speed.
    if (!effectiveShootRequested) {
      state = VolleyState.IDLE;
      solutionValidity = SolutionValidity.GLOBAL_INVALID;
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
      RobotContainer.spindexerSubsystem.stop();
      RobotContainer.shooterSubsystem.stop();

      if (teleopEnabled) {
        RobotContainer.hoodSubsystem.setTargetAngleRad(Constants.OperatorConstants.Hood.NEUTRAL_ANGLE_RAD);
      }

      publishTelemetry();
      return;
    }

    // Fully invalid shot: behave like shoot cannot run at all.
    if (solutionValidity == SolutionValidity.GLOBAL_INVALID) {
      state = VolleyState.NO_SOLUTION;
      RobotContainer.transferSubsystem.stop();
      RobotContainer.spindexerSubsystem.stop();
      RobotContainer.shooterSubsystem.stop();
      publishTelemetry();
      return;
    }

    // Ballistic solution exists: spin shooter/hood even if turret zone is invalid.
    double compensatedHoodRad = computeCompensatedHoodAngleRad(
        lastSolution.hoodCommandAngleRad,
        lastSolution.shooterRpmCommand);
    hoodCompensationRad = compensatedHoodRad - lastSolution.hoodCommandAngleRad;

    RobotContainer.shooterSubsystem.setTargetRpm(lastSolution.shooterRpmCommand);
    RobotContainer.hoodSubsystem.setTargetAngleRad(compensatedHoodRad);
    RobotContainer.spindexerSubsystem.runSupply();

    // Turret-only invalid: keep aiming and spun up, but do not feed.
    if (solutionValidity == SolutionValidity.TURRET_ONLY_INVALID) {
      state = VolleyState.NO_SOLUTION;
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
      publishTelemetry();
      return;
    }

    // If suppressing due to edge handling, do not feed.
    if (suppress) {
      state = VolleyState.ARMING;
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
      publishTelemetry();
      return;
    }

    boolean okToStartFeed = turretAimed && shooterReady && ballAtThroat;

    if (shotMode == ShotMode.STATIC_HUB_BASE
        || shotMode == ShotMode.STATIC_TOWER_BASE) {

      var speeds = RobotContainer.driveSubsystem.getState().Speeds;

      boolean stopped = Math.abs(speeds.vxMetersPerSecond) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_VX_MPS
          && Math.abs(speeds.vyMetersPerSecond) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_VY_MPS
          && Math.abs(Math.toDegrees(
              speeds.omegaRadiansPerSecond)) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_OMEGA_DEG_PER_S;

      okToStartFeed = okToStartFeed && stopped;
    }

    boolean hoodCompAvailable = isCompensatedHoodAllowed(compensatedHoodRad);
    boolean rpmDroppedTooFar = hasShooterDroppedTooFar(lastSolution.shooterRpmCommand);

    if (state == VolleyState.FIRING) {
      if (!hoodCompAvailable || rpmDroppedTooFar) {
        state = VolleyState.RECOVERING;
      }
    } else if (state == VolleyState.RECOVERING) {
      if (shooterReady) {
        state = VolleyState.ARMING;
      }
    }

    if (state == VolleyState.RECOVERING) {
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
      RobotContainer.spindexerSubsystem.runSlow();
      publishTelemetry();
      return;
    }

    if (state == VolleyState.FIRING || okToStartFeed) {
      state = VolleyState.FIRING;
      RobotContainer.transferSubsystem.runFeed();
    } else {
      state = VolleyState.ARMING;
      RobotContainer.transferSubsystem.runVelocityRps(
          Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
    }

    publishTelemetry();
  }
   

  private Translation2d estimateAccelerationField(double now, Translation2d vField) {
    if (lastVelTs < 0) {
      lastVelTs = now;
      lastVelXField = vField.getX();
      lastVelYField = vField.getY();
      return new Translation2d(0.0, 0.0);
    }
    double dt = Math.max(1e-3, now - lastVelTs);
    double ax = (vField.getX() - lastVelXField) / dt;
    double ay = (vField.getY() - lastVelYField) / dt;
    lastVelTs = now;
    lastVelXField = vField.getX();
    lastVelYField = vField.getY();
    return new Translation2d(ax, ay);
  }

  private static Translation2d getAllianceHubTarget() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
      return new Translation2d(Constants.FieldTargets.HUB_RED_X, Constants.FieldTargets.HUB_RED_Y);
    }
    return new Translation2d(Constants.FieldTargets.HUB_BLUE_X, Constants.FieldTargets.HUB_BLUE_Y);
  }

  // ------------------------------------------------------------
  // Trench safety zone helpers (teleop-only usage)
  // ------------------------------------------------------------
  private static boolean isInTrenchZoneAllianceAware(double poseX, double poseY) {
    // Define zones in BLUE-alliance field coordinates, then mirror across the field
    // length for RED. This assumes your field coordinate frame is the standard one
    // where alliance mirroring is an X flip. If your odometry uses a different
    // convention, you MUST adjust the mirroring (otherwise zones will be wrong).
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

    // Convert current pose into the BLUE frame for zone checks.
    double xBlueFrame = isRed
        ? (Constants.OperatorConstants.FieldGeometry.FIELD_LENGTH_METERS - poseX)
        : poseX;

    // NOTE: Y is not mirrored here. If you end up needing a Y mirror for your
    // coordinate system, mirror it consistently for both zones.
    double yBlueFrame = poseY;

    // Zone 1
    if (isInRect(
        xBlueFrame,
        yBlueFrame,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE1_MIN_X_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE1_MAX_X_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE1_MIN_Y_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE1_MAX_Y_METERS)) {
      return true;
    }

    // Zone 2
    if (isInRect(
        xBlueFrame,
        yBlueFrame,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE2_MIN_X_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE2_MAX_X_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE2_MIN_Y_METERS,
        Constants.OperatorConstants.FieldGeometry.BLUE_TRENCH_ZONE2_MAX_Y_METERS)) {
      return true;
    }

    return false;
  }

  private static boolean isInRect(
      double x,
      double y,
      double minX,
      double maxX,
      double minY,
      double maxY) {

    // Defensive: tolerate accidental min/max reversal.
    double loX = Math.min(minX, maxX);
    double hiX = Math.max(minX, maxX);
    double loY = Math.min(minY, maxY);
    double hiY = Math.max(minY, maxY);

    return (x >= loX) && (x <= hiX) && (y >= loY) && (y <= hiY);
  }

    private static Translation2d getAllianceAwareAimTarget(Constants.FieldTargets.AimTarget target) {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    return new Translation2d(target.getX(isRed), target.getY(isRed));
  }

  private static Constants.FieldTargets.AimTarget selectAimTargetForPose(Pose2d pose) {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

    double xBlueFrame = isRed
        ? (Constants.OperatorConstants.FieldGeometry.FIELD_LENGTH_METERS - pose.getX())
        : pose.getX();

    double yBlueFrame = pose.getY();

    if (xBlueFrame <= Constants.FieldTargets.ALLIANCE_ZONE_MAX_X_BLUE_FRAME_METERS) {
      return Constants.FieldTargets.AimTarget.HUB;
    }

    if (yBlueFrame <= Constants.FieldTargets.NEUTRAL_ZONE_Y_SPLIT_METERS) {
      return Constants.FieldTargets.AimTarget.NEUTRAL_LOW;
    }

    return Constants.FieldTargets.AimTarget.NEUTRAL_HIGH;
  }

  private static boolean isTurretWithinLegalShootZone(double turretDeg) {
    return Double.isFinite(turretDeg)
        && turretDeg >= Constants.OperatorConstants.Turret.MIN_ANGLE_DEG
        && turretDeg <= Constants.OperatorConstants.Turret.MAX_ANGLE_DEG;
  }

  private double computeCompensatedHoodAngleRad(double baseHoodRad, double targetRpm) {
    if (!Double.isFinite(baseHoodRad) || !Double.isFinite(targetRpm) || targetRpm <= 1.0) {
      return baseHoodRad;
    }

    double currentRpm = RobotContainer.shooterSubsystem.getVelocityRpm();
    double rpmFraction = MathUtil.clamp(currentRpm / targetRpm, 0.0, 1.0);
    double hoodCompDeg = (1.0 - rpmFraction)
        * Constants.OperatorConstants.AutoShoot.HOOD_COMP_DEG_PER_UNIT_RPM_DROP;
    hoodCompDeg = MathUtil.clamp(
        hoodCompDeg,
        0.0,
        Constants.OperatorConstants.AutoShoot.HOOD_COMP_MAX_DEG);

    return baseHoodRad + Math.toRadians(hoodCompDeg);
  }

  private static boolean isCompensatedHoodAllowed(double compensatedHoodRad) {
    return Double.isFinite(compensatedHoodRad)
        && compensatedHoodRad >= Constants.OperatorConstants.Hood.MIN_ANGLE_RAD
        && compensatedHoodRad <= Constants.OperatorConstants.Hood.MAX_ANGLE_RAD;
  }

  private static boolean hasShooterDroppedTooFar(double targetRpm) {
    if (!Double.isFinite(targetRpm) || targetRpm <= 1.0) {
      return true;
    }

    double currentRpm = RobotContainer.shooterSubsystem.getVelocityRpm();
    return currentRpm <= (targetRpm * Constants.OperatorConstants.AutoShoot.RECOVERY_RPM_FRACTION_LIMIT);
  }

  /**
   * Convert a desired field yaw (radians) into turret-forward-frame degrees.
   *
   * Steps:
   * - predict robot heading at release time: headingNow + omega * dtRelease
   * - compute field->robot relative angle: yawField - predictedHeading
   * - shift by 180 deg if turret zero points robot BACK
   * - output degrees in (-180..+180] then allow caller to choose equivalent ±360
   */
  private static double computeDesiredTurretDeg(
      double robotHeadingFieldRad,
      double omegaRadPerSec,
      double dtReleaseSec,
      double desiredYawFieldRad,
      double turretZeroOffsetFromRobotFwdDeg) {

    double predictedHeading = robotHeadingFieldRad + omegaRadPerSec * dtReleaseSec;
    double robotRelative = MathUtil.angleModulus(desiredYawFieldRad - predictedHeading);

    // Convert robot-forward-relative yaw into turret-frame degrees where turret "0"
    // is your defined zero direction.
    robotRelative = MathUtil.angleModulus(
        robotRelative - Math.toRadians(turretZeroOffsetFromRobotFwdDeg));

    return Math.toDegrees(robotRelative);
  }

  /** Simple aim check: compare current continuous turret angle to desired. */
  private boolean isTurretAimed(double desiredDeg) {
    if (!Double.isFinite(desiredDeg))
      return false;
    double err = Math.abs(desiredDeg - RobotContainer.turretSubsystem.getContinuousAngleDeg());
    return err <= Constants.OperatorConstants.Turret.AIM_TOLERANCE_DEG;
  }

  /**
   * Soft-limit selection to avoid living at the ends of turret travel.
   *
   * Also implements "flip suppression": if we unwrap/flip, we suppress feeding
   * briefly so we don't fire mid-swing.
   */
  private double chooseSoftLimitedEquivalent(double desiredDeg, double nowTs) {
    // Asymmetric soft limits (inside the hard mechanical stops).
    double softMin = Constants.OperatorConstants.Turret.SOFT_AIM_MIN_DEG;
    double softMax = Constants.OperatorConstants.Turret.SOFT_AIM_MAX_DEG;
    double margin = Constants.OperatorConstants.Turret.LIMIT_MARGIN_DEG;

    double currentDeg = RobotContainer.turretSubsystem.getContinuousAngleDeg();

    // Candidate(s): keep this structure so you can add ±360 equivalents later if
    // desired.
    double[] cands = new double[] { desiredDeg };
    double best = Double.NaN;
    double bestScore = Double.POSITIVE_INFINITY;

    for (double c : cands) {
      if (c < softMin || c > softMax) {
        continue;
      }

      // Penalty for living near edges
      double edgePenalty = 0.0;
      if (c < softMin + margin) {
        edgePenalty = (softMin + margin) - c;
      } else if (c > softMax - margin) {
        edgePenalty = c - (softMax - margin);
      }

      // Prefer small motion from current
      double motionPenalty = Math.abs(c - currentDeg);

      double score = motionPenalty + 10.0 * edgePenalty;
      if (score < bestScore) {
        bestScore = score;
        best = c;
      }
    }

    // If nothing fit, just clamp (and suppress firing briefly).
    if (!Double.isFinite(best)) {
      best = MathUtil.clamp(desiredDeg, softMin, softMax);

      // If we're near an edge, treat it as "avoiding" and suppress feed briefly.
      boolean nearEdge = best < softMin + margin || best > softMax - margin;
      if (nearEdge && !avoidingEdge) {
        avoidingEdge = true;
        suppressShootUntilTs = nowTs + Constants.OperatorConstants.AutoShoot.FLIP_SUPPRESS_SEC;
      } else if (!nearEdge) {
        avoidingEdge = false;
      }
    } else {
      avoidingEdge = false;
    }

    return best;
  }

  private void publishTelemetry() {
    if (!Constants.DebugTelemetrySubsystems.supervisor) {
      return;
    }

    SmartDashboard.putString("AutoShoot/State", state.toString());
    SmartDashboard.putString("AutoShoot/SolutionValidity", solutionValidity.toString());
    SmartDashboard.putString("AutoShoot/AimTarget", currentAimTarget.toString());
    SmartDashboard.putBoolean("AutoShoot/ShootRequested", shootRequested);
    SmartDashboard.putNumber("AutoShoot/RawDesiredTurretDeg", rawDesiredTurretDeg);
    SmartDashboard.putNumber("AutoShoot/DesiredTurretDeg", desiredTurretDeg);
    SmartDashboard.putNumber("AutoShoot/HoodCompDeg", Math.toDegrees(hoodCompensationRad));
    SmartDashboard.putBoolean("AutoShoot/AvoidingEdge", avoidingEdge);
    SmartDashboard.putNumber("AutoShoot/SuppressUntilTs", suppressShootUntilTs);
    SmartDashboard.putNumber("AutoShoot/LastSol/Rpm", lastSolution.shooterRpmCommand);
    SmartDashboard.putNumber("AutoShoot/LastSol/HoodDeg", Math.toDegrees(lastSolution.hoodCommandAngleRad));
    SmartDashboard.putNumber("AutoShoot/LastSol/YawFieldDeg", Math.toDegrees(lastSolution.yawFieldRad));
  }

}