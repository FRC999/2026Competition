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
 */
public class AutoShootSupervisorSubsystem extends SubsystemBase {
  private static final int PERF_PUBLISH_EVERY_LOOPS = 25;

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
  private TurretHelpers.MovingAutoShotTable movingAutoShotTable;

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

  // Soft-limit flip suppression
  private boolean avoidingEdge = false;
  private double suppressShootUntilTs = 0.0;

  // Cached desired turret command each loop (deg in turret-forward frame)
  private double desiredTurretDeg = Double.NaN;

  // Cached solver output
  private TurretHelpers.Solution lastSolution = TurretHelpers.makeInvalidSolution();

  private final Timer shotCooldownTimer = new Timer();
  private boolean shotCooldownActive = false;
  private boolean lastBallAtThroat = false;
  private double shootRequestStartTs = -1.0;
  private static final double FEED_FORCE_START_AFTER_S = 1.0;
  private long periodicRuntimeAccumNs = 0L;
  private long periodicRuntimeMaxNs = 0L;
  private int periodicRuntimeSamples = 0;

  public AutoShootSupervisorSubsystem() {

    if (!EnabledSubsystems.supervisor) {
      return;
    }

    this.movingAutoShotTable =
        TurretHelpers.MovingAutoShotTable.loadFromDeployCsv(
            Constants.OperatorConstants.MovingAutoShotTable.DEPLOY_CSV_PATH);

    // Load artillery table once. If missing/empty, hasAnyData() will be false and
    // solver will return invalid.
    this.table = TurretHelpers.ArtilleryTableIndexedByShooterRpmAndHoodAngle
        .loadFromDeployCsv(Constants.OperatorConstants.ArtilleryTable.DEPLOY_CSV_PATH);
    shotCooldownTimer.stop();
    shotCooldownTimer.reset();
  }

  private void recordPeriodicRuntime(long elapsedNs) {
    if (!Constants.DebugTelemetrySubsystems.perfLight) {
      return;
    }

    periodicRuntimeAccumNs += elapsedNs;
    periodicRuntimeMaxNs = Math.max(periodicRuntimeMaxNs, elapsedNs);
    periodicRuntimeSamples++;

    if (periodicRuntimeSamples >= PERF_PUBLISH_EVERY_LOOPS) {
      SmartDashboard.putNumber(
          "Perf/AutoShoot/PeriodicMsAvg",
          periodicRuntimeAccumNs / 1_000_000.0 / periodicRuntimeSamples);
      SmartDashboard.putNumber("Perf/AutoShoot/PeriodicMsMax", periodicRuntimeMaxNs / 1_000_000.0);
      periodicRuntimeAccumNs = 0L;
      periodicRuntimeMaxNs = 0L;
      periodicRuntimeSamples = 0;
    }
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
      shootRequestStartTs = Timer.getFPGATimestamp();
    }

    shootRequested = requested;

    if (!shootRequested) {
      shootRequestStartTs = -1.0;
      RobotContainer.transferSubsystem.runThroat();
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

    public TurretHelpers.Solution calculateDiagnosticSolution() {
    if (!EnabledSubsystems.supervisor) {
      return TurretHelpers.makeInvalidSolution();
    }

    var driveState = RobotContainer.driveSubsystem.getState();
    var poseField = driveState.Pose;

    currentAimTarget = Constants.FieldTargets.AimTarget.HUB;
    Translation2d target2d = getAllianceAwareAimTarget(currentAimTarget);

    final boolean isStatic = isStaticShotMode(shotMode);

    double omega = driveState.Speeds.omegaRadiansPerSecond;

    TurretHelpers.Solution solution;

        if (!isStatic) {
          solution = solveDistanceInterpolatedMovingAutoShot(poseField, target2d);
        } else {
      final double dx = target2d.getX() - poseField.getX();
      final double dy = target2d.getY() - poseField.getY();
      final double yawFieldRad = Math.atan2(dy, dx);

            final double shooterRpm;
      final double hoodAngleRad;

            if (shotMode == ShotMode.STATIC_TOWER_BASE) {
        shooterRpm = Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_RPM;
        hoodAngleRad =
            Math.toRadians(Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_HOOD_DEG);

        solution =
            new TurretHelpers.Solution(
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

      } else if (shotMode == ShotMode.STATIC_HUB_BASE) {
        shooterRpm = Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_RPM;
        hoodAngleRad =
            Math.toRadians(Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_HOOD_DEG);

        solution =
            new TurretHelpers.Solution(
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

      } else if (shotMode == ShotMode.MANUAL_FIXED) {
        double throttle =
            MathUtil.clamp(RobotContainer.getTurretStick().getThrottle(), -1.0, 1.0);
        shooterRpm =
            Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_BASE_RPM
                + throttle * Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_RPM_TRIM_RANGE;
        hoodAngleRad =
            Math.toRadians(Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_HOOD_DEG);

        solution =
            new TurretHelpers.Solution(
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

      } else {
        solution =
            solveManualPresetDistanceShot(
                poseField,
                target2d,
                getManualPresetDistanceMeters(shotMode));
      }
    }

        return solution;
  }


  // Shot mode: moving solver vs static presets (existing behavior)
    public enum ShotMode {
    MOVING_AUTO,
    STATIC_HUB_BASE,
    STATIC_TOWER_BASE,
    MANUAL_FIXED,
    MANUAL_PRESET_2M,
    MANUAL_PRESET_3M,
    MANUAL_PRESET_4M
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
    long startNs = Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() : 0L;

    if (!EnabledSubsystems.supervisor) {
      return;
    }

    if (RobotContainer.isPanicStopActive()) {
      shootRequested = false;
      state = VolleyState.IDLE;
      RobotContainer.transferSubsystem.stop();
      RobotContainer.spindexerSubsystem.stop();
      RobotContainer.shooterSubsystem.stop();
      publishTelemetry();
      recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
      return;
    }


    final double now = Timer.getFPGATimestamp();
    if (isCalibrationActive()) {
      state = VolleyState.IDLE;
      publishTelemetry();
      recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
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

    // --- 1) Read drive state first; target selection depends on pose ---
    var driveState = RobotContainer.driveSubsystem.getState();
    var poseField = driveState.Pose;
    //System.out.println("Pose to autoshoot: " + poseField.toString());
    boolean manualTurretMode = RobotContainer.isHubTrackingDisabledByButtonBox();
    boolean aimEnabled =
        (Constants.OperatorConstants.AutoShoot.ALWAYS_AIM || effectiveShootRequested)
            && !manualTurretMode;
    boolean shouldComputeAimTarget = aimEnabled || effectiveShootRequested;

    Translation2d target2d = null;
    if (shouldComputeAimTarget) {
      currentAimTarget = selectAimTargetForPose(poseField);
      target2d = getAllianceAwareAimTarget(currentAimTarget);
    }

    // CTRE state.Speeds is robot-relative chassis speeds.
    double omega = driveState.Speeds.omegaRadiansPerSecond;
    final boolean isStatic = isStaticShotMode(shotMode);

    // When not actively shooting, keep turret tracking cheap:
    // do geometric hub tracking only, and skip the full ballistic solver.
    if (!effectiveShootRequested) {
      lastSolution = TurretHelpers.makeInvalidSolution();
      hoodCompensationRad = 0.0;

      if (aimEnabled && target2d != null) {
        rawDesiredTurretDeg =
            TurretHelpers.computeStationaryRawTurretYawDeg(poseField, target2d);

        boolean turretZoneValid =
            Double.isFinite(rawDesiredTurretDeg)
                && isTurretWithinLegalShootZone(rawDesiredTurretDeg);

        solutionValidity =
            turretZoneValid
                ? SolutionValidity.VALID
                : SolutionValidity.TURRET_ONLY_INVALID;

        desiredTurretDeg =
            Double.isFinite(rawDesiredTurretDeg)
                ? chooseSoftLimitedEquivalent(rawDesiredTurretDeg, now)
                : Double.NaN;
      } else {
        rawDesiredTurretDeg = Double.NaN;
        desiredTurretDeg = Double.NaN;
        solutionValidity = SolutionValidity.GLOBAL_INVALID;
      }

    } else {
      // Actively shooting: run the existing full solution path.
      if (shotMode == ShotMode.MOVING_AUTO) {
        lastSolution = solveDistanceInterpolatedMovingAutoShot(poseField, target2d);
      } else {
        final double dx = target2d.getX() - poseField.getX();
        final double dy = target2d.getY() - poseField.getY();
        final double yawFieldRad = Math.atan2(dy, dx);

        final double shooterRpm;
        final double hoodAngleRad;

                if (shotMode == ShotMode.STATIC_TOWER_BASE) {
          shooterRpm = Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_RPM;
          hoodAngleRad =
              Math.toRadians(Constants.OperatorConstants.AutoShoot.STATIC_TOWER_BASE_HOOD_DEG);

          lastSolution =
              new TurretHelpers.Solution(
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

        } else if (shotMode == ShotMode.STATIC_HUB_BASE) {
          shooterRpm = Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_RPM;
          hoodAngleRad =
              Math.toRadians(Constants.OperatorConstants.AutoShoot.STATIC_HUB_BASE_HOOD_DEG);

          lastSolution =
              new TurretHelpers.Solution(
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

        } else if (shotMode == ShotMode.MANUAL_FIXED) {
          double twist = MathUtil.clamp(RobotContainer.getTurretStick().getThrottle(), -1.0, 1.0);
          shooterRpm =
              Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_BASE_RPM
                  + twist * Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_RPM_TRIM_RANGE;
          hoodAngleRad =
              Math.toRadians(Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_HOOD_DEG);

          lastSolution =
              new TurretHelpers.Solution(
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

        } else {
          lastSolution =
              solveManualPresetDistanceShot(
                  poseField,
                  target2d,
                  getManualPresetDistanceMeters(shotMode));
        }
      }

      // lastSolution = applyEmpiricalMovingAutoShotCorrection(lastSolution, poseField, target2d);

      boolean ballisticValid =
          lastSolution.valid
              && Double.isFinite(lastSolution.shooterRpmCommand)
              && Double.isFinite(lastSolution.hoodCommandAngleRad)
              && Double.isFinite(lastSolution.yawFieldRad);

      final boolean isStaticForPredict = isStaticShotMode(shotMode);
      final double omegaForPredict = isStaticForPredict ? 0.0 : omega;

      if (ballisticValid) {
        if (isStaticForPredict) {
          rawDesiredTurretDeg =
              TurretHelpers.computeStationaryRawTurretYawDeg(poseField, target2d);
        } else {
          rawDesiredTurretDeg =
              computeDesiredTurretDeg(
                  poseField.getRotation().getRadians(),
                  omegaForPredict,
                  Constants.OperatorConstants.AutoShoot.DT_RELEASE_SEC,
                  lastSolution.yawFieldRad,
                  Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG);
        }
      } else {
        rawDesiredTurretDeg = Double.NaN;
      }

      boolean turretZoneValid =
          ballisticValid && isTurretWithinLegalShootZone(rawDesiredTurretDeg);

      if (!ballisticValid) {
        solutionValidity = SolutionValidity.GLOBAL_INVALID;
      } else if (!turretZoneValid) {
        solutionValidity = SolutionValidity.TURRET_ONLY_INVALID;
      } else {
        solutionValidity = SolutionValidity.VALID;
      }

      desiredTurretDeg =
          Double.isFinite(rawDesiredTurretDeg)
              ? chooseSoftLimitedEquivalent(rawDesiredTurretDeg, now)
              : Double.NaN;
    }

    if (aimEnabled && Double.isFinite(desiredTurretDeg)) {
      RobotContainer.turretSubsystem.goToAngleDeg(desiredTurretDeg);
    }

    // --- 4) Decide state machine ---

        boolean suppress = now < suppressShootUntilTs;

    boolean turretAimed = manualTurretMode || isTurretAimed(desiredTurretDeg);
    boolean shooterReady = RobotContainer.shooterSubsystem.isReadyToShoot();
    boolean ballAtThroat = RobotContainer.transferSubsystem.hasBallAtThroat();
    if (lastBallAtThroat && !ballAtThroat) {
  shotCooldownTimer.reset();
  shotCooldownTimer.start();
  shotCooldownActive = true;
}

lastBallAtThroat = ballAtThroat;


    double yawFieldToUse = Double.NaN;
    if (target2d != null) {
      double dx = target2d.getX() - poseField.getX();
      double dy = target2d.getY() - poseField.getY();
      yawFieldToUse = Math.atan2(dy, dx);  // Yaw to face the hub
    }

    if (Constants.DebugTelemetrySubsystems.supervisor) {
      SmartDashboard.putString("AutoShoot/SolutionValidity", solutionValidity.toString());
      SmartDashboard.putBoolean("AutoShoot/TurretAimed", turretAimed);
      SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooterReady);
      SmartDashboard.putBoolean("AutoShoot/BallAtThroat", ballAtThroat);
      SmartDashboard.putBoolean("AutoShoot/Suppress", suppress);
      SmartDashboard.putBoolean("AutoShoot/TrenchLockoutActive", trenchLockoutActive);
      SmartDashboard.putString("AutoShoot/AimTarget", currentAimTarget.toString());

      // Log robot pose and velocity
      SmartDashboard.putNumber("TurretTesting/RobotPoseX", poseField.getX());
      SmartDashboard.putNumber("TurretTesting/RobotPoseY", poseField.getY());
      SmartDashboard.putNumber("TurretTesting/RobotRotation", poseField.getRotation().getDegrees());

      SmartDashboard.putNumber("TurretTesting/vxRobot", driveState.Speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("TurretTesting/vyRobot", driveState.Speeds.vyMetersPerSecond);

      // Log target position
      SmartDashboard.putNumber("TurretTesting/TargetX", target2d != null ? target2d.getX() : Double.NaN);
      SmartDashboard.putNumber("TurretTesting/TargetY", target2d != null ? target2d.getY() : Double.NaN);

      // Log yaw to face the hub (direct yaw calculation)
      SmartDashboard.putNumber("TurretTesting/TargetYawField", Math.toDegrees(yawFieldToUse));

      // Log raw desired turret angle
      SmartDashboard.putNumber("TurretTesting/RawDesiredTurretDeg", rawDesiredTurretDeg);
      SmartDashboard.putNumber("Turret/CurrentAngle", RobotContainer.turretSubsystem.getRelativePosition()); 

      // Log CTRE pose (if available)
      SmartDashboard.putNumber("TurretTesting/RobotPoseX", poseField.getX());
      SmartDashboard.putNumber("TurretTesting/RobotPoseY", poseField.getY());
      SmartDashboard.putNumber("TurretTesting/RobotRotation", poseField.getRotation().getDegrees());

      SmartDashboard.putNumber(
          "TurretTesting/StaticPivotAwareTurretDeg",
          target2d != null
              ? TurretHelpers.computeStationaryRawTurretYawDeg(poseField, target2d)
              : Double.NaN);

      SmartDashboard.putNumber("TurretTesting/StaticRobotCenterYawDeg",
          target2d != null
              ? computeDesiredTurretDeg(
                  poseField.getRotation().getRadians(),
                  0.0,
                  0.0,
                  Math.atan2(target2d.getY() - poseField.getY(), target2d.getX() - poseField.getX()),
                  Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG)
              : Double.NaN);

      SmartDashboard.putNumber("TurretTesting/DesiredTurretDegFinal", desiredTurretDeg);
      SmartDashboard.putNumber("TurretTesting/TurretZeroOffsetDeg",
          Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG);
      SmartDashboard.putNumber("TurretTesting/TurretPivotOffsetX",
          Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS.getX());
      SmartDashboard.putNumber("TurretTesting/TurretPivotOffsetY",
          Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS.getY());

      // Log turret command issuance
    }
    if (shotCooldownActive && shotCooldownTimer.hasElapsed(0.015)) {
      shotCooldownActive = false;
    }

    

    // If not requested (or lockout active), keep shooter off and hold transfer at blocked-stage speed.
    if (!effectiveShootRequested) {
      state = VolleyState.IDLE;
      solutionValidity = SolutionValidity.GLOBAL_INVALID;
      RobotContainer.transferSubsystem.stop();
      RobotContainer.spindexerSubsystem.stop();

      if (DriverStation.isEnabled()) {
        RobotContainer.shooterSubsystem.setTargetRpm(
            Constants.OperatorConstants.AutoShoot.IDLE_SHOOTER_RPM);
      } else {
        RobotContainer.shooterSubsystem.stop();
      }

      if (teleopEnabled) {
        RobotContainer.hoodSubsystem.setTargetAngleRad(Constants.OperatorConstants.Hood.NEUTRAL_ANGLE_RAD);
      }

      publishTelemetry();
      recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
      return;
    }

    // Fully invalid shot: behave like shoot cannot run at all.
    if (solutionValidity == SolutionValidity.GLOBAL_INVALID) {
      state = VolleyState.NO_SOLUTION;
      RobotContainer.transferSubsystem.stop();
      RobotContainer.spindexerSubsystem.stop();
      RobotContainer.shooterSubsystem.stop();
      publishTelemetry();
      recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
      return;
    }

    // Ballistic solution exists: spin shooter/hood even if turret zone is invalid.
    double compensatedHoodRad = computeCompensatedHoodAngleRad(
        lastSolution.hoodCommandAngleRad,
        lastSolution.shooterRpmCommand);
    hoodCompensationRad = compensatedHoodRad - lastSolution.hoodCommandAngleRad;

    RobotContainer.shooterSubsystem.setTargetRpm(lastSolution.shooterRpmCommand);
    RobotContainer.hoodSubsystem.setTargetAngleRad(compensatedHoodRad);
    // RobotContainer.spindexerSubsystem.runSupply();

    

    // If suppressing due to edge handling, do not feed.
  if (suppress) {
    state = VolleyState.ARMING;
    RobotContainer.transferSubsystem.stop();
    RobotContainer.spindexerSubsystem.stop();
    publishTelemetry();
    recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
    return;
  }

      boolean feedTimeoutElapsed =
        shootRequestStartTs >= 0.0
            && (now - shootRequestStartTs) >= FEED_FORCE_START_AFTER_S;

    boolean okToStartFeed = shooterReady || feedTimeoutElapsed;

    if (shotMode == ShotMode.STATIC_HUB_BASE
        || shotMode == ShotMode.STATIC_TOWER_BASE) {

      var speeds = driveState.Speeds;

      boolean stopped = Math.abs(speeds.vxMetersPerSecond) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_VX_MPS
          && Math.abs(speeds.vyMetersPerSecond) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_VY_MPS
          && Math.abs(Math.toDegrees(
              speeds.omegaRadiansPerSecond)) < Constants.OperatorConstants.AutoShoot.STATIC_MAX_OMEGA_DEG_PER_S;

      okToStartFeed = okToStartFeed && stopped;
    }

    boolean hoodCompAvailable = isCompensatedHoodAllowed(compensatedHoodRad);
        boolean shouldRecover = !hoodCompAvailable;

    if (state == VolleyState.FIRING) {
      if (shouldRecover) {
        state = VolleyState.RECOVERING;
      }
    } 
    
    else if (state == VolleyState.RECOVERING) {
      if (shooterReady) {
        state = VolleyState.ARMING;
      }
    }

    if (state == VolleyState.RECOVERING) {
      RobotContainer.transferSubsystem.stop();
      RobotContainer.spindexerSubsystem.stop();
      publishTelemetry();
      recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
      return;
    }

 if (state == VolleyState.FIRING || okToStartFeed) {

  if (state != VolleyState.FIRING) {
    // entering FIRING for first time
    shotCooldownTimer.reset();
    shotCooldownTimer.start();
    shotCooldownActive = true;
  }

  state = VolleyState.FIRING;

  if (!shotCooldownActive) {
    RobotContainer.transferSubsystem.runFeed();
    RobotContainer.spindexerSubsystem.runSupply();
  } else {
    RobotContainer.transferSubsystem.stop();
    RobotContainer.spindexerSubsystem.stop();
  }

} else {
  state = VolleyState.ARMING;
  RobotContainer.transferSubsystem.stop();
  RobotContainer.spindexerSubsystem.stop();
}

    publishTelemetry();
    recordPeriodicRuntime(Constants.DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
  }
   

  private void seedMovingAutoDistanceTables() {
    // movingAutoRpmByDistance.clear();
    // movingAutoHoodDegByDistance.clear();

    // double[] distances = Constants.OperatorConstants.AutoShoot.MOVING_AUTO_SHOT_DISTANCE_M;
    // double[] rpms = Constants.OperatorConstants.AutoShoot.MOVING_AUTO_SHOT_RPM;
    // double[] hoods = Constants.OperatorConstants.AutoShoot.MOVING_AUTO_SHOT_HOOD_DEG;

    // int n = Math.min(distances.length, Math.min(rpms.length, hoods.length));
    // for (int i = 0; i < n; i++) {
    //   movingAutoRpmByDistance.put(distances[i], rpms[i]);
    //   movingAutoHoodDegByDistance.put(distances[i], hoods[i]);
    // }
  }

    private boolean isStaticShotMode(ShotMode mode) {
    return mode == ShotMode.STATIC_HUB_BASE
        || mode == ShotMode.STATIC_TOWER_BASE
        || mode == ShotMode.MANUAL_FIXED
        || mode == ShotMode.MANUAL_PRESET_2M
        || mode == ShotMode.MANUAL_PRESET_3M
        || mode == ShotMode.MANUAL_PRESET_4M;
  }

  private double getManualPresetDistanceMeters(ShotMode mode) {
    switch (mode) {
      case MANUAL_PRESET_2M:
        return 2.0;
      case MANUAL_PRESET_3M:
        return 3.0;
      case MANUAL_PRESET_4M:
        return 4.0;
      default:
        return Double.NaN;
    }
  }

    private TurretHelpers.Solution solveManualPresetDistanceShot(
      Pose2d poseField,
      Translation2d target2d,
      double presetDistanceMeters) {

    if (movingAutoShotTable == null || !movingAutoShotTable.hasAnyData()) {
      return TurretHelpers.makeInvalidSolution();
    }

    double currentTurretAngleDeg = RobotContainer.turretSubsystem.getAngleDeg();
    double preferredShooterRpm = RobotContainer.shooterSubsystem.getTargetRpm();

    TurretHelpers.MovingAutoShotCommand shot =
        movingAutoShotTable.findInterpolatedShot(
            presetDistanceMeters,
            currentTurretAngleDeg,
            preferredShooterRpm);

    if (!shot.valid
        || !Double.isFinite(shot.shooterRpmCommand)
        || !Double.isFinite(shot.hoodCommandAngleRad)) {
      return TurretHelpers.makeInvalidSolution();
    }

    Translation2d turretCenterField =
        poseField.getTranslation().plus(
            Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
                .rotateBy(poseField.getRotation()));

    double yawFieldRad =
        Math.atan2(
            target2d.getY() - turretCenterField.getY(),
            target2d.getX() - turretCenterField.getX());

    return new TurretHelpers.Solution(
        true,
        0.0,
        yawFieldRad,
        Double.NaN,
        Double.NaN,
        new Translation3d(),
        shot.shooterRpmCommand,
        shot.hoodCommandAngleRad,
        Double.NaN,
        Double.NaN);
  }

    private TurretHelpers.Solution solveDistanceInterpolatedMovingAutoShot(
      Pose2d poseField,
      Translation2d target2d) {

    if (movingAutoShotTable == null || !movingAutoShotTable.hasAnyData()) {
      return TurretHelpers.makeInvalidSolution();
    }

    double distanceMeters = computeTurretCenterToTargetDistanceMeters(poseField, target2d);
    double currentTurretAngleDeg = RobotContainer.turretSubsystem.getAngleDeg();

        double preferredShooterRpm = RobotContainer.shooterSubsystem.getTargetRpm();

    TurretHelpers.MovingAutoShotCommand shot =
        movingAutoShotTable.findInterpolatedShot(
            distanceMeters,
            currentTurretAngleDeg,
            preferredShooterRpm);

    if (!shot.valid
        || !Double.isFinite(shot.shooterRpmCommand)
        || !Double.isFinite(shot.hoodCommandAngleRad)) {
      return TurretHelpers.makeInvalidSolution();
    }

    Translation2d turretCenterField =
        poseField.getTranslation().plus(
            Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
                .rotateBy(poseField.getRotation()));

    double yawFieldRad =
        Math.atan2(
            target2d.getY() - turretCenterField.getY(),
            target2d.getX() - turretCenterField.getX());

    return new TurretHelpers.Solution(
        true,
        0.0,
        yawFieldRad,
        Double.NaN,
        Double.NaN,
        new Translation3d(),
        shot.shooterRpmCommand,
        shot.hoodCommandAngleRad,
        Double.NaN,
        Double.NaN);
  }

  private static double computeTurretCenterToTargetDistanceMeters(
      Pose2d poseField,
      Translation2d target2d) {
    Translation2d turretCenterField =
        poseField.getTranslation().plus(
            Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
                .rotateBy(poseField.getRotation()));

    return turretCenterField.getDistance(target2d);
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

    public static Translation2d getAllianceAwareAimTarget(Constants.FieldTargets.AimTarget target) {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    return new Translation2d(target.getX(isRed), target.getY(isRed));
  }

  private static Constants.FieldTargets.AimTarget selectAimTargetForPose(Pose2d pose) {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

    if(!isRed && pose.getX()>=4.664 && pose.getY()<=4.002){
      return Constants.FieldTargets.AimTarget.NEUTRAL_LOW;
    }
    if(!isRed && pose.getX()>=4.664 && pose.getY()>4.002){
      return Constants.FieldTargets.AimTarget.NEUTRAL_HIGH;
    }
    if(isRed && pose.getX()<=11.942 && pose.getY()<=4.002){
      return Constants.FieldTargets.AimTarget.NEUTRAL_HIGH;
    }
    if(isRed && pose.getX()<=11.942 && pose.getY()>4.002){
      return Constants.FieldTargets.AimTarget.NEUTRAL_LOW;
    }

    return Constants.FieldTargets.AimTarget.HUB;

    // double xBlueFrame = isRed
    //     ? (Constants.OperatorConstants.FieldGeometry.FIELD_LENGTH_METERS - pose.getX())
    //     : pose.getX();

    // double yBlueFrame = pose.getY();

    // if (xBlueFrame <= Constants.FieldTargets.ALLIANCE_ZONE_MAX_X_BLUE_FRAME_METERS) {
    //   return Constants.FieldTargets.AimTarget.HUB;
    // }

    // if (yBlueFrame <= Constants.FieldTargets.NEUTRAL_ZONE_Y_SPLIT_METERS) {
    //   return Constants.FieldTargets.AimTarget.NEUTRAL_LOW;
    // }

    // return Constants.FieldTargets.AimTarget.NEUTRAL_HIGH;
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
