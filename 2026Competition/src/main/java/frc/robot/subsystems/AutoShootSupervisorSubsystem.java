package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldTargets.AimTarget;
import frc.robot.Constants.OperatorConstants.AutoShoot;
import frc.robot.Constants.OperatorConstants.Hood;
import frc.robot.Constants.OperatorConstants.Turret;
import frc.robot.RobotContainer;
import frc.robot.lib.AimGeometry;
import frc.robot.lib.FieldTargeting;
import frc.robot.lib.FieldRules;
import frc.robot.lib.ShotFlightTimeTable;
import frc.robot.lib.ShotIntent;
import frc.robot.lib.ShotPlanner;
import frc.robot.lib.ShotPlanner.Mode;
import frc.robot.lib.ShotPlanner.Solution;
import frc.robot.lib.ShotReadiness;
import frc.robot.lib.ShotTable;
import org.littletonrobotics.junction.Logger;

/** One shot plan and one feed decision per loop. Diagnostics never change control. */
public class AutoShootSupervisorSubsystem extends SubsystemBase {
  public enum VolleyState { IDLE, ARMING, FIRING, RECOVERING, NO_SOLUTION, EXTERNAL_CONTROL }
  public enum SolutionValidity { VALID, TURRET_ONLY_INVALID, GLOBAL_INVALID }
  private final ShotIntent intent = new ShotIntent();
  private final ShotPlanner planner;
  private final SlewRateLimiter turretLimiter = new SlewRateLimiter(Turret.TRACKING_SETPOINT_RATE_LIMIT_DEG_PER_SEC);
  private boolean filterInitialized, lastBallAtThroat;
  private double cooldownUntil;
  private Mode shotMode = Mode.MOVING_AUTO;
  private VolleyState state = VolleyState.IDLE;
  private SolutionValidity validity = SolutionValidity.GLOBAL_INVALID;
  private AimTarget targetKind = AimTarget.HUB;
  private Solution solution = Solution.invalid("IDLE");
  private double rawTurret = Double.NaN, commandedTurret = Double.NaN;
  private String feedReason = "IDLE";

  public AutoShootSupervisorSubsystem() {
    ShotFlightTimeTable flightTimes = new ShotFlightTimeTable();
    try {
      flightTimes = ShotFlightTimeTable.load(Filesystem.getDeployDirectory().toPath().resolve("artillery/flight_times.csv"));
    } catch (java.io.IOException ex) {
      DriverStation.reportError("Flight-time table rejected: " + ex.getMessage(), false);
    }
    ShotTable hub = loadShotTable("Hub", Constants.OperatorConstants.MovingAutoShotTable.DEPLOY_CSV_PATH);
    ShotTable pass = loadShotTable("Pass", "artillery/pass_shots.csv");
    planner = new ShotPlanner(hub, pass, flightTimes);
  }

  private static ShotTable loadShotTable(String name, String path) {
    ShotTable table = ShotTable.loadFromDeployCsv(path);
    Logger.recordOutput("AutoShoot/Tables/" + name + "/Status", table.loadStatus());
    Logger.recordOutput("AutoShoot/Tables/" + name + "/SHA256", table.sourceSha256());
    Logger.recordOutput("AutoShoot/Tables/" + name + "/Rows", table.sampleCount());
    if (table.loadStatus().startsWith("REJECTED") || table.loadStatus().equals("MISSING"))
      DriverStation.reportError("Shot table " + path + ": " + table.loadStatus(), false);
    return table;
  }

  public void setShootRequested(boolean requested) {
    intent.request(requested, inKnownTrench());
    if (!requested) { stopFeed(); cooldownUntil = 0; state = VolleyState.IDLE; }
  }
  public void setExternalControl(boolean active) {
    intent.externalControl(active); stopFeed(); filterInitialized = false;
  }
  public void setCalibrationActive(boolean active) { setExternalControl(active); }
  public boolean isShootRequested() { return intent.requested(); }
  public VolleyState getVolleyState() { return state; }
  public SolutionValidity getSolutionValidity() { return validity; }
  public AimTarget getCurrentAimTarget() { return targetKind; }
  public Mode getShotMode() { return shotMode; }
  public void setShotMode(Mode mode) { shotMode = java.util.Objects.requireNonNull(mode); }

  public static Translation2d getAllianceAwareAimTarget(AimTarget target) {
    return FieldTargeting.target(target, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red);
  }
  public double getHubTargetRelativeAngleDeg() {
    return AimGeometry.turretDegrees(RobotContainer.driveSubsystem.getPose(), getAllianceAwareAimTarget(AimTarget.HUB));
  }
  public double getHubCommandRelativeAngleDeg() {
    return AimGeometry.safeCommand(getHubTargetRelativeAngleDeg() + Turret.AUTO_AIM_TRIM_DEG);
  }
  /** Read-only calculation shares the control planner and target policy. */
  public Solution calculateDiagnosticSolution() {
    var drive = RobotContainer.driveSubsystem.getState();
    boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    AimTarget target = FieldTargeting.select(drive.Pose, red, shotMode == Mode.MOVING_AUTO, targetKind);
    return planner.solve(shotMode, drive.Pose, motionSpeeds(drive.Speeds), FieldTargeting.target(target, red), target,
        RobotContainer.shooterSubsystem.getTargetRpm(), manualThrottle(), AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT);
  }
  private double manualThrottle() {
    return shotMode == Mode.MANUAL_FIXED ? RobotContainer.getTurretStick().getThrottle() : 0;
  }
  private ChassisSpeeds motionSpeeds(ChassisSpeeds wheelSpeeds) {
    return new ChassisSpeeds(wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond,
        RobotContainer.driveSubsystem.getGyroYawRateRadiansPerSecond());
  }
  private boolean inKnownTrench() {
    var drive = RobotContainer.driveSubsystem.getState();
    return RobotContainer.vision.hasCompetitionAimFrame() && RobotContainer.driveSubsystem.hasFieldReference()
        && FieldTargeting.trenchInhibit(drive.Pose,
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.Speeds, drive.Pose.getRotation()));
  }
  private void stopFeed() {
    RobotContainer.transferSubsystem.stop(); RobotContainer.spindexerSubsystem.stop();
  }
  private boolean anotherCommandOwnsMechanisms() {
    // Command groups have one scheduler owner on every requirement. Direct calibration and SysId
    // commands own their hardware without receiving competing supervisor requests.
    for (Subsystem mechanism : new Subsystem[] {RobotContainer.shooterSubsystem, RobotContainer.hoodSubsystem,
        RobotContainer.turretSubsystem, RobotContainer.transferSubsystem, RobotContainer.spindexerSubsystem}) {
      var owner = mechanism.getCurrentCommand();
      if (owner != null && owner != getCurrentCommand()) return true;
    }
    return false;
  }

  @Override public void periodic() {
    Logger.recordOutput("AutoShoot/FeedAllowed", false);
    boolean enabled = DriverStation.isEnabled();
    intent.observe(enabled, inKnownTrench());
    if (!Constants.EnabledSubsystems.supervisor) return;
    if (!enabled || RobotContainer.isPanicStopActive()) {
      intent.request(false, false); stopFeed(); RobotContainer.shooterSubsystem.stop();
      resetOutputsForLog("DISABLED_OR_PANIC", VolleyState.IDLE); return;
    }
    if (intent.externalControl() || anotherCommandOwnsMechanisms()) {
      // The owning command stops its outputs in end(). Do not write competing outputs here.
      intent.request(false, false);
      // A shooter/turret-only owner must not leave an unowned hood raised in a trench.
      var hoodOwner = RobotContainer.hoodSubsystem.getCurrentCommand();
      if (intent.trenchLocked() && (hoodOwner == null || hoodOwner == getCurrentCommand()))
        RobotContainer.hoodSubsystem.setTargetAngleRad(Hood.NEUTRAL_ANGLE_RAD);
      resetOutputsForLog("EXTERNAL_CONTROL", VolleyState.EXTERNAL_CONTROL); return;
    }

    double now = Timer.getFPGATimestamp();
    var drive = RobotContainer.driveSubsystem.getState();
    var speeds = motionSpeeds(drive.Speeds);
    boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    boolean poseReady = DriverStation.getAlliance().isPresent()
        && RobotContainer.vision.hasCompetitionAimFrame() && RobotContainer.vision.isLocalizationReady();
    boolean manualAim = shotMode != Mode.MOVING_AUTO && RobotContainer.isHubTrackingDisabledByButtonBox();
    targetKind = FieldTargeting.select(drive.Pose, red, shotMode == Mode.MOVING_AUTO, targetKind);
    var target = FieldTargeting.target(targetKind, red);
    // Mentor-authorized manual fallback: driver confirms G407 position when localization is lost.
    boolean manualZoneConfirmation = manualAim && !poseReady;
    boolean fieldZoneAllowed = targetKind != AimTarget.HUB || manualZoneConfirmation
        || (poseReady && FieldRules.hubZoneConfirmed(drive.Pose,
            ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.Pose.getRotation()), red,
            AutoShoot.DT_RELEASE_SEC));
    SmartDashboard.putBoolean("AutoShoot/ManualZoneConfirmationRequired", manualZoneConfirmation);
    Logger.recordOutput("AutoShoot/ManualZoneConfirmationRequired", manualZoneConfirmation);
    Logger.recordOutput("AutoShoot/FieldZoneAllowed", fieldZoneAllowed);
    boolean active = intent.requested() && !intent.trenchLocked();
    solution = active ? planner.solve(shotMode, drive.Pose, speeds, target, targetKind,
        RobotContainer.shooterSubsystem.getTargetRpm(), manualThrottle(), AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT)
        : Solution.invalid("IDLE");
    rawTurret = active ? solution.turretDegrees()
        : AimGeometry.turretDegrees(drive.Pose, target) + Turret.AUTO_AIM_TRIM_DEG;
    double comfort = AutoShoot.STATIONARY_ILLEGAL_SHOT_COMFORT_MARGIN_DEG;
    boolean legalAngle = AimGeometry.inWindow(manualAim ? RobotContainer.turretSubsystem.getAngleDeg() : rawTurret,
        Turret.MIN_ANGLE_DEG + comfort, Turret.MAX_ANGLE_DEG - comfort);
    validity = !active || !solution.valid() || (!poseReady && !manualAim) ? SolutionValidity.GLOBAL_INVALID
        : legalAngle ? SolutionValidity.VALID : SolutionValidity.TURRET_ONLY_INVALID;
    boolean autoAim = (AutoShoot.ALWAYS_AIM || active) && poseReady
        && !RobotContainer.isHubTrackingDisabledByButtonBox();
    if (autoAim && Double.isFinite(rawTurret)) {
      if (!filterInitialized) {
        turretLimiter.reset(RobotContainer.turretSubsystem.getAngleDeg()); filterInitialized = true;
      }
      commandedTurret = turretLimiter.calculate(AimGeometry.safeCommand(rawTurret));
      RobotContainer.turretSubsystem.goToAngleDeg(commandedTurret);
    } else { filterInitialized = false; commandedTurret = Double.NaN; }

    boolean throat = RobotContainer.transferSubsystem.hasBallAtThroat();
    if (lastBallAtThroat && !throat && state == VolleyState.FIRING) cooldownUntil = now + .015;
    lastBallAtThroat = throat;
    boolean solutionUsable = active && solution.valid() && (poseReady || manualAim);
    if (solutionUsable) {
      RobotContainer.shooterSubsystem.setTargetRpm(solution.shooterRpmCommand());
      // An RPM dip inhibits feed; it must not invent an unmeasured hood correction.
      RobotContainer.hoodSubsystem.setTargetAngleRad(solution.hoodCommandAngleRad());
    } else {
      if (!intent.requested() && !intent.trenchLocked()) RobotContainer.shooterSubsystem.setTargetRpm(AutoShoot.IDLE_SHOOTER_RPM);
      else RobotContainer.shooterSubsystem.stop();
      RobotContainer.hoodSubsystem.setTargetAngleRad(Hood.NEUTRAL_ANGLE_RAD);
    }
    boolean rpmReady = RobotContainer.shooterSubsystem.isReadyToShoot();
    boolean hoodReady = solution.valid() && AimGeometry.inWindow(solution.hoodCommandAngleRad(), Hood.MIN_ANGLE_RAD, Hood.MAX_ANGLE_RAD)
        && RobotContainer.hoodSubsystem.isAtTarget();
    boolean turretReady = legalAngle && (manualAim
        || RobotContainer.turretSubsystem.atAngleDeg(rawTurret, Turret.AIM_TOLERANCE_DEG));
    boolean motionAllowed = shotMode == Mode.MOVING_AUTO
        || (Math.abs(speeds.vxMetersPerSecond) < AutoShoot.STATIC_MAX_VX_MPS
            && Math.abs(speeds.vyMetersPerSecond) < AutoShoot.STATIC_MAX_VY_MPS
            && Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) < AutoShoot.STATIC_MAX_OMEGA_DEG_PER_S);
    boolean pathReady = !DriverStation.isAutonomousEnabled() || !RobotContainer.driveSubsystem.hasAutonomousPrecisionFailure();
    var reason = ShotReadiness.evaluate(new ShotReadiness.Inputs(intent.requested(), solution.valid(),
        poseReady || manualAim, pathReady, intent.trenchLocked(), RobotContainer.turretSubsystem.isPositionTrusted(),
        turretReady, rpmReady, hoodReady, motionAllowed, now < cooldownUntil, fieldZoneAllowed));
    feedReason = reason.toString();
    state = switch (reason) {
      case READY -> VolleyState.FIRING;
      case IDLE -> VolleyState.IDLE;
      case NO_SOLUTION, POSE_UNREADY, PATH_FAILED, HUB_ZONE_UNCONFIRMED -> VolleyState.NO_SOLUTION;
      case RPM_UNREADY, HOOD_UNREADY, COOLDOWN -> VolleyState.RECOVERING;
      default -> VolleyState.ARMING;
    };
    if (reason == ShotReadiness.Reason.READY) {
      RobotContainer.transferSubsystem.runFeed(); RobotContainer.spindexerSubsystem.runSupply();
    } else stopFeed();
    Logger.recordOutput("AutoShoot/FeedAllowed", reason == ShotReadiness.Reason.READY);
    Logger.recordOutput("AutoShoot/PoseReady", poseReady);
    Logger.recordOutput("AutoShoot/AutonomousPathReady", pathReady);
    Logger.recordOutput("AutoShoot/TurretAimed", turretReady);
    Logger.recordOutput("AutoShoot/ShooterReady", rpmReady);
    Logger.recordOutput("AutoShoot/HoodReady", hoodReady);
    Logger.recordOutput("AutoShoot/MotionAllowed", motionAllowed);
    Logger.recordOutput("AutoShoot/TargetPosition", target);
    publish();
  }

  private void resetOutputsForLog(String reason, VolleyState nextState) {
    SmartDashboard.putBoolean("AutoShoot/ManualZoneConfirmationRequired", false);
    Logger.recordOutput("AutoShoot/ManualZoneConfirmationRequired", false);
    Logger.recordOutput("AutoShoot/FieldZoneAllowed", false);
    state = nextState; validity = SolutionValidity.GLOBAL_INVALID;
    solution = Solution.invalid(reason); rawTurret = commandedTurret = Double.NaN;
    filterInitialized = false; feedReason = reason; publish();
  }
  private void publish() {
    Logger.recordOutput("AutoShoot/State", state.toString());
    Logger.recordOutput("AutoShoot/ShotMode", shotMode.toString());
    Logger.recordOutput("AutoShoot/FeedReason", feedReason);
    Logger.recordOutput("AutoShoot/SolutionValidity", validity.toString());
    Logger.recordOutput("AutoShoot/AimTarget", targetKind.toString());
    Logger.recordOutput("AutoShoot/TrenchLocked", intent.trenchLocked());
    Logger.recordOutput("AutoShoot/ShootRequested", intent.requested());
    Logger.recordOutput("AutoShoot/RawDesiredTurretDeg", rawTurret);
    Logger.recordOutput("AutoShoot/FilteredDesiredTurretDeg", commandedTurret);
    Logger.recordOutput("AutoShoot/TurretTrackingErrorDeg", rawTurret - RobotContainer.turretSubsystem.getContinuousAngleDeg());
    Logger.recordOutput("AutoShoot/TargetRPM", solution.shooterRpmCommand());
    Logger.recordOutput("AutoShoot/TargetHoodRadians", solution.hoodCommandAngleRad());
    Logger.recordOutput("AutoShoot/MovingAim/LeadSource", solution.leadSource());
    Logger.recordOutput("AutoShoot/MovingAim/DistanceLookupMeters", solution.distanceMeters());
    Logger.recordOutput("AutoShoot/MovingAim/RadialLeadSeconds", solution.radialLeadSeconds());
    Logger.recordOutput("AutoShoot/MovingAim/LateralLeadSeconds", solution.lateralLeadSeconds());
    if (solution.movingAim() != null) {
      Logger.recordOutput("AutoShoot/MovingAim/ReleasePosition", solution.movingAim().releasePosition());
      Logger.recordOutput("AutoShoot/MovingAim/ReleaseVelocity", solution.movingAim().releaseVelocity());
    }
    SmartDashboard.putString("AutoShoot/FeedReason", feedReason);
    if (Constants.DebugTelemetrySubsystems.supervisor || Constants.DebugTelemetrySubsystems.turret) {
      SmartDashboard.putString("AutoShoot/State", state.toString());
      SmartDashboard.putNumber("Turret/HubTargetRelativeAngleDeg", getHubTargetRelativeAngleDeg());
      SmartDashboard.putNumber("Turret/HubCommandRelativeAngleDeg", getHubCommandRelativeAngleDeg());
    }
  }
}
