package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.PrecisionConstants;
import frc.robot.subsystems.PrecisionDrive;

/**
 * Final-pose controller used after an explicitly known pose reset, a coarse trajectory, or directly as a
 * positioning test. Drives in field coordinates until the robot holds a translation/rotation tolerance
 * for a continuous settle time, with a hard safety timeout.
 *
 * <p>Competition routes select brake-only {@link StopAtRouteEnd} instead. This controller is retained
 * for explicit precision alignment and may correct translation/rotation during settling. End always
 * holds module angles; timeout/interruption is not success. Targets retain the blue field origin.
 *
 * <p>Ported from the pinned FRC999 prototype; field gains still require 2026 robot validation.
 * Historical idea traceability:
 *
 * <ul>
 *   <li><b>Profiled control on x/y/theta with velocity feedforward</b> -- the trapezoid profile
 *       decelerates to zero velocity exactly at the goal, reducing an end-of-move overshoot risk. Idea: 1768 Nashoba {@code driveToPose} (profiled PID x/y/theta); the same
 *       deceleration behavior 6328 {@code DriveToPose} gets from an explicit profile + FF fade.
 *   <li><b>Settle gate</b> -- success requires staying inside tolerance for a continuous settle time,
 *       not just one instantaneous touch. Idea: 1768 {@code cmdWithAccuracy} settle stopwatch.
 *   <li><b>Safety timeout</b> -- ends (unsuccessfully) after a hard cap so a bad target can never hang
 *       the command. Idea: 1768 wraps accuracy commands in {@code .withTimeout(totalTime + slack)}.
 *   <li><b>Full logging</b> of target / measured / errors / settle, satisfying the AGENTS.md rule that
 *       "every precision test must log target pose, measured final pose, translation error, rotation
 *       error, and settle time." Idea: 6328 {@code DriveToPose} logs measured/setpoint/goal every loop.
 * </ul>
 *
 * <p>The coarse-trajectory -> precision handoff (6328 {@code DriveTrajectory.until(spatial).andThen(
 * DriveToPose)}) is provided by {@link #handoffFrom(edu.wpi.first.wpilibj2.command.Command,
 * java.util.function.BooleanSupplier)}.
 */
public class DriveToPosePrecisionCommand extends Command {
  public enum CompletionReason { RUNNING, SUCCEEDED, TIMED_OUT, INTERRUPTED, INVALID_STATE }
  private CompletionReason completionReason = CompletionReason.RUNNING;
  private java.util.function.BooleanSupplier finishPermission = () -> true;

  /** Additional continuously checked finish gate (normally fresh referenced vision); set before scheduling. */
  public DriveToPosePrecisionCommand withFinishPermission(java.util.function.BooleanSupplier permission) {
    finishPermission = java.util.Objects.requireNonNull(permission);
    return this;
  }

  public boolean succeeded() { return completionReason == CompletionReason.SUCCEEDED; }
  public CompletionReason getCompletionReason() { return completionReason; }
  /** Selects how tightly this command must finish its terminal heading. */
  public enum YawPrecision {
    PRECISE(PrecisionConstants.PRECISION_ROTATION_TOLERANCE_DEGREES),
    RELAXED(PrecisionConstants.RELAXED_ROTATION_TOLERANCE_DEGREES);

    private final double toleranceDegrees;

    YawPrecision(double toleranceDegrees) {
      this.toleranceDegrees = toleranceDegrees;
    }

    public double toleranceDegrees() {
      return toleranceDegrees;
    }
  }

  private final PrecisionDrive drive;
  private final Pose2d targetPose;
  private final YawPrecision yawPrecision;
  private final double rotationToleranceDegrees;
  private final double translationToleranceMeters;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          PrecisionConstants.PRECISION_DRIVE_KP,
          0.0,
          PrecisionConstants.PRECISION_DRIVE_KD,
          new TrapezoidProfile.Constraints(
              PrecisionConstants.PRECISION_MAX_SPEED_METERS_PER_SECOND,
              PrecisionConstants.PRECISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED),
          PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          PrecisionConstants.PRECISION_DRIVE_KP,
          0.0,
          PrecisionConstants.PRECISION_DRIVE_KD,
          new TrapezoidProfile.Constraints(
              PrecisionConstants.PRECISION_MAX_SPEED_METERS_PER_SECOND,
              PrecisionConstants.PRECISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED),
          PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          PrecisionConstants.PRECISION_THETA_KP,
          0.0,
          PrecisionConstants.PRECISION_THETA_KD,
          new TrapezoidProfile.Constraints(
              PrecisionConstants.PRECISION_MAX_OMEGA_RADIANS_PER_SECOND,
              PrecisionConstants.PRECISION_MAX_ANGULAR_ACCEL_RAD_PER_SECOND_SQUARED),
          PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS);

  private final Timer settleTimer = new Timer();
  private final Timer safetyTimer = new Timer();
  private boolean wasWithinPoseTolerance;
  private boolean settlingHoldLatched;
  private boolean strictFinishMotionRequired;
  private boolean finishQualified;
  private final SettleVelocityEscape velocityEscape =
      new SettleVelocityEscape(PrecisionConstants.PRECISION_SETTLE_VELOCITY_ESCAPE_CONFIRM_SECONDS);
  private final SettleVelocityEscape poseRequalification =
      new SettleVelocityEscape(PrecisionConstants.PRECISION_SETTLE_POSE_REQUALIFICATION_SECONDS);
  private final SettleVelocityEscape finishConfirmation =
      new SettleVelocityEscape(PrecisionConstants.PRECISION_SETTLE_SECONDS);
  private int poseToleranceEntryCount;
  private int atGoalEntryCount;
  private int settlingHoldExitCount;
  private double lastExecuteTimestampSeconds;
  private double profileTimeAccumulatorSeconds;
  private double profileElapsedSeconds;

  public DriveToPosePrecisionCommand(PrecisionDrive drive, Pose2d targetPose) {
    this(drive, targetPose, YawPrecision.PRECISE);
  }

  public DriveToPosePrecisionCommand(
      PrecisionDrive drive, Pose2d targetPose, YawPrecision yawPrecision) {
    this(drive, targetPose, yawPrecision, PrecisionConstants.PRECISION_TRANSLATION_TOLERANCE_METERS);
  }

  /** Selects endpoint accuracy for a route without changing its gains or motion constraints. */
  public DriveToPosePrecisionCommand(
      PrecisionDrive drive, Pose2d targetPose, YawPrecision yawPrecision,
      double translationToleranceMeters) {
    this.drive = drive;
    if (targetPose == null || !Double.isFinite(targetPose.getX()) || !Double.isFinite(targetPose.getY())
        || !Double.isFinite(targetPose.getRotation().getRadians())) throw new IllegalArgumentException("Target pose must be finite");
    this.targetPose = targetPose;
    this.yawPrecision = java.util.Objects.requireNonNull(yawPrecision);
    this.rotationToleranceDegrees = yawPrecision.toleranceDegrees();
    this.translationToleranceMeters = validateTranslationTolerance(translationToleranceMeters);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(this.translationToleranceMeters);
    yController.setTolerance(this.translationToleranceMeters);
    thetaController.setTolerance(Math.toRadians(rotationToleranceDegrees));
    addRequirements(drive);
  }

  /**
   * Registers every output type used by this command before the robot can move.
   *
   * <p>AdvantageKit creates a publisher/schema the first time an output key is seen. The 0586 robot
   * log registered 22 keys during the high-speed spatial handoff and blocked the scheduler for 468
   * ms, then registered another 69 keys on the first execute. Publishing safe defaults once during
   * robot construction moves that one-time cost to startup instead of paying it while coasting.
   */
  public static void primeTelemetrySchema() {
    double startSeconds = Timer.getFPGATimestamp();

    for (String key :
        new String[] {
          "DriveToPose/AtGoal",
          "DriveToPose/Controller/Active",
          "DriveToPose/Controller/RotationCommandClamped",
          "DriveToPose/Controller/TranslationCommandClamped",
          "DriveToPose/Finished",
          "DriveToPose/GoalQualifiedThisLoop",
          "DriveToPose/FinishMotionQualifiedThisLoop",
          "DriveToPose/Controller/StrictFinishMotionRequired",
          "DriveToPose/FinishQualified",
          "DriveToPose/VelocityEscapePending",
          "DriveToPose/VelocityEscapeConfirmed",
          "DriveToPose/PoseRequalificationPending",
          "DriveToPose/PoseRequalificationConfirmed",
          "DriveToPose/OutsideSettlingEscapePoseTolerance",
          "DriveToPose/OutsideSettlingEscapeTolerance",
          "DriveToPose/OutsideSettlingEscapeVelocityTolerance",
          "DriveToPose/SettlingHoldActive",
          "DriveToPose/TimedOut",
          "DriveToPose/WithinPoseTolerance",
          "DriveToPose/WithinVelocityTolerance"
        }) {
      Logger.recordOutput(key, false);
    }

    for (String key :
        new String[] {
          "DriveToPose/Controller/ConfiguredMaxAccelerationMetersPerSecondSquared",
          "DriveToPose/Controller/ConfiguredMaxSpeedMetersPerSecond",
          "DriveToPose/Controller/ConfiguredProfilePeriodSeconds",
          "DriveToPose/Controller/ConfiguredRotationToleranceDegrees",
          "DriveToPose/Controller/ConfiguredTranslationToleranceMeters",
          "DriveToPose/Controller/ConfiguredRotationVelocityDamping",
          "DriveToPose/Controller/ConfiguredSettleEscapeMaxRotationSpeedDegreesPerSecond",
          "DriveToPose/Controller/ConfiguredSettleEscapeMaxTranslationSpeedMetersPerSecond",
          "DriveToPose/Controller/ConfiguredSettleEscapeRotationDegrees",
          "DriveToPose/Controller/ConfiguredSettleEscapeTranslationMeters",
          "DriveToPose/Controller/ConfiguredSettleMaxRotationSpeedDegreesPerSecond",
          "DriveToPose/Controller/ConfiguredSettleMaxTranslationSpeedMetersPerSecond",
          "DriveToPose/Controller/ConfiguredSettleSeconds",
          "DriveToPose/Controller/ConfiguredFinishMaxRotationSpeedDegreesPerSecond",
          "DriveToPose/Controller/ConfiguredFinishMaxModuleSpeedMetersPerSecond",
          "DriveToPose/MaxAbsModuleSpeedMetersPerSecond",
          "DriveToPose/Controller/ConfiguredVelocityEscapeConfirmSeconds",
          "DriveToPose/VelocityEscapeSeconds",
          "DriveToPose/PoseRequalificationSeconds",
          "DriveToPose/FinishQualificationSeconds",
          "DriveToPose/Controller/ConfiguredPoseRequalificationSeconds",
          "DriveToPose/Controller/ConfiguredTranslationFfMaxRadiusMeters",
          "DriveToPose/Controller/ConfiguredTranslationFfMinRadiusMeters",
          "DriveToPose/Controller/ConfiguredTranslationKd",
          "DriveToPose/Controller/ConfiguredTranslationKp",
          "DriveToPose/Controller/ConfiguredTranslationVelocityDamping",
          "DriveToPose/Controller/ControllerRequestedOmegaDegreesPerSecond",
          "DriveToPose/Controller/ControllerRequestedVxRobotMetersPerSecond",
          "DriveToPose/Controller/DampingOmegaDegreesPerSecond",
          "DriveToPose/Controller/DampingVxFieldMetersPerSecond",
          "DriveToPose/Controller/DampingVyFieldMetersPerSecond",
          "DriveToPose/Controller/DistanceToTargetMeters",
          "DriveToPose/Controller/FeedbackOmegaDegreesPerSecond",
          "DriveToPose/Controller/FeedbackVxFieldMetersPerSecond",
          "DriveToPose/Controller/FeedbackVyFieldMetersPerSecond",
          "DriveToPose/Controller/KinematicMeasuredOmegaDegreesPerSecond",
          "DriveToPose/Controller/LoopDtMilliseconds",
          "DriveToPose/Controller/MeasuredOmegaDegreesPerSecond",
          "DriveToPose/Controller/MeasuredVxRobotMetersPerSecond",
          "DriveToPose/Controller/MeasuredVyRobotMetersPerSecond",
          "DriveToPose/Controller/ProfileClockLagMilliseconds",
          "DriveToPose/Controller/ProfileElapsedSeconds",
          "DriveToPose/Controller/ProfileOmegaDegreesPerSecond",
          "DriveToPose/Controller/ProfileTimeAccumulatorMilliseconds",
          "DriveToPose/Controller/ProfileVxFieldMetersPerSecond",
          "DriveToPose/Controller/ProfileVyFieldMetersPerSecond",
          "DriveToPose/Controller/RawProfileVxFieldMetersPerSecond",
          "DriveToPose/Controller/RawProfileVyFieldMetersPerSecond",
          "DriveToPose/Controller/RequestedOmegaDegreesPerSecond",
          "DriveToPose/Controller/RequestedVxRobotMetersPerSecond",
          "DriveToPose/Controller/RequestedVyRobotMetersPerSecond",
          "DriveToPose/Controller/TrackingErrorOmegaDegreesPerSecond",
          "DriveToPose/Controller/TrackingErrorVxRobotMetersPerSecond",
          "DriveToPose/Controller/TrackingErrorVyRobotMetersPerSecond",
          "DriveToPose/Controller/TranslationDampingScale",
          "DriveToPose/Controller/TranslationFeedforwardScale",
          "DriveToPose/Controller/TranslationVelocityTrackingErrorMetersPerSecond",
          "DriveToPose/Controller/WallElapsedSeconds",
          "DriveToPose/ErrorThetaSignedDegrees",
          "DriveToPose/ErrorXFieldMeters",
          "DriveToPose/ErrorYFieldMeters",
          "DriveToPose/MeasuredRotationSpeedDegreesPerSecond",
          "DriveToPose/MeasuredTranslationSpeedMetersPerSecond",
          "DriveToPose/RotationErrorDegrees",
          "DriveToPose/SettleSeconds",
          "DriveToPose/TranslationErrorMeters"
        }) {
      Logger.recordOutput(key, 0.0);
    }

    for (String key :
        new String[] {
          "DriveToPose/AtGoalEntryCount",
          "DriveToPose/Controller/ConfiguredMaxProfileStepsPerExecute",
          "DriveToPose/Controller/ProfileStepsThisExecute",
          "DriveToPose/PoseToleranceEntryCount",
          "DriveToPose/SettlingHoldExitCount"
        }) {
      Logger.recordOutput(key, 0L);
    }

    Logger.recordOutput("DriveToPose/Controller/DriveRequestType", "NOT_RUNNING");
    Logger.recordOutput("DriveToPose/Controller/YawPrecisionMode", "NOT_RUNNING");

    Logger.recordOutput("DriveToPose/TargetPose", Pose2d.kZero);
    Logger.recordOutput("DriveToPose/MeasuredPose", Pose2d.kZero);
    Logger.recordOutput("DriveToPose/Controller/ProfileSetpointPose", Pose2d.kZero);

    ChassisSpeeds zeroSpeeds = new ChassisSpeeds();
    for (String key :
        new String[] {
          "DriveToPose/Controller/ControllerRequestedVelocityField",
          "DriveToPose/Controller/ControllerRequestedVelocityRobot",
          "DriveToPose/Controller/DampingVelocityField",
          "DriveToPose/Controller/FeedbackVelocityField",
          "DriveToPose/Controller/MeasuredVelocityField",
          "DriveToPose/Controller/MeasuredVelocityRobot",
          "DriveToPose/Controller/ProfileVelocityField",
          "DriveToPose/Controller/RawProfileVelocityField",
          "DriveToPose/Controller/RequestedVelocityField",
          "DriveToPose/Controller/RequestedVelocityFieldUnclamped",
          "DriveToPose/Controller/RequestedVelocityRobot",
          "DriveToPose/Controller/VelocityTrackingErrorRobot"
        }) {
      Logger.recordOutput(key, zeroSpeeds);
    }

    Logger.recordOutput(
        "DriveToPose/Controller/TelemetryPrimeDurationMilliseconds",
        (Timer.getFPGATimestamp() - startSeconds) * 1000.0);
    Logger.recordOutput("DriveToPose/Controller/TelemetrySchemaPrimed", true);
  }

  @Override
  public void initialize() {
    completionReason = CompletionReason.RUNNING;
    Logger.recordOutput("DriveToPose/CompletionReason", completionReason.toString());
    Pose2d pose = drive.getPose();
    ChassisSpeeds speeds = drive.getFieldRelativeSpeeds();
    if (!Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())
        || !Double.isFinite(speeds.vxMetersPerSecond) || !Double.isFinite(speeds.vyMetersPerSecond)
        || !Double.isFinite(drive.getGyroYawRateRadiansPerSecond())) {
      completionReason = CompletionReason.INVALID_STATE;
      drive.stop();
      return;
    }
    // Seed each profile with the current state so the first command is continuous (no jump). Idea:
    // 6328 DriveToPose resets controllers with current pose + current field velocity in initialize().
    xController.reset(pose.getX(), speeds.vxMetersPerSecond);
    yController.reset(pose.getY(), speeds.vyMetersPerSecond);
    thetaController.reset(pose.getRotation().getRadians(), drive.getGyroYawRateRadiansPerSecond());
    settleTimer.stop();
    settleTimer.reset();
    safetyTimer.restart();
    wasWithinPoseTolerance = false;
    settlingHoldLatched = false;
    strictFinishMotionRequired = requiresRotatingFinish(
        Math.abs(targetPose.getRotation().minus(pose.getRotation()).getDegrees()),
        Math.abs(Math.toDegrees(drive.getGyroYawRateRadiansPerSecond())));
    finishQualified = false;
    velocityEscape.reset();
    poseRequalification.reset();
    finishConfirmation.reset();
    poseToleranceEntryCount = 0;
    atGoalEntryCount = 0;
    settlingHoldExitCount = 0;
    lastExecuteTimestampSeconds = Timer.getFPGATimestamp();
    // Preserve the original one nominal profile step on the first execute, then use elapsed wall time.
    profileTimeAccumulatorSeconds = PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS;
    profileElapsedSeconds = 0.0;
    // Clear stale end-of-run flags so the log reflects THIS run while it is in progress.
    Logger.recordOutput("DriveToPose/Finished", false);
    Logger.recordOutput("DriveToPose/TimedOut", false);
    Logger.recordOutput("DriveToPose/Controller/Active", true);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationToleranceMeters", translationToleranceMeters);
    Logger.recordOutput("DriveToPose/FinishQualified", false);
    Logger.recordOutput("DriveToPose/FinishMotionQualifiedThisLoop", false);
    Logger.recordOutput("DriveToPose/Controller/StrictFinishMotionRequired", strictFinishMotionRequired);
    Logger.recordOutput("DriveToPose/VelocityEscapePending", false);
    Logger.recordOutput("DriveToPose/VelocityEscapeConfirmed", false);
    Logger.recordOutput("DriveToPose/VelocityEscapeSeconds", 0.0);
    Logger.recordOutput("DriveToPose/PoseRequalificationPending", false);
    Logger.recordOutput("DriveToPose/PoseRequalificationConfirmed", false);
    Logger.recordOutput("DriveToPose/PoseRequalificationSeconds", 0.0);
    Logger.recordOutput("DriveToPose/FinishQualificationSeconds", 0.0);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredFinishMaxRotationSpeedDegreesPerSecond",
        PrecisionConstants.PRECISION_FINISH_MAX_ROTATION_SPEED_DEGREES_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredFinishMaxModuleSpeedMetersPerSecond",
        PrecisionConstants.PRECISION_FINISH_MAX_MODULE_SPEED_METERS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredPoseRequalificationSeconds",
        PrecisionConstants.PRECISION_SETTLE_POSE_REQUALIFICATION_SECONDS);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredVelocityEscapeConfirmSeconds",
        PrecisionConstants.PRECISION_SETTLE_VELOCITY_ESCAPE_CONFIRM_SECONDS);
    Logger.recordOutput("DriveToPose/Controller/DriveRequestType", "Velocity");
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredMaxSpeedMetersPerSecond",
        PrecisionConstants.PRECISION_MAX_SPEED_METERS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredMaxAccelerationMetersPerSecondSquared",
        PrecisionConstants.PRECISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationKp",
        PrecisionConstants.PRECISION_DRIVE_KP);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationKd",
        PrecisionConstants.PRECISION_DRIVE_KD);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationFfMinRadiusMeters",
        PrecisionConstants.PRECISION_TRANSLATION_FF_MIN_RADIUS_METERS);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationFfMaxRadiusMeters",
        PrecisionConstants.PRECISION_TRANSLATION_FF_MAX_RADIUS_METERS);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredTranslationVelocityDamping",
        PrecisionConstants.PRECISION_TRANSLATION_VELOCITY_DAMPING);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredRotationVelocityDamping",
        PrecisionConstants.PRECISION_ROTATION_VELOCITY_DAMPING);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleMaxTranslationSpeedMetersPerSecond",
        PrecisionConstants.PRECISION_SETTLE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleMaxRotationSpeedDegreesPerSecond",
        PrecisionConstants.PRECISION_SETTLE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleEscapeTranslationMeters",
        PrecisionConstants.PRECISION_SETTLE_ESCAPE_TRANSLATION_METERS);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleEscapeRotationDegrees",
        PrecisionConstants.PRECISION_SETTLE_ESCAPE_ROTATION_DEGREES);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleEscapeMaxTranslationSpeedMetersPerSecond",
        PrecisionConstants.PRECISION_SETTLE_ESCAPE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleEscapeMaxRotationSpeedDegreesPerSecond",
        PrecisionConstants.PRECISION_SETTLE_ESCAPE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredSettleSeconds",
        PrecisionConstants.PRECISION_SETTLE_SECONDS);
    Logger.recordOutput("DriveToPose/Controller/YawPrecisionMode", yawPrecision.name());
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredRotationToleranceDegrees",
        rotationToleranceDegrees);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredProfilePeriodSeconds",
        PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS);
    Logger.recordOutput(
        "DriveToPose/Controller/ConfiguredMaxProfileStepsPerExecute",
        PrecisionConstants.PRECISION_MAX_PROFILE_STEPS_PER_EXECUTE);
  }

  @Override
  public void execute() {
    var checkedPose = drive.getPose();
    var checkedSpeeds = drive.getRobotRelativeSpeeds();
    if (!Double.isFinite(checkedPose.getX()) || !Double.isFinite(checkedPose.getY())
        || !Double.isFinite(checkedPose.getRotation().getRadians())
        || !Double.isFinite(checkedSpeeds.vxMetersPerSecond) || !Double.isFinite(checkedSpeeds.vyMetersPerSecond)
        || !Double.isFinite(drive.getGyroYawRateRadiansPerSecond())) {
      completionReason = CompletionReason.INVALID_STATE;
      drive.stop();
      return;
    }
    double executeTimestampSeconds = Timer.getFPGATimestamp();
    double controllerLoopDtSeconds =
        Math.max(0.0, executeTimestampSeconds - lastExecuteTimestampSeconds);
    lastExecuteTimestampSeconds = executeTimestampSeconds;
    profileTimeAccumulatorSeconds =
        Math.min(
            profileTimeAccumulatorSeconds + controllerLoopDtSeconds,
            PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS
                * PrecisionConstants.PRECISION_MAX_PROFILE_STEPS_PER_EXECUTE);
    int profileStepsThisExecute =
        Math.min(
            (int)
                Math.floor(
                    profileTimeAccumulatorSeconds
                        / PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS),
            PrecisionConstants.PRECISION_MAX_PROFILE_STEPS_PER_EXECUTE);

    Pose2d pose = drive.getPose();
    ChassisSpeeds kinematicVelocityRobot = drive.getRobotRelativeSpeeds();
    double gyroYawRateRadiansPerSecond = drive.getGyroYawRateRadiansPerSecond();
    // Keep module-derived vx/vy, but use the Pigeon for omega. The 4844 log proved the module-
    // kinematic omega could disagree with the gyro-owned heading strongly enough for rotational
    // damping to oppose the correction that the pose error actually required.
    ChassisSpeeds measuredVelocityRobot =
        new ChassisSpeeds(
            kinematicVelocityRobot.vxMetersPerSecond,
            kinematicVelocityRobot.vyMetersPerSecond,
            gyroYawRateRadiansPerSecond);
    ChassisSpeeds measuredVelocityField =
        ChassisSpeeds.fromRobotRelativeSpeeds(measuredVelocityRobot, pose.getRotation());

    // Profiled PID returns the control effort; getSetpoint().velocity is the profile's feedforward
    // velocity, which trapezoids to zero at the goal. Keep the terms separate so the next log can
    // distinguish profile demand, pose-feedback correction, clamping, and low-level velocity error.
    // ProfiledPIDController advances by its configured 20 ms every time calculate() is called. The
    // 7b6d log averaged 34.5 ms between executions, so one call per execute left the profile far behind
    // both wall time and the physical robot. Advance an equal number of X/Y/theta steps from accumulated
    // elapsed time. Gains have zero I/D today, making repeated same-measurement catch-up deterministic;
    // revisit this mechanism before adding either term.
    double xFeedback = 0.0;
    double yFeedback = 0.0;
    double thetaFeedback = 0.0;
    for (int step = 0; step < profileStepsThisExecute; step++) {
      xFeedback = xController.calculate(pose.getX(), targetPose.getX());
      yFeedback = yController.calculate(pose.getY(), targetPose.getY());
      thetaFeedback =
          thetaController.calculate(
              pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    }
    profileTimeAccumulatorSeconds -=
        profileStepsThisExecute * PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS;
    profileElapsedSeconds +=
        profileStepsThisExecute * PrecisionConstants.PRECISION_PROFILE_PERIOD_SECONDS;

    // A loop can occasionally execute sooner than the nominal 20 ms immediately after an overrun.
    // Hold the trapezoid and compute current proportional feedback against the existing setpoint.
    // These expressions intentionally match the configured zero-I/zero-D controllers.
    if (profileStepsThisExecute == 0) {
      xFeedback =
          PrecisionConstants.PRECISION_DRIVE_KP * (xController.getSetpoint().position - pose.getX());
      yFeedback =
          PrecisionConstants.PRECISION_DRIVE_KP * (yController.getSetpoint().position - pose.getY());
      thetaFeedback =
          PrecisionConstants.PRECISION_THETA_KP
              * MathUtil.angleModulus(
                  thetaController.getSetpoint().position - pose.getRotation().getRadians());
    }
    var xSetpoint = xController.getSetpoint();
    var ySetpoint = yController.getSetpoint();
    var thetaSetpoint = thetaController.getSetpoint();

    double translationError = pose.getTranslation().getDistance(targetPose.getTranslation());
    double translationFeedforwardScale =
        feedforwardScaleForDistance(
            translationError,
            PrecisionConstants.PRECISION_TRANSLATION_FF_MIN_RADIUS_METERS,
            PrecisionConstants.PRECISION_TRANSLATION_FF_MAX_RADIUS_METERS);

    // Fade only translation feedforward based on the robot's measured remaining distance. The
    // profile remains responsible for the smooth acceleration ramp, while the fade prevents a
    // lagging internal profile from pushing the robot forward through the physical target. Pose
    // feedback remains at full authority so it can brake immediately when the robot is ahead.
    double fadedXProfileVelocity = xSetpoint.velocity * translationFeedforwardScale;
    double fadedYProfileVelocity = ySetpoint.velocity * translationFeedforwardScale;
    double translationDampingScale = 1.0 - translationFeedforwardScale;
    double xVelocityDamping =
        velocityDamping(
            measuredVelocityField.vxMetersPerSecond,
            PrecisionConstants.PRECISION_TRANSLATION_VELOCITY_DAMPING,
            translationDampingScale);
    double yVelocityDamping =
        velocityDamping(
            measuredVelocityField.vyMetersPerSecond,
            PrecisionConstants.PRECISION_TRANSLATION_VELOCITY_DAMPING,
            translationDampingScale);
    double omegaVelocityDamping =
        velocityDamping(
            measuredVelocityField.omegaRadiansPerSecond,
            PrecisionConstants.PRECISION_ROTATION_VELOCITY_DAMPING,
            1.0);
    double xSpeedUnclamped = xFeedback + fadedXProfileVelocity + xVelocityDamping;
    double ySpeedUnclamped = yFeedback + fadedYProfileVelocity + yVelocityDamping;
    double omegaUnclamped = thetaFeedback + thetaSetpoint.velocity + omegaVelocityDamping;

    // Clamp translational speed as a VECTOR (see clampTranslationToMax): per-axis clamping would let a
    // diagonal command reach sqrt(2) * max. Omega is bounded separately.
    double[] clamped =
        clampTranslationToMax(
            xSpeedUnclamped,
            ySpeedUnclamped,
            PrecisionConstants.PRECISION_MAX_SPEED_METERS_PER_SECOND);
    double xSpeed = clamped[0];
    double ySpeed = clamped[1];
    double omega = MathUtil.clamp(
        omegaUnclamped,
        -PrecisionConstants.PRECISION_MAX_OMEGA_RADIANS_PER_SECOND,
        PrecisionConstants.PRECISION_MAX_OMEGA_RADIANS_PER_SECOND);

    ChassisSpeeds rawProfileVelocityField =
        new ChassisSpeeds(xSetpoint.velocity, ySetpoint.velocity, thetaSetpoint.velocity);
    ChassisSpeeds profileVelocityField =
        new ChassisSpeeds(
            fadedXProfileVelocity, fadedYProfileVelocity, thetaSetpoint.velocity);
    ChassisSpeeds feedbackVelocityField = new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);
    ChassisSpeeds dampingVelocityField =
        new ChassisSpeeds(xVelocityDamping, yVelocityDamping, omegaVelocityDamping);
    ChassisSpeeds requestedVelocityFieldUnclamped =
        new ChassisSpeeds(xSpeedUnclamped, ySpeedUnclamped, omegaUnclamped);
    ChassisSpeeds controllerRequestedVelocityField = new ChassisSpeeds(xSpeed, ySpeed, omega);
    ChassisSpeeds controllerRequestedVelocityRobot =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, pose.getRotation());

    double rotationErrorDeg = Math.abs(pose.getRotation().minus(targetPose.getRotation()).getDegrees());
    double measuredTranslationSpeed =
        Math.hypot(
            measuredVelocityField.vxMetersPerSecond, measuredVelocityField.vyMetersPerSecond);
    double measuredRotationSpeedDeg =
        Math.abs(Math.toDegrees(measuredVelocityField.omegaRadiansPerSecond));
    double maxAbsModuleSpeed = 0.0;
    for (var moduleState : drive.getModuleStates()) {
      maxAbsModuleSpeed = Math.max(maxAbsModuleSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }
    boolean finishMotionQualifiedThisLoop =
        !strictFinishMotionRequired
            || isFinishMotionCalm(maxAbsModuleSpeed, measuredRotationSpeedDeg);
    boolean withinPoseTolerance =
        translationError <= translationToleranceMeters
            && rotationErrorDeg <= rotationToleranceDegrees;
    boolean withinVelocityTolerance =
        measuredTranslationSpeed
                <= PrecisionConstants.PRECISION_SETTLE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND
            && measuredRotationSpeedDeg
                <= PrecisionConstants.PRECISION_SETTLE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND;
    boolean goalQualifiedThisLoop =
        withinPoseTolerance && withinVelocityTolerance;
    boolean outsideSettlingEscapePoseTolerance =
        exceedsSettleEscapeTolerance(
            translationError,
            rotationErrorDeg,
            PrecisionConstants.PRECISION_SETTLE_ESCAPE_TRANSLATION_METERS,
            PrecisionConstants.PRECISION_SETTLE_ESCAPE_ROTATION_DEGREES);
    boolean outsideSettlingEscapeVelocityTolerance =
        exceedsSettleEscapeVelocityTolerance(
            measuredTranslationSpeed,
            measuredRotationSpeedDeg,
            PrecisionConstants.PRECISION_SETTLE_ESCAPE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND,
            PrecisionConstants.PRECISION_SETTLE_ESCAPE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND);
    boolean velocityEscapeConfirmed =
        velocityEscape.update(
            settlingHoldLatched,
            outsideSettlingEscapeVelocityTolerance,
            executeTimestampSeconds);
    boolean outsideSettlingEscapeTolerance =
        outsideSettlingEscapePoseTolerance || velocityEscapeConfirmed;
    boolean poseRequalificationConfirmed =
        poseRequalification.update(
            settlingHoldLatched, !withinPoseTolerance, executeTimestampSeconds);
    outsideSettlingEscapeTolerance |= poseRequalificationConfirmed;

    if (withinPoseTolerance && !wasWithinPoseTolerance) {
      poseToleranceEntryCount++;
    }
    wasWithinPoseTolerance = withinPoseTolerance;

    // Latch the zero-velocity hold after the first pose+velocity qualification. The 4b2a639a robot
    // log entered AtGoal five times because ordinary estimator/velocity noise released the hold and
    // restarted active correction. Pose escape is immediate; speed escape must persist for 80 ms.
    // Keep commanding zero while confirming motion instead of restarting corrections on one spike.
    // Also resume after 200 ms continuously outside the tight pose limits: otherwise a pose
    // between entry and escape limits can hold zero indefinitely without qualifying (e707).
    // Pending motion blocks completion, and successful completion also rechecks current tight
    // pose/speed limits: the 0b06 run showed why a historical qualification alone is insufficient.
    if (settlingHoldLatched && outsideSettlingEscapeTolerance) {
      settlingHoldLatched = false;
      settlingHoldExitCount++;
      settleTimer.stop();
      settleTimer.reset();
    }
    if (!settlingHoldLatched && goalQualifiedThisLoop) {
      settlingHoldLatched = true;
      atGoalEntryCount++;
      settleTimer.restart();
    }
    boolean atGoal = settlingHoldLatched;
    // Hold age is not continuous qualification: brief pose/speed failures leave the zero hold
    // latched. Require a fresh uninterrupted qualification window rather than finishing on one
    // good sample after an old hold has aged. Brief failures reset this clock, not the zero hold.
    // On a rotating final approach, all wheels and the gyro must also be calm for the full
    // interval. H4 6e71 reached pose but rotated ~6 deg after premature finish. Straight
    // handoffs retain their already-tested finish gate.
    boolean continuousFinishConfirmed =
        finishConfirmation.update(
            settlingHoldLatched,
            goalQualifiedThisLoop && finishMotionQualifiedThisLoop && !velocityEscape.isPending(),
            executeTimestampSeconds);
    finishQualified =
        canFinishHold(
            settlingHoldLatched,
            goalQualifiedThisLoop && finishMotionQualifiedThisLoop,
            velocityEscape.isPending(),
            continuousFinishConfirmed);

    // Once position and velocity are both acceptable, hold a closed-loop zero request instead of
    // continuing to chase sub-tolerance pose noise during the settle timer. Preserve the controller's
    // pre-hold request separately in the log for diagnosis.
    ChassisSpeeds requestedVelocityField =
        atGoal ? new ChassisSpeeds() : controllerRequestedVelocityField;
    ChassisSpeeds requestedVelocityRobot =
        atGoal ? new ChassisSpeeds() : controllerRequestedVelocityRobot;
    ChassisSpeeds trackingErrorRobot =
        new ChassisSpeeds(
            requestedVelocityRobot.vxMetersPerSecond - measuredVelocityRobot.vxMetersPerSecond,
            requestedVelocityRobot.vyMetersPerSecond - measuredVelocityRobot.vyMetersPerSecond,
            requestedVelocityRobot.omegaRadiansPerSecond - measuredVelocityRobot.omegaRadiansPerSecond);

    if (atGoal) {
      drive.holdPrecisionModuleAngles();
    } else {
      drive.driveRobotRelativeVelocity(requestedVelocityRobot);
    }
    Logger.recordOutput(
        "DriveToPose/Controller/DriveRequestType", atGoal ? "VelocityAngleHold" : "Velocity");

    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
    Logger.recordOutput("DriveToPose/MeasuredPose", pose);
    Logger.recordOutput("DriveToPose/TranslationErrorMeters", translationError);
    Logger.recordOutput("DriveToPose/RotationErrorDegrees", rotationErrorDeg);
    Logger.recordOutput("DriveToPose/WithinPoseTolerance", withinPoseTolerance);
    Logger.recordOutput("DriveToPose/WithinVelocityTolerance", withinVelocityTolerance);
    Logger.recordOutput("DriveToPose/GoalQualifiedThisLoop", goalQualifiedThisLoop);
    Logger.recordOutput("DriveToPose/FinishMotionQualifiedThisLoop", finishMotionQualifiedThisLoop);
    Logger.recordOutput("DriveToPose/MaxAbsModuleSpeedMetersPerSecond", maxAbsModuleSpeed);
    Logger.recordOutput("DriveToPose/FinishQualified", finishQualified);
    Logger.recordOutput("DriveToPose/VelocityEscapePending", velocityEscape.isPending());
    Logger.recordOutput("DriveToPose/VelocityEscapeConfirmed", velocityEscapeConfirmed);
    Logger.recordOutput("DriveToPose/PoseRequalificationPending", poseRequalification.isPending());
    Logger.recordOutput("DriveToPose/PoseRequalificationConfirmed", poseRequalificationConfirmed);
    Logger.recordOutput(
        "DriveToPose/PoseRequalificationSeconds",
        poseRequalification.elapsedSeconds(executeTimestampSeconds));
    Logger.recordOutput(
        "DriveToPose/VelocityEscapeSeconds", velocityEscape.elapsedSeconds(executeTimestampSeconds));
    Logger.recordOutput(
        "DriveToPose/OutsideSettlingEscapeTolerance", outsideSettlingEscapeTolerance);
    Logger.recordOutput(
        "DriveToPose/OutsideSettlingEscapePoseTolerance", outsideSettlingEscapePoseTolerance);
    Logger.recordOutput(
        "DriveToPose/OutsideSettlingEscapeVelocityTolerance",
        outsideSettlingEscapeVelocityTolerance);
    Logger.recordOutput("DriveToPose/AtGoal", atGoal);
    Logger.recordOutput("DriveToPose/SettlingHoldActive", settlingHoldLatched);
    Logger.recordOutput("DriveToPose/PoseToleranceEntryCount", poseToleranceEntryCount);
    Logger.recordOutput("DriveToPose/AtGoalEntryCount", atGoalEntryCount);
    Logger.recordOutput("DriveToPose/SettlingHoldExitCount", settlingHoldExitCount);
    Logger.recordOutput(
        "DriveToPose/MeasuredTranslationSpeedMetersPerSecond", measuredTranslationSpeed);
    Logger.recordOutput(
        "DriveToPose/MeasuredRotationSpeedDegreesPerSecond", measuredRotationSpeedDeg);
    Logger.recordOutput("DriveToPose/SettleSeconds", settleTimer.get());
    Logger.recordOutput(
        "DriveToPose/FinishQualificationSeconds",
        finishConfirmation.elapsedSeconds(executeTimestampSeconds));
    Logger.recordOutput("DriveToPose/ErrorXFieldMeters", targetPose.getX() - pose.getX());
    Logger.recordOutput("DriveToPose/ErrorYFieldMeters", targetPose.getY() - pose.getY());
    Logger.recordOutput(
        "DriveToPose/ErrorThetaSignedDegrees",
        targetPose.getRotation().minus(pose.getRotation()).getDegrees());
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileSetpointPose",
        new Pose2d(
            xSetpoint.position,
            ySetpoint.position,
            new Rotation2d(thetaSetpoint.position)));
    Logger.recordOutput("DriveToPose/Controller/RawProfileVelocityField", rawProfileVelocityField);
    Logger.recordOutput("DriveToPose/Controller/ProfileVelocityField", profileVelocityField);
    Logger.recordOutput("DriveToPose/Controller/FeedbackVelocityField", feedbackVelocityField);
    Logger.recordOutput("DriveToPose/Controller/DampingVelocityField", dampingVelocityField);
    Logger.recordOutput(
        "DriveToPose/Controller/RequestedVelocityFieldUnclamped",
        requestedVelocityFieldUnclamped);
    Logger.recordOutput(
        "DriveToPose/Controller/ControllerRequestedVelocityField",
        controllerRequestedVelocityField);
    Logger.recordOutput(
        "DriveToPose/Controller/ControllerRequestedVelocityRobot",
        controllerRequestedVelocityRobot);
    Logger.recordOutput("DriveToPose/Controller/RequestedVelocityField", requestedVelocityField);
    Logger.recordOutput("DriveToPose/Controller/RequestedVelocityRobot", requestedVelocityRobot);
    Logger.recordOutput("DriveToPose/Controller/MeasuredVelocityField", measuredVelocityField);
    Logger.recordOutput("DriveToPose/Controller/MeasuredVelocityRobot", measuredVelocityRobot);
    Logger.recordOutput("DriveToPose/Controller/VelocityTrackingErrorRobot", trackingErrorRobot);
    Logger.recordOutput(
        "DriveToPose/Controller/DistanceToTargetMeters", translationError);
    Logger.recordOutput(
        "DriveToPose/Controller/TranslationFeedforwardScale", translationFeedforwardScale);
    Logger.recordOutput(
        "DriveToPose/Controller/TranslationDampingScale", translationDampingScale);
    Logger.recordOutput(
        "DriveToPose/Controller/DampingVxFieldMetersPerSecond", xVelocityDamping);
    Logger.recordOutput(
        "DriveToPose/Controller/DampingVyFieldMetersPerSecond", yVelocityDamping);
    Logger.recordOutput(
        "DriveToPose/Controller/DampingOmegaDegreesPerSecond",
        Math.toDegrees(omegaVelocityDamping));
    Logger.recordOutput(
        "DriveToPose/Controller/ControllerRequestedVxRobotMetersPerSecond",
        controllerRequestedVelocityRobot.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/ControllerRequestedOmegaDegreesPerSecond",
        Math.toDegrees(controllerRequestedVelocityRobot.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/RawProfileVxFieldMetersPerSecond",
        rawProfileVelocityField.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/RawProfileVyFieldMetersPerSecond",
        rawProfileVelocityField.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileVxFieldMetersPerSecond",
        profileVelocityField.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/FeedbackVxFieldMetersPerSecond",
        feedbackVelocityField.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileVyFieldMetersPerSecond",
        profileVelocityField.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/FeedbackVyFieldMetersPerSecond",
        feedbackVelocityField.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileOmegaDegreesPerSecond",
        Math.toDegrees(profileVelocityField.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/FeedbackOmegaDegreesPerSecond",
        Math.toDegrees(feedbackVelocityField.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/RequestedVxRobotMetersPerSecond",
        requestedVelocityRobot.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/MeasuredVxRobotMetersPerSecond",
        measuredVelocityRobot.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/TrackingErrorVxRobotMetersPerSecond",
        trackingErrorRobot.vxMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/RequestedVyRobotMetersPerSecond",
        requestedVelocityRobot.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/MeasuredVyRobotMetersPerSecond",
        measuredVelocityRobot.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/TrackingErrorVyRobotMetersPerSecond",
        trackingErrorRobot.vyMetersPerSecond);
    Logger.recordOutput(
        "DriveToPose/Controller/RequestedOmegaDegreesPerSecond",
        Math.toDegrees(requestedVelocityRobot.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/MeasuredOmegaDegreesPerSecond",
        Math.toDegrees(measuredVelocityRobot.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/KinematicMeasuredOmegaDegreesPerSecond",
        Math.toDegrees(kinematicVelocityRobot.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/TrackingErrorOmegaDegreesPerSecond",
        Math.toDegrees(trackingErrorRobot.omegaRadiansPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/TranslationVelocityTrackingErrorMetersPerSecond",
        Math.hypot(trackingErrorRobot.vxMetersPerSecond, trackingErrorRobot.vyMetersPerSecond));
    Logger.recordOutput(
        "DriveToPose/Controller/TranslationCommandClamped",
        Math.hypot(xSpeedUnclamped, ySpeedUnclamped)
            > PrecisionConstants.PRECISION_MAX_SPEED_METERS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/RotationCommandClamped",
        Math.abs(omegaUnclamped) > PrecisionConstants.PRECISION_MAX_OMEGA_RADIANS_PER_SECOND);
    Logger.recordOutput(
        "DriveToPose/Controller/LoopDtMilliseconds", controllerLoopDtSeconds * 1000.0);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileStepsThisExecute", profileStepsThisExecute);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileTimeAccumulatorMilliseconds",
        profileTimeAccumulatorSeconds * 1000.0);
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileElapsedSeconds", profileElapsedSeconds);
    Logger.recordOutput(
        "DriveToPose/Controller/WallElapsedSeconds", safetyTimer.get());
    Logger.recordOutput(
        "DriveToPose/Controller/ProfileClockLagMilliseconds",
        (safetyTimer.get() - profileElapsedSeconds) * 1000.0);
  }

  @Override
  public boolean isFinished() {
    return completionReason == CompletionReason.INVALID_STATE || (finishQualified && finishPermission.getAsBoolean())
        || safetyTimer.hasElapsed(PrecisionConstants.PRECISION_SAFETY_TIMEOUT_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (interrupted) completionReason = CompletionReason.INTERRUPTED;
    else if (completionReason != CompletionReason.INVALID_STATE) {
      completionReason = finishQualified && finishPermission.getAsBoolean()
          ? CompletionReason.SUCCEEDED : CompletionReason.TIMED_OUT;
    }
    Logger.recordOutput("DriveToPose/CompletionReason", completionReason.toString());
    Logger.recordOutput("DriveToPose/Controller/Active", false);
    Logger.recordOutput("DriveToPose/Finished", true);
    Logger.recordOutput("DriveToPose/TimedOut",
        completionReason == CompletionReason.TIMED_OUT);
  }

  /**
   * Builds the coarse-then-precise handoff: run {@code coarse} (a PathPlanner/Choreo path) until
   * {@code handoffCondition} becomes true (e.g. crossing a spatial line near the endpoint), then bail
   * out of the timed path and finish on this position-tolerance controller.
   *
   * <p>Idea traceability: 6328 {@code AutoCommands}: {@code new DriveTrajectory(...).until(spatial)
   * .andThen(new DriveToPose(...).until(withinTolerance))}. A time-based path should never be what
   * *finishes* a precise move.
   */
  public Command handoffFrom(Command coarse, java.util.function.BooleanSupplier handoffCondition) {
    return coarse.until(handoffCondition).andThen(this);
  }

  /**
   * Scales a field translation velocity {@code (xSpeed, ySpeed)} so its magnitude never exceeds
   * {@code maxSpeed}, preserving direction; returns {@code {x, y}}. Clamping each axis independently
   * would allow {@code sqrt(2) * maxSpeed} on a diagonal. Static + pure so it is unit-testable.
   */
  static double[] clampTranslationToMax(double xSpeed, double ySpeed, double maxSpeed) {
    double norm = Math.hypot(xSpeed, ySpeed);
    if (norm > maxSpeed) {
      double scale = maxSpeed / norm;
      return new double[] {xSpeed * scale, ySpeed * scale};
    }
    return new double[] {xSpeed, ySpeed};
  }

  /**
   * Returns a linear 0..1 velocity-feedforward scale based on measured remaining distance. The scale
   * is zero inside {@code minRadius}, one beyond {@code maxRadius}, and linear between them. Static +
   * pure so the target-crossing behavior is pinned by unit tests.
   */
  static double feedforwardScaleForDistance(double distance, double minRadius, double maxRadius) {
    if (maxRadius <= minRadius) {
      throw new IllegalArgumentException("maxRadius must be greater than minRadius");
    }
    return MathUtil.clamp((Math.max(0.0, distance) - minRadius) / (maxRadius - minRadius), 0.0, 1.0);
  }

  /**
   * Opposes measured velocity by {@code gain * scale}. A scale of zero leaves cruise motion
   * unchanged; a scale of one applies full near-target damping. Static + pure for unit coverage.
   */
  static double velocityDamping(double measuredVelocity, double gain, double scale) {
    return -measuredVelocity * Math.max(0.0, gain) * MathUtil.clamp(scale, 0.0, 1.0);
  }

  /** Final completion needs a calmer robot than the threshold used to enter zero hold. */
  static boolean isFinishMotionCalm(double maxAbsModuleSpeed, double absGyroYawRateDegreesPerSecond) {
    return Double.isFinite(maxAbsModuleSpeed)
        && Double.isFinite(absGyroYawRateDegreesPerSecond)
        && maxAbsModuleSpeed >= 0.0
        && absGyroYawRateDegreesPerSecond >= 0.0
        && maxAbsModuleSpeed <= PrecisionConstants.PRECISION_FINISH_MAX_MODULE_SPEED_METERS_PER_SECOND
        && absGyroYawRateDegreesPerSecond
            <= PrecisionConstants.PRECISION_FINISH_MAX_ROTATION_SPEED_DEGREES_PER_SECOND;
  }

  /** Selects the rotating-stop policy from handoff state, never from an auto name. */
  static boolean requiresRotatingFinish(double absYawCorrectionDegrees, double absGyroYawRateDegreesPerSecond) {
    // An invalid handoff reading must not silently select the less cautious finish policy.
    return !Double.isFinite(absYawCorrectionDegrees)
        || !Double.isFinite(absGyroYawRateDegreesPerSecond)
        || absYawCorrectionDegrees < 0.0
        || absGyroYawRateDegreesPerSecond < 0.0
        || absYawCorrectionDegrees > PrecisionConstants.PRECISION_SETTLE_ESCAPE_ROTATION_DEGREES
            || absGyroYawRateDegreesPerSecond
                > PrecisionConstants.PRECISION_SETTLE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND;
  }

  /** Returns true when position or heading requires immediate release of zero hold. */
  static boolean exceedsSettleEscapeTolerance(
      double translationError,
      double rotationErrorDegrees,
      double translationEscapeMeters,
      double rotationEscapeDegrees) {
    return translationError > translationEscapeMeters
        || rotationErrorDegrees > rotationEscapeDegrees;
  }

  /** Returns the raw speed violation; confirmation is required before releasing zero hold. */
  static boolean exceedsSettleEscapeVelocityTolerance(
      double translationSpeedMetersPerSecond,
      double rotationSpeedDegreesPerSecond,
      double translationSpeedEscapeMetersPerSecond,
      double rotationSpeedEscapeDegreesPerSecond) {
    return Math.abs(translationSpeedMetersPerSecond) > translationSpeedEscapeMetersPerSecond
        || Math.abs(rotationSpeedDegreesPerSecond) > rotationSpeedEscapeDegreesPerSecond;
  }

  /** Confirms a continuous hold-release condition using wall time; clear samples reset the window. */
  static final class SettleVelocityEscape {
    private final double confirmationSeconds;
    private double startSeconds = Double.NaN;

    SettleVelocityEscape(double confirmationSeconds) {
      this.confirmationSeconds = confirmationSeconds;
    }

    boolean update(boolean holding, boolean outsideLimits, double nowSeconds) {
      if (!holding || !outsideLimits) {
        reset();
        return false;
      }
      if (!isPending()) {
        startSeconds = nowSeconds;
      }
      return elapsedSeconds(nowSeconds) >= confirmationSeconds;
    }

    boolean isPending() {
      return !Double.isNaN(startSeconds);
    }

    double elapsedSeconds(double nowSeconds) {
      return isPending() ? Math.max(0.0, nowSeconds - startSeconds) : 0.0;
    }

    void reset() {
      startSeconds = Double.NaN;
    }
  }

  /** Rejects invalid route-specific accuracy settings before any drivetrain output. */
  static double validateTranslationTolerance(double toleranceMeters) {
    if (!Double.isFinite(toleranceMeters) || toleranceMeters <= 0.0
        || toleranceMeters > PrecisionConstants.PRECISION_SETTLE_ESCAPE_TRANSLATION_METERS) {
      throw new IllegalArgumentException("Translation tolerance must be positive and within escape limit");
    }
    return toleranceMeters;
  }

  /** An aged hold alone cannot finish while current pose/speed checks fail or motion is pending. */
  static boolean canFinishHold(
      boolean holding, boolean currentGoalQualified, boolean velocityPending, boolean timeElapsed) {
    return holding && currentGoalQualified && !velocityPending && timeElapsed;
  }
}
