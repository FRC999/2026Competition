package frc.robot.OdometryUpdates;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.RobotContainer;
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.VisionHelpers;

public class OdometryUpdatesSubsystem extends SubsystemBase {
  private static final int PERF_PUBLISH_EVERY_LOOPS = 25;
  /**
   * Limelight AprilTag pose estimation needs a good robot yaw to disambiguate tags,
   * especially when using MegaTag2. This subsystem assumes the drivetrain IMU
   * is the only yaw source and keeps that yaw mirrored into all Limelights.
   *
   * Startup flow:
   * INITIALIZE
   * The drivetrain has either just powered up or has not yet been re-anchored.
   * The subsystem publishes drivetrain yaw to the Limelights and immediately moves
   * into the seeking state.
   *
   * SEEKING_TAGS
   * The robot is waiting for the first good AprilTag-based field pose.
   * Limelight yaw continues to come from drivetrain odometry/Pigeon.
   * Once a valid pose estimate is found, the drivetrain IMU yaw and CTRE pose are
   * reset to that field pose and the state transitions to CALIBRATED.
   *
   * CALIBRATED
   * The robot has a field anchor and now runs LL-only vision fusion.
   * Each loop, drivetrain yaw is pushed to the Limelights and the single best
   * pose estimate is selected and fused if it passes gating.
   *
   * State transitions:
   * INITIALIZE -> SEEKING_TAGS
   *   Enter normal LL acquisition mode.
   *
   * SEEKING_TAGS -> CALIBRATED
   *   A valid Limelight pose estimate was accepted as the field anchor.
   *
   * CALIBRATED -> SEEKING_TAGS
   *   A manual yaw reset or delayed MegaTag1 recalibration requests a fresh LL anchor.
   *
   * Notes:
   * Yaw reset/re-anchor requests force gate bypass temporarily so the next accepted
   * Limelight measurement can establish a fresh anchor.
   * The subsystem deliberately does not use a blind fallback pose when tags are absent.
   */
  private enum VisionState {
    INITIALIZE,
    SEEKING_TAGS,
    CALIBRATED
  }

  private VisionState state = VisionState.INITIALIZE;
  private VisionState prevState = VisionState.INITIALIZE;
  private String lastTransition = "START";
  private double lastTransitionTime = 0.0;
  private int transitionSeq = 0;

  private boolean gatePassOverride = true;
  private boolean hasRequestedReanchor = false;
  private int loopsSinceSeed = 0;
  private boolean visionReady = false;

  private final Timer delayedMegaTag1RecalTimer = new Timer();
  private boolean waitingForMegaTag1Recal = false;
  private long periodicRuntimeAccumNs = 0L;
  private long periodicRuntimeMaxNs = 0L;
  private int periodicRuntimeSamples = 0;

  public OdometryUpdatesSubsystem() {
    if (!EnabledSubsystems.odometry) {
      return;
    }
  }

  private void transitionTo(VisionState newState, String reason) {
    if (newState == state) {
      return;
    }

    prevState = state;
    state = newState;
    transitionSeq++;
    lastTransitionTime = Timer.getFPGATimestamp();
    lastTransition = prevState.name() + " -> " + state.name()
        + (reason != null && !reason.isBlank() ? " | " + reason : "");

    if (Constants.DebugTelemetrySubsystems.odometry) {
      SmartDashboard.putString("Odometry/State", state.name());
      SmartDashboard.putString("Odometry/StateColor", ElasticHelpers.questStatesColors(state.name()));
      SmartDashboard.putString("Odometry/LastTransition", lastTransition);
      SmartDashboard.putNumber("Odometry/TransitionSeq", transitionSeq);
      SmartDashboard.putNumber("Odometry/LastTransitionTimeSec", lastTransitionTime);
    }
  }

  private boolean isReasonablePose(Pose2d pose) {
    if (pose == null) {
      return false;
    }

    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians())
        && Math.abs(pose.getX()) < OdometryConstants.MAX_REASONABLE_FIELD_COORD_ABS_METERS
        && Math.abs(pose.getY()) < OdometryConstants.MAX_REASONABLE_FIELD_COORD_ABS_METERS;
  }

  private boolean gateMeasurement(
      Pose2d robotPose,
      double timestamp,
      boolean strict,
      double speedNow,
      Pose2d poseNow) {
    if (gatePassOverride) {
      return true;
    }

    Pose2d chassisPoseAtTimestamp = RobotContainer.driveSubsystem.getSample(timestamp).orElse(poseNow);
    var twist = chassisPoseAtTimestamp.minus(robotPose);
    double transErr = Math.hypot(twist.getX(), twist.getY());
    double rotErr = Math.abs(twist.getRotation().getRadians());

    final double transGate = (strict ? 0.6 : 0.8) + 0.5 * speedNow;
    final double rotGate = Units.degreesToRadians(strict ? 15.0 : 20.0);

    return transErr <= transGate && rotErr <= rotGate;
  }

  private boolean shouldRejectPoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate == null || poseEstimate.tagCount <= 0 || poseEstimate.rawFiducials == null
        || poseEstimate.rawFiducials.length == 0) {
      return true;
    }

    double ambiguity = MathUtil.clamp(poseEstimate.rawFiducials[0].ambiguity, 0.0, 1.0);
    return (poseEstimate.tagCount == 1 && ambiguity > LLVisionConstants.kMaxSingleTagAmbiguity)
        || poseEstimate.rawFiducials[0].distToCamera > LLVisionConstants.kMaxCameraToTargetDistance
        || !isReasonablePose(poseEstimate.pose);
  }

  private boolean fusePoseEstimate(LimelightHelpers.PoseEstimate poseEstimate, String cameraName, boolean strict) {
    if (shouldRejectPoseEstimate(poseEstimate)) {
      VisionHelpers.clearLLTelemetry(cameraName);
      return false;
    }

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;
    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;
    double timestampSeconds = poseEstimate.timestampSeconds > 1.0
        ? poseEstimate.timestampSeconds
        : Timer.getFPGATimestamp() - poseEstimate.latency;

    if (!gateMeasurement(poseEstimate.pose, timestampSeconds, strict, speedNow, poseNow)) {
      VisionHelpers.clearLLTelemetry(cameraName);
      return false;
    }

    if (timestampSeconds < RobotContainer.driveSubsystem.getYawSeedTimestamp()) {
      return false;
    }

    double ambiguity = MathUtil.clamp(poseEstimate.rawFiducials[0].ambiguity, 0.0, 1.0);
    Matrix<N3, N1> std = LimelightHelpers.llStdDev(
        poseEstimate.avgTagDist,
        poseEstimate.tagCount,
        ambiguity);

    if (visionReady) {
      RobotContainer.driveSubsystem.addVisionMeasurement(poseEstimate.pose, timestampSeconds, std);
    }
    VisionHelpers.updateLLTelemetry(poseEstimate, cameraName);
    return true;
  }

  private void resetRobotPoseFromVision(LimelightHelpers.PoseEstimate poseEstimate) {
    RobotContainer.driveSubsystem.resetChassisIMUToAngle(poseEstimate.pose.getRotation().getDegrees());
    RobotContainer.driveSubsystem.resetCTREPose(poseEstimate.pose);
    gatePassOverride = false;

    if (RobotContainer.llAprilTagSubsystem.wasLastBestPoseMegaTag1()) {
      scheduleDelayedMegaTag1Recalibration();
    } else {
      cancelDelayedMegaTag1Recalibration();
    }
  }

  public void handlePostYawSeed() {
    if (hasRequestedReanchor) {
      return;
    }

    loopsSinceSeed++;
    if (loopsSinceSeed <= 1) {
      return;
    }

    requestReanchorFromLimelightAfterYawReset();
    visionReady = true;
    hasRequestedReanchor = true;
  }

  public void requestReanchorFromLimelightAfterYawReset() {
    if (!RobotContainer.driveSubsystem.hasFinishedSeeding()) {
      return;
    }

    gatePassOverride = true;

    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    RobotContainer.llAprilTagSubsystem.setLLOrientation(
        robotPose.getRotation().getDegrees(),
        RobotContainer.driveSubsystem.getTurnRate());

    transitionTo(VisionState.SEEKING_TAGS, "Driver yaw reset; re-seek LL");
  }

  private void scheduleDelayedMegaTag1Recalibration() {
    delayedMegaTag1RecalTimer.reset();
    delayedMegaTag1RecalTimer.start();
    waitingForMegaTag1Recal = true;
    if (DebugTelemetrySubsystems.odometry || DebugTelemetrySubsystems.llLight) {
      SmartDashboard.putBoolean("Odometry/WaitingForMegaTag1Recal", true);
    }
  }

  private void recordPeriodicRuntime(long elapsedNs) {
    if (!DebugTelemetrySubsystems.perfLight) {
      return;
    }

    periodicRuntimeAccumNs += elapsedNs;
    periodicRuntimeMaxNs = Math.max(periodicRuntimeMaxNs, elapsedNs);
    periodicRuntimeSamples++;

    if (periodicRuntimeSamples >= PERF_PUBLISH_EVERY_LOOPS) {
      SmartDashboard.putNumber(
          "Perf/Odometry/PeriodicMsAvg",
          periodicRuntimeAccumNs / 1_000_000.0 / periodicRuntimeSamples);
      SmartDashboard.putNumber("Perf/Odometry/PeriodicMsMax", periodicRuntimeMaxNs / 1_000_000.0);
      periodicRuntimeAccumNs = 0L;
      periodicRuntimeMaxNs = 0L;
      periodicRuntimeSamples = 0;
    }
  }

  private void cancelDelayedMegaTag1Recalibration() {
    delayedMegaTag1RecalTimer.stop();
    delayedMegaTag1RecalTimer.reset();
    waitingForMegaTag1Recal = false;
    if (DebugTelemetrySubsystems.odometry || DebugTelemetrySubsystems.llLight) {
      SmartDashboard.putBoolean("Odometry/WaitingForMegaTag1Recal", false);
    }
  }

  private void handleDelayedMegaTag1Recalibration() {
    if (!waitingForMegaTag1Recal) {
      return;
    }

    if (delayedMegaTag1RecalTimer.hasElapsed(5.0)) {
      cancelDelayedMegaTag1Recalibration();
      requestReanchorFromLimelightAfterYawReset();
      if (DebugTelemetrySubsystems.odometry) {
        System.out.println("Triggered delayed LL recalibration 5s after MegaTag1 anchor");
      }
    }
  }

  @Override
  public void periodic() {
    long startNs = DebugTelemetrySubsystems.perfLight ? System.nanoTime() : 0L;
    if (!EnabledSubsystems.odometry || RobotBase.isSimulation()) {
      return;
    }

    if (DebugTelemetrySubsystems.odometry) {
      SmartDashboard.putString("Odometry/UpdatesState", state.name());
      SmartDashboard.putBoolean("Odometry/GatePassOverride", gatePassOverride);
    }

    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    RobotContainer.llAprilTagSubsystem.setLLOrientation(
        robotPose.getRotation().getDegrees(),
        RobotContainer.driveSubsystem.getTurnRate());

    LimelightHelpers.PoseEstimate bestPoseEstimate = Constants.EnabledSubsystems.ll
        ? RobotContainer.llAprilTagSubsystem.getBestPoseEstimateFromAllLL()
        : null;
    String bestCameraName = RobotContainer.llAprilTagSubsystem.getLastBestPoseCameraName();

    switch (state) {
      case INITIALIZE -> transitionTo(VisionState.SEEKING_TAGS, "Initialized LL-only odometry");
      case SEEKING_TAGS -> {
        if (bestPoseEstimate != null && bestCameraName != null) {
          resetRobotPoseFromVision(bestPoseEstimate);
          transitionTo(VisionState.CALIBRATED, "Good LL fix; anchored field pose");
        }
      }
      case CALIBRATED -> {
        if (bestPoseEstimate != null && bestCameraName != null) {
          fusePoseEstimate(bestPoseEstimate, bestCameraName, true);
        }
        handleDelayedMegaTag1Recalibration();
      }
    }
    recordPeriodicRuntime(DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
  }
}
