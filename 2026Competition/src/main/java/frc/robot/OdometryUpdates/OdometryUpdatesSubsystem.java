package frc.robot.OdometryUpdates;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.lib.QuestHelpers;
import frc.robot.lib.VisionHelpers;
import gg.questnav.questnav.PoseFrame;

/**
 * Hybrid odometry state machine.
 *
 * Core responsibilities:
 * - keep Limelight IMU/yaw inputs synchronized with the heading source currently being trusted
 * - acquire the first valid field anchor from Limelight
 * - promote Quest to the primary pose source once Quest is healthy and field-calibrated
 * - preserve the current LL-only fallback behavior if Quest is unavailable or stale
 * - force fresh LL re-anchors after yaw resets, delayed MegaTag1 recalibration, or sustained tag loss
 * - re-anchor Quest from LL before returning to Quest-primary mode
 *
 * Important operating rules:
 * - Limelight startup seeding still uses the current strategy:
 *   prefer MT1 multi-tag first, then allow MT2 fallback only after the configured delay.
 * - When LL fallback is active, the code keeps the current LL behavior rather than the old Quest-era LL behavior:
 *   current LL IMU mode selection, MT1 assist when reliable, best-pose LL fusion,
 *   delayed MegaTag1 recalibration, and tag-loss recovery/re-anchor.
 * - Yaw reset and re-anchor requests temporarily enable gatePassOverride so the next accepted
 *   LL measurement can establish a fresh anchor instead of being rejected by normal innovation gating.
 * - If all tags are lost for long enough while LL is primary, the subsystem arms a re-anchor request;
 *   once tags return, LL is allowed to re-establish the anchor.
 *
 * States:
 * - INITIALIZE:
 *   Startup placeholder. Immediately routes to Quest-assisted seeking if fresh Quest
 *   tracking is already present, otherwise to LL fallback seeking.
 *
 * - SEEKING_TAGS_Q:
 *   Quest is present, but the robot does not yet have a trusted field anchor.
 *   LL is still responsible for the initial field pose. During this phase, LL yaw
 *   orientation is fed from Quest when Quest robot pose is reasonable.
 *   Once a valid LL anchor is accepted, drivetrain odometry is reset to that pose,
 *   Quest is reset to the same field pose, and the state moves to CALIBRATED_Q.
 *
 * - SEEKING_TAGS_NO_Q:
 *   LL-only startup/fallback seek mode. The robot waits for a valid LL field pose using the
 *   modern LL selection logic. If Quest comes online during this state, Quest yaw is first aligned
 *   to the current robot yaw, then the state moves to SEEKING_TAGS_Q.
 *
 * - CALIBRATED_Q:
 *   Quest is primary. Quest frames are fused into the drivetrain estimator while Quest
 *   remains fresh and calibrated. LL remains available for re-anchoring and recovery,
 *   but does not replace the primary Quest tracking path here.
 *   If Quest is lost or stale for longer than the hold timer, the system falls back to CALIBRATED_NO_Q.
 *
 * - CALIBRATED_NO_Q:
 *   Current LL-only operating mode. LL continues to drive pose updates using the current
 *   LL logic. Sustained tag loss arms a re-anchor so the first reliable returning LL fix can
 *   re-establish the anchor cleanly. If Quest returns, LL must first provide a good field pose
 *   so Quest can be re-anchored before switching back to CALIBRATED_Q.
 */
public class OdometryUpdatesSubsystem extends SubsystemBase {
  private static final int PERF_PUBLISH_EVERY_LOOPS = 25;
  private static final double INITIAL_MT1_SEED_WAIT_BEFORE_MT2_FALLBACK_SEC = 5.0;

  private enum VisionState {
    INITIALIZE,
    SEEKING_TAGS_Q,
    SEEKING_TAGS_NO_Q,
    CALIBRATED_Q,
    CALIBRATED_NO_Q
  }

  private VisionState state = VisionState.INITIALIZE;
  private VisionState prevState = VisionState.INITIALIZE;
  private String lastTransition = "START";
  private double lastTransitionTime = 0.0;
  private int transitionSeq = 0;

  private boolean gatePassOverride = true;
  private boolean initialVisionAnchorComplete = false;
  private boolean hasRequestedReanchor = false;
  private int loopsSinceSeed = 0;
  private boolean visionReady = false;
  private double allTagsLostStartTs = Double.NaN;
  private boolean pendingReanchorOnVisionReturn = false;

  private final Timer questLossHoldTimer = new Timer();
  private final Timer delayedMegaTag1RecalTimer = new Timer();
  private boolean waitingForMegaTag1Recal = false;
  private long periodicRuntimeAccumNs = 0L;
  private long periodicRuntimeMaxNs = 0L;
  private int periodicRuntimeSamples = 0;

  public OdometryUpdatesSubsystem() {
    if (!EnabledSubsystems.odometry) {
      return;
    }

    SmartDashboard.putNumber(
        OdometryConstants.TAG_LOSS_REANCHOR_ARM_DELAY_DASHBOARD_KEY,
        SmartDashboard.getNumber(
            OdometryConstants.TAG_LOSS_REANCHOR_ARM_DELAY_DASHBOARD_KEY,
            OdometryConstants.TAG_LOSS_REANCHOR_ARM_DELAY_SEC_DEFAULT));
  }

  private double getTagLossReanchorArmDelaySec() {
    return SmartDashboard.getNumber(
        OdometryConstants.TAG_LOSS_REANCHOR_ARM_DELAY_DASHBOARD_KEY,
        OdometryConstants.TAG_LOSS_REANCHOR_ARM_DELAY_SEC_DEFAULT);
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

    if (prevState == VisionState.CALIBRATED_Q) {
      questLossHoldTimer.stop();
      questLossHoldTimer.reset();
    }

    // When Quest is primary, LL must not be able to inject delayed MT1 re-anchors
    // back into robot odometry. LL-only recalibration remains available in LL states.
    if (state == VisionState.CALIBRATED_Q) {
      cancelDelayedMegaTag1Recalibration();
    }

    if (DebugTelemetrySubsystems.odometry) {
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
        && Math.abs(pose.getY()) < OdometryConstants.MAX_REASONABLE_FIELD_COORD_ABS_METERS
        && !pose.equals(QuestNavConstants.NULL_POSE);
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

  private boolean shouldRejectInitialSeedPoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate == null || poseEstimate.tagCount <= 0 || poseEstimate.rawFiducials == null
        || poseEstimate.rawFiducials.length == 0) {
      return true;
    }

    double ambiguity = MathUtil.clamp(poseEstimate.rawFiducials[0].ambiguity, 0.0, 1.0);
    return (poseEstimate.tagCount == 1 && ambiguity > LLVisionConstants.kMaxSingleTagAmbiguity)
        || poseEstimate.rawFiducials[0].distToCamera > LLVisionConstants.kMaxInitialSeedCameraToTargetDistance
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

  private void fuseQuestNavAllUnread() {
    PoseFrame[] frames = RobotContainer.questNavSubsystem.getAllCurrentPoseframes();
    if (frames == null || frames.length == 0) {
      return;
    }

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;
    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;
    boolean gatePassOverrideIntermediate = gatePassOverride;

    for (PoseFrame poseFrame : frames) {
      if (poseFrame == null) {
        continue;
      }

      Pose2d robotPose;
      try {
        Pose2d questPose = poseFrame.questPose3d().toPose2d();
        robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
      } catch (Throwable t) {
        continue;
      }

      if (!isReasonablePose(robotPose)) {
        continue;
      }

      double measurementTimestamp;
      try {
        measurementTimestamp = poseFrame.dataTimestamp() > 1.0
            ? poseFrame.dataTimestamp()
            : Timer.getFPGATimestamp();
      } catch (Throwable t) {
        continue;
      }

      if (!gateMeasurement(robotPose, measurementTimestamp, false, speedNow, poseNow)) {
        continue;
      }

      if (measurementTimestamp < RobotContainer.driveSubsystem.getYawSeedTimestamp()) {
        continue;
      }

      if (visionReady) {
        RobotContainer.driveSubsystem.addVisionMeasurement(
            robotPose,
            measurementTimestamp,
            QuestHelpers.questStdDev(speedNow));
      }
      gatePassOverrideIntermediate = false;
    }

    gatePassOverride = gatePassOverrideIntermediate;
  }

  private void resetRobotPoseFromVision(LimelightHelpers.PoseEstimate poseEstimate) {
    RobotContainer.driveSubsystem.resetChassisIMUToAngle(poseEstimate.pose.getRotation().getDegrees());
    RobotContainer.driveSubsystem.resetCTREPose(poseEstimate.pose);
    gatePassOverride = false;
    initialVisionAnchorComplete = true;
    clearVisionLossReanchorState();

    if (RobotContainer.llAprilTagSubsystem.wasLastBestPoseMegaTag1()) {
      scheduleDelayedMegaTag1Recalibration();
    } else {
      cancelDelayedMegaTag1Recalibration();
    }
  }

  private void calibrateQuestFromLL(Pose2d robotPose) {
    RobotContainer.questNavSubsystem.resetQuestOdometry(new Pose3d(robotPose));
    RobotContainer.questNavSubsystem.setInitialPoseSet(true);
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

  private void clearVisionLossReanchorState() {
    allTagsLostStartTs = Double.NaN;
    pendingReanchorOnVisionReturn = false;
  }

  private boolean isQuestPrimaryAvailable() {
    return EnabledSubsystems.questnav
        && RobotContainer.questNavSubsystem.hasFreshTracking()
        && RobotContainer.questNavSubsystem.isInitialPoseSet();
  }

  private int getDesiredLimelightImuMode() {
    if (state == VisionState.INITIALIZE
        || state == VisionState.SEEKING_TAGS_Q
        || state == VisionState.SEEKING_TAGS_NO_Q) {
      return LLVisionConstants.LL_IMU_MODE_SEED;
    }

    if (RobotContainer.llAprilTagSubsystem.hasReliableMultiTagMegaTag1Observation()) {
      return LLVisionConstants.LL_IMU_MODE_TRACKING_MT1_ASSIST;
    }

    return LLVisionConstants.LL_IMU_MODE_TRACKING_INTERNAL;
  }

  private void requestReanchorFromLimelight(String reason) {
    if (!RobotContainer.driveSubsystem.hasFinishedSeeding()) {
      return;
    }

    gatePassOverride = true;
    clearVisionLossReanchorState();
    cancelDelayedMegaTag1Recalibration();

    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    RobotContainer.llAprilTagSubsystem.setLLOrientation(
        robotPose.getRotation().getDegrees(),
        RobotContainer.driveSubsystem.getTurnRate());

    transitionTo(
        isQuestPrimaryAvailable() ? VisionState.SEEKING_TAGS_Q : VisionState.SEEKING_TAGS_NO_Q,
        reason);
  }

  public void requestReanchorFromLimelightAfterYawReset() {
    requestReanchorFromLimelight("Driver yaw reset; re-seek LL");
  }

  private void handleVisionLossReturnReanchor(double now) {
    if (state != VisionState.CALIBRATED_NO_Q || !Constants.EnabledSubsystems.ll) {
      clearVisionLossReanchorState();
      return;
    }

    boolean anyTagsVisible = RobotContainer.llAprilTagSubsystem.isAprilTagVisibleAny();

    if (!anyTagsVisible) {
      if (!Double.isFinite(allTagsLostStartTs)) {
        allTagsLostStartTs = now;
      }

      if (now - allTagsLostStartTs >= getTagLossReanchorArmDelaySec()) {
        pendingReanchorOnVisionReturn = true;
      }
      return;
    }

    allTagsLostStartTs = Double.NaN;
    if (pendingReanchorOnVisionReturn) {
      requestReanchorFromLimelight("All tags lost; re-seek LL on return");
    }
  }

  private void scheduleDelayedMegaTag1Recalibration() {
    delayedMegaTag1RecalTimer.reset();
    delayedMegaTag1RecalTimer.start();
    waitingForMegaTag1Recal = true;
    if (DebugTelemetrySubsystems.odometry || DebugTelemetrySubsystems.llLight) {
      SmartDashboard.putBoolean("Odometry/WaitingForMegaTag1Recal", true);
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
        //System.out.println("Triggered delayed LL recalibration 5s after MegaTag1 anchor");
      }
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

  @Override
  public void periodic() {
    long startNs = DebugTelemetrySubsystems.perfLight ? System.nanoTime() : 0L;
    if (!EnabledSubsystems.odometry || RobotBase.isSimulation()) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    RobotContainer.llAprilTagSubsystem.ensureIMUMode(getDesiredLimelightImuMode());

    Pose2d drivePose = RobotContainer.driveSubsystem.getPose();
    RobotContainer.llAprilTagSubsystem.setLLOrientation(
        drivePose.getRotation().getDegrees(),
        RobotContainer.driveSubsystem.getTurnRate());

    if (state == VisionState.SEEKING_TAGS_Q) {
      Pose2d questRobotPose = RobotContainer.questNavSubsystem.getQuestRobotPose2d();
      if (isReasonablePose(questRobotPose)) {
        RobotContainer.llAprilTagSubsystem.setLLOrientation(
            questRobotPose.getRotation().getDegrees(),
            RobotContainer.driveSubsystem.getTurnRate());
      }
    }

    handleVisionLossReturnReanchor(now);

    if (DebugTelemetrySubsystems.odometry) {
      SmartDashboard.putString("Odometry/UpdatesState", state.name());
      SmartDashboard.putBoolean("Odometry/GatePassOverride", gatePassOverride);
      SmartDashboard.putBoolean(
          "Odometry/AnyTagsVisible",
          Constants.EnabledSubsystems.ll && RobotContainer.llAprilTagSubsystem.isAprilTagVisibleAny());
      SmartDashboard.putNumber(
          "Odometry/AllTagsLostForSec",
          Double.isFinite(allTagsLostStartTs) ? now - allTagsLostStartTs : 0.0);
      SmartDashboard.putBoolean("Odometry/PendingVisionReturnReanchor", pendingReanchorOnVisionReturn);
      SmartDashboard.putBoolean("Odometry/QuestPrimaryAvailable", isQuestPrimaryAvailable());
    }

    switch (state) {
      case INITIALIZE -> transitionTo(
          EnabledSubsystems.questnav && RobotContainer.questNavSubsystem.hasFreshTracking()
              ? VisionState.SEEKING_TAGS_Q
              : VisionState.SEEKING_TAGS_NO_Q,
          EnabledSubsystems.questnav && RobotContainer.questNavSubsystem.hasFreshTracking()
              ? "Initialized with Quest tracking available"
              : "Initialized LL fallback seeking");
      case SEEKING_TAGS_Q -> {
        if (!EnabledSubsystems.questnav || !RobotContainer.questNavSubsystem.hasFreshTracking()) {
          transitionTo(VisionState.SEEKING_TAGS_NO_Q, "Quest unavailable during initial seek");
          break;
        }

        boolean allowMegaTag2SeedFallback =
            initialVisionAnchorComplete
                || Timer.getFPGATimestamp() - lastTransitionTime
                    >= INITIAL_MT1_SEED_WAIT_BEFORE_MT2_FALLBACK_SEC;
        LimelightHelpers.PoseEstimate seedPoseEstimate = Constants.EnabledSubsystems.ll
            ? RobotContainer.llAprilTagSubsystem.getInitialSeedPoseEstimateFromAllLL(allowMegaTag2SeedFallback)
            : null;
        String seedCameraName = RobotContainer.llAprilTagSubsystem.getLastBestPoseCameraName();
        if (seedPoseEstimate != null
            && seedCameraName != null
            && !shouldRejectInitialSeedPoseEstimate(seedPoseEstimate)) {
          resetRobotPoseFromVision(seedPoseEstimate);
          calibrateQuestFromLL(seedPoseEstimate.pose);
          transitionTo(VisionState.CALIBRATED_Q, "Good LL fix; Quest anchored to field");
        }
      }
      case SEEKING_TAGS_NO_Q -> {
        if (EnabledSubsystems.questnav && RobotContainer.questNavSubsystem.hasFreshTracking()) {
          RobotContainer.questNavSubsystem.resetQuestIMUToAngle(
              RobotContainer.driveSubsystem.getPose().getRotation().getDegrees());
          transitionTo(VisionState.SEEKING_TAGS_Q, "Quest came online; switching to Quest-assisted seek");
          break;
        }

        boolean allowMegaTag2SeedFallback =
            initialVisionAnchorComplete
                || Timer.getFPGATimestamp() - lastTransitionTime
                    >= INITIAL_MT1_SEED_WAIT_BEFORE_MT2_FALLBACK_SEC;
        LimelightHelpers.PoseEstimate seedPoseEstimate = Constants.EnabledSubsystems.ll
            ? RobotContainer.llAprilTagSubsystem.getInitialSeedPoseEstimateFromAllLL(allowMegaTag2SeedFallback)
            : null;
        String seedCameraName = RobotContainer.llAprilTagSubsystem.getLastBestPoseCameraName();
        if (seedPoseEstimate != null
            && seedCameraName != null
            && !shouldRejectInitialSeedPoseEstimate(seedPoseEstimate)) {
          resetRobotPoseFromVision(seedPoseEstimate);
          transitionTo(VisionState.CALIBRATED_NO_Q, "Good LL fix; LL fallback anchored");
        }
      }
      case CALIBRATED_Q -> {
        if (isQuestPrimaryAvailable()) {
          questLossHoldTimer.stop();
          questLossHoldTimer.reset();
          fuseQuestNavAllUnread();
        } else {
          if (!questLossHoldTimer.isRunning()) {
            questLossHoldTimer.reset();
            questLossHoldTimer.start();
          }

          if (questLossHoldTimer.hasElapsed(OdometryConstants.QUEST_LOSS_HOLD_SEC)) {
            transitionTo(VisionState.CALIBRATED_NO_Q, "Quest lost/stale; falling back to LL");
          } else if (DebugTelemetrySubsystems.odometry) {
            SmartDashboard.putNumber(
                "Odometry/QuestLossHoldRemainingSec",
                Math.max(0.0, OdometryConstants.QUEST_LOSS_HOLD_SEC - questLossHoldTimer.get()));
          }
        }
      }
      case CALIBRATED_NO_Q -> {
        LimelightHelpers.PoseEstimate bestPoseEstimate = Constants.EnabledSubsystems.ll
            ? RobotContainer.llAprilTagSubsystem.getBestPoseEstimateFromAllLL()
            : null;
        String bestCameraName = RobotContainer.llAprilTagSubsystem.getLastBestPoseCameraName();
        if (bestPoseEstimate != null && bestCameraName != null) {
          fusePoseEstimate(bestPoseEstimate, bestCameraName, true);

          if (EnabledSubsystems.questnav && RobotContainer.questNavSubsystem.hasFreshTracking()) {
            calibrateQuestFromLL(bestPoseEstimate.pose);
            RobotContainer.driveSubsystem.resetChassisIMUToAngle(bestPoseEstimate.pose.getRotation().getDegrees());
            RobotContainer.driveSubsystem.resetCTREPose(bestPoseEstimate.pose);
            transitionTo(VisionState.CALIBRATED_Q, "Quest regained; re-anchored from LL");
            break;
          }
        }
        handleDelayedMegaTag1Recalibration();
      }
      default -> {
      }
    }

    recordPeriodicRuntime(DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
  }
}
