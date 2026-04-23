package frc.robot.OdometryUpdates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * QuestNav ingestion subsystem.
 *
 * This class is intentionally not the top-level odometry state machine. Its job is to:
 * - talk to the QuestNav library
 * - call commandPeriodic() every loop for QuestNav 2026-2.2.0
 * - cache the latest unread Quest pose frames
 * - expose Quest tracking/freshness/pose helpers to the odometry state machine
 *
 * Effective local state model:
 * - not tracking:
 *   Quest data should not be trusted
 *
 * - tracking but stale:
 *   Quest still reports tracking, but no fresh unread frames have arrived within the
 *   configured timeout, so odometry should treat Quest as unavailable
 *
 * - tracking and fresh:
 *   Quest may be used by the odometry state machine if it has also been field-calibrated
 *
 * Field calibration itself is owned by OdometryUpdatesSubsystem via initialPoseSet.
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;

  private PoseFrame[] poseFrames = new PoseFrame[0];
  private boolean initialPoseSet = false;
  private double lastFreshFrameFpgaTs = Double.NEGATIVE_INFINITY;
  private int telemetryLoopCounter = 0;

  public QuestNavSubsystem() {
    questNav = new QuestNav();
    if (EnabledSubsystems.questnav) {
      resetToZeroPose();
    }
  }

  public boolean isInitialPoseSet() {
    return initialPoseSet;
  }

  public void setInitialPoseSet(boolean initialPoseSet) {
    this.initialPoseSet = initialPoseSet;
  }

  public void resetToZeroPose() {
    Pose3d questPose = QuestNavConstants.ROBOT_ZERO_POSE_3D.transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D);
    questNav.setPose(questPose);
  }

  public Pose2d getQuestRobotPose2d() {
    if (poseFrames == null || poseFrames.length == 0) {
      return QuestNavConstants.NULL_POSE;
    }

    return poseFrames[poseFrames.length - 1]
        .questPose3d()
        .toPose2d()
        .transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
  }

  public Pose3d getQuestRobotPose3d() {
    if (poseFrames == null || poseFrames.length == 0) {
      return QuestNavConstants.NULL_POSE_3D;
    }

    return poseFrames[poseFrames.length - 1]
        .questPose3d()
        .transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D.inverse());
  }

  public Pose2d getQuestPose2d() {
    if (poseFrames == null || poseFrames.length == 0) {
      return QuestNavConstants.NULL_POSE;
    }

    return poseFrames[poseFrames.length - 1].questPose3d().toPose2d();
  }

  public Pose3d getQuestPose3d() {
    if (poseFrames == null || poseFrames.length == 0) {
      return QuestNavConstants.NULL_POSE_3D;
    }

    return poseFrames[poseFrames.length - 1].questPose3d();
  }

  public double getQuestRobotYaw() {
    return getQuestRobotPose2d().getRotation().getDegrees();
  }

  public double getQTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ? poseFrames[poseFrames.length - 1].dataTimestamp() : 0.0;
  }

  public double getQAppTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ? poseFrames[poseFrames.length - 1].appTimestamp() : 0.0;
  }

  public double getLastFreshFrameAgeSec() {
    if (!Double.isFinite(lastFreshFrameFpgaTs)) {
      return Double.POSITIVE_INFINITY;
    }

    return Timer.getFPGATimestamp() - lastFreshFrameFpgaTs;
  }

  public boolean isTracking() {
    try {
      return EnabledSubsystems.questnav && questNav.isTracking();
    } catch (Throwable t) {
      return false;
    }
  }

  public boolean hasFreshTracking() {
    return isTracking()
        && poseFrames != null
        && poseFrames.length > 0
        && getLastFreshFrameAgeSec() <= OdometryConstants.QUEST_STALE_TIMEOUT_SEC;
  }

  public void resetQuestIMUToAngle(double angleDeg) {
    Pose2d currentRobotPose = getQuestRobotPose2d();
    if (!isReasonableQuestRobotPose(currentRobotPose)) {
      return;
    }

    Pose2d newRobotPose = new Pose2d(currentRobotPose.getTranslation(), Rotation2d.fromDegrees(angleDeg));
    questNav.setPose(new Pose3d(newRobotPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST)));
  }

  public void resetQuestOdometry(Pose3d robotPose) {
    questNav.setPose(robotPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D));
  }

  public PoseFrame[] getAllCurrentPoseframes() {
    return poseFrames == null ? new PoseFrame[0] : poseFrames.clone();
  }

  private boolean isReasonableQuestRobotPose(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians())
        && Math.abs(pose.getX()) < OdometryConstants.MAX_REASONABLE_FIELD_COORD_ABS_METERS
        && Math.abs(pose.getY()) < OdometryConstants.MAX_REASONABLE_FIELD_COORD_ABS_METERS
        && !pose.equals(QuestNavConstants.NULL_POSE);
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.questnav) {
      return;
    }

    questNav.commandPeriodic();

    PoseFrame[] unreadPoseFrames = questNav.getAllUnreadPoseFrames();
    if (unreadPoseFrames != null && unreadPoseFrames.length > 0) {
      poseFrames = unreadPoseFrames;
      lastFreshFrameFpgaTs = Timer.getFPGATimestamp();
    }

    if (!DebugTelemetrySubsystems.questnav) {
      return;
    }

    telemetryLoopCounter++;
    if (telemetryLoopCounter < 5) {
      return;
    }
    telemetryLoopCounter = 0;

    SmartDashboard.putBoolean("QuestNav/Tracking", isTracking());
    SmartDashboard.putBoolean("QuestNav/FreshTracking", hasFreshTracking());
    SmartDashboard.putNumber("QuestNav/LastFreshFrameAgeSec", getLastFreshFrameAgeSec());
    SmartDashboard.putString("QuestNav/RobotPose/Translation", getQuestRobotPose3d().getTranslation().toString());
    SmartDashboard.putNumber("QuestNav/RobotPose/YawDeg", getQuestRobotYaw());
    SmartDashboard.putString("QuestNav/QuestPose/Translation", getQuestPose3d().getTranslation().toString());
    SmartDashboard.putNumber("QuestNav/Timestamp/DataSec", getQTimeStamp());
    SmartDashboard.putNumber("QuestNav/Timestamp/AppSec", getQAppTimeStamp());
    SmartDashboard.putNumber("QuestNav/FramesCount", poseFrames != null ? poseFrames.length : 0);
  }
}
