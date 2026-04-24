package frc.robot.OdometryUpdates;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.QuestHelpers;
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
  private int characterizationCounter = 0;
  private Pose2d characterizationStartPose = QuestNavConstants.NULL_POSE;
  private double savedQuestAngleDeg = 0.0;

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

  public double getQuestYaw() {
    return getQuestPose2d().getRotation().getDegrees();
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

  /**
   * Rotate in place, sample raw Quest X/Y, and print the fitted circle center.
   * This is a manual calibration helper only and does not run unless explicitly scheduled.
   */
  public Command offsetTranslationCharacterizationCommand() {
    final double rotationalSpeed = SwerveConstants.MaxAngularRate / 6.0;
    final ArrayList<Double[]> questPoses = new ArrayList<>();
    final int everyN = 10;

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              questPoses.clear();
              characterizationCounter = 0;
              RobotContainer.driveSubsystem.drive(0.0, 0.0, rotationalSpeed);
            },
            RobotContainer.driveSubsystem),
        Commands.run(
            () -> {
              if (characterizationCounter++ % everyN == 0) {
                Pose2d pose = getQuestPose2d();
                if (!pose.equals(QuestNavConstants.NULL_POSE)) {
                  questPoses.add(new Double[] { pose.getX(), pose.getY() });
                }
              }
            },
            this).finallyDo(interrupted -> {
              try {
                var center = QuestHelpers.estimateCircleCenter(questPoses);
                System.out.println(
                    "Quest translation characterization | CenterX: " + center.getX()
                        + " CenterY: " + center.getY()
                        + " OffsetDeltaX: " + (center.getX() - QuestNavConstants.ROBOT_TO_QUEST.getX())
                        + " OffsetDeltaY: " + (center.getY() - QuestNavConstants.ROBOT_TO_QUEST.getY())
                        + " Samples: " + questPoses.size());
              } catch (Exception e) {
                System.out.println("Quest translation characterization failed: " + e.getMessage());
              }
              RobotContainer.driveSubsystem.drive(0.0, 0.0, 0.0);
            }))
        .until(() -> false);
  }

  /**
   * Drive forward after resetting Quest pose and print the measured raw Quest heading line.
   * This is a manual calibration helper only and does not run unless explicitly scheduled.
   */
  public Command offsetAngleCharacterizationCommand() {
    final double driveSpeed = SwerveConstants.MaxSpeed / 6.0;

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              characterizationCounter = 0;
              characterizationStartPose = QuestNavConstants.NULL_POSE;
              savedQuestAngleDeg = getQuestYaw();
              questNav.setPose(new Pose3d(new Pose2d()));
            },
            RobotContainer.driveSubsystem),
        Commands.run(
            () -> {
              if (characterizationCounter++ == 10) {
                characterizationStartPose = getQuestPose2d();
                RobotContainer.driveSubsystem.drive(driveSpeed, 0.0, 0.0);
              }
            },
            this).finallyDo(interrupted -> {
              try {
                Pose2d endPose = getQuestPose2d();
                double dx = endPose.getX() - characterizationStartPose.getX();
                double dy = endPose.getY() - characterizationStartPose.getY();
                double angleRad = Math.atan2(dy, dx);
                System.out.println(
                    "Quest angle characterization | Start: " + characterizationStartPose
                        + " End: " + endPose
                        + " HeadingRad: " + angleRad
                        + " HeadingDeg: " + Math.toDegrees(angleRad));
              } catch (Exception e) {
                System.out.println("Quest angle characterization failed: " + e.getMessage());
              }
              RobotContainer.driveSubsystem.drive(0.0, 0.0, 0.0);
              questNav.setPose(new Pose3d(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(savedQuestAngleDeg))));
            }))
        .until(() -> false);
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

    SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav/Tracking", isTracking());
    SmartDashboard.putBoolean("QuestNav/FreshTracking", hasFreshTracking());
    SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
    questNav.getBatteryPercent().ifPresent(
        batteryPercent -> SmartDashboard.putNumber("QuestNav/Battery%", batteryPercent));
    questNav.getTrackingLostCounter().ifPresent(
        trackingLostCount -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", trackingLostCount));
    SmartDashboard.putNumber("QuestNav/LastFreshFrameAgeSec", getLastFreshFrameAgeSec());
    SmartDashboard.putString("QuestNav/RobotPose/Translation", getQuestRobotPose3d().getTranslation().toString());
    SmartDashboard.putNumber("QuestNav/RobotPose/YawDeg", getQuestRobotYaw());
    SmartDashboard.putString("QuestNav/QuestPose/Translation", getQuestPose3d().getTranslation().toString());
    SmartDashboard.putNumber("QuestNav/QuestPose/YawDeg", getQuestYaw());
    SmartDashboard.putNumber("QuestNav/Timestamp/DataSec", getQTimeStamp());
    SmartDashboard.putNumber("QuestNav/Timestamp/AppSec", getQAppTimeStamp());
    SmartDashboard.putNumber("QuestNav/FramesCount", poseFrames != null ? poseFrames.length : 0);
  }
}
