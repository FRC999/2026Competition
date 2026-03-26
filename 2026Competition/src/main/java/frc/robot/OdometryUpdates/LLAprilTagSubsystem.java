// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OdometryUpdates;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.PoseEstimate;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants.LLCamera;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.lib.VisionHelpers;
import frc.robot.lib.ElasticHelpers;
import java.util.Comparator;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLAprilTagSubsystem extends SubsystemBase {
  private static final double ORIENTATION_UPDATE_MIN_INTERVAL_SEC = 0.05;
  private static final double ORIENTATION_UPDATE_YAW_DELTA_DEG = 0.5;
  private static final double ORIENTATION_UPDATE_YAW_RATE_DELTA_DEG_PER_SEC = 2.0;
  private static final LLCamera[] APRILTAG_CAMERAS = LLCamera.values();
  public static AprilTagFieldLayout fieldLayout;
  
  private boolean imuModeSet = false;
  private double lastOrientationYawDeg = Double.NaN;
  private double lastOrientationYawRateDegPerSec = Double.NaN;
  private double lastOrientationUpdateTs = Double.NEGATIVE_INFINITY;

  private double maxBestAmbiguity = 0.5; // Puts pretty high standard on AprilTag position determination
  private boolean lastPoseEstimateUsedMegaTag1 = false;
  private boolean lastBestPoseUsedMegaTag1 = false;
  private String lastBestPoseCameraName = null;

  Map<Pose2d, Integer> allianceTagPoses;

  /** Creates a new LLVisionSubsystem. */
  public LLAprilTagSubsystem() {

    if(!EnabledSubsystems.ll){
      return;
    }
    fieldLayout = AprilTagFieldLayout.loadField(LLAprilTagConstants.LLVisionConstants.FIELD_LAYOUT);

  }

  public Pose2d getRobotAprilTagPose() {
    return null;
  }

  public Pose2d getKnownPose(String poseName) {
    //System.out.println(RobotPoseConstants.visionRobotPoses.keySet());
    if(RobotPoseConstants.visionRobotPoses.containsKey(poseName)){
      return RobotPoseConstants.visionRobotPoses.get(poseName);
    } else {
      return null; 
    }
  }

  public boolean isAprilTagVisible(String cameraName) {
    return LimelightHelpers.getTV(cameraName); 
  }

  public boolean isAprilTagVisibleAny() {
    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      if (isAprilTagVisible(llcamera.getCameraName())) {
        return true;
      }
    }
    return false;
  }

  public boolean isRedReefTagID(int tag) {
    return ( tag>=6 && tag <=11);
  }
  public boolean isBlueReefTagID(int tag) {
    return ( tag>=17 && tag <=22);
  }
  public boolean isAnyReefTagID(int tag) {
    return isRedReefTagID(tag) || isBlueReefTagID(tag);
  }



  public double getClosestTag(RawFiducial[] rf) { // Closest tag to camera
    double ldr = Double.MAX_VALUE; //lowest distance to robot;
    int ldrid = 0; // id of the closest target
    for (RawFiducial irf : rf) {
      if(irf.distToRobot<ldr) {
        ldr = irf.distToRobot;
        ldrid = irf.id ;
      }
    }
    return ldrid;
  }

  /**
   * Return pose of the alliance apriltag with the IMU closest to the current robot IMU.
   * So, if the bot is oriented generally in the same direction as the tag, that will be the tag returned
   * @return
   */
  public Pose2d getTagPerAllianceAndIMU() {
    Pose2d nearest = new Pose2d();
    double rotationdiff = 180;
    for (Pose2d tagPose : allianceTagPoses.keySet()) {
      double d = Math.abs( RobotContainer.driveSubsystem.getYaw() - tagPose.getRotation().getDegrees() );
      if( d < rotationdiff ) {
        rotationdiff = d;
        nearest = tagPose;
      }
    }
    return nearest;
  }

  public LLCamera[] getListOfApriltagLLCameras() {
    return APRILTAG_CAMERAS;
  }

  public void setLLOrientation(double yaw, double yawrate){
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    boolean shouldUpdate =
        !Double.isFinite(lastOrientationYawDeg)
            || Math.abs(yaw - lastOrientationYawDeg) >= ORIENTATION_UPDATE_YAW_DELTA_DEG
            || Math.abs(yawrate - lastOrientationYawRateDegPerSec)
                >= ORIENTATION_UPDATE_YAW_RATE_DELTA_DEG_PER_SEC
            || now - lastOrientationUpdateTs >= ORIENTATION_UPDATE_MIN_INTERVAL_SEC;

    if (!shouldUpdate) {
      return;
    }

    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      LimelightHelpers.SetRobotOrientation_NoFlush(llcamera.getCameraName(), yaw, yawrate, 0, 0, 0, 0);
    }
    lastOrientationYawDeg = yaw;
    lastOrientationYawRateDegPerSec = yawrate;
    lastOrientationUpdateTs = now;
  }

  /**
   * return PoseEstimate from a given camera or NULL if nothing is visible
   * @param cn - camera name
   * @return
   */
  public LimelightHelpers.PoseEstimate getPoseEstimateFromLL(String cn) {
    LimelightHelpers.PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cn);

    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cn);

    boolean mt1Valid = megaTag1 != null && megaTag1.tagCount > 0;
    boolean mt2Valid = megaTag2 != null && megaTag2.tagCount > 0;

    if (!mt1Valid && !mt2Valid) {
      lastPoseEstimateUsedMegaTag1 = false;
      return null;
    }

    if (!mt2Valid) {
      lastPoseEstimateUsedMegaTag1 = true;
      return megaTag1;
    }

    if (!mt1Valid) {
      lastPoseEstimateUsedMegaTag1 = false;
      return megaTag2;
    }

    double imuYawDiffDeg =
        Math.abs(megaTag1.pose.getRotation().minus(megaTag2.pose.getRotation()).getDegrees());

    if (imuYawDiffDeg > 180.0) {
      imuYawDiffDeg = 360.0 - imuYawDiffDeg;
    }

    if (DebugTelemetrySubsystems.ll) {
      SmartDashboard.putNumber("Vision/" + cn + "/MegaTagYawDiffDeg", imuYawDiffDeg);
    }

    if (imuYawDiffDeg > 90.0) {
      if (DebugTelemetrySubsystems.ll) {
        SmartDashboard.putBoolean("Vision/" + cn + "/UsingMegaTag1YawFailsafe", true);
        SmartDashboard.putString("LL MegaTag1: ", megaTag1.pose.toString());
      }
      lastPoseEstimateUsedMegaTag1 = true;
      return megaTag1;
    }

    if (DebugTelemetrySubsystems.ll) {
      SmartDashboard.putBoolean("Vision/" + cn + "/UsingMegaTag1YawFailsafe", false);
      SmartDashboard.putString("LL MegaTag2: ", megaTag2.pose.toString());
    }
    lastPoseEstimateUsedMegaTag1 = false;
    return megaTag2;
  }

  private double getPoseRankingScore(LimelightHelpers.PoseEstimate pe) {
    if (pe == null || pe.rawFiducials == null || pe.rawFiducials.length == 0) {
      return Double.NEGATIVE_INFINITY;
    }

    double ambiguity = pe.rawFiducials[0].ambiguity;
    double avgTagDistance = pe.avgTagDist;
    double latencyPenalty = pe.latency;

    return pe.tagCount * 1000.0
        - ambiguity * 100.0
        - avgTagDistance * 10.0
        - latencyPenalty;
  }

  public LimelightHelpers.PoseEstimate getBestPoseEstimateFromAllLL() {
    LimelightHelpers.PoseEstimate bestPose = null;
    double bestAmbiguity = 99;
    boolean bestPoseUsedMegaTag1 = false;
    String bestCameraName = null;
    Comparator<LimelightHelpers.PoseEstimate> poseComparator =
        Comparator.comparingInt((LimelightHelpers.PoseEstimate poseEstimate) -> poseEstimate.tagCount)
            .thenComparingDouble(this::getPoseRankingScore);

    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      String cn = llcamera.getCameraName();
      LimelightHelpers.PoseEstimate pe = getPoseEstimateFromLL(cn);

      if (pe != null) {
        VisionHelpers.updateLLTelemetry(pe, cn);

        if (pe.rawFiducials != null
            && pe.rawFiducials.length > 0
            && (bestPose == null || poseComparator.compare(pe, bestPose) > 0)) {
          bestAmbiguity = pe.rawFiducials[0].ambiguity;
          bestPose = pe;
          bestPoseUsedMegaTag1 = lastPoseEstimateUsedMegaTag1;
          bestCameraName = cn;
        }
      } else {
        VisionHelpers.clearLLTelemetry(cn);
      }
    }

    if (bestAmbiguity > maxBestAmbiguity) {
      lastBestPoseUsedMegaTag1 = false;
      lastBestPoseCameraName = null;
      if (DebugTelemetrySubsystems.llLight) {
        SmartDashboard.putString("Vision/BestPose/Camera", "");
        SmartDashboard.putNumber("Vision/BestPose/TagCount", 0);
        SmartDashboard.putNumber("Vision/BestPose/Ambiguity", 10.0);
        SmartDashboard.putNumber("Vision/BestPose/AvgTagDist", 0.0);
        SmartDashboard.putNumber("Vision/BestPose/LatencySec", 0.0);
        SmartDashboard.putBoolean("Vision/BestPose/UsedMegaTag1", false);
      }
      return null;
    }

    lastBestPoseUsedMegaTag1 = bestPoseUsedMegaTag1;
    lastBestPoseCameraName = bestCameraName;
    if (DebugTelemetrySubsystems.llLight) {
      SmartDashboard.putString("Vision/BestPose/Camera", bestCameraName != null ? bestCameraName : "");
      SmartDashboard.putNumber("Vision/BestPose/TagCount", bestPose != null ? bestPose.tagCount : 0);
      SmartDashboard.putNumber(
          "Vision/BestPose/Ambiguity",
          bestPose != null && bestPose.rawFiducials != null && bestPose.rawFiducials.length > 0
              ? bestPose.rawFiducials[0].ambiguity
              : 10.0);
      SmartDashboard.putNumber("Vision/BestPose/AvgTagDist", bestPose != null ? bestPose.avgTagDist : 0.0);
      SmartDashboard.putNumber("Vision/BestPose/LatencySec", bestPose != null ? bestPose.latency : 0.0);
      SmartDashboard.putBoolean("Vision/BestPose/UsedMegaTag1", lastBestPoseUsedMegaTag1);
    }

    if (DebugTelemetrySubsystems.ll) {
      if (bestPose != null) {
        SmartDashboard.putNumber("Vision/BestPoseEst/TagCount", bestPose.tagCount);
        SmartDashboard.putNumber("Vision/BestPoseEst/Ambiguity", bestPose.rawFiducials[0].ambiguity);
      } else {
        SmartDashboard.putNumber("Vision/BestPoseEst/TagCount", 0);
        SmartDashboard.putNumber("Vision/BestPoseEst/Ambiguity", 10);
      }
    }

    return bestPose;
  }

  public boolean wasLastBestPoseMegaTag1() {
    return lastBestPoseUsedMegaTag1;
  }

  public String getLastBestPoseCameraName() {
    return lastBestPoseCameraName;
  }

  @Override
  public void periodic() {
    
    if (!EnabledSubsystems.ll) {
      return;
    }

    // One-time IMU mode set: 1 = mirror external yaw into LL IMU (keeps MT2/IMU consistent).
    if (!imuModeSet) {
      for (LLCamera llcamera : APRILTAG_CAMERAS) {
        LimelightHelpers.SetIMUMode(llcamera.getCameraName(),  LLAprilTagConstants.LLVisionConstants.LL_IMU_MODE);
      }
      imuModeSet = true;
    }

    if (DebugTelemetrySubsystems.ll) {
      SmartDashboard.putString("Vision/LL4/VisibleColors", ElasticHelpers.LLAnyVisibleColors(isAprilTagVisibleAny()));
    }

  }
}


