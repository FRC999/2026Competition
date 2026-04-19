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
  private static final int INITIAL_SEED_MT1_MIN_TAGS = 2;
  private static final double ORIENTATION_UPDATE_MIN_INTERVAL_SEC = 0.05;
  private static final double ORIENTATION_UPDATE_YAW_DELTA_DEG = 0.5;
  private static final double ORIENTATION_UPDATE_YAW_RATE_DELTA_DEG_PER_SEC = 2.0;
  private static final int PERF_PUBLISH_EVERY_LOOPS = 25;
  private static final LLCamera[] APRILTAG_CAMERAS = LLCamera.values();
  public static AprilTagFieldLayout fieldLayout;
  
  private int currentIMUMode = Integer.MIN_VALUE;
  private double lastOrientationYawDeg = Double.NaN;
  private double lastOrientationYawRateDegPerSec = Double.NaN;
  private double lastOrientationUpdateTs = Double.NEGATIVE_INFINITY;
  private long bestPoseRuntimeAccumNs = 0L;
  private long bestPoseRuntimeMaxNs = 0L;
  private int bestPoseRuntimeSamples = 0;

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

  public void ensureIMUMode(int mode) {
    if (currentIMUMode == mode) {
      return;
    }

    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      String cameraName = llcamera.getCameraName();
      LimelightHelpers.SetIMUMode(cameraName, mode);
      if (mode == LLAprilTagConstants.LLVisionConstants.LL_IMU_MODE_TRACKING_MT1_ASSIST) {
        LimelightHelpers.setLimelightNTDouble(
            cameraName,
            "imuassistalpha_set",
            LLAprilTagConstants.LLVisionConstants.LL_IMU_ASSIST_ALPHA);
      }
    }
    currentIMUMode = mode;
  }

  private boolean hasValidPoseEstimate(PoseEstimate pe) {
    return pe != null && pe.tagCount > 0 && pe.rawFiducials != null && pe.rawFiducials.length > 0;
  }

  private PoseEstimate getMegaTag1PoseEstimate(String cameraName) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    return hasValidPoseEstimate(poseEstimate) ? poseEstimate : null;
  }

  private PoseEstimate getMegaTag2PoseEstimate(String cameraName) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    return hasValidPoseEstimate(poseEstimate) ? poseEstimate : null;
  }

  public boolean hasReliableMultiTagMegaTag1Observation() {
    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      PoseEstimate megaTag1 = getMegaTag1PoseEstimate(llcamera.getCameraName());
      if (megaTag1 != null && megaTag1.tagCount >= INITIAL_SEED_MT1_MIN_TAGS) {
        return true;
      }
    }
    return false;
  }

  /**
   * return PoseEstimate from a given camera or NULL if nothing is visible
   * @param cn - camera name
   * @return
   */
  public LimelightHelpers.PoseEstimate getPoseEstimateFromLL(String cn) {
    LimelightHelpers.PoseEstimate megaTag1 = getMegaTag1PoseEstimate(cn);
    LimelightHelpers.PoseEstimate megaTag2 = getMegaTag2PoseEstimate(cn);

    boolean mt1Valid = megaTag1 != null;
    boolean mt2Valid = megaTag2 != null;

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

  public LimelightHelpers.PoseEstimate getInitialSeedPoseEstimateFromAllLL(boolean allowMegaTag2Fallback) {
    long startNs = DebugTelemetrySubsystems.perfLight ? System.nanoTime() : 0L;
    PoseEstimate bestMegaTag1MultiTag = null;
    PoseEstimate bestMegaTag2 = null;
    String bestMegaTag1Camera = null;
    String bestMegaTag2Camera = null;
    Comparator<LimelightHelpers.PoseEstimate> poseComparator =
        Comparator.comparingInt((LimelightHelpers.PoseEstimate poseEstimate) -> poseEstimate.tagCount)
            .thenComparingDouble(this::getPoseRankingScore);

    for (LLCamera llcamera : APRILTAG_CAMERAS) {
      String cameraName = llcamera.getCameraName();
      PoseEstimate megaTag1 = getMegaTag1PoseEstimate(cameraName);
      PoseEstimate megaTag2 = getMegaTag2PoseEstimate(cameraName);

      if (megaTag1 != null
          && megaTag1.tagCount >= INITIAL_SEED_MT1_MIN_TAGS
          && (bestMegaTag1MultiTag == null || poseComparator.compare(megaTag1, bestMegaTag1MultiTag) > 0)) {
        bestMegaTag1MultiTag = megaTag1;
        bestMegaTag1Camera = cameraName;
      }

      if (megaTag2 != null
          && (bestMegaTag2 == null || poseComparator.compare(megaTag2, bestMegaTag2) > 0)) {
        bestMegaTag2 = megaTag2;
        bestMegaTag2Camera = cameraName;
      }
    }

    PoseEstimate selectedPose =
        bestMegaTag1MultiTag != null ? bestMegaTag1MultiTag : (allowMegaTag2Fallback ? bestMegaTag2 : null);
    String selectedCamera =
        bestMegaTag1MultiTag != null ? bestMegaTag1Camera : (allowMegaTag2Fallback ? bestMegaTag2Camera : null);
    boolean selectedMegaTag1 = bestMegaTag1MultiTag != null;

    lastBestPoseUsedMegaTag1 = selectedPose != null && selectedMegaTag1;
    lastBestPoseCameraName = selectedCamera;

    if (DebugTelemetrySubsystems.llLight) {
      SmartDashboard.putString(
          "Vision/SeedPose/Source",
          selectedPose == null
              ? (allowMegaTag2Fallback ? "NONE" : "WAITING_FOR_MT1_MULTI_TAG")
              : (selectedMegaTag1 ? "MT1_MULTI_TAG" : "MT2_FALLBACK"));
      SmartDashboard.putString("Vision/SeedPose/Camera", selectedCamera != null ? selectedCamera : "");
      SmartDashboard.putNumber("Vision/SeedPose/TagCount", selectedPose != null ? selectedPose.tagCount : 0);
    }

    recordBestPoseRuntime(DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
    return selectedPose;
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

  private void recordBestPoseRuntime(long elapsedNs) {
    if (!DebugTelemetrySubsystems.perfLight) {
      return;
    }

    bestPoseRuntimeAccumNs += elapsedNs;
    bestPoseRuntimeMaxNs = Math.max(bestPoseRuntimeMaxNs, elapsedNs);
    bestPoseRuntimeSamples++;

    if (bestPoseRuntimeSamples >= PERF_PUBLISH_EVERY_LOOPS) {
      SmartDashboard.putNumber(
          "Perf/LL/BestPoseMsAvg",
          bestPoseRuntimeAccumNs / 1_000_000.0 / bestPoseRuntimeSamples);
      SmartDashboard.putNumber("Perf/LL/BestPoseMsMax", bestPoseRuntimeMaxNs / 1_000_000.0);
      bestPoseRuntimeAccumNs = 0L;
      bestPoseRuntimeMaxNs = 0L;
      bestPoseRuntimeSamples = 0;
    }
  }

  public LimelightHelpers.PoseEstimate getBestPoseEstimateFromAllLL() {
    long startNs = DebugTelemetrySubsystems.perfLight ? System.nanoTime() : 0L;
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
      recordBestPoseRuntime(DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
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

    recordBestPoseRuntime(DebugTelemetrySubsystems.perfLight ? System.nanoTime() - startNs : 0L);
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

    if (DebugTelemetrySubsystems.ll) {
      SmartDashboard.putString("Vision/LL4/VisibleColors", ElasticHelpers.LLAnyVisibleColors(isAprilTagVisibleAny()));
    }

  }
}


