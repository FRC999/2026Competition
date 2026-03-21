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
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLAprilTagSubsystem extends SubsystemBase {
  public static AprilTagFieldLayout fieldLayout;
  
  private boolean imuModeSet = false;

  private double maxBestAmbiguity = 0.5; // Puts pretty high standard on AprilTag position determination
    private boolean lastPoseEstimateUsedMegaTag1 = false;
  private boolean lastBestPoseUsedMegaTag1 = false;

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
    String[] cns = {"limelight-fl", "limelight-fr", "limelight-l"};
    int counter = 0;
    for(int i = 0; i<=2; i++){
      if(isAprilTagVisible(cns[i])){
        counter++;
      }
    }
    if(counter>=1){
      return true;
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
    return LLCamera.values();
  }

  public void setLLOrientation(double yaw, double yawrate){
    for (LLCamera llcamera : LLCamera.values()) {
      LimelightHelpers.SetRobotOrientation(llcamera.getCameraName(),  yaw, yawrate,0,0,0,0);
    }
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

    SmartDashboard.putNumber("Vision/" + cn + "/MegaTagYawDiffDeg", imuYawDiffDeg);

    if (imuYawDiffDeg > 90.0) {
      SmartDashboard.putBoolean("Vision/" + cn + "/UsingMegaTag1YawFailsafe", true);
      SmartDashboard.putString("LL MegaTag1: ", megaTag1.pose.toString());
      lastPoseEstimateUsedMegaTag1 = true;
      return megaTag1;
    }

    SmartDashboard.putBoolean("Vision/" + cn + "/UsingMegaTag1YawFailsafe", false);
    SmartDashboard.putString("LL MegaTag2: ", megaTag2.pose.toString());
    lastPoseEstimateUsedMegaTag1 = false;
    return megaTag2;
  }

   public LimelightHelpers.PoseEstimate getBestPoseEstimateFromAllLL() {
    LimelightHelpers.PoseEstimate bestPose = null;
    double bestAmbiguity = 99;
    boolean bestPoseUsedMegaTag1 = false;

    for (LLCamera llcamera : LLCamera.values()) {
      String cn = llcamera.getCameraName();
      LimelightHelpers.PoseEstimate pe = getPoseEstimateFromLL(cn);

      if (pe != null) {
        VisionHelpers.updateLLTelemetry(pe, cn);

        if (pe.rawFiducials[0].ambiguity < bestAmbiguity) {
          bestAmbiguity = pe.rawFiducials[0].ambiguity;
          bestPose = pe;
          bestPoseUsedMegaTag1 = lastPoseEstimateUsedMegaTag1;
        }
      } else {
        VisionHelpers.clearLLTelemetry(cn);
      }
    }

    if (bestAmbiguity > maxBestAmbiguity) {
      lastBestPoseUsedMegaTag1 = false;
      return null;
    }

    lastBestPoseUsedMegaTag1 = bestPoseUsedMegaTag1;
    SmartDashboard.putBoolean("Vision/BestPoseUsedMegaTag1", lastBestPoseUsedMegaTag1);

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

  @Override
  public void periodic() {
    
    if (!EnabledSubsystems.ll) {
  return;
    }

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;

    // One-time IMU mode set: 1 = mirror external yaw into LL IMU (keeps MT2/IMU consistent).
    if (!imuModeSet) {
      for (LLCamera llcamera : LLCamera.values()) {
        LimelightHelpers.SetIMUMode(llcamera.getCameraName(),  LLAprilTagConstants.LLVisionConstants.LL_IMU_MODE);
      }
      imuModeSet = true;
    }

    if (DebugTelemetrySubsystems.ll) {
      SmartDashboard.putString("Vision/LL4/VisibleColors", ElasticHelpers.LLAnyVisibleColors(isAprilTagVisibleAny()));
    }

  }
}


