// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants.LLCamera;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.QuestHelpers;
import frc.robot.lib.VisionHelpers;
import gg.questnav.questnav.PoseFrame;

/**
 * This class is rewritten with Finite State Machine logic (FMS)
 */
public class OdometryUpdatesSubsystem extends SubsystemBase {

  /**
   * LL YAW needs to set as precise as possible before LL can tell you the Pose2D of the bot based on AT.
   * That Yaw is necessary for disambiguation. However, if you rotate the bot after you turn it on, but before
   * it sees any AT, its internal IMU (Pigeon) may not be good enough to track the move.
   * So, the IMU updates necessary for LL disambiguation will be coming from Quest rather that the robot odometry
   * until the SEEDING is done.
   * 
   * The robot posing on the field will be - put it against the game element with exact known angle (e.g reef in 2025)
   * Make sure that angle is known in OdometryConstants. Once the bot is UP, put it to whatever position you need in a starting area,
   * so it sees ATs.
   * Once it sees the ATs, the LL Yaw disambiguation updates will come from the robot odometry.
   * 
   * The robot YAW will be initially set to whatever value is indicated in the OdometryConstants. So, if the
   * ATs do not become visible at the start of auto, or Quest did not come on, the Yaw currently in the odometry will be used. 
   * 
   * FMS states:
   * INITIALIZE - initial location on the field, against a game element that has knowb Rotation
   * 
   * -> SEEKING_TAGS_Q - quest tracking is detected
   *      Set Quest IMU to known initial Rotation
   * -> SEEKING_TAGS_NO_Q - quest tracking is not detected for N iterations
   * 
   * all cases - set LL IMU to the initial known rotation
   * 
   * SEEKING_TAGS_Q - looking for AT
   * 
   * -> SEEKING_TAGS_NO_Q - quest suddenly stopped working
   *    LL IMU is updated by odometry
   * 
   * -> CALIBRATED_Q - saw AT, determined my Pose2d
   *    Calibrate Quest based on the Pose2D gathered from the camera
   *    set robot odometry based on pose detected
   *    LL IMU is updated by Quest
   * 
   *  all non-transition cases - LL IMU is updated by Quest
   *
   * SEEKING_TAGS_NO_Q -  looking for AT
   *   
   * -> CALIBRATED_NO_Q - saw AT
   *    set robot odometry based on pose detected
   * 
   * -> SEEKING_TAGS_Q - quest suddently came up
   *    Set Quest IMU to current robot yaw
   *    LL IMU is updated by current robot yaw since we set Quest to the same value
   * 
   * CALIBRATED_Q - update odometry with both Q and LL
   * 
   *   -> CALIBRATED_NO_Q - quest stopped working
   *     LL IMU is updated by odometry
   * 
   * CALIBRATED_NO_Q - update odometry with LL only
   * 
   *   -> CALIBRATED_Q - quest suddently came up
   *     Set Quest IMU to current robot yaw
   * 
   */
  /* ========= State machine for initial alignment ========= */
  private enum VisionState {
    INITIALIZE, SEEKING_TAGS_Q, SEEKING_TAGS_NO_Q, CALIBRATED_Q, CALIBRATED_NO_Q
  }

  private VisionState state = VisionState.INITIALIZE;
  private int forceTransitionFromInitialize = 50*10; // Cycles to wait in initialize if quest is not on in beginning

  public boolean gatePassOverride;

  private VisionState prevState = VisionState.INITIALIZE;
  private String lastTransition = "START";
  private double lastTransitionTime = 0.0;
  private int transitionSeq = 0;
  private Timer llTimer = new Timer();

    private final Timer questLossHoldTimer = new Timer();
  private boolean hasQuestEverBeenFieldCalibrated = false;
    private boolean allowStartupFallbackAnchor = true;
  

  /** Creates a new OdometryUpdatesSubsystem. */
  public OdometryUpdatesSubsystem() {
    if(!EnabledSubsystems.odometry){
      return;
    }

    gatePassOverride = true;
    
    // The chassis yaw should be initialized regardless of quest or LL being operational
    // TODO: add LED lights and turn it on another color when zero chassis runs
    RobotContainer.driveSubsystem.zeroChassisYaw();
  }

  //Update odometry using Quest
  private void fuseQuestNavAllUnread() {
   // System.out.println(Timer.getFPGATimestamp());
    PoseFrame[] frames = RobotContainer.questNavSubsystem.getAllCurrentPoseframes();
   // System.out.println(Timer.getFPGATimestamp());
    if (frames == null || frames.length == 0) return;
   // System.out.println(Timer.getFPGATimestamp());
    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;


    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;
    boolean gatePassOverrideIntermediate = gatePassOverride;
    for (PoseFrame pf : frames) {
      if (pf == null) continue;

      // Docs: questPose() is the Quest’s Pose2d; convert to robot center with ROBOT_TO_QUEST
      Pose2d questPose = pf.questPose3d().toPose2d();                         // :contentReference[oaicite:2]{index=2}
      if (questPose == null) continue;
      Pose2d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse()); // :contentReference[oaicite:3]{index=3}
      if (!isReasonablePose(robotPose)) continue;

      // Docs: use the data timestamp directly
      Double tMeas = pf.dataTimestamp();                         // seconds (FPGA time) :contentReference[oaicite:4]{index=4}
      double t = (tMeas != null && tMeas > 1.0) ? tMeas : Timer.getFPGATimestamp();

      if (gateMeasurement(robotPose, t, /*strict*/ false, speedNow, poseNow)) {
        Matrix<N3, N1> std = QuestHelpers.questStdDev(speedNow);
        RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, t, QuestNavConstants.QUESTNAV_STD_DEVS);
        //System.out.println("T");
        //System.out.println(Timer.getFPGATimestamp());
        gatePassOverrideIntermediate = false;
        // System.out.println("TEST");
      } else {
        if (DebugTelemetrySubsystems.questnav) {
        //  SmartDashboard.putString("Odometry/QuestNavRejectedPose", robotPose.toString());
        //  SmartDashboard.putNumber("Odometry/QuestNavRejectedTransErr", robotPose.getTranslation().getDistance(poseNow.getTranslation()));
        }
      }
    }
    gatePassOverride = gatePassOverrideIntermediate;
  }

  /** Innovation gating with latency compensation via buffered prediction. */
  private boolean gateMeasurement(Pose2d robotPose, double timestamp, boolean strict, double speedNow, Pose2d poseNow) { 
    if (gatePassOverride) {
      return true;
    }
    //Either pose from driveSubsystem at timestamp or current pose of bot
    Pose2d chassisPoseAtTimestamp = RobotContainer.driveSubsystem.getSample(timestamp).orElse(poseNow);

    var twist = chassisPoseAtTimestamp.minus(robotPose); // log(SE2)
    double transErr = Math.hypot(twist.getX(), twist.getY());
    double rotErr = Math.abs(twist.getRotation().getRadians());

    //Tolerances for translation and rotation that determine validity of vision Pose
    final double transGate = (strict ? 0.6 : 0.8) + 0.5 * speedNow;  // meters
    final double rotGate   = Units.degreesToRadians(strict ? 15.0 : 20.0);

    return transErr <= transGate && rotErr <= rotGate;
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
        && !pose.equals(QuestNavConstants.nullPose);
  }

  private void fuseLLCamera(LLCamera llcamera) {
    String cn = llcamera.getCameraName();
    if (cn == null || cn.isBlank()) return;

    LimelightHelpers.PoseEstimate pe = RobotContainer.llAprilTagSubsystem.getPoseEstimateFromLL(cn);

    if (pe == null) {
      VisionHelpers.clearLLTelemetry(cn);
      return;
    } // Already get NULL if pe.tagCount == 0

    double ambiguity = MathUtil.clamp(pe.rawFiducials[0].ambiguity, 0.0, 1.0); // if your Helpers expose it; else set from MT2 metrics
    
    if ((pe.tagCount == 1 && ambiguity > LLVisionConstants.kMaxSingleTagAmbiguity) 
        || (pe.rawFiducials[0].distToCamera > LLVisionConstants.kMaxCameraToTargetDistance)) 
      {
        VisionHelpers.clearLLTelemetry(cn);
        return;
      }

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;

    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;

    Pose2d robotPose = pe.pose; // robot-in-field (requires correct cam extrinsics in LL UI)
    double timestampLL = (pe.timestampSeconds > 1.0) ? pe.timestampSeconds
      : Timer.getFPGATimestamp() - pe.latency;


    if (!gatePassOverride && !gateMeasurement(robotPose, timestampLL, /*strict*/ true, speedNow, poseNow)) {
      VisionHelpers.clearLLTelemetry(cn);
      return;
    }

    Matrix<N3, N1> std = LimelightHelpers.llStdDev(pe.avgTagDist, pe.tagCount, ambiguity);
    RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, timestampLL, std);
    VisionHelpers.updateLLTelemetry(pe, cn);
  }

  private void calibrateQuestFromLL(Pose2d robotPose) {
    RobotContainer.questNavSubsystem.resetQuestOdometry(new Pose3d(robotPose));
  }
  
  /** Centralized state change: logs FROM→TO (+reason), time, and bumps a counter. */
  private void transitionTo(VisionState newState, String reason) {
    if (newState == state) return; // no-op
    prevState = state;
    state = newState;
    transitionSeq++;
    lastTransitionTime = Timer.getFPGATimestamp();
    lastTransition = prevState.name() + " -> " + state.name() + (reason != null && !reason.isBlank() ? " | " + reason : "");

    switch (prevState) {
      case SEEKING_TAGS_Q:
        // Task #10: when leaving the "seek tags w/ Quest" state, stop + reset so elapsed time
        // does not carry across re-entries.
        llTimer.stop();
        llTimer.reset();
        break;
      case CALIBRATED_Q:
        questLossHoldTimer.stop();
        questLossHoldTimer.reset();
        break;
      default:
        break;
    }

    switch (newState) {
      case SEEKING_TAGS_Q:
        // Task #10: on entry, always start the LL timer fresh from 0.
        llTimer.stop();
        llTimer.reset();
        llTimer.start();

        if (DebugTelemetrySubsystems.odometry) {
          System.out.println("Odometry: llTimer reset+started (enter SEEKING_TAGS_Q)");
        }
        break;
      case CALIBRATED_Q:
        questLossHoldTimer.stop();
        questLossHoldTimer.reset();
        break;
        default:
        break;
    }


    // SmartDashboard: concise and stable paths
    if(Constants.DebugTelemetrySubsystems.odometry) {
      //System.out.println("*******Inside transition to: Odometry is true");
      SmartDashboard.putString("Odometry/State", state.name());
      SmartDashboard.putString("Odometry/StateColor", ElasticHelpers.questStatesColors(state.name()));
      SmartDashboard.putString("Odometry/LastTransition", lastTransition);
      SmartDashboard.putNumber("Odometry/TransitionSeq", transitionSeq);
      SmartDashboard.putNumber("Odometry/LastTransitionTimeSec", lastTransitionTime);
    }
  }

  //Limelight Override for States
  public void updateQuestAndState(Pose2d pose2d) {
    if (state == VisionState.SEEKING_TAGS_Q) {
      calibrateQuestFromLL(pose2d);
      RobotContainer.driveSubsystem.resetCTREPose(pose2d);
      gatePassOverride = false;
      RobotContainer.questNavSubsystem.setInitialPoseSet(true);
      hasQuestEverBeenFieldCalibrated = true;
      allowStartupFallbackAnchor = false;
      

      transitionTo(VisionState.CALIBRATED_Q, "Manual override with good pose"); // SEEKING_TAGS_Q -> CALIBRATED_Q
    } else if (state == VisionState.SEEKING_TAGS_NO_Q) {
      RobotContainer.driveSubsystem.resetCTREPose(pose2d);
      gatePassOverride = false;

      transitionTo(VisionState.CALIBRATED_NO_Q, "Manual override with good pose"); // SEEKING_TAGS_NO_Q -> CALIBRATED_NO_Q
    }
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.odometry) {
      return;
    }

    // This method will be called once per scheduler run
    // System.out.println(Timer.getFPGATimestamp());

    if(RobotBase.isSimulation()){
      return;
    }

    if (DebugTelemetrySubsystems.odometry) {
      // Telemetry for the state machine
      String stateString = switch(state) {
        case INITIALIZE -> "INITIALIZE";
        case SEEKING_TAGS_Q -> "SEEKING_TAGS_Q";
        case SEEKING_TAGS_NO_Q -> "SEEKING_TAGS_NO_Q";
        case CALIBRATED_Q -> "CALIBRATED_Q";
        case CALIBRATED_NO_Q -> "CALIBRATED_NO_Q";
      };
      SmartDashboard.putString("Odometry/UpdatesState", stateString);
      SmartDashboard.putBoolean("Odometry/GatePassOverride", gatePassOverride);
      //System.out.println("Test0***");
    }

    switch (state) {
      case INITIALIZE -> {
        // If Quest is On
        if (RobotContainer.questNavSubsystem.isTracking()){
          // Set quest IMU to initial yaw based on alliance
          RobotContainer.questNavSubsystem.resetQuestIMUToAngle(OdometryConstants.initialYawForAlliance().getDegrees());
          //System.out.println("Test1***");
          transitionTo(VisionState.SEEKING_TAGS_Q, "Quest tracking true; seeded Quest IMU"); // initialize -> SEEKING_TAGS_Q
          //state = VisionState.SEEKING_TAGS_Q;
        } else { // No Quest detected at INITIALIZE
          if(forceTransitionFromInitialize-- < 0){ // Do not give up looking for Quest for some iterations in case it's late to start
          
      //System.out.println("Test2***");
          transitionTo(VisionState.SEEKING_TAGS_NO_Q, "Quest not tracking; timeout from initialize"); // initialize -> SEEKING_NO_Q
          //state = VisionState.SEEKING_TAGS_NO_Q;
          }
        }
        // Set LL IMU to the initial known pose assuming the robot is not moving
        RobotContainer.llAprilTagSubsystem.setLLOrientation(OdometryConstants.initialYawForAlliance().getDegrees(),0);

      }
      case SEEKING_TAGS_Q -> { // Quest was present before
    //System.out.println("Test3***");

        if (RobotContainer.questNavSubsystem.isTracking()){ // Quest is still working
          Pose2d rp = RobotContainer.questNavSubsystem.getQuestRobotPose2d();
          if (isReasonablePose(rp)) { // Only use valid Quest poses to seed LL orientation
            RobotContainer.llAprilTagSubsystem.setLLOrientation(
                rp.getRotation().getDegrees(),
                RobotContainer.driveSubsystem.getTurnRate());
          }

          var bestPoseEstimate = RobotContainer.llAprilTagSubsystem.getBestPoseEstimateFromAllLL();
          if (bestPoseEstimate != null) {
            // We see AT and have a good pose estimate
            calibrateQuestFromLL(bestPoseEstimate.pose);
            SmartDashboard.putString("Pose to Quest", bestPoseEstimate.pose.toString());
            RobotContainer.driveSubsystem.resetCTREPose(bestPoseEstimate.pose);
            gatePassOverride = false;
            RobotContainer.questNavSubsystem.setInitialPoseSet(true);
            hasQuestEverBeenFieldCalibrated = true;
            allowStartupFallbackAnchor = false;
    //System.out.println("Test5***");

            transitionTo(VisionState.CALIBRATED_Q, "Good LL fix; anchored field pose"); // SEEKING_TAGS_Q -> CALIBRATED_Q
            //state = VisionState.CALIBRATED_Q; // Now we're calibrated with Quest working
            return; // No need to do anything else this cycle
          }

        } else { // Quest is not working anymore, so transition to no-quest state while still seeking the tags
    //System.out.println("Test6***");

          transitionTo(VisionState.SEEKING_TAGS_NO_Q, "Quest lost during seeking"); // SEEKING_TAGS_Q -> SEEKING_TAGS_NO_Q
          //state = VisionState.SEEKING_TAGS_NO_Q;
          // Update LL Yaw based on Robot Yaw
          RobotContainer.llAprilTagSubsystem.setLLOrientation(
              RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),RobotContainer.driveSubsystem.getTurnRate());
        }
    //System.out.println("Test7***");

          if (allowStartupFallbackAnchor
              && !hasQuestEverBeenFieldCalibrated
              && llTimer.hasElapsed(QuestNavConstants.llWait)) {
          if (DebugTelemetrySubsystems.odometry) {
            System.out.println("Odometry: llTimer elapsed; using fallback anchor pose");
          }
    //System.out.println("Test8***");

          calibrateQuestFromLL(QuestNavConstants.startingPositionNoLL);
          RobotContainer.driveSubsystem.resetCTREPose(QuestNavConstants.startingPositionNoLL);

          gatePassOverride = false;
          RobotContainer.questNavSubsystem.setInitialPoseSet(true);
          allowStartupFallbackAnchor = false;

          transitionTo(VisionState.CALIBRATED_Q, "Startup fallback anchor after LL timeout"); // SEEKING_TAGS_Q -> CALIBRATED_Q

        } else {
          if (DebugTelemetrySubsystems.odometry) {
            SmartDashboard.putString("Odometry/LLTimer", llTimer.toString());
          }
        }

      }
      case SEEKING_TAGS_NO_Q -> {
    //System.out.println("Test9***");

        if (RobotContainer.questNavSubsystem.isTracking()){ // Quest came up!!! note that this will result in extra 20ms cycle since Quest Pose is not set yet
          Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
          // Set Quest IMU to current robot yaw
          RobotContainer.questNavSubsystem.resetQuestIMUToAngle(robotPose.getRotation().getDegrees());
    //System.out.println("Test10***");

          // Update LL Yaw based on Robot Yaw, since we updated the Quest to the same pose
          RobotContainer.llAprilTagSubsystem.setLLOrientation(
            robotPose.getRotation().getDegrees(),RobotContainer.driveSubsystem.getTurnRate());

          transitionTo(VisionState.SEEKING_TAGS_Q, "Quest came online; IMU reseeded to robot yaw"); // SEEKING_TAGS_NO_Q -> SEEKING_TAGS_Q
          //state = VisionState.SEEKING_TAGS_Q; // Transition state indicating that we have Quest now. The LL Yaw will be tracked by Quest then.

          return; // No need to do anything else this cycle; even if I see AT, I really want to deal with it if Quest is UP
        }

        var poseEstimate = RobotContainer.llAprilTagSubsystem.getBestPoseEstimateFromAllLL();

        if (poseEstimate != null) { // We see AT and have a good pose estimate, though Quest is still not UP
          // Set robot odometry to the pose detected
          RobotContainer.driveSubsystem.resetCTREPose(poseEstimate.pose);
          gatePassOverride = false;
    //System.out.println("Test11***");

          transitionTo(VisionState.CALIBRATED_NO_Q, "Good LL fix w/o Quest; anchored"); // SEEKING_TAGS_NO_Q -> CALIBRATED_NO_Q
          //state = VisionState.CALIBRATED_NO_Q; // Now we're calibrated without Quest
          return; // No need to do anything else this cycle
        }
      }
      case CALIBRATED_Q -> {
    //System.out.println("Test12***");

                if (RobotContainer.questNavSubsystem.isTracking()) { // Quest is working
          questLossHoldTimer.stop();
          questLossHoldTimer.reset();

          fuseQuestNavAllUnread(); // Update poses from Quest

        } else { // Quest temporarily lost; hold CALIBRATED_Q before falling back to LL-only
          if (!questLossHoldTimer.isRunning()) {
            questLossHoldTimer.reset();
            questLossHoldTimer.start();
          }

          if (questLossHoldTimer.hasElapsed(OdometryConstants.QUEST_LOSS_HOLD_SEC)) {
            transitionTo(VisionState.CALIBRATED_NO_Q, "Quest lost while calibrated for 5s");
          } else if (DebugTelemetrySubsystems.odometry) {
            SmartDashboard.putNumber(
                "Odometry/QuestLossHoldRemainingSec",
                Math.max(0.0, OdometryConstants.QUEST_LOSS_HOLD_SEC - questLossHoldTimer.get()));
          }
        }
   // System.out.println("Test14***");

                if (Constants.EnabledSubsystems.ll) {
          RobotContainer.llAprilTagSubsystem.setLLOrientation(
              RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
              RobotContainer.driveSubsystem.getTurnRate());

          boolean questCurrentlyTracking = RobotContainer.questNavSubsystem.isTracking();
          boolean questLossHoldExpired = questLossHoldTimer.isRunning()
              && questLossHoldTimer.hasElapsed(OdometryConstants.QUEST_LOSS_HOLD_SEC);

          if (questCurrentlyTracking || questLossHoldExpired) {
            for (LLCamera llcamera : RobotContainer.llAprilTagSubsystem.getListOfApriltagLLCameras()) {
              fuseLLCamera(llcamera);
            }
          }
        }
    //System.out.println("Test15***");
        
          //RobotContainer.AutonomousConfigure();
    //System.out.println("Test16***");

      }
      case CALIBRATED_NO_Q -> {
        Pose2d robotPose = RobotContainer.driveSubsystem.getPose();

        // Update LL Yaw based on Robot Yaw while LL remains primary
        RobotContainer.llAprilTagSubsystem.setLLOrientation(
            robotPose.getRotation().getDegrees(), RobotContainer.driveSubsystem.getTurnRate());

        // Keep fusing LL while operating without Quest as primary
        for (LLCamera llcamera : RobotContainer.llAprilTagSubsystem.getListOfApriltagLLCameras()) {
          fuseLLCamera(llcamera);
        }

        // If Quest has returned, do NOT switch back immediately.
        // Wait until LL has a good field pose, then use LL to re-anchor Quest.
        if (RobotContainer.questNavSubsystem.isTracking()) {
          var bestPoseEstimate = RobotContainer.llAprilTagSubsystem.getBestPoseEstimateFromAllLL();

          if (bestPoseEstimate != null) {
            calibrateQuestFromLL(bestPoseEstimate.pose);
            RobotContainer.driveSubsystem.resetCTREPose(bestPoseEstimate.pose);
            RobotContainer.questNavSubsystem.setInitialPoseSet(true);
            hasQuestEverBeenFieldCalibrated = true;
            allowStartupFallbackAnchor = false;
            gatePassOverride = false;

            transitionTo(VisionState.CALIBRATED_Q, "Quest regained; re-anchored from LL");
            return;
          }
        }
      }
    }
  }

  
}
