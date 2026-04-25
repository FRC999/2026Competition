// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMainOneRight extends SequentialCommandGroup {
  /** Creates a new AutoMainOneRight. */
  public AutoMainOneRight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        //  new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MANUAL_FIXED, false)
        //     .raceWith(new WaitCommand(3)), 
        Commands.defer(
            InitialAutoDeployWhileHeld::new,
            Set.of(RobotContainer.intakeSubsystem)), 
         new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              //TrajectoryHelper.AutoDesiredPoses.BlueTrenchRight,
              //new Pose2d(3.884, 6.966, new Rotation2d()),
              RobotContainer.driveSubsystem.getPose(),
              TrajectoryHelper.AutoDesiredPoses.BlueTrenchRight2),    
          Set.of(RobotContainer.driveSubsystem)),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueNeutralRight", false, false),
          //new PrintCommand("Past hub right"),
            // .alongWith(new InstantCommand(
            // () -> RobotContainer.intakeSubsystem.setStayDeployedAfterTriggerRelease(true),
            // RobotContainer.intakeSubsystem).andThen(new DeployIntakeSequence())),
            // .alongWith(new DeployIntakeSequence()),
          //   .raceWith(new WaitCommand(0.2)),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRight_BlueNeutralRightMiddle", false, false))
            .raceWith(new StartIntake()),
          // new RetractIntakeSequence()
          //   .raceWith(new WaitCommand(0.2)),
          //new PrintCommand("Past middle"),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRightMiddle_BlueNeutralHubRight", false, false))
            .raceWith(new StartIntake()),
            // .alongWith(new StopIntake()),
          // RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRight_BlueTrenchRight2", false, false),
          // RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueTrenchRight", false, false),
          // RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRightTurn", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRight_BlueNearBump", false, false),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            .alongWith(new PulseIntakeForBallSettle()))
            .raceWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNearBump_BlueTrenchRight", false, false)),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRightUnTurn", false, false),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight_BlueAllianceRight", false, false),
          // new DeployIntakeSequence() 
          //   .raceWith(new WaitCommand(0.2)),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNearBump_BlueTrenchRight", false, false),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueAllianceRight_BlueTrenchRight", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight_BlueTrenchRight2", false, false),
          // new RetractIntakeSequence()
          //   .raceWith(new WaitCommand(0.2)),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueNeutralHubRight", false, false))
            .raceWith(new StartIntake()),
          //  .alongWith(new StartIntake()),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNearBump_BlueNeutralBump", false, false),
          //RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralBump_BlueNeutralHub", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRight2_BlueNearBump", false, false),
          //  .alongWith(new StopIntake()),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            .alongWith(new PulseIntakeForBallSettle()))
            .raceWith(new WaitCommand(5))
    );
  }
}
