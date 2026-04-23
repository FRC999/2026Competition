// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoWorldsHubSweep extends SequentialCommandGroup {
  /** Creates a new AutoWorldsHubSweep. */
  public AutoWorldsHubSweep() {
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
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueNeutralHubRightMore", false, false))
            .raceWith(new StartIntake()),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRightMore_BlueOffCenter", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueOffCenter_BlueNearTower", false, false),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            .alongWith(new PulseIntakeForBallSettle()))
            .raceWith(new WaitCommand(5)), 
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNearTower_BlueDepot", false, false))
            .raceWith(new StartIntake()),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            .alongWith(new PulseIntakeForBallSettle()))
            .raceWith(new WaitCommand(5)),  
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueDepot_BlueLeftLine", false, false)
    );
  }
}
