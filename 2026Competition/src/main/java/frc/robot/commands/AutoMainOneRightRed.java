// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

public class AutoMainOneRightRed extends SequentialCommandGroup {
  public AutoMainOneRightRed() {
    addCommands(
        Commands.defer(
            InitialAutoDeployWhileHeld::new,
            Set.of(RobotContainer.intakeSubsystem)),
         new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              RobotContainer.driveSubsystem.getPose(),
              TrajectoryHelper.AutoDesiredPoses.RedTrenchRight2),
          Set.of(RobotContainer.driveSubsystem)),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueNeutralRight", false, false),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRight_BlueNeutralRightMiddle", false, false))
            .raceWith(new StartIntake()),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRightMiddle_BlueNeutralHubRight", false, false))
            .raceWith(new StartIntake()),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRight_BlueNearBump", false, false),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            //.alongWith(new PulseIntakeForBallSettle())
            )
            .raceWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNearBump_BlueTrenchRight", false, false)),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight_BlueTrenchRight2", false, false),
          (RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight2_BlueNeutralHubRight", false, false))
            .raceWith(new StartIntake()),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubRight2_BlueNearBump", false, false),
          ((new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false))
            //.alongWith(new PulseIntakeForBallSettle())
            )
            .raceWith(new WaitCommand(5))
    );
  }
}
