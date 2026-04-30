// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

public class AutoWorldsHubSweepBlue extends SequentialCommandGroup {
  public AutoWorldsHubSweepBlue() {
    addCommands(
        new WaitCommand(5),
        Commands.defer(
            InitialAutoDeployWhileHeld::new,
            java.util.Set.of(RobotContainer.intakeSubsystem)),
        new DeferredCommand(
            () -> RobotContainer.runTrajectory2Poses(
                true,
                RobotContainer.driveSubsystem.getPose(),
                TrajectoryHelper.AutoDesiredPoses.BlueTrenchRight2),
            java.util.Set.of(RobotContainer.driveSubsystem)),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "BlueTrenchRight2_BlueNeutralHubRightMore",
            false,
            false)
        .raceWith(new StartIntake()),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "BlueNeutralHubRightMore_BlueOffCenter",
            false,
            false)
        .raceWith(new StartIntake()),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "BlueOffCenter_BlueNearTower",
            false,
            false),
        (new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
           .alongWith(new RepeatIntakeRetractDeploy())
           )
            .raceWith(new WaitCommand(3.75)),
        new DeployIntakeSequence().raceWith(new WaitCommand(1.5)),

        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "BlueNearTower_BlueDepotThrough",
            false,
            false)
        .raceWith(new StartIntake()),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "BlueDepotThrough_BlueLeftLine",
            false,
            false),
        (new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
            .alongWith(new RepeatIntakeRetractDeploy())
            )
            .raceWith(new WaitCommand(2)));
  }
}
