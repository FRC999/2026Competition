// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/**
 * Blue-only Worlds sweep with retained start delay and shot/pickup phases. The complete original
 * sequence is over budget; strategy revision is pending. Every phase is interruptible by the outer
 * AUTO deadline and cleans up through its child command's end method.
 */
public class AutoWorldsHubSweepBlue extends SequentialCommandGroup {
  public AutoWorldsHubSweepBlue() {
    addCommands(
        PrecisionPathCommands.requireAlliance(RobotContainer.driveSubsystem, edu.wpi.first.wpilibj.DriverStation.Alliance.Blue),
        new WaitCommand(1),
        Commands.defer(
            InitialAutoDeployWhileHeld::new,
            java.util.Set.of(RobotContainer.intakeSubsystem)),
        RobotContainer.approachCompetitionPath("BlueTrenchRight2_BlueNeutralHubRightMore"),
        RobotContainer.followCompetitionPath(
            "BlueTrenchRight2_BlueNeutralHubRightMore",
            false,
            frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)
        .raceWith(new StartIntake()),
        RobotContainer.followCompetitionPath(
            "BlueNeutralHubRightMore_BlueOffCenter",
            false,
            frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)
        .raceWith(new StartIntake()),
        RobotContainer.followCompetitionPath(
            "BlueOffCenter_BlueNearTower",
            false,
            frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE),
        (new ShootWhileHeld(frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO, false)
           .alongWith(new RepeatIntakeRetractDeploy())
           )
            .raceWith(new WaitCommand(3.75)),
        new DeployIntakeSequence().raceWith(new WaitCommand(1.5)),

        RobotContainer.followCompetitionPath(
            "BlueNearTower_BlueDepotThrough",
            false,
            frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)
        .raceWith(new StartIntake()),
        RobotContainer.followCompetitionPath(
            "BlueDepotThrough_BlueLeftLine",
            false,
            frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE),
        (new ShootWhileHeld(frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO, false)
            .alongWith(new RepeatIntakeRetractDeploy())
            )
            .raceWith(new WaitCommand(2)));
  }
}
