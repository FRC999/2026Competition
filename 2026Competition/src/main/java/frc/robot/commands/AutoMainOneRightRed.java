// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class AutoMainOneRightRed extends SequentialCommandGroup {
  public AutoMainOneRightRed() {
    addCommands(
        PrecisionPathCommands.requireAlliance(RobotContainer.driveSubsystem, edu.wpi.first.wpilibj.DriverStation.Alliance.Red),
        Commands.defer(
            InitialAutoDeployWhileHeld::new,
            Set.of(RobotContainer.intakeSubsystem)),
         RobotContainer.approachCompetitionPath("BlueTrenchRight2_BlueNeutralRight"),
          RobotContainer.followCompetitionPath("BlueTrenchRight2_BlueNeutralRight", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE),
          (RobotContainer.followCompetitionPath("BlueNeutralRight_BlueNeutralRightMiddle", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE))
            .raceWith(new StartIntake()),
          (RobotContainer.followCompetitionPath("BlueNeutralRightMiddle_BlueNeutralHubRight", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE))
            .raceWith(new StartIntake()),
          RobotContainer.followCompetitionPath("BlueNeutralHubRight_BlueNearBump", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)
            .raceWith(new StartIntake()),
          ((new ShootWhileHeld(frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO, false))
            .alongWith(new RepeatIntakeRetractDeploy()))
            .raceWith(RobotContainer.followCompetitionPath("BlueNearBump_BlueTrenchRight", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)),
          RobotContainer.followCompetitionPath("BlueTrenchRight_BlueTrenchRight2", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE)
            .raceWith(new StartIntake()),
          (RobotContainer.followCompetitionPath("BlueTrenchRight2_BlueNeutralHubRight", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE))
            .raceWith(new StartIntake()),
          RobotContainer.followCompetitionPath("BlueNeutralHubRight2_BlueNearBump", false, frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE),
          ((new ShootWhileHeld(frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO, false))
            .alongWith(new RepeatIntakeRetractDeploy()))
            .raceWith(new WaitCommand(5))
    );
  }
}
