package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class StopIntakeAndMaybeRetract extends SequentialCommandGroup {
  public StopIntakeAndMaybeRetract() {
    addCommands(
        new StopIntake(),
        Commands.either(
            Commands.none(),
            new RetractIntakeSequence(),
            RobotContainer::isIntakeStayOutAfterTriggerReleaseEnabled));
  }
}