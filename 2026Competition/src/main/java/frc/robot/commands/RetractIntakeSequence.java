package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class RetractIntakeSequence extends Command {
  public RetractIntakeSequence() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.onDriverIntakeTriggerReleased();
    RobotContainer.intakeSubsystem.selectRetractedMode();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.onDriverIntakeTriggerReleased();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeRetracted);
  }
}