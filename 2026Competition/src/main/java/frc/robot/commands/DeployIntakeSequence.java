package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class DeployIntakeSequence extends Command {
  public DeployIntakeSequence() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.onDriverIntakeTriggerReleased();
    RobotContainer.intakeSubsystem.selectDeployedMode();
  }

  @Override
  public void execute() {
    if (RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg)) {
      RobotContainer.intakeSubsystem.onDriverIntakeTriggerPressed();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.onDriverIntakeTriggerReleased();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}