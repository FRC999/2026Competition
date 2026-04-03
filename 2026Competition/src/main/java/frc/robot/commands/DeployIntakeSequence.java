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
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeDeployedDeg);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      RobotContainer.intakeSubsystem.releaseDeployHoldToCoast();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg);
  }
}
