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
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeRetracted);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeRetracted);
  }
}
