package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants.IntakeConstants;

public class ReverseIntake extends Command {
  public ReverseIntake() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.runIntake(IntakeConstants.ROLLER_REVERSE_RPS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}