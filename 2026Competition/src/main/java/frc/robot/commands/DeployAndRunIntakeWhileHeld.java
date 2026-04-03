package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class DeployAndRunIntakeWhileHeld extends Command {
  public DeployAndRunIntakeWhileHeld() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.runIntakeNoPid(IntakeConstants.INTAKE_ROLLER_DUTY);
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeDeployedDeg);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
