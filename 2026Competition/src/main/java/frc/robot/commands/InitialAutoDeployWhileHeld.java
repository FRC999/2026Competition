package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.RobotContainer;

public class InitialAutoDeployWhileHeld extends Command {
  public InitialAutoDeployWhileHeld() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.beginInitialAutoDeploy(
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_DUTY);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.stopPivotAndHoldCurrentPosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
