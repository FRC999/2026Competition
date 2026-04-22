package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class InitialAutoDeployWhileHeld extends Command {
  private final Timer timeoutTimer = new Timer();

  public InitialAutoDeployWhileHeld() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timeoutTimer.restart();
    RobotContainer.intakeSubsystem.enableInitialAutoDeployCurrentBoost();
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.beginInitialAutoDeploy(
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_DUTY);
  }

  @Override
  public void execute() {
    RobotContainer.intakeSubsystem.enableInitialAutoDeployCurrentBoost();
    RobotContainer.intakeSubsystem.setPivotDutyCycle(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_DUTY);
  }

  @Override
  public void end(boolean interrupted) {
    timeoutTimer.stop();
    RobotContainer.intakeSubsystem.disableInitialAutoDeployCurrentBoost();
    RobotContainer.intakeSubsystem.stopPivotAndHoldCurrentPosition();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeInitialDeployDeg)
        || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_TIMEOUT_SEC);
  }
}
