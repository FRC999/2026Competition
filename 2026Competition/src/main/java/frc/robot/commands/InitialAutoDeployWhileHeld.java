package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

/**
 * Bounded opening deployment. Owns intake, stops rollers, and temporarily raises current limits.
 * Boost ends at its separate maximum; target arrival or overall timeout ends the command. Every end
 * restores current limits and stops/brakes the pivot. Timeout does not establish a new encoder zero.
 */
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
    if (timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_BOOST_MAX_SEC))
      RobotContainer.intakeSubsystem.disableInitialAutoDeployCurrentBoost();
    RobotContainer.intakeSubsystem.setPivotDutyCycle(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_DUTY);
  }

  @Override
  public void end(boolean interrupted) {
    timeoutTimer.stop();
    RobotContainer.intakeSubsystem.disableInitialAutoDeployCurrentBoost();
    RobotContainer.intakeSubsystem.stopPivotInBrake();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeInitialDeployDeg)
        || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_TIMEOUT_SEC);
  }
}
