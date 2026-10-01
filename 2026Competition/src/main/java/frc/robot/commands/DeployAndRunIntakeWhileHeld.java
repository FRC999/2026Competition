package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

/**
 * LT intake action: deploy once while running rollers, then coast after successful deployment.
 * A blocked deployment times out and brakes the pivot while rollers remain requested until release.
 * Cancellation stops pivot and rollers; RobotContainer decides whether release schedules a retract.
 */
public class DeployAndRunIntakeWhileHeld extends Command {
  private boolean waitingForDeployRelease = false;
  private final edu.wpi.first.wpilibj.Timer deployTimer = new edu.wpi.first.wpilibj.Timer();

  public DeployAndRunIntakeWhileHeld() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    deployTimer.restart();
    RobotContainer.intakeSubsystem.runIntakeNoPid(IntakeConstants.INTAKE_ROLLER_DUTY);
    waitingForDeployRelease = !RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg);
    if (waitingForDeployRelease) {
      RobotContainer.intakeSubsystem.setIntakePositionWithAngle(
          IntakePositions.IntakeDeployedDeg,
          IntakeConstants.LEFT_TRIGGER_DEPLOY_EXTRA_FEEDFORWARD_VOLTS);
      return;
    }

    RobotContainer.intakeSubsystem.releaseDeployHoldToCoast();
  }

  @Override
  public void execute() {
    if (waitingForDeployRelease
        && RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg)) {
      RobotContainer.intakeSubsystem.releaseDeployHoldToCoast();
      waitingForDeployRelease = false;
    }
    if (waitingForDeployRelease && deployTimer.hasElapsed(
        IntakeConstants.INTAKE_PIVOT_POSITION_COMMAND_TIMEOUT_SEC)) {
      RobotContainer.intakeSubsystem.stopPivotInBrake();
      waitingForDeployRelease = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    deployTimer.stop();
    if (interrupted) RobotContainer.intakeSubsystem.stopPivotInBrake();
    RobotContainer.intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
