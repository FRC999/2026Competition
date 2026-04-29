package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class DeployAndRunIntakeWhileHeld extends Command {
  private boolean waitingForDeployRelease = false;

  public DeployAndRunIntakeWhileHeld() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
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
