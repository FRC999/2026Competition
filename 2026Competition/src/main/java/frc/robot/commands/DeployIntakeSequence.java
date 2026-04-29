package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class DeployIntakeSequence extends Command {
  private final Timer timeoutTimer = new Timer();
  private final boolean useTeleopPowerBoost;

  public DeployIntakeSequence() {
    this(false);
  }

  public DeployIntakeSequence(boolean useTeleopPowerBoost) {
    this.useTeleopPowerBoost = useTeleopPowerBoost;
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timeoutTimer.restart();
    if (useTeleopPowerBoost) {
      RobotContainer.intakeSubsystem.enableTeleopDeployPivotPowerBoost();
    }
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeDeployedDeg);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    timeoutTimer.stop();
    if (useTeleopPowerBoost) {
      RobotContainer.intakeSubsystem.disableTeleopPivotPowerBoost();
    }
    if (!interrupted) {
      RobotContainer.intakeSubsystem.releaseDeployHoldToCoast();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg)
        || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_POSITION_COMMAND_TIMEOUT_SEC);
  }
}
