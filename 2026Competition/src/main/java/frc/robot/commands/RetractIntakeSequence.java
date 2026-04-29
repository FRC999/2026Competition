package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class RetractIntakeSequence extends Command {
  private final Timer timeoutTimer = new Timer();
  private final boolean useTeleopPowerBoost;
  private boolean keepRollersRunningDuringRetract;

  public RetractIntakeSequence() {
    this(false);
  }

  public RetractIntakeSequence(boolean useTeleopPowerBoost) {
    this.useTeleopPowerBoost = useTeleopPowerBoost;
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timeoutTimer.restart();
    keepRollersRunningDuringRetract = DriverStation.isTeleopEnabled();
    if (useTeleopPowerBoost) {
      RobotContainer.intakeSubsystem.enableTeleopRetractPivotPowerBoost();
    }
    if (!keepRollersRunningDuringRetract) {
      RobotContainer.intakeSubsystem.stopIntake();
    }
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeRetracted);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    timeoutTimer.stop();
    if (keepRollersRunningDuringRetract) {
      RobotContainer.intakeSubsystem.stopIntake();
    }
    if (useTeleopPowerBoost) {
      RobotContainer.intakeSubsystem.disableTeleopPivotPowerBoost();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeRetracted)
        || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_POSITION_COMMAND_TIMEOUT_SEC);
  }
}
