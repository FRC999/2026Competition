package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class RetractIntakeSequenceWithTimeout extends Command {
  private final Timer timeoutTimer = new Timer();

  public RetractIntakeSequenceWithTimeout() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timeoutTimer.restart();
    RobotContainer.intakeSubsystem.stopIntake();
    RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeRetracted);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    timeoutTimer.stop();
    if (interrupted) {
      RobotContainer.intakeSubsystem.stopPivotAndHoldCurrentPosition();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeRetracted)
        || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_POSITION_COMMAND_TIMEOUT_SEC);
  }
}
