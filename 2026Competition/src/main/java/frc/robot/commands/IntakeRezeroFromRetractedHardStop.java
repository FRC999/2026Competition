package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.RobotContainer;

public class IntakeRezeroFromRetractedHardStop extends Command {
  private final Timer timer = new Timer();
  private final Timer currentDebounceTimer = new Timer();

  public IntakeRezeroFromRetractedHardStop() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    currentDebounceTimer.stop();
    currentDebounceTimer.reset();

    RobotContainer.intakeSubsystem.setPivotDutyCycle(
        -IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_DUTY);
  }

  @Override
  public void execute() {
    final boolean minTimeElapsed =
        timer.hasElapsed(IntakeConstants.INTAKE_PIVOT_REZERO_MIN_TIME_SEC);

    final boolean currentHigh =
        RobotContainer.intakeSubsystem.getPivotStatorCurrentAmps()
            >= IntakeConstants.INTAKE_PIVOT_REZERO_STATOR_CURRENT_TRIGGER_A;

    if (minTimeElapsed && currentHigh) {
      if (!currentDebounceTimer.isRunning()) {
        currentDebounceTimer.reset();
        currentDebounceTimer.start();
      }
    } else {
      currentDebounceTimer.stop();
      currentDebounceTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.exitOpenLoopHold();

    timer.stop();
    currentDebounceTimer.stop();
    currentDebounceTimer.reset();

    if (!interrupted) {
      RobotContainer.intakeSubsystem.seedZeroFromRetractedHardStop();
    }
  }

  @Override
  public boolean isFinished() {
    return currentDebounceTimer.hasElapsed(
            IntakeConstants.INTAKE_PIVOT_REZERO_CURRENT_DEBOUNCE_SEC)
        || timer.hasElapsed(IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_TIME_SEC);
  }
}