package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.RobotContainer;

public class IntakeRezeroFromRetractedHardStop extends Command {
  private final Timer timer = new Timer();

  public IntakeRezeroFromRetractedHardStop() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // Retract into the hard stop using open-loop power.
    // Follower remains active because only the leader is being commanded.
    RobotContainer.intakeSubsystem.setPivotDutyCycle(
        -IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_DUTY);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.exitOpenLoopHold();
    timer.stop();

    // Only accept the new zero if the full retract routine completed.
    if (!interrupted) {
      RobotContainer.intakeSubsystem.seedZeroFromRetractedHardStop();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_TIME_SEC);
  }
}