package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.HardStopHoming;

/** Bounded, operator-requested retract homing. A stall is a proxy requiring an unobstructed mechanism. */
public class IntakeRezeroFromRetractedHardStop extends Command {
  private final HardStopHoming homing = new HardStopHoming(
      IntakeConstants.INTAKE_PIVOT_REZERO_MIN_TIME_SEC,
      IntakeConstants.INTAKE_PIVOT_REZERO_CURRENT_DEBOUNCE_SEC,
      IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_TIME_SEC);
  public IntakeRezeroFromRetractedHardStop() { addRequirements(RobotContainer.intakeSubsystem); }
  @Override public void initialize() { homing.start(Timer.getFPGATimestamp()); }
  @Override public void execute() {
    var intake = RobotContainer.intakeSubsystem;
    intake.runRetractedHoming();
    boolean evidence = intake.hasFreshHomingCurrents()
        && intake.getPivotLeaderStatorCurrentAmps() >= IntakeConstants.INTAKE_PIVOT_REZERO_STATOR_CURRENT_TRIGGER_A
        && intake.getPivotFollowerStatorCurrentAmps() >= IntakeConstants.INTAKE_PIVOT_REZERO_STATOR_CURRENT_TRIGGER_A;
    homing.update(Timer.getFPGATimestamp(), evidence);
  }
  @Override public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setPivotDutyCycle(0);
    if (!interrupted && homing.result() == HardStopHoming.Result.CONFIRMED_STALL) {
      RobotContainer.intakeSubsystem.seedAfterConfirmedHoming();
    } else {
      DriverStation.reportWarning("Intake homing did not qualify; encoder zero was not changed.", false);
    }
  }
  @Override public boolean isFinished() { return homing.result() != HardStopHoming.Result.MOVING; }
}
