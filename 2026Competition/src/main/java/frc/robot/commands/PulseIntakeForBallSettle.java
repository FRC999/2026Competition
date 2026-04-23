package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class PulseIntakeForBallSettle extends Command {
  private enum PulseTarget {
    DOWN,
    MID
  }

  private final Timer stepTimer = new Timer();
  private PulseTarget currentTarget = PulseTarget.DOWN;

  public PulseIntakeForBallSettle() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.runIntakeNoPid(IntakeConstants.INTAKE_ROLLER_DUTY);
    commandTarget(PulseTarget.DOWN);
  }

  @Override
  public void execute() {
    if (!atCurrentTarget() && !stepTimer.hasElapsed(IntakeConstants.INTAKE_PULSE_STEP_TIMEOUT_SEC)) {
      return;
    }

    if (currentTarget == PulseTarget.DOWN) {
      commandTarget(PulseTarget.MID);
    } else {
      commandTarget(PulseTarget.DOWN);
    }
  }

  @Override
  public void end(boolean interrupted) {
    stepTimer.stop();
    RobotContainer.intakeSubsystem.stopIntake();
    CommandScheduler.getInstance().schedule(new DeployIntakeSequence());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void commandTarget(PulseTarget nextTarget) {
    currentTarget = nextTarget;
    stepTimer.restart();

    if (nextTarget == PulseTarget.DOWN) {
      RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeDeployedDeg);
      return;
    }

    RobotContainer.intakeSubsystem.setTargetPivotDeg(IntakeConstants.INTAKE_PULSE_MID_DEG);
  }

  private boolean atCurrentTarget() {
    if (currentTarget == PulseTarget.DOWN) {
      return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg);
    }

    return RobotContainer.intakeSubsystem.isAtPositionDeg(IntakeConstants.INTAKE_PULSE_MID_DEG);
  }
}
