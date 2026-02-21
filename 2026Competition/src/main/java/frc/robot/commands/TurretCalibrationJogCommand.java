package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

/** Calibration: hold to jog turret open-loop at a safe duty cycle. */
public class TurretCalibrationJogCommand extends Command {
  private final double duty;

  public TurretCalibrationJogCommand(double duty) {
    this.duty = duty;
    addRequirements(RobotContainer.turretSubsystem);
  }

  @Override
  public void execute() {
    RobotContainer.turretSubsystem.setDutyCycle(duty);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // hold-to-run
  }
}