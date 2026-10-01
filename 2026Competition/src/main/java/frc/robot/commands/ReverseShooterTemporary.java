package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Owns the shooter while clearing. A fresh shoot press is required afterward. */
public class ReverseShooterTemporary extends Command {
  public ReverseShooterTemporary() {
    addRequirements(RobotContainer.shooterSubsystem, RobotContainer.autoShootSupervisorSubsystem);
  }
  @Override public void initialize() {
    RobotContainer.autoShootSupervisorSubsystem.setExternalControl(true);
    execute();
  }
  @Override public void execute() {
    RobotContainer.shooterSubsystem.setReverseTargetRpm(Constants.OperatorConstants.Shooter.REVERSE_CLEAR_RPM);
  }
  @Override public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.stop();
    RobotContainer.autoShootSupervisorSubsystem.setExternalControl(false);
  }
}
