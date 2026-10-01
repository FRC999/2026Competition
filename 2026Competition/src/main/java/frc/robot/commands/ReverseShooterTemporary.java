package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** One owner clears the complete feed train; a fresh shoot press is required afterward. */
public class ReverseShooterTemporary extends Command {
  public ReverseShooterTemporary() {
    addRequirements(RobotContainer.shooterSubsystem, RobotContainer.autoShootSupervisorSubsystem,
        RobotContainer.transferSubsystem, RobotContainer.spindexerSubsystem);
  }
  @Override public void initialize() {
    RobotContainer.autoShootSupervisorSubsystem.setExternalControl(true);
    execute();
  }
  @Override public void execute() {
    RobotContainer.shooterSubsystem.setReverseTargetRpm(Constants.OperatorConstants.Shooter.REVERSE_CLEAR_RPM);
    RobotContainer.transferSubsystem.reverseTransfer();
    RobotContainer.spindexerSubsystem.runSupplyReverse();
  }
  @Override public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.stop();
    RobotContainer.transferSubsystem.stop();
    RobotContainer.spindexerSubsystem.stop();
    RobotContainer.autoShootSupervisorSubsystem.setExternalControl(false);
  }
}
