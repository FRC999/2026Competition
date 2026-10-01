package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Owns a turret move to its trusted integrated zero. Normal arrival retains the position hold;
 * interruption stops output. Subsystem trust/limit guards remain active throughout.
 */
public class TurretGoToZeroCommand extends Command {

  public TurretGoToZeroCommand() {
    addRequirements(RobotContainer.turretSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.turretSubsystem.goToAngleDeg(0.0);
  }

  @Override
  public void execute() {
    RobotContainer.turretSubsystem.goToAngleDeg(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      RobotContainer.turretSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.turretSubsystem.atAngleDeg(
        0.0, Constants.OperatorConstants.Turret.TURRET_POSITION_TOLERANCE_DEG);
  }
}
