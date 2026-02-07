
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * One-button "shoot until empty" command.
 *
 * This command is intentionally thin:
 * - It asserts driver intent while scheduled.
 * - The volley state machine (and telemetry) lives in {@link AutoShootSupervisorSubsystem}.
 *
 * Requiring the relevant subsystems prevents other commands from fighting the superstructure
 * while a volley is active.
 */
public class AutoShootUntilEmpty extends Command {

  public AutoShootUntilEmpty() {
    addRequirements(RobotContainer.autoShootSupervisorSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);
  }

  @Override
  public void execute() {
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
