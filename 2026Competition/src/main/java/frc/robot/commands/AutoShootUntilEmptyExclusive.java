package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

/**
 * Auto-only: shoot until empty, but EXCLUSIVE (locks the real hardware subsystems).
 *
 * This prevents parallel auto commands from fighting shooter/hood/turret/transfer/spindexer
 * while the AutoShootSupervisorSubsystem state machine is driving them.
 *
 * Intended as a drop-in replacement for AutoShootUntilEmpty in autos.
 */
public class AutoShootUntilEmptyExclusive extends Command {

  public AutoShootUntilEmptyExclusive() {

    // Lock the real subsystems that the supervisor commands each periodic().
    addRequirements(
        RobotContainer.shooterSubsystem,
        RobotContainer.hoodSubsystem,
        RobotContainer.turretSubsystem,
        RobotContainer.transferSubsystem,
        RobotContainer.spindexerSubsystem
    );

    // Optional: also require the supervisor subsystem itself (doesn't hurt if it's a SubsystemBase).
    // If your AutoShootSupervisorSubsystem is not a subsystem requirement target, remove this line.
    addRequirements(RobotContainer.autoShootSupervisorSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);
  }

  @Override
  public void execute() {
    // Intentionally empty.
    // AutoShootSupervisorSubsystem runs the volley state machine continuously in periodic().
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
  }

  @Override
  public boolean isFinished() {
    // Prefer a supervisor-provided "done" signal if you have one.
    // If you already have a robust finish condition inside AutoShootUntilEmpty, mirror it here.

    // If your supervisor has a method like `isEmpty()` or `isDoneShooting()`, use that.
    // Example (uncomment if exists):
    // return supervisor.isDoneShooting();

    // Conservative fallback (never finishes on its own) — replace with your project’s real condition.
    return false;
  }
}