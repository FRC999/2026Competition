package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/**
 * Evaluates permission when scheduled and stops on revocation, cancellation or completion.
 * The wrapper retains the routine's requirements even when denied. Publish guarded tests separately
 * from normal operator bindings; the routine's output callback must also guard the current mode.
 */
public final class GuardedSysId {
  private GuardedSysId() {}

  /** stop must be safe to call even if the underlying routine never initialized. */
  public static Command wrap(Command routine, BooleanSupplier allowed, Runnable stop) {
    return routine.until(() -> !allowed.getAsBoolean()).onlyIf(allowed)
        .finallyDo(interrupted -> stop.run());
  }
}
