package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Evaluate permission when scheduled and stop on revocation, cancellation or completion. */
public final class GuardedSysId {
  private GuardedSysId() {}

  public static Command wrap(Command routine, BooleanSupplier allowed, Runnable stop) {
    return routine.until(() -> !allowed.getAsBoolean()).onlyIf(allowed)
        .finallyDo(interrupted -> stop.run());
  }
}
