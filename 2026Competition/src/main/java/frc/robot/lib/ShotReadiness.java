package frc.robot.lib;

/** Continuous feed permission, checked on every loop (including while already firing). */
public final class ShotReadiness {
  private ShotReadiness() {}

  public enum Reason { READY, IDLE, NO_SOLUTION, POSE_UNREADY, PATH_FAILED, TRENCH_LOCKED,
    TURRET_UNTRUSTED, TURRET_NOT_AIMED, RPM_UNREADY, HOOD_UNREADY, MOVING_IN_STATIC_MODE, COOLDOWN }
  public record Inputs(boolean requested, boolean solution, boolean pose, boolean path,
      boolean trenchLocked, boolean turretTrusted, boolean turretAimed, boolean rpm,
      boolean hood, boolean motion, boolean coolingDown) {}
  public static Reason evaluate(Inputs in) {
    if (!in.requested()) return Reason.IDLE;
    if (in.trenchLocked()) return Reason.TRENCH_LOCKED;
    if (!in.pose()) return Reason.POSE_UNREADY;
    if (!in.path()) return Reason.PATH_FAILED;
    if (!in.solution()) return Reason.NO_SOLUTION;
    if (!in.turretTrusted()) return Reason.TURRET_UNTRUSTED;
    if (!in.turretAimed()) return Reason.TURRET_NOT_AIMED;
    if (!in.rpm()) return Reason.RPM_UNREADY;
    if (!in.hood()) return Reason.HOOD_UNREADY;
    if (!in.motion()) return Reason.MOVING_IN_STATIC_MODE;
    if (in.coolingDown()) return Reason.COOLDOWN;
    return Reason.READY;
  }
}
