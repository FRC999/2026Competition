package frc.robot.lib;

/**
 * Scheduler-thread shot intent, separate from computed readiness. Trench entry latches an inhibit;
 * exit alone does not clear it. The owning command must clear/reissue its request outside the guard.
 * External control clears requests on both entry and exit, so jam clear cannot revive an old volley.
 */
public final class ShotIntent {
  private boolean requested, externalControl, trenchLocked;
  /** A false-to-true request outside the trench releases the trench latch. */
  public void request(boolean value, boolean inTrench) {
    if (value && !requested && !inTrench) trenchLocked = false;
    requested = value && !externalControl;
    if (inTrench) trenchLocked = true;
  }
  public void externalControl(boolean active) { externalControl = active; requested = false; }
  /** Per-loop mode/position observation; disabling clears both the request and trench latch. */
  public void observe(boolean enabled, boolean inTrench) {
    if (!enabled) { requested = false; trenchLocked = false; }
    else if (inTrench) trenchLocked = true;
  }
  public boolean requested() { return requested; }
  public boolean externalControl() { return externalControl; }
  public boolean trenchLocked() { return trenchLocked; }
}
