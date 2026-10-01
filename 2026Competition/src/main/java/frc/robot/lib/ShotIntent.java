package frc.robot.lib;

/** Intent belongs to the active command; clearing a jam cannot resurrect an earlier request. */
public final class ShotIntent {
  private boolean requested, externalControl, trenchLocked;
  public void request(boolean value, boolean inTrench) {
    if (value && !requested && !inTrench) trenchLocked = false;
    requested = value && !externalControl;
    if (inTrench) trenchLocked = true;
  }
  public void externalControl(boolean active) { externalControl = active; requested = false; }
  public void observe(boolean enabled, boolean inTrench) {
    if (!enabled) { requested = false; trenchLocked = false; }
    else if (inTrench) trenchLocked = true;
  }
  public boolean requested() { return requested; }
  public boolean externalControl() { return externalControl; }
  public boolean trenchLocked() { return trenchLocked; }
}
