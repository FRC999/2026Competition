package frc.robot.lib;

/** A held input cannot restart a canceled action when its operating mode or panic gate reopens. */
public final class FreshPress {
  private boolean armed;
  /**
   * Call every button-loop poll, including while disallowed. Returns the armed held state, not a
   * one-loop pulse; WPILib Trigger derives edges. A closed gate disarms until a release is observed.
   */
  public boolean update(boolean allowed, boolean pressed) {
    if (!allowed) { armed = false; return false; }
    if (!pressed) armed = true;
    return armed && pressed;
  }
}
