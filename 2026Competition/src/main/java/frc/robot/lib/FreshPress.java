package frc.robot.lib;

/** A held input cannot restart a canceled action when its operating mode or panic gate reopens. */
public final class FreshPress {
  private boolean armed;
  public boolean update(boolean allowed, boolean pressed) {
    if (!allowed) { armed = false; return false; }
    if (!pressed) armed = true;
    return armed && pressed;
  }
}
