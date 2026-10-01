package frc.robot.lib;

import edu.wpi.first.math.MathUtil;

/**
 * Finite, bounded joystick shaping with a raw-input deadband and optional cubic response.
 */
public final class DriverInput {
  private DriverInput() {}
  /** Retains the season's cubic curve outside the raw-input deadband, without sign reversal inside. */
  public static double shape(double raw, double deadband, boolean cubic) {
    if (!Double.isFinite(raw) || !Double.isFinite(deadband) || deadband < 0 || deadband >= 1) return 0;
    raw = MathUtil.clamp(raw, -1, 1);
    if (Math.abs(raw) <= deadband) return 0;
    if (!cubic) return MathUtil.applyDeadband(raw, deadband);
    double threshold = deadband * deadband * deadband;
    return (raw * raw * raw - Math.copySign(threshold, raw)) / (1 - threshold);
  }
}
