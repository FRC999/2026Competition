package frc.robot.lib;

/** Target tracking must not erase readiness on every small moving-shot RPM correction. */
public final class ShooterReadinessPolicy {
  private ShooterReadinessPolicy() {}
  public static boolean instantaneouslyReady(double measuredRpm, double targetRpm, double toleranceFraction) {
    return Double.isFinite(measuredRpm) && Double.isFinite(targetRpm) && targetRpm > 1
        && Double.isFinite(toleranceFraction) && toleranceFraction > 0 && toleranceFraction < 1
        && Math.abs(measuredRpm - targetRpm) <= toleranceFraction * targetRpm;
  }
  public static boolean requiresNewWindow(double previous, double next, double toleranceFraction) {
    return !instantaneouslyReady(previous, next, toleranceFraction);
  }
}
