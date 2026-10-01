package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import java.util.OptionalDouble;

/** Turret commands use continuous angles; never wrap through an extension-restricted region. */
public final class TurretMotionPolicy {
  private TurretMotionPolicy() {}

  public static OptionalDouble motorPositionTarget(double desiredDegrees, double minDegrees,
      double maxDegrees, double motorRotationsPerTurretTurn, double motorSign, boolean positionTrusted) {
    if (!positionTrusted || !Double.isFinite(desiredDegrees)
        || !Double.isFinite(minDegrees) || !Double.isFinite(maxDegrees) || minDegrees >= maxDegrees
        || !Double.isFinite(motorRotationsPerTurretTurn) || motorRotationsPerTurretTurn <= 0
        || Math.abs(motorSign) != 1) return OptionalDouble.empty();
    return OptionalDouble.of(motorSign * MathUtil.clamp(desiredDegrees, minDegrees, maxDegrees)
        / 360.0 * motorRotationsPerTurretTurn);
  }

  public static boolean atTarget(double measuredDegrees, double desiredDegrees, double toleranceDegrees,
      double minDegrees, double maxDegrees, boolean positionTrusted) {
    return positionTrusted && Double.isFinite(measuredDegrees) && Double.isFinite(desiredDegrees)
        && Double.isFinite(toleranceDegrees) && toleranceDegrees >= 0
        && desiredDegrees >= minDegrees && desiredDegrees <= maxDegrees
        && Math.abs(measuredDegrees - desiredDegrees) <= toleranceDegrees;
  }
}
