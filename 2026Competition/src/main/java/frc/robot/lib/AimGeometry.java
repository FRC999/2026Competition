package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants.Turret;
import frc.robot.Constants.OperatorConstants.TurretGeometry;

/** Robot coordinates are +X forward, +Y left. Turret zero changes bearing, never pivot position. */
public final class AimGeometry {
  private AimGeometry() {}
  /** Rotates the measured robot-frame mount offset into the blue-origin field frame (meters). */
  public static Translation2d pivot(Pose2d pose) {
    return pose.getTranslation().plus(
        TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS.rotateBy(pose.getRotation()));
  }
  /** Field bearing in radians from the turret pivot; coincident target/pivot returns NaN. */
  public static double fieldYaw(Pose2d pose, Translation2d target) {
    Translation2d delta = target.minus(pivot(pose));
    return delta.getNorm() > 1e-6 ? delta.getAngle().getRadians() : Double.NaN;
  }
  /** Robot-relative turret angle in degrees, accounting for its mechanical forward-zero offset. */
  public static double turretDegrees(Pose2d pose, Translation2d target) {
    return Math.toDegrees(MathUtil.angleModulus(fieldYaw(pose, target)
        - pose.getRotation().getRadians() - Math.toRadians(Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG)));
  }
  /** Limits a setpoint only. Check the original aim for reachability; never clamp a measured angle. */
  public static double safeCommand(double angle) {
    return Double.isFinite(angle) ? MathUtil.clamp(angle, Turret.SOFT_AIM_MIN_DEG, Turret.SOFT_AIM_MAX_DEG)
        : Double.NaN;
  }
  public static boolean inWindow(double angle, double min, double max) {
    return Double.isFinite(angle) && angle >= min && angle <= max;
  }
  /** Signed chassis rotation in degrees needed to bring the turret demand inside the given window. */
  public static double chassisTurnToWindow(double turretAngle, double min, double max) {
    return Double.isFinite(turretAngle) && min <= max
        ? turretAngle - MathUtil.clamp(turretAngle, min, max) : Double.NaN;
  }
}
