package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants.Turret;
import frc.robot.Constants.OperatorConstants.TurretGeometry;

/** Robot coordinates are +X forward, +Y left. Turret zero changes bearing, never pivot position. */
public final class AimGeometry {
  private AimGeometry() {}
  public static Translation2d pivot(Pose2d pose) {
    return pose.getTranslation().plus(
        TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS.rotateBy(pose.getRotation()));
  }
  public static double fieldYaw(Pose2d pose, Translation2d target) {
    Translation2d delta = target.minus(pivot(pose));
    return delta.getNorm() > 1e-6 ? delta.getAngle().getRadians() : Double.NaN;
  }
  public static double turretDegrees(Pose2d pose, Translation2d target) {
    return Math.toDegrees(MathUtil.angleModulus(fieldYaw(pose, target)
        - pose.getRotation().getRadians() - Math.toRadians(Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG)));
  }
  public static double safeCommand(double angle) {
    return Double.isFinite(angle) ? MathUtil.clamp(angle, Turret.SOFT_AIM_MIN_DEG, Turret.SOFT_AIM_MAX_DEG)
        : Double.NaN;
  }
  public static boolean inWindow(double angle, double min, double max) {
    return Double.isFinite(angle) && angle >= min && angle <= max;
  }
  public static double chassisTurnToWindow(double turretAngle, double min, double max) {
    return Double.isFinite(turretAngle) && min <= max
        ? turretAngle - MathUtil.clamp(turretAngle, min, max) : Double.NaN;
  }
}
