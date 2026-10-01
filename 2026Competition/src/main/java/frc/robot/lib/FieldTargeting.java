package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldTargets.AimTarget;
import frc.robot.Constants.OperatorConstants.FieldGeometry;
import frc.robot.config.VisionConstants;

/** Target selection and physical trench regions use the same rotational alliance transform as paths. */
public final class FieldTargeting {
  private FieldTargeting() {}
  public static final double PASS_BOUNDARY_X = 4.664; // Retained season boundary, blue-side coordinates.
  public static final double HYSTERESIS_METERS = .15; // Initial setting; validate against surveyed spots.

  public static Translation2d target(AimTarget target, boolean red) {
    return new Translation2d(target.getX(red), target.getY(red));
  }
  public static AimTarget select(Pose2d pose, boolean red, boolean allowPassing, AimTarget previous) {
    if (!allowPassing) return AimTarget.HUB;
    double x = red ? VisionConstants.FIELD_LAYOUT.getFieldLength() - pose.getX() : pose.getX();
    double y = red ? VisionConstants.FIELD_LAYOUT.getFieldWidth() - pose.getY() : pose.getY();
    double boundary = PASS_BOUNDARY_X + (previous == AimTarget.HUB ? HYSTERESIS_METERS : -HYSTERESIS_METERS);
    if (x < boundary) return AimTarget.HUB;
    double mid = VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2;
    if (previous == AimTarget.NEUTRAL_LOW && y < mid + HYSTERESIS_METERS) return previous;
    if (previous == AimTarget.NEUTRAL_HIGH && y > mid - HYSTERESIS_METERS) return previous;
    return y <= mid ? AimTarget.NEUTRAL_LOW : AimTarget.NEUTRAL_HIGH;
  }
  /** All four physical regions matter on either alliance; geometry is provisional season data. */
  public static boolean inTrench(Pose2d pose) {
    return inBlueTrenches(pose.getX(), pose.getY())
        || inBlueTrenches(VisionConstants.FIELD_LAYOUT.getFieldLength() - pose.getX(),
            VisionConstants.FIELD_LAYOUT.getFieldWidth() - pose.getY());
  }
  private static boolean inBlueTrenches(double x, double y) {
    return inRect(x, y, FieldGeometry.BLUE_TRENCH_ZONE1_MIN_X_METERS, FieldGeometry.BLUE_TRENCH_ZONE1_MAX_X_METERS,
        FieldGeometry.BLUE_TRENCH_ZONE1_MIN_Y_METERS, FieldGeometry.BLUE_TRENCH_ZONE1_MAX_Y_METERS)
        || inRect(x, y, FieldGeometry.BLUE_TRENCH_ZONE2_MIN_X_METERS, FieldGeometry.BLUE_TRENCH_ZONE2_MAX_X_METERS,
            FieldGeometry.BLUE_TRENCH_ZONE2_MIN_Y_METERS, FieldGeometry.BLUE_TRENCH_ZONE2_MAX_Y_METERS);
  }
  private static boolean inRect(double x, double y, double minX, double maxX, double minY, double maxY) {
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }
}
