package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldTargets.AimTarget;
import frc.robot.Constants.OperatorConstants.FieldGeometry;
import frc.robot.config.VisionConstants;

/** Target selection and physical trench regions use the same rotational alliance transform as paths. */
public final class FieldTargeting {
  private FieldTargeting() {}
  public static final double PASS_BOUNDARY_X = 4.664; // Retained season boundary, blue-side coordinates.
  public static final double HYSTERESIS_METERS = .15; // Initial setting; validate against surveyed spots.
  // Provisional software guards, NOT measured footprint or hood-lowering time.
  public static final double TRENCH_PADDING_METERS = .45;
  public static final double TRENCH_LOOKAHEAD_SECONDS = .35;

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
  /** All four nominal structures matter on either alliance. */
  public static boolean inTrench(Pose2d pose) {
    return inBlueTrenches(pose.getX(), pose.getY())
        || inBlueTrenches(VisionConstants.FIELD_LAYOUT.getFieldLength() - pose.getX(),
            VisionConstants.FIELD_LAYOUT.getFieldWidth() - pose.getY());
  }
  /** Lower before entry and remain lowered until clear; also detects crossing the entire region. */
  public static boolean trenchInhibit(Pose2d pose, ChassisSpeeds fieldSpeeds) {
    double x = pose.getX(), y = pose.getY();
    double dx = fieldSpeeds.vxMetersPerSecond * TRENCH_LOOKAHEAD_SECONDS;
    double dy = fieldSpeeds.vyMetersPerSecond * TRENCH_LOOKAHEAD_SECONDS;
    if (!Double.isFinite(x + y + dx + dy)) return true;
    for (boolean oppositeEnd : new boolean[] {false, true}) {
      double sx = oppositeEnd ? VisionConstants.FIELD_LAYOUT.getFieldLength() - x : x;
      double sy = oppositeEnd ? VisionConstants.FIELD_LAYOUT.getFieldWidth() - y : y;
      double ex = sx + (oppositeEnd ? -dx : dx), ey = sy + (oppositeEnd ? -dy : dy);
      double p = TRENCH_PADDING_METERS;
      double minX = FieldGeometry.BLUE_TRENCH_ZONE1_MIN_X_METERS - p;
      double maxX = FieldGeometry.BLUE_TRENCH_ZONE1_MAX_X_METERS + p;
      if (crosses(sx, sy, ex, ey, minX, maxX, FieldGeometry.BLUE_TRENCH_ZONE1_MIN_Y_METERS - p,
              FieldGeometry.BLUE_TRENCH_ZONE1_MAX_Y_METERS + p)
          || crosses(sx, sy, ex, ey, minX, maxX, FieldGeometry.BLUE_TRENCH_ZONE2_MIN_Y_METERS - p,
              FieldGeometry.BLUE_TRENCH_ZONE2_MAX_Y_METERS + p)) return true;
    }
    return false;
  }
  private static boolean crosses(double sx, double sy, double ex, double ey,
      double minX, double maxX, double minY, double maxY) {
    double enter = 0, leave = 1;
    double[] start = {sx, sy}, delta = {ex - sx, ey - sy}, min = {minX, minY}, max = {maxX, maxY};
    for (int axis = 0; axis < 2; axis++) {
      if (Math.abs(delta[axis]) < 1e-9) {
        if (start[axis] < min[axis] || start[axis] > max[axis]) return false;
      } else {
        double a = (min[axis] - start[axis]) / delta[axis], b = (max[axis] - start[axis]) / delta[axis];
        enter = Math.max(enter, Math.min(a, b)); leave = Math.min(leave, Math.max(a, b));
        if (enter > leave) return false;
      }
    }
    return true;
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
