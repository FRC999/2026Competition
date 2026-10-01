package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.VisionConstants;

/** Conservative software checks for REBUILT G403/G407; these are not bumper inspection geometry. */
public final class FieldRules {
  private FieldRules() {}
  public static final double ALLIANCE_ZONE_DEPTH_METERS = 158.6 * .0254;
  public static final double ZONE_MARGIN_METERS = .10; // Provisional localization allowance.

  public static double allianceX(Pose2d pose, boolean red) {
    return red ? VisionConstants.FIELD_LAYOUT.getFieldLength() - pose.getX() : pose.getX();
  }

  public static boolean onOwnAutoHalf(Pose2d pose, boolean red) {
    return finiteAndInField(pose) && allianceX(pose, red) <= VisionConstants.FIELD_LAYOUT.getFieldLength() / 2;
  }

  /** Require center inside the zone now and at estimated release; no invented bumper footprint. */
  public static boolean hubZoneConfirmed(Pose2d pose, ChassisSpeeds fieldSpeeds, boolean red, double releaseDelay) {
    if (!finiteAndInField(pose) || !Double.isFinite(fieldSpeeds.vxMetersPerSecond)
        || !Double.isFinite(releaseDelay) || releaseDelay < 0) return false;
    double x = allianceX(pose, red);
    double releaseX = x + (red ? -1 : 1) * fieldSpeeds.vxMetersPerSecond * releaseDelay;
    return Math.max(x, releaseX) <= ALLIANCE_ZONE_DEPTH_METERS - ZONE_MARGIN_METERS;
  }

  private static boolean finiteAndInField(Pose2d pose) {
    return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians()) && pose.getX() >= 0 && pose.getY() >= 0
        && pose.getX() <= VisionConstants.FIELD_LAYOUT.getFieldLength()
        && pose.getY() <= VisionConstants.FIELD_LAYOUT.getFieldWidth();
  }
}
