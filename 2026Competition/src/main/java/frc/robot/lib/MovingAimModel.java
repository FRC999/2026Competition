package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

/** Constant-velocity release prediction. Mount offset and omega-cross-offset velocity are explicit. */
public final class MovingAimModel {
  private MovingAimModel() {}

  public record Aim(Translation2d releasePosition, Translation2d releaseVelocity,
      double fieldYawRadians, double turretDegrees, double effectiveDistanceMeters,
      double radialSpeedMps, double lateralSpeedMps) {}

  public static Optional<Aim> predict(Pose2d robotPose, ChassisSpeeds robotSpeeds,
      Translation2d pivotOffset, Translation2d target, double releaseDelaySeconds,
      double radialLeadSeconds, double lateralLeadSeconds, double turretZeroDegrees) {
    if (robotPose == null || robotSpeeds == null || pivotOffset == null || target == null
        || !finite(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians(),
            robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, robotSpeeds.omegaRadiansPerSecond,
            pivotOffset.getX(), pivotOffset.getY(), target.getX(), target.getY(),
            releaseDelaySeconds, radialLeadSeconds, lateralLeadSeconds, turretZeroDegrees)
        || releaseDelaySeconds < 0 || radialLeadSeconds < 0 || lateralLeadSeconds < 0) return Optional.empty();
    Translation2d centerVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond,
        robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
    double omega = robotSpeeds.omegaRadiansPerSecond;
    Rotation2d releaseHeading = robotPose.getRotation().plus(new Rotation2d(omega * releaseDelaySeconds));
    Translation2d fieldOffset = pivotOffset.rotateBy(releaseHeading);
    Translation2d releasePosition = robotPose.getTranslation()
        .plus(centerVelocity.times(releaseDelaySeconds)).plus(fieldOffset);
    Translation2d releaseVelocity = centerVelocity.plus(
        new Translation2d(-omega * fieldOffset.getY(), omega * fieldOffset.getX()));
    Translation2d delta = target.minus(releasePosition);
    if (delta.getNorm() < 1e-6) return Optional.empty();
    Translation2d radial = delta.div(delta.getNorm());
    Translation2d lateral = new Translation2d(-radial.getY(), radial.getX());
    double radialSpeed = dot(releaseVelocity, radial);
    double lateralSpeed = dot(releaseVelocity, lateral);
    Translation2d compensated = delta.minus(radial.times(radialSpeed * radialLeadSeconds))
        .minus(lateral.times(lateralSpeed * lateralLeadSeconds));
    if (compensated.getNorm() < 1e-6 || dot(compensated, radial) <= 0) return Optional.empty();
    double yaw = compensated.getAngle().getRadians();
    double turret = Math.toDegrees(MathUtil.angleModulus(yaw - releaseHeading.getRadians()
        - Math.toRadians(turretZeroDegrees)));
    return Optional.of(new Aim(releasePosition, releaseVelocity, yaw, turret,
        compensated.getNorm(), radialSpeed, lateralSpeed));
  }

  private static double dot(Translation2d a, Translation2d b) { return a.getX()*b.getX() + a.getY()*b.getY(); }
  private static boolean finite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }
}
