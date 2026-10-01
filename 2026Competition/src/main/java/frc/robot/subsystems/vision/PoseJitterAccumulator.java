package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Fixed-count pose statistics for a stationary camera calibration capture.
 *
 * <p>Uses Welford's online algorithm, so no image-rate sample array or allocation is required. Yaw is
 * unwrapped around the first sample before accumulation, avoiding the +/-180 degree discontinuity.
 * Standard deviations are population values for the captured set.
 */
public final class PoseJitterAccumulator {
  private final int targetSamples;
  private int count;
  private double yawReferenceRadians;
  private double meanX;
  private double meanY;
  private double meanYawDelta;
  private double m2X;
  private double m2Y;
  private double m2Yaw;
  private double minX;
  private double maxX;
  private double minY;
  private double maxY;
  private double minYawDelta;
  private double maxYawDelta;

  public PoseJitterAccumulator(int targetSamples) {
    if (targetSamples <= 0) {
      throw new IllegalArgumentException("targetSamples must be positive");
    }
    this.targetSamples = targetSamples;
    reset();
  }

  public void reset() {
    count = 0;
    yawReferenceRadians = 0.0;
    meanX = 0.0;
    meanY = 0.0;
    meanYawDelta = 0.0;
    m2X = 0.0;
    m2Y = 0.0;
    m2Yaw = 0.0;
    minX = Double.POSITIVE_INFINITY;
    maxX = Double.NEGATIVE_INFINITY;
    minY = Double.POSITIVE_INFINITY;
    maxY = Double.NEGATIVE_INFINITY;
    minYawDelta = Double.POSITIVE_INFINITY;
    maxYawDelta = Double.NEGATIVE_INFINITY;
  }

  /** Adds a pose unless the requested sample count has already been reached. */
  public void add(Pose2d pose) {
    if (isReady()) {
      return;
    }
    if (count == 0) {
      yawReferenceRadians = pose.getRotation().getRadians();
    }
    double yawDelta =
        MathUtil.angleModulus(pose.getRotation().getRadians() - yawReferenceRadians);

    count++;
    double deltaX = pose.getX() - meanX;
    meanX += deltaX / count;
    m2X += deltaX * (pose.getX() - meanX);
    double deltaY = pose.getY() - meanY;
    meanY += deltaY / count;
    m2Y += deltaY * (pose.getY() - meanY);
    double deltaYaw = yawDelta - meanYawDelta;
    meanYawDelta += deltaYaw / count;
    m2Yaw += deltaYaw * (yawDelta - meanYawDelta);

    minX = Math.min(minX, pose.getX());
    maxX = Math.max(maxX, pose.getX());
    minY = Math.min(minY, pose.getY());
    maxY = Math.max(maxY, pose.getY());
    minYawDelta = Math.min(minYawDelta, yawDelta);
    maxYawDelta = Math.max(maxYawDelta, yawDelta);
  }

  public int getCount() {
    return count;
  }

  public int getTargetSamples() {
    return targetSamples;
  }

  public boolean isReady() {
    return count >= targetSamples;
  }

  public Pose2d getMeanPose() {
    if (count == 0) {
      return new Pose2d();
    }
    return new Pose2d(
        meanX,
        meanY,
        Rotation2d.fromRadians(yawReferenceRadians + meanYawDelta));
  }

  public double getStdDevX() {
    return standardDeviation(m2X);
  }

  public double getStdDevY() {
    return standardDeviation(m2Y);
  }

  public double getStdDevTranslation() {
    return Math.hypot(getStdDevX(), getStdDevY());
  }

  public double getStdDevYawDegrees() {
    return Math.toDegrees(standardDeviation(m2Yaw));
  }

  public double getPeakToPeakX() {
    return count == 0 ? 0.0 : maxX - minX;
  }

  public double getPeakToPeakY() {
    return count == 0 ? 0.0 : maxY - minY;
  }

  public double getPeakToPeakYawDegrees() {
    return count == 0 ? 0.0 : Math.toDegrees(maxYawDelta - minYawDelta);
  }

  private double standardDeviation(double m2) {
    return count == 0 ? 0.0 : Math.sqrt(m2 / count);
  }
}
