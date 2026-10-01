package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.config.VisionConstants;

/**
 * Stateless observation validity, covariance and timestamp calculations with no hardware IO.
 * {@link Vision} owns camera configuration, per-loop orchestration and logging;
 * {@link LocalizationBootstrap} owns stable disabled initialization. Split out in the prototype
 * on 2026-07-16 (was static methods inside
 * {@code Vision}).
 *
 * <p>Idea traceability is preserved from the original in-class versions: 6328 (covariance shape, single-tag heading = +Infinity), 3467 (rejection reason enums), 125 (conservative
 * single-tag heading), 5940 (anisotropic log-fitted covariance -- 2026-07-16 survey), plus this
 * project's capture-timestamp rejection for frames recorded before a pose reset.
 */
public final class VisionPolicy {
  private VisionPolicy() {}

  /** Stable, log-filterable categories for why a frame was discarded. Idea: 3467 reason enums. */
  public enum RejectionReason {
    ACCEPTED,
    NO_TAGS,
    NON_FINITE,
    INVALID_DISTANCE,
    STALE_TIMESTAMP,
    FUTURE_TIMESTAMP,
    DUPLICATE_OR_OUT_OF_ORDER,
    UNCALIBRATED_OR_LAYOUT_UNCONFIRMED,
    BAD_Z,
    OUTSIDE_FIELD,
    TOO_FAR,
    SINGLE_TAG_AMBIGUOUS
  }

  /**
   * How a single-tag frame becomes a fused pose.
   *
   * <ul>
   *   <li>{@link SingleTagStrategy#PNP}: use the coprocessor's single-tag PnP robot pose (translation
   *       only; heading is never fused for single tags). Retained prototype baseline, not physical
   *       validation on this competition robot.
   *   <li>{@link SingleTagStrategy#TRIG_SOLVE}: recompute XY from the camera-to-tag translation + the odometry-buffer
   *       heading at the frame timestamp ({@link SingleTagTrigSolver}). Idea: 6328 via PhotonVision
   *       {@code PNP_DISTANCE_TRIG_SOLVE}; 1678 C2026 runs it in production. 2026-07-16 survey.
   * </ul>
   */
  public enum SingleTagStrategy {
    PNP,
    TRIG_SOLVE
  }

  /**
   * Which measurement-noise model weights an accepted frame.
   *
   * <ul>
   *   <li>{@link CovarianceModel#ISOTROPIC}: retained prototype baseline -- {@code baseline * dist^2 / tagCount^2 *
   *       cameraFactor}, same in X and Y (6328/6995 shape).
   *   <li>{@link CovarianceModel#ANISOTROPIC}: 5940-style (2026-07-16 survey) -- separate power-law sigmas parallel
   *       and perpendicular to the camera->tag ray, rotated into field axes. Range error grows faster
   *       than bearing error, so the noise ellipse is real; coefficients are PROVISIONAL until fitted
   *       from robot logs (see docs/offseason-195/testing.md, stage R2).
   * </ul>
   */
  public enum CovarianceModel {
    ISOTROPIC,
    ANISOTROPIC
  }

  /** Returns {@link RejectionReason#ACCEPTED} when the observation is usable, else the failing gate. */
  public static RejectionReason rejectionReason(VisionIO.PoseObservation obs) {
    if (obs.tagCount() <= 0) {
      return RejectionReason.NO_TAGS;
    }
    Pose3d p = obs.pose();
    // Reject NaN/Inf in the pose AND the downstream scalars: a NaN distance would make the covariance
    // hand CTRE NaN std devs, and a NaN ambiguity would silently pass the ambiguity gate below (any
    // comparison with NaN is false). Idea: v2 strategy doc rule 7.
    if (!(Double.isFinite(p.getX())
        && Double.isFinite(p.getY())
        && Double.isFinite(p.getZ())
        && Double.isFinite(p.getRotation().getX())
        && Double.isFinite(p.getRotation().getY())
        && Double.isFinite(p.getRotation().getZ())
        && Double.isFinite(obs.timestamp())
        && Double.isFinite(obs.averageTagDistance())
        && Double.isFinite(obs.ambiguity()))) {
      return RejectionReason.NON_FINITE;
    }
    if (obs.averageTagDistance() <= 0.0) return RejectionReason.INVALID_DISTANCE;
    if (Math.abs(p.getZ()) > VisionConstants.MAX_ACCEPTED_Z_METERS) {
      return RejectionReason.BAD_Z;
    }
    double x = p.getX();
    double y = p.getY();
    double m = VisionConstants.FIELD_BORDER_MARGIN_METERS;
    if (x < -m
        || x > VisionConstants.FIELD_LENGTH_METERS + m
        || y < -m
        || y > VisionConstants.FIELD_WIDTH_METERS + m) {
      return RejectionReason.OUTSIDE_FIELD;
    }
    if (obs.averageTagDistance() > VisionConstants.MAX_AVERAGE_TAG_DISTANCE_METERS) {
      return RejectionReason.TOO_FAR;
    }
    // PhotonVision ambiguity is [0,1], or -1 when it could not be computed. Reject a single-tag frame
    // with unknown (negative) OR high ambiguity: we do not gyro-disambiguate the PnP pose itself, so an
    // unverifiable single-tag pose could be the flipped solution. NOTE (2026-07-16): the gate is kept
    // even in TRIG_SOLVE mode so the A/B comparison fuses the SAME frames -- only the pose math and
    // weights differ. Loosening it under trig-solve (which is ambiguity-immune for XY) is a possible
    // later knob. (Multi-tag solves never reach this gate.)
    if (obs.tagCount() == 1
        && (obs.ambiguity() < 0.0 || obs.ambiguity() > VisionConstants.MAX_SINGLE_TAG_AMBIGUITY)) {
      return RejectionReason.SINGLE_TAG_AMBIGUOUS;
    }
    return RejectionReason.ACCEPTED;
  }

  /** Check a camera's original FPGA timestamp before using it in either fusion or manual seeding. */
  public static RejectionReason timestampRejectionReason(double timestamp, double now, double last) {
    if (!Double.isFinite(timestamp) || !Double.isFinite(now)) return RejectionReason.NON_FINITE;
    if (timestamp > now + VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS) return RejectionReason.FUTURE_TIMESTAMP;
    if (now - timestamp > VisionConstants.MAX_FRAME_AGE_SECONDS) return RejectionReason.STALE_TIMESTAMP;
    if (timestamp <= last) return RejectionReason.DUPLICATE_OR_OUT_OF_ORDER;
    return RejectionReason.ACCEPTED;
  }

  /**
   * Baseline isotropic covariance: distance-squared / tag-count-squared with a per-camera trust
   * factor; heading is ignored for single-tag solves.
   *
   * <p>Idea traceability: 6328/Northstar and 6995 adaptive covariance ({@code k * dist^2 / tagCount^2
   * * cameraFactor} -- tag count is SQUARED, so multi-tag solves are trusted much more); 125 NUTRONs
   * conservative single-tag heading (theta = +Infinity, never fuse one-tag rotation).
   */
  public static Matrix<N3, N1> standardDeviations(
      int cameraIndex, double averageDistanceMeters, int tagCount, boolean trustRotation) {
    double distanceFactor =
        averageDistanceMeters * averageDistanceMeters / ((double) tagCount * tagCount);
    double cameraFactor = cameraFactor(cameraIndex);
    double xy = Math.max(VisionConstants.MIN_LINEAR_STD_DEV_METERS,
        VisionConstants.LINEAR_STD_DEV_BASELINE * distanceFactor * cameraFactor);
    double theta =
        trustRotation
            ? Math.max(VisionConstants.MIN_ANGULAR_STD_DEV_RADIANS, VisionConstants.ANGULAR_STD_DEV_BASELINE
                * distanceFactor
                * angularCameraFactor(cameraIndex))
            : Double.POSITIVE_INFINITY;
    return VecBuilder.fill(xy, xy, theta);
  }

  /**
   * 5940-style anisotropic covariance (2026-07-16 survey; see their 2026
   * {@code subsystems/vision/Vision.java} ~lines 495-555): a camera measures the tag's <em>bearing</em>
   * much better than its <em>range</em>, so the noise ellipse is elongated along the camera->tag ray.
   * Two fitted power laws, {@code sigma = C * d^E}, one parallel to the ray and one perpendicular, are
   * rotated into field axes through the ray angle {@code alpha}:
   *
   * <pre>
   *   varX = cos^2(a)*varPar + sin^2(a)*varPerp
   *   varY = sin^2(a)*varPar + cos^2(a)*varPerp
   * </pre>
   *
   * <p>Theta keeps the baseline isotropic theta model (and +Infinity when rotation is untrusted) --
   * 5940 also blends theta by ray angle, but that refinement needs fitted data we do not have yet.
   * All coefficients are PROVISIONAL (chosen to roughly match the isotropic model at 2 m) until fitted
   * from real-robot logs -- fitting procedure: docs/offseason-195/testing.md stage R2.
   *
   * @param cameraIndex index into {@code CAMERA_STD_DEV_FACTORS}
   * @param distanceMeters average camera-to-tag distance for the frame
   * @param tagCount number of tags in the solve (selects the single- vs multi-tag power law)
   * @param trustRotation whether theta may be fused (multi-tag only)
   * @param rayAngleFieldRadians field-frame angle of the robot->tag ray (from {@code atan2})
   */
  public static Matrix<N3, N1> anisotropicStandardDeviations(
      int cameraIndex,
      double distanceMeters,
      int tagCount,
      boolean trustRotation,
      double rayAngleFieldRadians) {
    boolean multiTag = tagCount >= 2;
    double sigmaPar =
        powerLawSigma(
            distanceMeters,
            multiTag
                ? VisionConstants.ANISO_MULTI_TAG_PARALLEL_COEFF
                : VisionConstants.ANISO_SINGLE_TAG_PARALLEL_COEFF,
            multiTag
                ? VisionConstants.ANISO_MULTI_TAG_PARALLEL_EXP
                : VisionConstants.ANISO_SINGLE_TAG_PARALLEL_EXP);
    double sigmaPerp =
        powerLawSigma(
            distanceMeters,
            multiTag
                ? VisionConstants.ANISO_MULTI_TAG_PERP_COEFF
                : VisionConstants.ANISO_SINGLE_TAG_PERP_COEFF,
            multiTag
                ? VisionConstants.ANISO_MULTI_TAG_PERP_EXP
                : VisionConstants.ANISO_SINGLE_TAG_PERP_EXP);
    double cameraFactor = cameraFactor(cameraIndex);

    double varPar = sigmaPar * sigmaPar;
    double varPerp = sigmaPerp * sigmaPerp;
    double cos = Math.cos(rayAngleFieldRadians);
    double sin = Math.sin(rayAngleFieldRadians);
    double varX = cos * cos * varPar + sin * sin * varPerp;
    double varY = sin * sin * varPar + cos * cos * varPerp;

    // Theta: reuse the validated baseline model rather than inventing an unfitted second power law.
    double distanceFactor = distanceMeters * distanceMeters / ((double) tagCount * tagCount);
    double theta =
        trustRotation
            ? Math.max(VisionConstants.MIN_ANGULAR_STD_DEV_RADIANS, VisionConstants.ANGULAR_STD_DEV_BASELINE
                * distanceFactor
                * angularCameraFactor(cameraIndex))
            : Double.POSITIVE_INFINITY;
    return VecBuilder.fill(
        Math.max(VisionConstants.MIN_LINEAR_STD_DEV_METERS, Math.sqrt(varX) * cameraFactor),
        Math.max(VisionConstants.MIN_LINEAR_STD_DEV_METERS, Math.sqrt(varY) * cameraFactor), theta);
  }

  /** {@code sigma = coeff * distance^exponent} -- the 5940 fitted-noise primitive. */
  static double powerLawSigma(double distanceMeters, double coeff, double exponent) {
    return coeff * Math.pow(distanceMeters, exponent);
  }

  /** Per-camera trust multiplier with a safe default for unknown indices. Idea: 6328 stdDevFactor. */
  static double cameraFactor(int cameraIndex) {
    return cameraIndex >= 0 && cameraIndex < VisionConstants.CAMERA_STD_DEV_FACTORS.length
        ? VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex]
        : 1.0;
  }

  /** Additional per-camera theta multiplier, independent from the XY factor. */
  static double angularCameraFactor(int cameraIndex) {
    return cameraIndex >= 0 && cameraIndex < VisionConstants.CAMERA_ANGULAR_STD_DEV_FACTORS.length
        ? VisionConstants.CAMERA_ANGULAR_STD_DEV_FACTORS[cameraIndex]
        : 1.0;
  }

  /** Whether this camera is permitted to contribute MultiTag heading. XY remains usable either way. */
  static boolean cameraRotationTrustEnabled(int cameraIndex) {
    return cameraIndex >= 0 && cameraIndex < VisionConstants.CAMERA_ROTATION_TRUST_ENABLED.length
        && VisionConstants.CAMERA_ROTATION_TRUST_ENABLED[cameraIndex];
  }

  /**
   * Whether an observation may contribute theta to the running estimator. A camera can remain
   * eligible for a disabled/manual MultiTag seed while enabled fusion deliberately uses gyro heading.
   */
  static boolean shouldFuseRotation(int cameraIndex, int tagCount, boolean robotEnabled) {
    return tagCount >= 2
        && cameraRotationTrustEnabled(cameraIndex)
        && (!robotEnabled || VisionConstants.FUSE_VISION_ROTATION_WHILE_ENABLED);
  }

  /**
   * True when a vision frame's capture timestamp predates the last pose reset, so it must not be
   * fused (an in-flight frame still sees the pre-reset pose).
   */
  public static boolean isPreResetFrame(double obsTimestampSeconds, double lastResetTimeSeconds) {
    return obsTimestampSeconds < lastResetTimeSeconds;
  }

  /**
   * Pure freshness check for {@code Vision.getTargetX}: returns the target yaw only when a target is
   * present AND its frame timestamp is within {@code TARGET_OBSERVATION_MAX_STALENESS_SECONDS} of
   * {@code nowSeconds} in EITHER direction. Using the absolute difference also rejects a future-dated
   * timestamp (a clock glitch), so the check is fully bounded.
   */
  public static Optional<Rotation2d> freshTargetX(VisionIO.TargetObservation obs, double nowSeconds) {
    if (!obs.hasTarget() || !Double.isFinite(nowSeconds) || !Double.isFinite(obs.timestampSeconds())
        || !Double.isFinite(obs.tx().getRadians()) || !Double.isFinite(obs.ty().getRadians())) {
      return Optional.empty();
    }
    if (Math.abs(nowSeconds - obs.timestampSeconds())
        > VisionConstants.TARGET_OBSERVATION_MAX_STALENESS_SECONDS) {
      return Optional.empty();
    }
    return Optional.of(obs.tx());
  }
}
