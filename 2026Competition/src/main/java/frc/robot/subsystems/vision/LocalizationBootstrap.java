package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

/** Disabled-only absolute initialization, independent of alliance and of the current pose's accuracy. */
public final class LocalizationBootstrap {
  /** Caller-vetted, calibrated MultiTag robot pose in the blue frame, stamped in FPGA seconds. */
  public record Sample(int camera, double timestamp, Pose2d pose) {}
  private Sample first;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;
  private int count;
  private String status = "WAITING_FOR_MULTITAG";

  // Initial acceptance settings, to be checked on hardware. Repeated frames cannot qualify.
  public static final double MAX_AGE_SECONDS = .25;
  public static final double MIN_SPAN_SECONDS = .10;
  public static final double MAX_SPREAD_METERS = .10;
  public static final double MAX_SPREAD_DEGREES = 3;
  public static final int MIN_SAMPLES = 4;

  /**
   * Accumulates distinct fresh observations from one stable camera; fresh camera disagreement vetoes
   * initialization. The caller supplies only trusted MultiTag samples. A returned pose requests a
   * disabled estimator reset; an empty result can mean waiting, rejection or an already adequate
   * reference, as distinguished by status(). This class never writes the estimator itself.
   */
  public Optional<Pose2d> update(double now, boolean disabled, boolean stationary,
      boolean referenced, Pose2d estimate, List<Sample> samples) {
    if (!disabled || !stationary) {
      clear();
      status = disabled ? "ROBOT_MOVING" : referenced ? "REFERENCED" : "NEEDS_DISABLED_REFERENCE";
      return Optional.empty();
    }
    var fresh = samples.stream().filter(s -> s != null && finite(s.pose())
        && Double.isFinite(s.timestamp()) && now >= s.timestamp()
        && now - s.timestamp() <= MAX_AGE_SECONDS).toList();
    if (fresh.isEmpty()) {
      clear(); status = "WAITING_FOR_MULTITAG"; return Optional.empty();
    }
    Sample newest = fresh.stream().max(java.util.Comparator.comparingDouble(Sample::timestamp)).orElseThrow();
    // Do not pick an arbitrary camera when fresh calibrated cameras disagree.
    if (fresh.stream().anyMatch(s -> !close(s.pose(), newest.pose()))) {
      clear(); status = "CAMERAS_DISAGREE"; return Optional.empty();
    }
    Sample selected = first == null ? newest : fresh.stream().filter(s -> s.camera() == first.camera())
        .max(java.util.Comparator.comparingDouble(Sample::timestamp)).orElse(newest);
    return observeStable(now, referenced, estimate, selected);
  }

  private Optional<Pose2d> observeStable(double now, boolean referenced, Pose2d estimate, Sample newest) {
    if (first == null || newest.camera() != first.camera() || now - lastTimestamp > MAX_AGE_SECONDS
        || !close(first.pose(), newest.pose())) {
      first = newest; lastTimestamp = newest.timestamp(); count = 1;
    } else if (newest.timestamp() > lastTimestamp) {
      lastTimestamp = newest.timestamp(); count++;
    }
    status = "COLLECTING_STABLE_MULTITAG";
    if (count < MIN_SAMPLES || newest.timestamp() - first.timestamp() < MIN_SPAN_SECONDS) {
      return Optional.empty();
    }
    // A carried/placed robot may already have a pit reference. Re-anchor only when that estimate
    // materially disagrees; normal small corrections remain weighted estimator measurements.
    boolean needsReset = !referenced || !finite(estimate)
        || estimate.getTranslation().getDistance(newest.pose().getTranslation()) > .25
        || Math.abs(estimate.getRotation().minus(newest.pose().getRotation()).getDegrees()) > 3;
    status = needsReset ? "SEEDED_MULTITAG" : "REFERENCED";
    if (!needsReset) return Optional.empty();
    clear();
    return Optional.of(newest.pose());
  }

  public String status() { return status; }
  public int sampleCount() { return count; }
  public void clear() { first = null; count = 0; lastTimestamp = Double.NEGATIVE_INFINITY; }
  private static boolean close(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation()) <= MAX_SPREAD_METERS
        && Math.abs(a.getRotation().minus(b.getRotation()).getDegrees()) <= MAX_SPREAD_DEGREES;
  }
  private static boolean finite(Pose2d pose) {
    return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
}
