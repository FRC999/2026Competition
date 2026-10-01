package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

/** Logged camera input boundary shared by real and simulated PhotonVision.
 * Vision IO supports replay-oriented analysis; the drivetrain/mechanisms still use direct CTRE IO.
 * Adapted from the pinned FRC999 prototype; see docs/offseason-195/audit.md for provenance.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    /** True when the coprocessor camera is reachable. Drives the pit "disconnected" alert. */
    public boolean connected = false;

    /** Newest raw MultiTag field-to-camera solve: timestamp, xyz, quaternion wxyz, tag count. */
    public double[] rawFieldToCamera = new double[0];

    /**
     * Angle to the single best target. Not used for pose fusion; kept so a future boresight/turret
     * aiming loop can servo directly on a tag bearing (the 2910/6328 "local" signal). Idea: 1768
     * template {@code getTargetX}.
     */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, false, 0.0);

    /** Newest solvable robot pose from the drained frame burst (MultiTag or single-tag). */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /** Number of PhotonVision results drained from NetworkTables during this robot loop. */
    public int unreadResultCount = 0;

    /** Most recent raw result timestamp, including frames with no visible tags. */
    public double lastResultTimestampSeconds = Double.NEGATIVE_INFINITY;

    /**
     * Older solvable poses from the same unread-result burst that were superseded by the newest pose.
     * NetworkTables is still drained completely; this count makes the intentional estimator-rate
     * reduction visible instead of silently hiding dropped intermediate corrections.
     */
    public int supersededPoseObservationCount = 0;

    /** IDs of all tags seen this loop, for field visualization in AdvantageScope. */
    public int[] tagIds = new int[0];
  }

  /**
   * Bearing to the best visible target (camera-relative). {@code hasTarget} distinguishes "target dead
   * ahead (0,0)" from "no target this frame", and {@code timestampSeconds} (FPGA time of the frame) lets
   * a consumer reject a stale bearing left over from an earlier loop, so a future boresight loop never
   * acts on a phantom or stale zero.
   */
  public static record TargetObservation(
      Rotation2d tx, Rotation2d ty, boolean hasTarget, double timestampSeconds) {}

  /**
   * One field-relative robot-pose estimate from one frame.
   *
   * @param timestamp capture time in the WPILib FPGA time base (converted to the CTRE time base by the
   *     consumer -- see {@link Vision} and {@code RobotContainer}).
   * @param pose field-relative robot pose solved by PhotonVision
   * @param ambiguity PnP ambiguity (single-tag only; ~0 for multi-tag)
   * @param tagCount number of tags used in the solve
   * @param averageTagDistance mean camera-to-tag distance, used for distance-squared covariance
   * @param primaryTagId fiducial ID anchoring this solve: THE tag for a single-tag solve, the first
   *     tag used for a multi-tag solve. Added 2026-07-16 so the fusion layer can (a) reconstruct the
   *     camera-to-tag transform for the trig-solve strategy and (b) compute the field-frame robot->tag
   *     ray angle for the anisotropic covariance model -- and so logs name the tag that produced each
   *     pose. {@code -1} when unknown.
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      int primaryTagId) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
