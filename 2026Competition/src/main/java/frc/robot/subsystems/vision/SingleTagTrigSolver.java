package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Pure math for the "trig solve" single-tag pose strategy: recompute the robot's field XY from the
 * camera-to-tag <em>translation</em> and the robot's <em>known heading</em> (from the odometry/gyro
 * history), instead of trusting the full single-tag PnP pose.
 *
 * <p>Why this is better than raw single-tag PnP (2026-07-16 survey):
 *
 * <ul>
 *   <li>Single-tag PnP has a well-known two-solution <em>rotation</em> ambiguity; the flipped solution
 *       drags the translated robot pose with it. The camera-to-tag translation (bearing + range),
 *       however, comes from where the tag sits in the image and is essentially ambiguity-free.
 *   <li>By substituting the gyro heading for the PnP rotation (a "drivebase flat on the floor" +
 *       "heading is known" constraint), the tag's field position pins the robot's XY exactly.
 * </ul>
 *
 * <p>Idea traceability: 6328's 2025 trig-solve reference implementation, now shipped upstream in
 * PhotonVision as {@code PoseStrategy.PNP_DISTANCE_TRIG_SOLVE} (PhotonLib v2026.x, requires
 * {@code addHeadingData} each loop); run in production by 1678 in C2026
 * ({@code frc/lib/io/vision/photon/AprilTagPhotonCameraIO.java}). We implement the same math as a pure
 * helper behind our AdvantageKit IO layer instead of adopting {@code PhotonPoseEstimator}, so it stays
 * replayable and headless-unit-testable, and the heading source stays explicit (CTRE odometry buffer,
 * sampled at the frame timestamp).
 *
 * <p>IMPORTANT fusion rule: the returned pose's rotation is the heading we were GIVEN. Never fuse it
 * back into the estimator (sigma-theta stays +Infinity for single-tag) -- fusing our own heading back
 * would be circular feedback, the exact 2026 bug this project exists to fix.
 */
public final class SingleTagTrigSolver {
  private SingleTagTrigSolver() {}

  /**
   * Recovers the camera-to-tag transform that produced a single-tag PnP pose observation.
   *
   * <p>The IO layer composes {@code fieldToRobot = fieldToTag * cameraToTag^-1 * robotToCamera^-1}
   * (see {@code VisionIOPhotonVision}); this inverts that composition, so no extra field had to be
   * added to the logged {@code PoseObservation}. Exact algebra, not an approximation.
   *
   * @param observedRobotPose the single-tag PnP robot pose from the IO layer
   * @param robotToCamera the (calibrated) robot-to-camera mount transform
   * @param tagPose the known field pose of the observed tag from the layout
   * @return the camera-to-tag transform embedded in the observation
   */
  public static Transform3d reconstructCameraToTag(
      Pose3d observedRobotPose, Transform3d robotToCamera, Pose3d tagPose) {
    Pose3d cameraPose = observedRobotPose.transformBy(robotToCamera);
    return new Transform3d(cameraPose, tagPose);
  }

  /**
   * Trig-solve: robot field XY from the known tag pose, the camera-to-tag translation, the camera
   * mount transform, and the known robot heading.
   *
   * <p>Chain: place a phantom robot at the field origin with the gyro heading (flat on the floor),
   * push the camera mount and the camera-to-tag transform through it -- the resulting translation is
   * the field-frame robot-to-tag vector. Subtracting it from the tag's known field position yields the
   * robot position. Only the <em>translation</em> of {@code cameraToTag} affects the result; its
   * (possibly ambiguity-flipped) rotation drops out -- which is the whole point.
   *
   * @param tagPose known field pose of the observed tag
   * @param robotToCamera the (calibrated) robot-to-camera mount transform
   * @param cameraToTag camera-frame transform to the tag (from PnP; only translation is used)
   * @param robotHeading field-relative robot heading at the frame timestamp (odometry buffer sample)
   * @return field-relative robot pose; rotation is {@code robotHeading} verbatim (do not fuse it)
   */
  public static Pose2d solve(
      Pose3d tagPose, Transform3d robotToCamera, Transform3d cameraToTag, Rotation2d robotHeading) {
    // Phantom robot at the origin, flat on the floor, with the known heading.
    Pose3d robotAtOrigin =
        new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, robotHeading.getRadians()));
    // Field-frame vector from robot center to the tag (position of the tag if the robot WERE at the
    // origin). cameraToTag's rotation is last in the chain, so it cannot influence this translation.
    Translation3d robotToTagField =
        robotAtOrigin.transformBy(robotToCamera).transformBy(cameraToTag).getTranslation();
    Translation3d robotPosition = tagPose.getTranslation().minus(robotToTagField);
    return new Pose2d(robotPosition.getX(), robotPosition.getY(), robotHeading);
  }
}
