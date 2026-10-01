package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.config.VisionConstants;
import org.junit.jupiter.api.Test;

/**
 * Headless verification of the trig-solve single-tag strategy (2026-07-16; idea: 6328 via PhotonVision
 * {@code PNP_DISTANCE_TRIG_SOLVE}, run in production by 1678).
 *
 * <p>Method: build synthetic ground truth (a robot pose, a camera mount, a tag), derive the exact
 * camera-to-tag transform a perfect PnP would produce, then check the solver recovers the ground-truth
 * pose -- including when the PnP <em>rotation</em> is deliberately corrupted, which is the entire value
 * proposition (ambiguity flips rotation; the trig solve must not care).
 */
class SingleTagTrigSolverTest {
  private static final double EPS = 1e-9;

  /** The tag from the real layout, so tests exercise the actual board geometry. */
  private static final Pose3d TAG =
      VisionConstants.FIELD_LAYOUT.getTagPose(1).orElseThrow();

  /** Exact camera-to-tag transform for a ground-truth robot pose and camera mount. */
  private static Transform3d groundTruthCameraToTag(Pose2d robotPose, Transform3d robotToCamera) {
    Pose3d cameraPose = new Pose3d(robotPose).transformBy(robotToCamera);
    return new Transform3d(cameraPose, TAG);
  }

  @Test
  void recoversPoseWithIdentityCameraMount() {
    // Simplest geometry: camera at robot center, robot facing the tag straight on.
    Pose2d truth = new Pose2d(4.0, 1.75, Rotation2d.kZero);
    Transform3d identity = new Transform3d();
    Transform3d cameraToTag = groundTruthCameraToTag(truth, identity);

    Pose2d solved = SingleTagTrigSolver.solve(TAG, identity, cameraToTag, truth.getRotation());
    assertEquals(truth.getX(), solved.getX(), EPS);
    assertEquals(truth.getY(), solved.getY(), EPS);
    assertEquals(0.0, solved.getRotation().getRadians(), EPS);
  }

  @Test
  void recoversPoseWithRealCameraMountAndRotatedRobot() {
    // Real front-left mount (offset, pitched, yawed) and a robot heading of 25 degrees.
    Pose2d truth = new Pose2d(3.2, 2.4, Rotation2d.fromDegrees(25.0));
    Transform3d mount = new Transform3d(0.3, 0.2, 0.4, new Rotation3d(0.02, -0.3, 2.8));
    Transform3d cameraToTag = groundTruthCameraToTag(truth, mount);

    Pose2d solved = SingleTagTrigSolver.solve(TAG, mount, cameraToTag, truth.getRotation());
    assertEquals(truth.getX(), solved.getX(), EPS);
    assertEquals(truth.getY(), solved.getY(), EPS);
    assertEquals(truth.getRotation().getRadians(), solved.getRotation().getRadians(), EPS);
  }

  @Test
  void corruptedPnpRotationDoesNotMoveTheSolution() {
    // THE value proposition: flip/corrupt the PnP rotation (what single-tag ambiguity does) and the
    // trig-solved position must not change, because only the camera-to-tag TRANSLATION is used.
    Pose2d truth = new Pose2d(3.2, 2.4, Rotation2d.fromDegrees(25.0));
    Transform3d mount = new Transform3d(0.3, 0.2, 0.4, new Rotation3d(0.02, -0.3, 2.8));
    Transform3d clean = groundTruthCameraToTag(truth, mount);
    Transform3d corrupted =
        new Transform3d(clean.getTranslation(), new Rotation3d(0.4, -0.9, 2.2)); // garbage rotation

    Pose2d solvedClean = SingleTagTrigSolver.solve(TAG, mount, clean, truth.getRotation());
    Pose2d solvedCorrupted = SingleTagTrigSolver.solve(TAG, mount, corrupted, truth.getRotation());
    assertEquals(solvedClean.getX(), solvedCorrupted.getX(), EPS);
    assertEquals(solvedClean.getY(), solvedCorrupted.getY(), EPS);
  }

  @Test
  void headingErrorShiftsPoseButBoundedByGeometry() {
    // A wrong heading rotates the robot->tag vector, so the solved pose moves -- the error magnitude is
    // bounded by (heading error in radians) * (distance to tag). Sanity-pin that behavior so nobody
    // mistakes the trig solve for something that can fix a bad gyro.
    Pose2d truth = new Pose2d(4.0, 1.75, Rotation2d.kZero);
    Transform3d identity = new Transform3d();
    Transform3d cameraToTag = groundTruthCameraToTag(truth, identity);
    double distance = cameraToTag.getTranslation().getNorm();

    double headingErrorRad = Math.toRadians(2.0);
    Pose2d solved =
        SingleTagTrigSolver.solve(TAG, identity, cameraToTag, new Rotation2d(headingErrorRad));
    double positionError = solved.getTranslation().getDistance(truth.getTranslation());
    // Small-angle bound: error <= 2 * sin(err/2) * d ~= err * d, with a little slack.
    assertEquals(0.0, positionError, headingErrorRad * distance * 1.01);
  }

  @Test
  void reconstructCameraToTagInvertsTheIoComposition() {
    // The IO layer builds fieldToRobot = fieldToTag * cameraToTag^-1 * robotToCamera^-1. Feed a known
    // cameraToTag through that composition and verify reconstruction returns it exactly.
    Transform3d mount = new Transform3d(-0.3, -0.2, 0.4, new Rotation3d(-0.03, -0.3, -2.8));
    Pose2d truth = new Pose2d(2.8, 1.5, Rotation2d.fromDegrees(-15.0));
    Transform3d original = groundTruthCameraToTag(truth, mount);

    // Recreate what VisionIOPhotonVision does with a real PnP result:
    Transform3d fieldToTarget = new Transform3d(TAG.getTranslation(), TAG.getRotation());
    Transform3d fieldToCamera = fieldToTarget.plus(original.inverse());
    Transform3d fieldToRobot = fieldToCamera.plus(mount.inverse());
    Pose3d observedPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

    Transform3d reconstructed = SingleTagTrigSolver.reconstructCameraToTag(observedPose, mount, TAG);
    assertEquals(original.getX(), reconstructed.getX(), EPS);
    assertEquals(original.getY(), reconstructed.getY(), EPS);
    assertEquals(original.getZ(), reconstructed.getZ(), EPS);
    assertEquals(
        0.0, original.getRotation().minus(reconstructed.getRotation()).getAngle(), EPS);
  }

  @Test
  void endToEndReconstructionPlusSolveRecoversTruth() {
    // Full pipeline as Vision.trigSolve runs it: observation pose -> reconstruct -> solve.
    Transform3d mount = new Transform3d(0.3, 0.2, 0.4, new Rotation3d(0.02, -0.3, 2.8));
    Pose2d truth = new Pose2d(4.4, 2.1, Rotation2d.fromDegrees(10.0));
    Pose3d observedPose = new Pose3d(truth); // perfect PnP would report the truth

    Transform3d cameraToTag = SingleTagTrigSolver.reconstructCameraToTag(observedPose, mount, TAG);
    Pose2d solved = SingleTagTrigSolver.solve(TAG, mount, cameraToTag, truth.getRotation());
    assertEquals(truth.getX(), solved.getX(), EPS);
    assertEquals(truth.getY(), solved.getY(), EPS);
  }
}
