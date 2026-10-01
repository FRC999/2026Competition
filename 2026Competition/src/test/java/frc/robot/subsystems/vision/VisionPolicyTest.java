package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.config.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionPolicy.RejectionReason;
import org.junit.jupiter.api.Test;

/**
 * Headless verification of the vision fusion policy (the rejection gates and both covariance models).
 * These encode this project's whole thesis -- the measurement discipline our 2026 robot got wrong -- so
 * they are worth pinning with tests. The policy lives in the pure {@link VisionPolicy} (extracted
 * 2026-07-16) so this suite can cover it completely without HAL dependencies.
 */
class VisionPolicyTest {
  private static PoseObservation obs(double x, double y, double z, double ambiguity, int tagCount, double dist) {
    return new PoseObservation(
        0.0, new Pose3d(x, y, z, new Rotation3d()), ambiguity, tagCount, dist,
        1);
  }

  @Test
  void acceptsGoodMultiTagPose() {
    assertEquals(RejectionReason.ACCEPTED, VisionPolicy.rejectionReason(obs(4.0, 2.0, 0.0, 0.0, 2, 2.0)));
  }

  @Test
  void rejectsNoTags() {
    assertEquals(RejectionReason.NO_TAGS, VisionPolicy.rejectionReason(obs(4.0, 2.0, 0.0, 0.0, 0, 2.0)));
  }

  @Test
  void rejectsNonFinite() {
    assertEquals(
        RejectionReason.NON_FINITE,
        VisionPolicy.rejectionReason(obs(Double.NaN, 2.0, 0.0, 0.0, 2, 2.0)));
  }

  @Test
  void rejectsNonFiniteDistance() {
    // A NaN average tag distance would make standardDeviations emit NaN std devs into CTRE.
    assertEquals(
        RejectionReason.NON_FINITE, VisionPolicy.rejectionReason(obs(4.0, 2.0, 0.0, 0.0, 2, Double.NaN)));
  }

  @Test
  void rejectsNonFiniteAmbiguity() {
    // A NaN ambiguity would silently pass the "> MAX_SINGLE_TAG_AMBIGUITY" gate (NaN comparisons false).
    assertEquals(
        RejectionReason.NON_FINITE, VisionPolicy.rejectionReason(obs(4.0, 2.0, 0.0, Double.NaN, 1, 2.0)));
  }

  @Test
  void rejectsImpossibleZ() {
    assertEquals(
        RejectionReason.BAD_Z,
        VisionPolicy.rejectionReason(obs(4.0, 2.0, VisionConstants.MAX_ACCEPTED_Z_METERS + 0.5, 0.0, 2, 2.0)));
  }

  @Test
  void rejectsOutsideField() {
    assertEquals(
        RejectionReason.OUTSIDE_FIELD, VisionPolicy.rejectionReason(obs(-5.0, 2.0, 0.0, 0.0, 2, 2.0)));
  }

  @Test
  void rejectsTooFar() {
    assertEquals(
        RejectionReason.TOO_FAR,
        VisionPolicy.rejectionReason(
            obs(4.0, 2.0, 0.0, 0.0, 2, VisionConstants.MAX_AVERAGE_TAG_DISTANCE_METERS + 1.0)));
  }

  @Test
  void rejectsAmbiguousSingleTag() {
    assertEquals(
        RejectionReason.SINGLE_TAG_AMBIGUOUS,
        VisionPolicy.rejectionReason(
            obs(4.0, 2.0, 0.0, VisionConstants.MAX_SINGLE_TAG_AMBIGUITY + 0.1, 1, 2.0)));
  }

  @Test
  void rejectsSingleTagWithUnknownAmbiguity() {
    // PhotonVision reports -1 when ambiguity is uncomputable; a single-tag frame we can't verify must
    // not pass the gate ( -1 > threshold is false, so this used to slip through).
    assertEquals(
        RejectionReason.SINGLE_TAG_AMBIGUOUS, VisionPolicy.rejectionReason(obs(4.0, 2.0, 0.0, -1.0, 1, 2.0)));
  }

  @Test
  void singleTagHeadingIsNeverTrusted() {
    // tagCount 1 -> trustRotation false -> theta std dev must be +Infinity (6328 discipline).
    var stdDevs = VisionPolicy.standardDeviations(0, 2.0, 1, false);
    assertTrue(Double.isInfinite(stdDevs.get(2, 0)), "single-tag theta std dev must be infinite");
    assertTrue(Double.isFinite(stdDevs.get(0, 0)), "single-tag xy std dev must stay finite");
  }

  @Test
  void covarianceScalesWithDistanceSquaredOverTagCountSquared() {
    // dist=2, tags=2 -> factor 4 / (2*2) = 1.0 (tag count squared, 6328/6995 convention).
    var stdDevs = VisionPolicy.standardDeviations(0, 2.0, 2, true);
    assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE * 1.0, stdDevs.get(0, 0), 1e-9);
    assertEquals(
        VisionConstants.ANGULAR_STD_DEV_BASELINE
            * VisionConstants.CAMERA_ANGULAR_STD_DEV_FACTORS[0],
        stdDevs.get(2, 0),
        1e-9);
  }

  @Test
  void moreTagsMeansTighterCovariance() {
    double twoTag = VisionPolicy.standardDeviations(0, 3.0, 2, true).get(0, 0);
    double threeTag = VisionPolicy.standardDeviations(0, 3.0, 3, true).get(0, 0);
    assertTrue(threeTag < twoTag, "more tags should reduce the std dev");
  }

  @Test
  void unknownCameraIndexUsesUnityFactor() {
    // An out-of-roster camera index must degrade to factor 1.0, not crash or zero the covariance.
    assertEquals(1.0, VisionPolicy.cameraFactor(99), 1e-9);
    assertEquals(1.0, VisionPolicy.cameraFactor(-1), 1e-9);
    assertEquals(VisionConstants.CAMERA_STD_DEV_FACTORS[0], VisionPolicy.cameraFactor(0), 1e-9);
    assertEquals(1.0, VisionPolicy.angularCameraFactor(99), 1e-9);
    assertEquals(
        VisionConstants.CAMERA_ANGULAR_STD_DEV_FACTORS[0],
        VisionPolicy.angularCameraFactor(0),
        1e-9);
  }

  @Test
  void rotationTrustIsExplicitPerCameraAndUnknownCameraIsUntrusted() {
    assertEquals(
        VisionConstants.CAMERA_ROTATION_TRUST_ENABLED[0],
        VisionPolicy.cameraRotationTrustEnabled(0));
    assertFalse(VisionPolicy.cameraRotationTrustEnabled(-1));
    assertFalse(VisionPolicy.cameraRotationTrustEnabled(99));
  }

  @Test
  void enabledRotationFusionIsIndependentFromDisabledManualSeedEligibility() {
    assertFalse(VisionPolicy.shouldFuseRotation(0, 1, false));
    assertTrue(VisionPolicy.shouldFuseRotation(0, 2, false));
    assertEquals(
        VisionConstants.FUSE_VISION_ROTATION_WHILE_ENABLED,
        VisionPolicy.shouldFuseRotation(0, 2, true));
    assertEquals(VisionConstants.CAMERA_ROTATION_TRUST_ENABLED[1], VisionPolicy.shouldFuseRotation(1, 2, false));
    assertFalse(VisionPolicy.shouldFuseRotation(99, 2, false));
  }

  // ---------------------------------------------------------------------------------------------
  // Anisotropic covariance (5940-style, 2026-07-16). sigma = C * d^E parallel/perpendicular to the
  // camera->tag ray, rotated into field axes by the ray angle alpha.
  // ---------------------------------------------------------------------------------------------

  @Test
  void powerLawSigmaMatchesDefinition() {
    assertEquals(0.06 * 4.0, VisionPolicy.powerLawSigma(2.0, 0.06, 2.0), 1e-9);
    assertEquals(0.05, VisionPolicy.powerLawSigma(1.0, 0.05, 3.7), 1e-9); // d=1 -> coeff itself
  }

  @Test
  void anisotropicRayAlongXPutsParallelSigmaOnX() {
    // Ray angle 0 (tag straight down +X): X gets the (larger) parallel sigma, Y the perpendicular one.
    var std = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 1, false, 0.0);
    double expectedPar =
        VisionPolicy.powerLawSigma(2.0, VisionConstants.ANISO_SINGLE_TAG_PARALLEL_COEFF,
            VisionConstants.ANISO_SINGLE_TAG_PARALLEL_EXP);
    double expectedPerp =
        VisionPolicy.powerLawSigma(2.0, VisionConstants.ANISO_SINGLE_TAG_PERP_COEFF,
            VisionConstants.ANISO_SINGLE_TAG_PERP_EXP);
    assertEquals(expectedPar, std.get(0, 0), 1e-9);
    assertEquals(expectedPerp, std.get(1, 0), 1e-9);
    assertTrue(std.get(0, 0) > std.get(1, 0), "range (parallel) error must exceed bearing error");
  }

  @Test
  void anisotropicRayAlongYSwapsAxes() {
    // Ray angle 90 deg: the parallel (range) sigma now lands on Y.
    var std = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 1, false, Math.PI / 2.0);
    assertTrue(std.get(1, 0) > std.get(0, 0), "at 90 deg the larger sigma must be on Y");
  }

  @Test
  void anisotropicAt45DegreesMixesEqually()  {
    // cos^2 = sin^2 = 0.5 -> X and Y variances identical.
    var std = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 1, false, Math.PI / 4.0);
    assertEquals(std.get(0, 0), std.get(1, 0), 1e-9);
  }

  @Test
  void anisotropicMultiTagTighterThanSingleTag() {
    double single = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 1, false, 0.0).get(0, 0);
    double multi = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 2, true, 0.0).get(0, 0);
    assertTrue(multi < single, "multi-tag anisotropic sigma must be tighter");
  }

  @Test
  void anisotropicSingleTagHeadingStillNeverTrusted() {
    // The new covariance model must not weaken the core discipline: one tag -> infinite theta.
    var std = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 1, false, 0.7);
    assertTrue(Double.isInfinite(std.get(2, 0)), "single-tag theta must stay infinite (anisotropic too)");
  }

  @Test
  void anisotropicMultiTagThetaUsesBaselineModel() {
    // Theta intentionally reuses the validated isotropic model (no unfitted second power law).
    var std = VisionPolicy.anisotropicStandardDeviations(0, 2.0, 2, true, 0.0);
    assertEquals(
        VisionConstants.ANGULAR_STD_DEV_BASELINE
            * VisionConstants.CAMERA_ANGULAR_STD_DEV_FACTORS[0],
        std.get(2, 0),
        1e-9);
  }

  @Test
  void anisotropicGrowsWithDistance() {
    double near = VisionPolicy.anisotropicStandardDeviations(0, 1.0, 1, false, 0.0).get(0, 0);
    double far = VisionPolicy.anisotropicStandardDeviations(0, 4.0, 1, false, 0.0).get(0, 0);
    assertTrue(far > near, "sigma must grow with distance");
  }

  // ---------------------------------------------------------------------------------------------
  // Timing / gating rules (unchanged behavior, now on VisionPolicy).
  // ---------------------------------------------------------------------------------------------

  @Test
  void teleopAlwaysFusesVision() {
    // Not autonomous -> always accept, regardless of the timer value.
    assertTrue(VisionPolicy.shouldAcceptDuringAuto(false, 0.0));
    assertTrue(VisionPolicy.shouldAcceptDuringAuto(false, 100.0));
  }

  @Test
  void earlyAutoSuppressesThenResumes() {
    double ignore = VisionConstants.AUTO_VISION_IGNORE_SECONDS;
    assertTrue(!VisionPolicy.shouldAcceptDuringAuto(true, ignore - 0.05), "early auto must suppress fusion");
    assertTrue(VisionPolicy.shouldAcceptDuringAuto(true, ignore + 0.05), "fusion resumes after the window");
  }

  @Test
  void suppressesVisionFrameFromBeforeReset() {
    // A frame captured (t=1.0) before the reset (t=1.5) still sees the old pose -> suppress it.
    assertTrue(VisionPolicy.isPreResetFrame(1.0, 1.5), "pre-reset frame must be suppressed");
    // A frame captured after the reset -> fuse it.
    assertTrue(!VisionPolicy.isPreResetFrame(2.0, 1.5), "post-reset frame must be fused");
  }

  @Test
  void resetQuarantineSuppressesRecentPostResetFrames() {
    // Frame timestamped just AFTER the reset but within the quarantine window -> still suppressed
    // (catches queued/latency-delayed frames whose timestamp slipped past the reset).
    assertTrue(
        VisionPolicy.isResetSuppressed(100.05, 100.0, 100.1, 0.35), "post-reset frame in quarantine suppressed");
  }

  @Test
  void resetQuarantineEndsAfterWindow() {
    // Fresh frame well past the quarantine window -> fused.
    assertTrue(
        !VisionPolicy.isResetSuppressed(100.4, 100.0, 100.5, 0.35), "frame past the quarantine must be fused");
  }

  @Test
  void targetXEmptyWhenNoTarget() {
    var obs = new VisionIO.TargetObservation(Rotation2d.fromDegrees(10), Rotation2d.kZero, false, 5.0);
    assertTrue(VisionPolicy.freshTargetX(obs, 5.0).isEmpty(), "no target -> empty");
  }

  @Test
  void targetXPresentWhenFresh() {
    var obs = new VisionIO.TargetObservation(Rotation2d.fromDegrees(10), Rotation2d.kZero, true, 5.0);
    var r = VisionPolicy.freshTargetX(obs, 5.05); // 0.05 s old, within the staleness window
    assertTrue(r.isPresent());
    assertEquals(10.0, r.get().getDegrees(), 1e-6);
  }

  @Test
  void targetXEmptyWhenStale() {
    var obs = new VisionIO.TargetObservation(Rotation2d.fromDegrees(10), Rotation2d.kZero, true, 5.0);
    double stale = 5.0 + VisionConstants.TARGET_OBSERVATION_MAX_STALENESS_SECONDS + 0.1;
    assertTrue(VisionPolicy.freshTargetX(obs, stale).isEmpty(), "stale bearing must be rejected");
  }

  @Test
  void targetXEmptyWhenFutureDated() {
    // A frame timestamped in the future (clock glitch) must also be rejected, not treated as fresh.
    var obs = new VisionIO.TargetObservation(Rotation2d.fromDegrees(10), Rotation2d.kZero, true, 5.0);
    double now = 5.0 - VisionConstants.TARGET_OBSERVATION_MAX_STALENESS_SECONDS - 0.1;
    assertTrue(VisionPolicy.freshTargetX(obs, now).isEmpty(), "future-dated bearing must be rejected");
  }
}
