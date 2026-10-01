package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/**
 * Headless verification of the precision command's pure math. Pins the vector-clamp fix (per-axis
 * clamping would let a diagonal command exceed the configured max by up to sqrt(2)).
 */
class DriveToPosePrecisionMathTest {
  @Test
  void rotatingFinishSelectionUsesHandoffStateNotRouteName() {
    assertFalse(DriveToPosePrecisionCommand.requiresRotatingFinish(0.18, 0.70));
    assertFalse(DriveToPosePrecisionCommand.requiresRotatingFinish(0.64, 3.04));
    assertTrue(DriveToPosePrecisionCommand.requiresRotatingFinish(16.28, 42.63));
    assertFalse(DriveToPosePrecisionCommand.requiresRotatingFinish(2.5, 8.0));
    assertTrue(DriveToPosePrecisionCommand.requiresRotatingFinish(3.0, 0.0));
    assertTrue(DriveToPosePrecisionCommand.requiresRotatingFinish(0.0, 9.0));
    assertTrue(DriveToPosePrecisionCommand.requiresRotatingFinish(Double.NaN, 0.0));
    assertTrue(DriveToPosePrecisionCommand.requiresRotatingFinish(0.0, Double.POSITIVE_INFINITY));
  }

  @Test
  void finalMotionGateRejectsH4ExitEvenWhenHoldEntryGatePassed() {
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(0.16, 3.5));
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(0.02, 3.5));
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(0.16, 1.0));
    assertTrue(DriveToPosePrecisionCommand.isFinishMotionCalm(0.05, 1.5));
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(Double.NaN, 0.0));
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(0.0, Double.NaN));
    assertFalse(DriveToPosePrecisionCommand.isFinishMotionCalm(-0.01, 0.0));
  }

  @Test
  void successfulFinishRequiresFreshContinuousQualificationNotOldHoldAge() {
    var finish = new DriveToPosePrecisionCommand.SettleVelocityEscape(0.05);
    assertFalse(finish.update(true, true, 1.0));
    assertFalse(finish.update(true, false, 1.04));
    assertFalse(finish.update(true, true, 1.20));
    assertFalse(finish.update(true, true, 1.24));
    assertTrue(finish.update(true, true, 1.26));
    assertFalse(finish.update(true, false, 1.27));
    assertFalse(finish.update(true, true, 1.30));
    assertFalse(finish.update(false, true, 1.40));
    finish.reset();
    assertFalse(finish.isPending());
  }

  @Test
  void routeTranslationToleranceMustBeFinitePositiveAndInsideEscapeLimit() {
    assertEquals(0.02, DriveToPosePrecisionCommand.validateTranslationTolerance(0.02));
    assertEquals(0.04, DriveToPosePrecisionCommand.validateTranslationTolerance(0.04));
    for (double invalid : new double[] {0.0, -0.01, Double.NaN, Double.POSITIVE_INFINITY, 0.07}) {
      assertThrows(IllegalArgumentException.class,
          () -> DriveToPosePrecisionCommand.validateTranslationTolerance(invalid));
    }
  }

  @Test
  void persistentLossOfTightPoseResumesCorrectionButBriefNoiseDoesNot() {
    var confirmation = new DriveToPosePrecisionCommand.SettleVelocityEscape(0.20);
    assertFalse(confirmation.update(true, true, 1.0));
    assertFalse(confirmation.update(true, true, 1.19));
    assertFalse(confirmation.update(true, false, 1.195));
    assertFalse(confirmation.isPending());
    assertFalse(confirmation.update(true, true, 2.0));
    assertTrue(confirmation.update(true, true, 2.21));
    assertFalse(confirmation.update(false, true, 2.22));
    assertFalse(confirmation.isPending());
    assertFalse(confirmation.update(true, true, 3.0));
    confirmation.reset();
    assertFalse(confirmation.isPending());
  }

  @Test
  void agedHoldCannotFinishWithPendingMotionOrFailedCurrentQualification() {
    assertFalse(DriveToPosePrecisionCommand.canFinishHold(true, true, true, true));
    assertFalse(DriveToPosePrecisionCommand.canFinishHold(true, false, false, true));
    assertFalse(DriveToPosePrecisionCommand.canFinishHold(false, true, false, true));
    assertFalse(DriveToPosePrecisionCommand.canFinishHold(true, true, false, false));
    assertTrue(DriveToPosePrecisionCommand.canFinishHold(true, true, false, true));
  }

  @Test
  void briefSpeedExcursionStaysPendingThenClearsWithoutReleasingHold() {
    var escape = new DriveToPosePrecisionCommand.SettleVelocityEscape(0.08);
    assertFalse(escape.update(true, true, 1.0));
    assertTrue(escape.isPending());
    assertFalse(escape.update(true, true, 1.06));
    assertFalse(escape.update(true, false, 1.07));
    assertFalse(escape.isPending());
    assertEquals(0.0, escape.elapsedSeconds(1.07));
  }

  @Test
  void sustainedSpeedExcursionReleasesHoldUsingActualElapsedTime() {
    var escape = new DriveToPosePrecisionCommand.SettleVelocityEscape(0.08);
    assertFalse(escape.update(true, true, 1.0));
    assertTrue(escape.update(true, true, 1.09));
    assertEquals(0.09, escape.elapsedSeconds(1.09), 1e-9);
  }

  @Test
  void separatedSpikesDoNotAccumulateAndInactiveHoldClearsConfirmation() {
    var escape = new DriveToPosePrecisionCommand.SettleVelocityEscape(0.08);
    assertFalse(escape.update(true, true, 1.0));
    assertFalse(escape.update(true, false, 1.05));
    assertFalse(escape.update(true, true, 1.06));
    assertFalse(escape.update(true, true, 1.12));
    assertFalse(escape.update(false, true, 1.20));
    assertFalse(escape.isPending());
    assertFalse(escape.update(true, true, 1.21));
    escape.reset();
    assertFalse(escape.isPending());
  }

  @Test
  void clampLeavesSubMaxVectorUnchanged() {
    double[] r = DriveToPosePrecisionCommand.clampTranslationToMax(0.3, 0.4, 1.0); // norm 0.5 < 1.0
    assertEquals(0.3, r[0], 1e-9);
    assertEquals(0.4, r[1], 1e-9);
  }

  @Test
  void clampScalesOversizedVectorToMaxNorm() {
    double[] r = DriveToPosePrecisionCommand.clampTranslationToMax(3.0, 4.0, 2.5); // norm 5 -> 2.5
    assertEquals(2.5, Math.hypot(r[0], r[1]), 1e-9);
  }

  @Test
  void clampBoundsDiagonalThatPerAxisWouldLeak() {
    // Per-axis clamping to max=1 would leave (1,1) -> norm sqrt(2)=1.414. Vector clamp must bound to 1.
    double[] r = DriveToPosePrecisionCommand.clampTranslationToMax(1.0, 1.0, 1.0);
    assertEquals(1.0, Math.hypot(r[0], r[1]), 1e-9);
  }

  @Test
  void feedforwardFadeIsZeroAtAndInsideTargetRadius() {
    assertEquals(0.0, DriveToPosePrecisionCommand.feedforwardScaleForDistance(0.0, 0.04, 0.35), 1e-9);
    assertEquals(0.0, DriveToPosePrecisionCommand.feedforwardScaleForDistance(0.04, 0.04, 0.35), 1e-9);
  }

  @Test
  void feedforwardFadeIsLinearBetweenRadii() {
    assertEquals(0.5, DriveToPosePrecisionCommand.feedforwardScaleForDistance(0.195, 0.04, 0.35), 1e-9);
  }

  @Test
  void feedforwardFadeIsOneOutsideMaxRadius() {
    assertEquals(1.0, DriveToPosePrecisionCommand.feedforwardScaleForDistance(0.35, 0.04, 0.35), 1e-9);
    assertEquals(1.0, DriveToPosePrecisionCommand.feedforwardScaleForDistance(2.0, 0.04, 0.35), 1e-9);
  }

  @Test
  void velocityDampingOpposesMotionAndRespectsScale() {
    assertEquals(-0.225, DriveToPosePrecisionCommand.velocityDamping(1.0, 0.45, 0.5), 1e-9);
    assertEquals(0.225, DriveToPosePrecisionCommand.velocityDamping(-1.0, 0.45, 0.5), 1e-9);
    assertEquals(0.0, DriveToPosePrecisionCommand.velocityDamping(1.0, 0.45, 0.0), 1e-9);
  }

  @Test
  void settleEscapeIgnoresOrdinaryToleranceNoise() {
    assertFalse(DriveToPosePrecisionCommand.exceedsSettleEscapeTolerance(0.05, 2.0, 0.06, 2.5));
  }

  @Test
  void settleEscapeReleasesForLargeTranslationOrRotationError() {
    assertTrue(DriveToPosePrecisionCommand.exceedsSettleEscapeTolerance(0.061, 0.0, 0.06, 2.5));
    assertTrue(DriveToPosePrecisionCommand.exceedsSettleEscapeTolerance(0.0, 2.51, 0.06, 2.5));
  }

  @Test
  void settleVelocityEscapeIgnoresMotionInsideWiderEnvelope() {
    assertFalse(
        DriveToPosePrecisionCommand.exceedsSettleEscapeVelocityTolerance(
            0.18, 12.0, 0.18, 12.0));
  }

  @Test
  void settleVelocityEscapeReleasesForRenewedTranslationOrRotation() {
    assertTrue(
        DriveToPosePrecisionCommand.exceedsSettleEscapeVelocityTolerance(
            0.181, 0.0, 0.18, 12.0));
    assertTrue(
        DriveToPosePrecisionCommand.exceedsSettleEscapeVelocityTolerance(
            0.0, -12.01, 0.18, 12.0));
  }
}
