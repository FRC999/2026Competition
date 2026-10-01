package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
class ShooterReadinessPolicyTest {
  @Test void smallMovingCorrectionsKeepHistoryButLargeStepsRearm() {
    assertFalse(ShooterReadinessPolicy.requiresNewWindow(2500, 2505, .1));
    assertTrue(ShooterReadinessPolicy.requiresNewWindow(2200, 3000, .1));
    assertTrue(ShooterReadinessPolicy.requiresNewWindow(0, 2200, .1));
  }
  @Test void readyFromPreviousTargetCannotPermitNewWrongSpeedOrStop() {
    assertFalse(ShooterReadinessPolicy.instantaneouslyReady(2200, 3000, .1));
    assertFalse(ShooterReadinessPolicy.instantaneouslyReady(Double.NaN, 2200, .1));
    assertFalse(ShooterReadinessPolicy.instantaneouslyReady(0, 0, .1));
    assertTrue(ShooterReadinessPolicy.instantaneouslyReady(2500, 2505, .1));
  }
}
