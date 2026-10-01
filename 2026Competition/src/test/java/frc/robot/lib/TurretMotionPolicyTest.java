package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class TurretMotionPolicyTest {
  @Test void continuousTargetsDoNotWrapAndMatchMotorSign() {
    assertEquals(-110.0 / 360 * 11, TurretMotionPolicy.motorPositionTarget(180, -110, 110, 11, -1, true).orElseThrow(), 1e-9);
    assertEquals(110.0 / 360 * 11, TurretMotionPolicy.motorPositionTarget(-180, -110, 110, 11, -1, true).orElseThrow(), 1e-9);
    assertEquals(-0.05 / 360 * 11, TurretMotionPolicy.motorPositionTarget(0.05, -110, 110, 11, -1, true).orElseThrow(), 1e-9);
  }
  @Test void rejectsUntrustedAndNonfiniteCommands() {
    assertTrue(TurretMotionPolicy.motorPositionTarget(10, -110, 110, 11, -1, false).isEmpty());
    assertTrue(TurretMotionPolicy.motorPositionTarget(Double.NaN, -110, 110, 11, -1, true).isEmpty());
    assertTrue(TurretMotionPolicy.motorPositionTarget(10, 110, -110, 11, -1, true).isEmpty());
    assertTrue(TurretMotionPolicy.motorPositionTarget(10, -110, 110, 0, -1, true).isEmpty());
  }
  @Test void unreachableOrOvershotAnglesAreNeverReady() {
    assertFalse(TurretMotionPolicy.atTarget(110, 180, 2, -110, 110, true));
    assertFalse(TurretMotionPolicy.atTarget(125, 110, 2, -110, 110, true));
    assertFalse(TurretMotionPolicy.atTarget(0, 0, 2, -110, 110, false));
    assertTrue(TurretMotionPolicy.atTarget(99.5, 100, 1, -110, 110, true));
  }
}
