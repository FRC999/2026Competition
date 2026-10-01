package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class PrecisionModuleAngleHoldRequestTest {
  @Test
  void capturesMeasuredAnglesNotOldTargetsAndSetsEverySpeedToZero() {
    SwerveModuleState[] measured = {
        new SwerveModuleState(0.1, Rotation2d.fromDegrees(83.0)),
        new SwerveModuleState(-0.2, Rotation2d.fromDegrees(-21.0))
    };
    SwerveModuleState[] held = PrecisionModuleAngleHoldRequest.captureStoppedStates(measured);
    assertEquals(2, held.length);
    assertEquals(0.0, held[0].speedMetersPerSecond);
    assertEquals(0.0, held[1].speedMetersPerSecond);
    assertEquals(83.0, held[0].angle.getDegrees(), 1e-9);
    assertEquals(-21.0, held[1].angle.getDegrees(), 1e-9);
    assertNotSame(measured[0], held[0]);
    measured[0].angle = Rotation2d.fromDegrees(12.0);
    assertEquals(83.0, held[0].angle.getDegrees(), 1e-9);
  }
}
