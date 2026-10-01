package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class PoseJitterAccumulatorTest {
  @Test
  void rejectsNonPositiveTarget() {
    assertThrows(IllegalArgumentException.class, () -> new PoseJitterAccumulator(0));
  }

  @Test
  void constantPoseHasZeroJitter() {
    PoseJitterAccumulator accumulator = new PoseJitterAccumulator(3);
    Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(15.0));
    accumulator.add(pose);
    accumulator.add(pose);
    accumulator.add(pose);

    assertTrue(accumulator.isReady());
    assertEquals(3, accumulator.getCount());
    assertEquals(0.0, accumulator.getStdDevTranslation(), 1e-12);
    assertEquals(0.0, accumulator.getStdDevYawDegrees(), 1e-12);
    assertEquals(0.0, accumulator.getPeakToPeakX(), 1e-12);
    assertEquals(0.0, accumulator.getPeakToPeakY(), 1e-12);
    assertEquals(0.0, accumulator.getPeakToPeakYawDegrees(), 1e-12);
    assertEquals(pose, accumulator.getMeanPose());
  }

  @Test
  void computesPopulationStatisticsAndFreezesAtTargetCount() {
    PoseJitterAccumulator accumulator = new PoseJitterAccumulator(2);
    accumulator.add(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(10.0)));
    accumulator.add(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(12.0)));
    accumulator.add(new Pose2d(100.0, 100.0, Rotation2d.fromDegrees(100.0)));

    assertEquals(2, accumulator.getCount());
    assertEquals(2, accumulator.getTargetSamples());
    assertEquals(2.0, accumulator.getMeanPose().getX(), 1e-12);
    assertEquals(3.0, accumulator.getMeanPose().getY(), 1e-12);
    assertEquals(11.0, accumulator.getMeanPose().getRotation().getDegrees(), 1e-12);
    assertEquals(1.0, accumulator.getStdDevX(), 1e-12);
    assertEquals(1.0, accumulator.getStdDevY(), 1e-12);
    assertEquals(Math.sqrt(2.0), accumulator.getStdDevTranslation(), 1e-12);
    assertEquals(1.0, accumulator.getStdDevYawDegrees(), 1e-12);
    assertEquals(2.0, accumulator.getPeakToPeakX(), 1e-12);
    assertEquals(2.0, accumulator.getPeakToPeakY(), 1e-12);
    assertEquals(2.0, accumulator.getPeakToPeakYawDegrees(), 1e-12);
  }

  @Test
  void unwrapsYawAcrossPlusMinus180Degrees() {
    PoseJitterAccumulator accumulator = new PoseJitterAccumulator(2);
    accumulator.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(179.0)));
    accumulator.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.0)));

    assertEquals(
        180.0, Math.abs(accumulator.getMeanPose().getRotation().getDegrees()), 1e-9);
    assertEquals(1.0, accumulator.getStdDevYawDegrees(), 1e-9);
  }

  @Test
  void resetClearsFrozenCapture() {
    PoseJitterAccumulator accumulator = new PoseJitterAccumulator(1);
    accumulator.add(new Pose2d(1.0, 1.0, Rotation2d.kZero));
    assertTrue(accumulator.isReady());

    accumulator.reset();
    assertFalse(accumulator.isReady());
    assertEquals(0, accumulator.getCount());
    assertEquals(new Pose2d(), accumulator.getMeanPose());
  }
}
