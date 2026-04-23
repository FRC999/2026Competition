package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class QuestHelpers {
  private QuestHelpers() {}

  public static Matrix<N3, N1> questStdDev(double chassisSpeedMps) {
    double translationStdDevMeters =
        MathUtil.clamp(0.05 + 0.02 * chassisSpeedMps, 0.05, 0.15);
    double yawStdDevRad =
        MathUtil.clamp(
            Units.degreesToRadians(1.0 + 0.5 * chassisSpeedMps),
            Units.degreesToRadians(0.7),
            Units.degreesToRadians(2.5));
    return VecBuilder.fill(translationStdDevMeters, translationStdDevMeters, yawStdDevRad);
  }
}
