package frc.robot.lib;

import java.awt.geom.Point2D;
import java.util.List;
import java.util.Objects;

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

  /**
   * Estimate the center of the best-fit circle for points given as Double[2] =
   * {x, y}. Requires at least 3 points.
   *
   * Model: x^2 + y^2 + a x + b y + c = 0 -> center = (-a/2, -b/2)
   */
  public static Point2D.Double estimateCircleCenter(List<Double[]> points) {
    if (points == null || points.size() < 3) {
      throw new IllegalArgumentException("Need at least 3 (x,y) points.");
    }

    double sxx = 0.0;
    double sxy = 0.0;
    double sx = 0.0;
    double syy = 0.0;
    double sy = 0.0;
    double n = 0.0;
    double sx2y2 = 0.0;
    double sx_x2y2 = 0.0;
    double sy_x2y2 = 0.0;

    for (Double[] p : points) {
      if (p == null || p.length != 2) {
        throw new IllegalArgumentException("Each element must be a Double[2] {x, y}.");
      }
      double x = requireFinite(p[0], "x");
      double y = requireFinite(p[1], "y");
      double r2 = x * x + y * y;

      sxx += x * x;
      sxy += x * y;
      syy += y * y;
      sx += x;
      sy += y;
      n += 1.0;

      sx2y2 += -r2;
      sx_x2y2 += -r2 * x;
      sy_x2y2 += -r2 * y;
    }

    double[][] m = {
        { sxx, sxy, sx },
        { sxy, syy, sy },
        { sx, sy, n }
    };
    double[] v = { sx_x2y2, sy_x2y2, sx2y2 };

    double[] theta = solve3x3(m, v);
    return new Point2D.Double(-0.5 * theta[0], -0.5 * theta[1]);
  }

  private static double requireFinite(Double d, String name) {
    Objects.requireNonNull(d, name + " is null");
    if (!Double.isFinite(d)) {
      throw new IllegalArgumentException(name + " must be finite");
    }
    return d;
  }

  private static double[] solve3x3(double[][] m, double[] v) {
    double[][] a = new double[3][4];
    for (int i = 0; i < 3; i++) {
      a[i][0] = m[i][0];
      a[i][1] = m[i][1];
      a[i][2] = m[i][2];
      a[i][3] = v[i];
    }

    for (int col = 0; col < 3; col++) {
      int pivot = col;
      double maxAbs = Math.abs(a[col][col]);
      for (int r = col + 1; r < 3; r++) {
        double val = Math.abs(a[r][col]);
        if (val > maxAbs) {
          maxAbs = val;
          pivot = r;
        }
      }

      if (maxAbs < 1e-12) {
        throw new IllegalArgumentException("Degenerate/collinear points: cannot fit a unique circle.");
      }

      if (pivot != col) {
        double[] tmp = a[col];
        a[col] = a[pivot];
        a[pivot] = tmp;
      }

      double diag = a[col][col];
      for (int c = col; c < 4; c++) {
        a[col][c] /= diag;
      }

      for (int r = 0; r < 3; r++) {
        if (r == col) {
          continue;
        }
        double f = a[r][col];
        if (f != 0.0) {
          for (int c = col; c < 4; c++) {
            a[r][c] -= f * a[col][c];
          }
        }
      }
    }

    return new double[] { a[0][3], a[1][3], a[2][3] };
  }
}
