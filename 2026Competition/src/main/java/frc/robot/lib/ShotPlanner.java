package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldTargets.AimTarget;
import frc.robot.Constants.OperatorConstants.AutoShoot;
import frc.robot.Constants.OperatorConstants.Turret;
import frc.robot.Constants.OperatorConstants.TurretGeometry;
import java.util.OptionalDouble;

/** Pure shot calculation shared by control and diagnostics. No hardware reads or state mutation. */
public final class ShotPlanner {
  public enum Mode { MOVING_AUTO, STATIC_HUB_BASE, STATIC_TOWER_BASE, MANUAL_FIXED,
    MANUAL_PRESET_2M, MANUAL_PRESET_3M, MANUAL_PRESET_4M }
  public record Solution(boolean valid, double yawFieldRad, double turretDegrees,
      double shooterRpmCommand, double hoodCommandAngleRad, double distanceMeters,
      String leadSource, double radialLeadSeconds, double lateralLeadSeconds,
      MovingAimModel.Aim movingAim) {
    public static Solution invalid(String reason) {
      return new Solution(false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
          reason, 0, 0, null);
    }
  }
  private final ShotTable hubTable;
  private final ShotTable passTable;
  private final ShotFlightTimeTable flightTimes;

  public ShotPlanner(ShotTable hubTable, ShotTable passTable, ShotFlightTimeTable flightTimes) {
    this.hubTable = hubTable; this.passTable = passTable; this.flightTimes = flightTimes;
  }

  public Solution solve(Mode mode, Pose2d pose, ChassisSpeeds speeds, Translation2d target,
      AimTarget targetKind, double preferredRpm, double throttle, double manualRpmTrim) {
    double distance = AimGeometry.pivot(pose).getDistance(target);
    double yaw = AimGeometry.fieldYaw(pose, target);
    double turret = AimGeometry.turretDegrees(pose, target);
    double rpm, hood;
    double radial = 0, lateral = 0;
    String source = "STATIC_PRESET";
    MovingAimModel.Aim aim = null;
    if (mode == Mode.STATIC_HUB_BASE) {
      rpm = AutoShoot.STATIC_HUB_BASE_RPM; hood = Math.toRadians(AutoShoot.STATIC_HUB_BASE_HOOD_DEG);
    } else if (mode == Mode.STATIC_TOWER_BASE) {
      rpm = AutoShoot.STATIC_TOWER_BASE_RPM; hood = Math.toRadians(AutoShoot.STATIC_TOWER_BASE_HOOD_DEG);
    } else if (mode == Mode.MANUAL_FIXED) {
      rpm = (AutoShoot.MANUAL_FIXED_SHOT_BASE_RPM
          + MathUtil.clamp(throttle, -1, 1) * AutoShoot.MANUAL_FIXED_SHOT_RPM_TRIM_RANGE) * (1 + manualRpmTrim);
      hood = Math.toRadians(AutoShoot.MANUAL_FIXED_SHOT_HOOD_DEG);
    } else {
      ShotTable table = targetKind == AimTarget.HUB ? hubTable : passTable;
      if (mode == Mode.MOVING_AUTO) {
        OptionalDouble measured = targetKind == AimTarget.HUB ? flightTimes.atDistance(distance) : OptionalDouble.empty();
        radial = measured.orElse(AutoShoot.MOVING_DISTANCE_LOOKUP_LEAD_SEC);
        lateral = measured.orElse(AutoShoot.MOVING_AIM_LATERAL_LEAD_SEC);
        source = measured.isPresent() ? "MEASURED_FLIGHT_TIME" : "LEGACY_EMPIRICAL_LEAD";
        var predicted = MovingAimModel.predict(pose, speeds,
            TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS, target,
            AutoShoot.DT_RELEASE_SEC, radial, lateral, Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG);
        if (predicted.isEmpty()) return Solution.invalid("INVALID_MOTION_PREDICTION");
        aim = predicted.get(); yaw = aim.fieldYawRadians(); turret = aim.turretDegrees();
        distance = aim.effectiveDistanceMeters();
      } else {
        distance = switch (mode) {
          case MANUAL_PRESET_2M -> 2; case MANUAL_PRESET_3M -> 3; case MANUAL_PRESET_4M -> 4;
          default -> Double.NaN;
        };
        source = "MEASURED_DISTANCE_PRESET";
      }
      var setting = table.findInterpolatedShot(distance, turret, preferredRpm);
      if (!setting.valid) return Solution.invalid(targetKind == AimTarget.HUB
          ? "OUTSIDE_MEASURED_HUB_TABLE" : "NO_MEASURED_PASS_SOLUTION");
      rpm = setting.shooterRpmCommand * (mode == Mode.MOVING_AUTO ? 1 : 1 + manualRpmTrim);
      hood = setting.hoodCommandAngleRad;
    }
    if (!Double.isFinite(yaw) || !Double.isFinite(turret) || !Double.isFinite(rpm) || rpm <= 0
        || !Double.isFinite(hood)) return Solution.invalid("NONFINITE_OR_NONPOSITIVE_SOLUTION");
    return new Solution(true, yaw, turret + Turret.AUTO_AIM_TRIM_DEG, rpm, hood, distance,
        source, radial, lateral, aim);
  }
}
