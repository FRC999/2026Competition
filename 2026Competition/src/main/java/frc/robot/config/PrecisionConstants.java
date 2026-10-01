package frc.robot.config;

/** Ported from 2027Prototyping d20594a. Provisional on the 2026 chassis; validate physically. */
public final class PrecisionConstants {
  private PrecisionConstants() {}
  public static final double PRECISION_TRANSLATION_TOLERANCE_METERS = 0.04;
  public static final double PRECISION_ROTATION_TOLERANCE_DEGREES = 1.5;
  public static final double RELAXED_ROTATION_TOLERANCE_DEGREES = 1.8;
  public static final double PRECISION_SETTLE_SECONDS = 0.05;
  public static final double PRECISION_MAX_SPEED_METERS_PER_SECOND = 1.6;
  public static final double PRECISION_MAX_OMEGA_RADIANS_PER_SECOND = Math.toRadians(180.0);
  public static final double PRECISION_MAX_ACCEL_METERS_PER_SECOND_SQUARED = 2.5;
  public static final double PRECISION_MAX_ANGULAR_ACCEL_RAD_PER_SECOND_SQUARED = Math.toRadians(360.0);
  public static final double PRECISION_PROFILE_PERIOD_SECONDS = 0.020;
  public static final int PRECISION_MAX_PROFILE_STEPS_PER_EXECUTE = 5;
  public static final double PRECISION_DRIVE_KP = 3.2;
  public static final double PRECISION_DRIVE_KD = 0.0;
  public static final double PRECISION_THETA_KP = 4.5;
  public static final double PRECISION_THETA_KD = 0.0;
  public static final double PRECISION_TRANSLATION_FF_MIN_RADIUS_METERS =
        PRECISION_TRANSLATION_TOLERANCE_METERS;
  public static final double PRECISION_TRANSLATION_FF_MAX_RADIUS_METERS = 0.35;
  public static final double PRECISION_TRANSLATION_VELOCITY_DAMPING = 0.45;
  public static final double PRECISION_ROTATION_VELOCITY_DAMPING = 0.70;
  public static final double PRECISION_SETTLE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND = 0.12;
  public static final double PRECISION_SETTLE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND = 8.0;
  public static final double PRECISION_FINISH_MAX_ROTATION_SPEED_DEGREES_PER_SECOND = 1.5;
  public static final double PRECISION_FINISH_MAX_MODULE_SPEED_METERS_PER_SECOND = 0.05;
  public static final double PRECISION_SETTLE_ESCAPE_TRANSLATION_METERS = 0.06;
  public static final double PRECISION_SETTLE_ESCAPE_ROTATION_DEGREES = 2.5;
  public static final double
        PRECISION_SETTLE_ESCAPE_MAX_TRANSLATION_SPEED_METERS_PER_SECOND = 0.18;
  public static final double PRECISION_SETTLE_ESCAPE_MAX_ROTATION_SPEED_DEGREES_PER_SECOND = 12.0;
  public static final double PRECISION_SETTLE_VELOCITY_ESCAPE_CONFIRM_SECONDS = 0.08;
  public static final double PRECISION_SETTLE_POSE_REQUALIFICATION_SECONDS = 0.20;
  public static final double PRECISION_SAFETY_TIMEOUT_SECONDS = 4.0;
}
