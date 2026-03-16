// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.OperatorConstants.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Field targets (meters in WPILib field coordinates).
   *
   * You indicated you'll provide these experimentally (HUB_BLUE_X/Y,
   * HUB_RED_X/Y).
   */
    public static final class FieldTargets {
    public static final double HUB_BLUE_X = 4.611624;
    public static final double HUB_BLUE_Y = 4.021328;
    public static final double HUB_RED_X = 11.90142;
    public static final double HUB_RED_Y = 4.021328;

    public static final double NEUTRAL_LOW_BLUE_X = 0.0; // TODO set
    public static final double NEUTRAL_LOW_BLUE_Y = 0.0; // TODO set
    public static final double NEUTRAL_LOW_RED_X = 0.0; // TODO set
    public static final double NEUTRAL_LOW_RED_Y = 0.0; // TODO set

    public static final double NEUTRAL_HIGH_BLUE_X = 0.0; // TODO set
    public static final double NEUTRAL_HIGH_BLUE_Y = 0.0; // TODO set
    public static final double NEUTRAL_HIGH_RED_X = 0.0; // TODO set
    public static final double NEUTRAL_HIGH_RED_Y = 0.0; // TODO set

    /**
     * Zone selection is evaluated in BLUE-frame coordinates.
     * For RED alliance, the robot X is mirrored using FIELD_LENGTH_METERS.
     */
    public static final double ALLIANCE_ZONE_MAX_X_BLUE_FRAME_METERS = 0.0; // TODO set
    public static final double NEUTRAL_ZONE_Y_SPLIT_METERS = 0.0; // TODO set

    public enum AimTarget {
      HUB(HUB_BLUE_X, HUB_BLUE_Y, HUB_RED_X, HUB_RED_Y),
      NEUTRAL_LOW(NEUTRAL_LOW_BLUE_X, NEUTRAL_LOW_BLUE_Y, NEUTRAL_LOW_RED_X, NEUTRAL_LOW_RED_Y),
      NEUTRAL_HIGH(NEUTRAL_HIGH_BLUE_X, NEUTRAL_HIGH_BLUE_Y, NEUTRAL_HIGH_RED_X, NEUTRAL_HIGH_RED_Y);

      private final double blueX;
      private final double blueY;
      private final double redX;
      private final double redY;

      AimTarget(double blueX, double blueY, double redX, double redY) {
        this.blueX = blueX;
        this.blueY = blueY;
        this.redX = redX;
        this.redY = redY;
      }

      public double getX(boolean isRed) {
        return isRed ? redX : blueX;
      }

      public double getY(boolean isRed) {
        return isRed ? redY : blueY;
      }
    }
  }

  public static final class EnabledSubsystems {

    public static final boolean chasis = true;
    public static final boolean odometry = true;
    public static final boolean ll = true;
    public static final boolean questnav = true;
    public static final boolean intake = true;
    public static final boolean shooter = true;
    public static final boolean turret = true;
    public static final boolean hood = true;
    public static final boolean hopper = true;
    public static final boolean spindexer = true;
    public static final boolean transfer = true;
    public static final boolean climber = true;
    public static final boolean supervisor = true;
    public static boolean calibration = false;
  }

  public static final class DebugTelemetrySubsystems {
    public static final boolean odometry = false;
    public static final boolean imu = false;
    public static final boolean chassis = false;
    public static final boolean ll = false;
    public static final boolean questnav = true;
    public static final boolean intake = false;
    public static final boolean shooter = false;
    public static final boolean turret = false; 
    public static final boolean hood = false;
    public static final boolean hopper = false;
    public static final boolean spindexer = false;
    public static final boolean transfer = false;
    public static final boolean climber = false;
    public static final boolean supervisor = false;
    // Task #12: Gate SmartDashboardSubsystem output (global dashboards only).
    public static final boolean smartDashboard = false;

    // Calibration-only telemetry gate (NetworkTables/SmartDashboard).
    public static final boolean calibration = false; // PLACEHOLDER set true only while calibrating
  }

  public static final class AutoConstants {
    public static PathConstraints pathConstraints = new PathConstraints(
        SwerveConstants.MaxSpeed,
        SwerveConstants.maxAcceleration,
        SwerveConstants.MaxAngularRate,
        SwerveConstants.maxAngularAcceleration,
        12, // Volts - nomonal battery
        false // constraints shold not be unlimited
    );

    public static PathConstraints testPathCconstraints = new PathConstraints(
        2.5,
        2.0,
        SwerveConstants.MaxAngularRate,
        SwerveConstants.maxAngularAcceleration,
        12, // Volts - nomonal battery
        false // constraints shold not be unlimited
    );

    public static enum autoPoses {

      //

    }
  }

  public static class OperatorConstants {
    /** CTRE Phoenix CAN bus name for roboRIO CAN. */
    public static final CANBus RIO_CANBUS = CANBus.roboRIO();

    public static final int kDriverControllerPort = 0;

    public static class OIContants {

      public static enum ControllerDeviceType {
        LOGITECH,
        PS5,
        XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
        XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
      }

      public static record ControllerDevice(int portNumber, ControllerDeviceType controllerDeviceType,
          double deadbandX, double deadbandY, double deadbandOmega,
          boolean cubeControllerLeftStick, boolean cubeControllerRightStick) {
      }

      public static ControllerDevice XBOX_CONTROLLER = new ControllerDevice(
          5,
          ControllerDeviceType.XBOX,
          0.03,
          0.05,
          0.03,
          false,
          false);

    }

    /** Swerve-wide constants and module mappings */
    public static final class SwerveConstants {

      public static final double CHASSIS_POSE_HISTORY_TIME = 0.6; // seconds

      public static final boolean CTR_ODOMETRY_UPDATE_FROM_QUEST = true;

      public static final double MaxSpeed = 5.85; // m/s
      public static final double MaxAngularRate = 4.71238898038469; // rad/s
      public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
                                                                   // rad/s^2
      public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
      public static final double DeadbandRatioLinear = 0.05; // determined by calibration method
      public static final double DeadbandRatioAngular = 0.05; // determined by calibration method

      public static final CANBus kCANBus = new CANBus("can", "./logs/example.hoot"); // 2025
      // public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");
      // // 2024 no canivore

      public static final Pigeon2Configuration pigeonConfigs = null;
      public static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100).withKI(0).withKD(0.5)
          .withKS(0.1).withKV(2.49).withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      public static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1).withKI(0).withKD(0)
          .withKS(0).withKV(0.124);
      public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
      public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

      // Added from original TunerConstants (auto-merged):

      // Auto-merged constant declarations from original TunerConstants:
      public static final double kCoupleRatio = 3.0;
      public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
      public static final double kDriveGearRatio = 5.2734375;
      public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
      public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

      public static final int kPigeonId = 40; // 2025
      // public static final int kPigeonId = 15; // 2024

      public static final Current kSlipCurrent = Amps.of(120.0);
      public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.85);
      public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
      public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
      public static final double kSteerGearRatio = 26.09090909090909;
      public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
      public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
      public static final Distance kWheelRadius = Inches.of(2 * 0.97883494);

      public static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(kWheelRadius)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType)
          .withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withEncoderInitialConfigs(encoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);
      public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

      public static record SwerveModuleConstantsRecord(int driveMotorID, int angleMotorID, int cancoderID,
          double angleOffset,
          boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInverted) {
      }

      // 2024 SWERVE CONSTANTS

      /*
       * public static final SwerveModuleConstantsRecord MOD0 = new
       * SwerveModuleConstantsRecord(
       * 1,
       * 2,
       * 20,
       * -0.282470578125,
       * false,
       * true,
       * false);
       * 
       * public static final SwerveModuleConstantsRecord MOD1 = new
       * SwerveModuleConstantsRecord(
       * 3,
       * 4,
       * 21,
       * 0.029541015625,
       * true,
       * true,
       * false);
       * 
       * public static final SwerveModuleConstantsRecord MOD2 = new
       * SwerveModuleConstantsRecord(
       * 5,
       * 6,
       * 22,
       * 0.317138875,
       * false,
       * true,
       * false);
       * 
       * public static final SwerveModuleConstantsRecord MOD3 = new
       * SwerveModuleConstantsRecord(
       * 7,
       * 8,
       * 23,
       * 0.044677734375,
       * true,
       * true,
       * false);
       */

      // 2026 Constants

      public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord( // Front Left,
          11, // driveMotorID
          12, // angleMotorID
          21, // CanCoder Id
          // -0.296142578125, // angleOffset of cancoder to mark zero-position
          -0.474365, // angleOffset of cancoder to mark zero-position
          false, // Inversion for drive motor
          false, // Inversion for angle motor
          false // inversion for CANcoder
      );

      public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord( // Front Right
          13, // driveMotorID
          14, // angleMotorID
          23, // CanCoder ID // 0.041015625, // angleOffset of cancoder to mark zero-position
          -0.498047, // angleOffset of cancoder to mark zero-position
          true, // Inversion for drive motor
          false, // Inversion for angle motor
          false // inversion for CANcoder
      );

      public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord( // Back Left
          15, // driveMotorID
          16, // angleMotorID
          25, // CanCoder ID
          // -0.296142578125, // angleOffset of cancoder to mark zero-position
          0.003174, // angleOffset of cancoder to mark zero-position
          false, // Inversion for drive motor
          false, // Inversion for angle motor
          false // inversion for CANcoder
      );
      public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord( // Back Right
          17, // driveMotorID
          18, // angleMotorID
          27, // CanCoder ID
          // 0.326171875, // angleOffset of cancoder to mark zero-position
          // 0.0576171875, // angleOffset of cancoder to mark zero-position
          0.001953, // angleOffset of cancoder to mark zero-position
          true, // Inversion for drive motor
          false, // Inversion for angle motor
          false // inversion for CANcoder
      );

    }

    public static final class Hopper {
      public static final int MOTOR_ID = 53;
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;
      public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
      public static final NeutralModeValue NEUTRAL_COAST = NeutralModeValue.Coast;
      public static final boolean ENABLE_CURRENT_LIMIT = true;
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.0;
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = 40.0;

      public static final double MAX_DUTY_CYCLE = 0.8;
      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double STATOR_CURRENT_LIMIT_A = 40.0;

      /** Placeholder gains (Position control). Tune after SysId. */
      public static final double kP = 40.0;
      public static final double kI = 0.0;
      public static final double kD = 2.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      /** MotionMagic placeholders (rotations-based). */
      public static final double MM_CRUISE_VEL_RPS = 1.0;
      public static final double MM_ACCEL_RPS2 = 2.0;

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;
    }

    public static final class Turret {
      public static final int MOTOR_ID = 41;
      public static final int CAN_ENCODER_ID = 45;

      /** Turret is NOT drivetrain; it lives on the roboRIO CAN bus. */
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

            /** CANcoder magnet offset in rotations. Matches Phoenix Tuner. */
      public static final double CANCODER_MAGNET_OFFSET_ROT = -0.134521;

      /** After applying magnet offset, turret-zero should read 0.0 rotations. */
      public static final double ABS_ZERO_ROTATIONS = 0.0;

      /**
       * Boot assumption: at robot power-on, turret is within +/- 120 degrees of
       * turret zero.
       */
      public static final double BOOT_MAX_ABS_DEG = 120.0;

      /** Mechanical safe range relative to forward (degrees). */
      public static final double MIN_ANGLE_DEG = -110.0; // CW hard stop -100
      public static final double MAX_ANGLE_DEG = 110.0; // CCW hard stop 100

      /**
       * "Soft" limit for auto-aiming (degrees from your turret ZERO). Your notes
       * indicate ~±200°.
       *
       * This does NOT replace the hard umbilical safety
       * (MIN_ANGLE_DEG/MAX_ANGLE_DEG). It is used
       * only by the auto-aim / auto-shoot logic to prefer flipping before you reach
       * the edge.
       */
      // public static final double SOFT_AIM_LIMIT_DEG = 200.0;
      public static final double SOFT_AIM_MARGIN_DEG = 5.0;

      /**
       * Soft limits used by auto-aim to avoid living on the hard stops.
       * These should stay INSIDE MIN_ANGLE_DEG/MAX_ANGLE_DEG.
       */
      public static final double SOFT_AIM_MIN_DEG = MIN_ANGLE_DEG + SOFT_AIM_MARGIN_DEG;
      public static final double SOFT_AIM_MAX_DEG = MAX_ANGLE_DEG - SOFT_AIM_MARGIN_DEG;
      /**
       * Soft limit for auto-aiming (degrees from your turret zero).
       * For a ±180 turret, this MUST be <= 180 or you can accidentally select ±360
       * "equivalents".
       */
      // verify real margins.

      /**
       * Turret "0 deg" direction, expressed as an offset from ROBOT FORWARD.
       *
       * Convention: robot-relative angles are +CCW (left). If turret zero points 90°
       * left,
       * then this constant is +90.
       */
      public static final double ZERO_OFFSET_FROM_ROBOT_FWD_DEG = 180.0; //180.0
      /**
       * When within this margin of a limit, prefer turning the other direction when
       * possible.
       */
      public static final double LIMIT_MARGIN_DEG = 10.0;

      public static final double AIM_TOLERANCE_DEG = 2.0;

      /**
       * Control direction conventions: CCW is positive. Start with false; adjust as
       * needed on-robot.
       */
      public static final boolean MOTOR_INVERTED = true;

      /** If angle changes the wrong direction relative to motor output, flip this. */
      public static final boolean SENSOR_PHASE_INVERTED = false;

      /** Output safety limits. */
      public static final double MAX_DUTY_CYCLE = 0.8;
      // Simulation-only: allow full duty for responsive visualization.
      public static final double SIM_MAX_DUTY_CYCLE = 1.0;
      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double STATOR_CURRENT_LIMIT_A = 40.0;
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = SUPPLY_CURRENT_LIMIT_A; // <Set turret supply lower
                                                                                        // limit (A) for brief spikes>
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.2; // <Set turret supply lower time (s)>

      /** Placeholder gains (Position control). Tune after SysId. */
      public static final double kP = 38.0; // 40.0
      public static final double kI = 0.0;
      public static final double kD = 0.80; // 2.0
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      public static final double TURRET_POSITION_TOLERANCE_DEG = 1.0; // Tolerance in degrees for turret position

      /** MotionMagic placeholders (rotations-based). */
      public static final double MM_CRUISE_VEL_RPS = 1.0 / 60.0;
      public static final double MM_ACCEL_RPS2 = 2.0;

      /**
       * Motion Magic defaults expressed in turret physical units (deg/s, deg/s^2).
       * These are initial approximations to track the hub while driving quickly.
       *
       * Reasoning: omega ≈ v/r. With v=5.5 m/s and close range r~1.5–2.0 m,
       * omega ~ 158–210 deg/s. Use cruise ~240 deg/s for headroom.
       */
      // TODO: Tune on real robot.
      public static final double MM_CRUISE_DEG_PER_SEC = 240.0; // 240
      public static final double MM_ACCEL_DEG_PER_SEC2 = 1200.0; // 1200

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 220.0 / 20.0; // pinion has 20 teeth, turret ring has 280
      public static final double SIM_TURRET_J_KGM2 = 0.002;
      // Simulation-only: use a fixed supply so this subsystem doesn't collapse
      // RoboRIO voltage.
      public static final double SIM_SUPPLY_VOLTS = 12.0;

      // Simulation-only "brake" feel when commanded voltage is near zero.
      public static final double SIM_BRAKE_DEADBAND_VOLTS = 0.15;
      public static final double SIM_BRAKE_KS_VOLTS = 2.0; // static-like braking
      public static final double SIM_BRAKE_KV_VOLTS_PER_RAD_PER_SEC = 0.25; // viscous braking
      public static final double SIM_STOP_OMEGA_EPS_RAD_PER_SEC = 0.10; // snap-to-zero threshold

      /** Gear ratio: motor pinion 11 teeth, turret ring 280 teeth. */
      public static final double GEAR_RATIO_TURRET_ROT_PER_MOTOR_ROT = 20.0 / 220.0; // output / input
      public static final double GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT = 220.0 / 20.0; // input / output

      /** Conversions for integrated sensor (motor rotations) <-> turret degrees. */
      public static final double MOTOR_ROT_PER_TURRET_DEG = GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT / 360.0;
      public static final double TURRET_DEG_PER_MOTOR_ROT = 1.0 / MOTOR_ROT_PER_TURRET_DEG;

      /** Calibration-only safe jog limit (do NOT use MAX_DUTY_CYCLE for testing). */
      // TODO: Tune to a safe value for your turret.
      public static final double CAL_JOG_MAX_DUTY = 0.40;

      /** Calibration step targets. */
      // TODO: Adjust if needed
      public static final double CAL_STEP_SMALL_DEG = 30.0;
      public static final double CAL_STEP_LARGE_DEG = 90.0;

      /** Sweep test parameters. */
      // TODO: Ensure safe and within hard limits
      public static final double CAL_SWEEP_MIN_DEG = -90.0;
      public static final double CAL_SWEEP_MAX_DEG = 90.0;
      // TODO: Tune
      public static final double CAL_SWEEP_PERIOD_SEC = 1.5;

      /** Live tuning increments (testing only). */
      // TODO: Tune increments
      public static final double CAL_KP_STEP = 1.0;
      public static final double CAL_KD_STEP = 0.1;
    }

    /** Hood (pitch) motor. Hardware TBD; reserved ID for future implementation. */
    public static final class Hood {
      public static final int MOTOR_ID = 42;
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /** Set true/false once the hood is installed and tested. */
      public static final boolean MOTOR_INVERTED = false;

      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double STATOR_CURRENT_LIMIT_A = 40.0;
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = SUPPLY_CURRENT_LIMIT_A; // <Set hood supply lower limit
                                                                                        // (A) for brief spikes>
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.2; // <Set hood supply lower time (s)>

      // --- Range + conversion (PLACEHOLDERS until measured on real robot) ---
      // Fully down = 0 degrees and 0 motor rotations.
      public static final double HOOD_MIN_ANGLE_DEG = 0.0; // TODO: PLACEHOLDER confirm 0 is correct
      public static final double HOOD_MAX_ANGLE_DEG = 62.0; // TODO: PLACEHOLDER measure real max angle
      public static final double HOOD_MAX_MOTOR_ROT = 22.0; // TODO: PLACEHOLDER measure real motor rotations at max

      // Motion Magic profile (sensor units: motor rotations, rotations/sec,
      // rotations/sec^2, rotations/sec^3)
      // Start conservative; raise cruise/accel if too slow.
      public static final double MM_CRUISE_VEL_RPS = 48.0; // motor rotations per second
      public static final double MM_ACCEL_RPS2 = 72.0; // motor rotations per second^2
      public static final double MM_JERK_RPS3 = 0.0; // 0 disables jerk limiting (fine to start)
                                                     // angle

      // Motor rotations tolerance for considering the hood "at target"
      public static final double AT_TARGET_TOL_ROT = 0.02;

      // Conversion derived from measurements.
      public static final double MOTOR_ROT_PER_DEG = HOOD_MAX_MOTOR_ROT / HOOD_MAX_ANGLE_DEG; // 1:2.67 Ratio; 2.67 rot
                                                                                              // of motor = 1 of hood
                                                                                              // 18 teeth on hood
      public static final double MOTOR_ROT_PER_RAD = MOTOR_ROT_PER_DEG * (180.0 / Math.PI); // TODO: PLACEHOLDER derived
                                                                                            // from above

      // --- Software limit margin ---
      // Since you have NO hard-stop at the top, keep a conservative margin.
      public static final double SOFT_LIMIT_MARGIN_FRACTION = 0.10; // PLACEHOLDER (10% margin)

      // Motor-rotation soft limits (0 = down hard-stop, up is constrained by forward
      // soft limit)
      public static final double REVERSE_SOFT_LIMIT_ROT = 0.0; // TODO: PLACEHOLDER assumes down is exactly 0 rot
      public static final double FORWARD_SOFT_LIMIT_ROT = HOOD_MAX_MOTOR_ROT * (1.0 - SOFT_LIMIT_MARGIN_FRACTION); // TODO:
                                                                                                                   // PLACEHOLDER

      // Physical angle clamps used by setTargetAngleRad()
      public static final double MIN_ANGLE_RAD = Math.toRadians(HOOD_MIN_ANGLE_DEG); // TODO: PLACEHOLDER
      public static final double MAX_ANGLE_RAD = Math.toRadians(HOOD_MAX_ANGLE_DEG); // TODO: PLACEHOLDER
      /**
       * Neutral (safe) hood angle used in teleop when not shooting and during trench
       * lockout.
       *
       * IMPORTANT: Set this to the maximum hood angle that stays under your 22-inch
       * height rule.
       */
      public static final double NEUTRAL_ANGLE_DEG = 0.0; // TODO: SET ME (safe default = 0 deg/down)
      public static final double NEUTRAL_ANGLE_RAD = Math.toRadians(NEUTRAL_ANGLE_DEG);

      /** Placeholder gains (Position control). Tune after SysId. */
      public static final double kP = 55.0;
      public static final double kI = 0.0;
      public static final double kD = 2.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_HOOD_J_KGM2 = 0.02;
    }

    /** Shooter (2x Kraken X60 on TalonFX, Phoenix 6). */
    public static final class Shooter {
      public static final int LEADER_CAN_ID = 43;
      public static final int FOLLOWER_CAN_ID = 44;
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /**
       * Shooter motors are linked by equal sprockets.
       * Set this true if the follower must spin opposite the leader due to mirrored
       * mounting.
       */
      public static final boolean FOLLOWER_OPPOSE_MASTER = true;

      /** Shooter expels ball on negative output, so invert motor. */
      public static final boolean MOTOR_INVERTED = false;
      public static final boolean NEUTRAL_COAST = true;

      public static final double MAX_DUTY_CYCLE = 0.8;
      public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
      public static final double STATOR_CURRENT_LIMIT_A = 60.0;
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = SUPPLY_CURRENT_LIMIT_A; // TODO: <Set shooter supply
                                                                                        // lower limit (A) for brief
                                                                                        // spikes; often >
                                                                                        // SUPPLY_CURRENT_LIMIT_A>
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.2; // TODO: <Set shooter supply lower time (s) before
                                                                    // clamping to SUPPLY_CURRENT_LIMIT_A>

      /** Velocity control gains (placeholders). */
      public static final double kP = 0.2; // 0.165
      public static final double kI = 0.0;
      public static final double kD = 0.001; // 0.0008
      public static final double kS = 0.18; // 0.18
      public static final double kV = 0.121;
      public static final double kA = 0.015; // 0.001

      /** Setpoint logic. */
      public static final double DEFAULT_RPM = 2000.0;
      public static final double RPM_STEP = 50.0;
      public static final double READY_MIN_TIME_S = 0.20;
      public static final double DIP_DETECT_DROP_RPM = 250.0;

      /** Simulation: motor rotations / wheel rotations. 1.0 for your 1:1 belt. */

      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;

      public static final int READY_WINDOW_SAMPLES = 5; // 60ms @ 20ms loop
      public static final double READY_RPM_TOLERANCE = 0.97; // 97% of target RPM
      public static final double READY_STDDEV_MAX = 0.015;

    }

    /**
     * Auto-shoot orchestration constants.
     *
     * These are used by the one-button "shoot until empty" command.
     */
    public static final class AutoShoot {
      /** If true, turret aims at target continuously even when not shooting. */
      public static final boolean ALWAYS_AIM = true;

      /**
       * Default RPM used if you have not yet integrated the artillery table / hood.
       */
      public static final double DEFAULT_SHOOT_RPM = Shooter.DEFAULT_RPM;

      /**
       * Intake/indexer feed duty while firing (replace with your real feeder/indexer
       * subsystem).
       */
      public static final double FEED_DUTY = 0.55;

      /** Minimum time between "dip" events to avoid double-counting (seconds). */
      public static final double DIP_DEBOUNCE_S = 0.10;

      /**
       * If you don't have beam breaks yet, the command can use an operator-provided
       * estimate of balls remaining. This is the default value placed on
       * SmartDashboard.
       */
      public static final int DEFAULT_BALLS_ESTIMATE = 5;

      /**
       * Release prediction horizon (seconds). This compensates for rotation while
       * shooting.
       * Tune by observing misses while rotating.
       */
      public static final double DT_RELEASE_SEC = 0.12;

      /** Suppress feeding/shooting for this long after a flip decision (seconds). */
      public static final double FLIP_SUPPRESS_SEC = 0.35;

      // ---------------------------------------------------------------------
      // STATIC FAILSAFE SHOTS (pose-only, hardwired presets)
      // ---------------------------------------------------------------------
      // TODO: PLACEHOLDER: Tune these for your real "Hub Base" static spot.
      public static final double STATIC_HUB_BASE_RPM = 4200.0;
      public static final double STATIC_HUB_BASE_HOOD_DEG = 25.0;

      // TODO: PLACEHOLDER: Tune these for your real "Tower Base" static spot.
      public static final double STATIC_TOWER_BASE_RPM = 4600.0;
      public static final double STATIC_TOWER_BASE_HOOD_DEG = 30.0;

      // "Robot stopped" gating for static shots (prevents feeding while sliding).
      // TODO: PLACEHOLDER: Tune thresholds (start conservative).
      public static final double STATIC_MAX_VX_MPS = 0.15;
      public static final double STATIC_MAX_VY_MPS = 0.15;
      public static final double STATIC_MAX_OMEGA_DEG_PER_S = 12.0;

      // Drivetrain heading-hold controller (used while static-shot button held).
      // TODO: PLACEHOLDER: Tune on carpet.
      public static final double STATIC_HOLD_HEADING_kP = 0.08;
      public static final double STATIC_HOLD_HEADING_kI = 0.0;
      public static final double STATIC_HOLD_HEADING_kD = 0.0;
      public static final double STATIC_HOLD_MAX_OMEGA_DEG_PER_S = 180.0;

      /** RT threshold for the stationary illegal-shot auto-turn assist. */
      public static final double STATIONARY_ASSIST_TRIGGER_THRESHOLD = 0.30;

      /**
       * Additional comfort margin inside turret hard limits for deciding when the
       * chassis should auto-turn to make a shot legal.
       *
       * With turret hard limits of [-110, +110], a value of 10 creates a comfort
       * window of [-100, +100].
       */
      public static final double STATIONARY_ILLEGAL_SHOT_COMFORT_MARGIN_DEG = 10.0;

            /**
       * Fixed robot angular speed for the stationary illegal-shot auto-turn assist.
       * Units are actual robot angular speed in rad/s.
       */
      public static final double STATIONARY_ILLEGAL_SHOT_FIXED_AUTO_TURN_RAD_PER_SEC = 1.5;

      /**
       * Driver omega deadband for allowing the stationary illegal-shot auto-turn
       * assist to take over.
       */
      public static final double STATIONARY_ASSIST_OMEGA_DEADBAND = 0.30;
      /** Shooter enters recovery if actual RPM <= targetRPM * this fraction. */
  public static final double RECOVERY_RPM_FRACTION_LIMIT = 0.50;

  /**
   * Hood compensation model:
   * hoodCompDeg = (1 - rpmFraction) * HOOD_COMP_DEG_PER_UNIT_RPM_DROP,
   * clamped to HOOD_COMP_MAX_DEG.
   */
  public static final double HOOD_COMP_DEG_PER_UNIT_RPM_DROP = 6.0; // TODO tune
  public static final double HOOD_COMP_MAX_DEG = 4.0; // TODO tune

  /** Rumble strengths for shoot-while-held invalid states. */
  public static final double TURRET_ONLY_INVALID_LEFT_RUMBLE = 0.25;
  public static final double GLOBAL_INVALID_RIGHT_RUMBLE = 0.80;
  public static final double GLOBAL_INVALID_PULSE_PERIOD_S = 0.30;
    }

    /** Spindexer motor + tuning. */
    public static final class Spindexer {
      public static final int MOTOR_ID = 50; // TODO set
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /** Low velocity for circulation / keeping balls flowing. Units: rotor RPS. */
      public static final double BASE_RPS = 3.0;
      /** Higher velocity for supplying transfer while shooting. Units: rotor RPS. */
      public static final double SUPPLY_RPS = 35.0;
      public static final double SLOW_RPS = 10.0;

      /**
       * Current limits:
       * Stator limit must be ABOVE the jam threshold, or anti-jam will never see the
       * spike.
       */
      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = 60.0;
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.2;
      public static final double STATOR_CURRENT_LIMIT_A = 60.0;

      // ---------------- Velocity Voltage tuning ----------------
      // Units are in rotor rotations/sec and Phoenix slot gains.
      public static final double VEL_kS = 0.0;
      public static final double VEL_kV = 0.12925;
      public static final double VEL_kP = 0.51;
      public static final double VEL_kI = 0.0;
      public static final double VEL_kD = 0.0;

      // ---------------- Anti-jam state machine ----------------
      public static final boolean ANTI_JAM_ENABLED = false;

      /** Jam trigger threshold. User approved 40-60 A range; start in the middle. */
      public static final double JAM_CURRENT_THRESHOLD_A = 50.0;

      /** Require jam condition to persist this long before reversing. */
      public static final double JAM_CONFIRM_TIME_S = 0.10;

      /**
       * Only evaluate jam logic when target speed is meaningfully above zero.
       * Prevents false triggers during tiny test speeds.
       */
      public static final double JAM_MIN_TARGET_RPS = 8.0;

      /**
       * Velocity-collapse detector:
       * if actual speed falls below this fraction of target while current is high,
       * we treat it as a real jam.
       */
      public static final double JAM_MIN_VELOCITY_RATIO = 0.50;

      /** Brief reverse to release ball compression. */
      public static final double UNJAM_REVERSE_DUTY = -0.30;
      public static final double UNJAM_REVERSE_TIME_S = 0.20;

      /** Optional gentle forward settle phase after reverse. */
      public static final boolean ENABLE_UNJAM_SETTLE_FORWARD = true;
      public static final double UNJAM_SETTLE_FORWARD_RPS = 6.0;
      public static final double UNJAM_SETTLE_TIME_S = 0.20;

      /** Minimum time between unjam events to prevent oscillation. */
      public static final double UNJAM_COOLDOWN_S = 0.50;

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;
    }

    /** Transfer motor + sensors + tuning. */
    public static final class Transfer {
      public static final int MOTOR_ID = 51; // TODO set
      public static final int MOTOR2_ID = 52; // reserved optional second transfer motor
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /** IR beam-break at transfer entry (just AFTER spindexer handoff). */
      public static final int ENTRY_SENSOR_DIO = 2;
      /** Default assumes HIGH when blocked. */
      // TODO: Verify IR beam-break polarity (HIGH when blocked?)
      public static final boolean ENTRY_SENSOR_INVERTED = true; // will say true when blocked

      /** IR beam-break at shooter throat (exit of transfer). */
      public static final int THROAT_SENSOR_DIO = 1;
      /** Default assumes HIGH when blocked. */
      // TODO: Verify IR beam-break polarity (HIGH when blocked?)
      public static final boolean THROAT_SENSOR_INVERTED = true;

      /** Number of balls to fire in one burst before pausing for shooter recovery. */
      public static final int BALLS_PER_BURST = 10;

      /**
       * Minimum time between throat-sensor count events to prevent double-counting.
       */
      public static final double THROAT_COUNT_DEBOUNCE_S = 0.03;

      /** Slow speed to keep balls staged without slamming them into the shooter. */
      public static final double STAGE_DUTY = 0.20;
      /** Fast speed to inject a ball into the shooter. */
      public static final double FEED_DUTY = 0.85;

      // ---------------- Closed-loop velocity targets (RPS) ----------------
      // TODO: These setpoints are placeholders until the robot is fully built and you
      // can test/measure
      // ideal transfer speeds with real balls.
      /** Staging target speed in rotor RPS (closed-loop). */
      public static final double STAGE_RPS = -20;
      /** Feeding target speed in rotor RPS (closed-loop). */
      public static final double FEED_RPS = -90;

      // ---------------- Metered firing (rate + speed) ----------------
      // Goal: eject ONE ball at a controlled speed, then wait a minimum interval
      // before ejecting next.
      // TODO: Placeholders until robot is fully built and ball dynamics are tested.
      /** Desired ejection rate in balls/sec (how often you allow an eject). */
      public static final double EJECT_BALLS_PER_SEC = 2.0; // TODO: placeholder (ex: 2 balls/sec)

      /**
       * Minimum time between the *start* of ejections (derived from
       * EJECT_BALLS_PER_SEC).
       */
      public static final double EJECT_MIN_INTERVAL_S = 1.0 / EJECT_BALLS_PER_SEC; // TODO: placeholder

      /**
       * Safety timeout: if throat never clears during an eject, stop anyway to avoid
       * running forever.
       * This is a protection against sensor issues or unexpected ball behavior.
       */
      public static final double EJECT_MAX_TIME_S = 0.35; // TODO: placeholder

      /**
       * When a ball is already at the throat, staging should stop to avoid
       * jamming/compressing.
       * If you later prefer a very slow "creep hold", change this to a small nonzero
       * value.
       */
      public static final double THROAT_BLOCKED_STAGE_RPS = -0; // TODO: placeholder (0 = stop)

      // ---------------- Closed-loop gains (Phoenix 6 Slot0) ----------------
      // TODO: All gains are placeholders and MUST be tuned on the real robot.
      // Units:
      // - kS, kV are in "duty" terms because we use VelocityDutyCycle.
      // - kP is duty per (RPS error).
      public static final double VEL_kS = 0.05;
      public static final double VEL_kV = 0.012;
      public static final double VEL_kP = 0.04;
      public static final double VEL_kI = 0.0;
      public static final double VEL_kD = 0.0;

      // ---------------- Motor safety defaults ----------------
      // Reasonable defaults (you authorized defaults). Tune as needed after measuring
      // performance.
      public static final double SUPPLY_CURRENT_LIMIT_A = 35.0; // TODO: verify/tune
      public static final double STATOR_CURRENT_LIMIT_A = 60.0; // TODO: verify/tune
      public static final double SUPPLY_CURRENT_LOWER_LIMIT_A = SUPPLY_CURRENT_LIMIT_A; // TODO: <Enter transfer supply
                                                                                        // lower limit (A)>
      public static final double SUPPLY_CURRENT_LOWER_TIME_S = 0.2; // TODO: <Enter transfer supply lower time (s)>

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;
    }

    /** Where the artillery table CSV lives under src/main/deploy. */
    public static final class ArtilleryTable {
      /** Example: "artillery/rebuilt_shots.csv" */
      public static final String DEPLOY_CSV_PATH = "artillery/rebuilt_shots.csv";
    }

    /** Solver tuning and physics constants. */
    public static final class ArtillerySolver {
      public static final double TOF_MIN_SEC = 0.10;
      public static final double TOF_MAX_SEC = 1.10;
      public static final double TOF_STEP_SEC = 0.01;

      /** Use 9.80665 unless you have a reason to change. */
      public static final double GRAVITY_MPS2 = 9.80665;

      /**
       * Weighting between matching angle vs speed in your measured table inverse
       * lookup.
       */
      public static final double ANGLE_WEIGHT = 1.0;
      public static final double SPEED_WEIGHT = 0.25;
    }

    /**
     * Field geometry values needed by the solver. You said you'll supply HUB X/Y
     * experimentally.
     */
    public static final class FieldGeometry {
      /**
       * Height of HUB opening center above the field, meters. TODO: set from manual
       * measurement.
       */
      public static final double HUB_OPENING_CENTER_Z_METERS = 1.8288;

      /**
       * Field length in meters, used for alliance mirroring (RED <-> BLUE).
       *
       * IMPORTANT: Set this to the official 2026 field length for your coordinate
       * frame.
       */
      public static final double FIELD_LENGTH_METERS = 16.54;

      /**
       * Teleop trench safety zones, defined in BLUE-alliance field coordinates
       * (meters).
       *
       * These are axis-aligned rectangles (min/max X/Y). For RED alliance, the pose X
       * is
       * mirrored using FIELD_LENGTH_METERS.
       *
       * IMPORTANT: You MUST set these numbers; leaving them at 0 will make the zone
       * detection wrong.
       */
      public static final double BLUE_TRENCH_ZONE1_MIN_X_METERS = 4.060; 
      public static final double BLUE_TRENCH_ZONE1_MAX_X_METERS = 5.200; 
      public static final double BLUE_TRENCH_ZONE1_MIN_Y_METERS = 7.270; 
      public static final double BLUE_TRENCH_ZONE1_MAX_Y_METERS = 7.585; 

      public static final double BLUE_TRENCH_ZONE2_MIN_X_METERS = 4.060; 
      public static final double BLUE_TRENCH_ZONE2_MAX_X_METERS = 5.200;
      public static final double BLUE_TRENCH_ZONE2_MIN_Y_METERS = 0.500;
      public static final double BLUE_TRENCH_ZONE2_MAX_Y_METERS = 0.785;
    }

    /** Turret geometry needed by the solver. */
    public static final class TurretGeometry {
      /**
       * Turret pivot position relative to robot origin, in the ROBOT frame (meters).
       */
      public static final Translation2d TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS = new Translation2d(
          -0.15, -0.06); //-0.1778, -0.07

      /** Ball release height above field when leaving shooter, meters. TODO set. */
      public static final double BALL_RELEASE_HEIGHT_METERS = 0.4318;
    }

    public static final class SysId {
      /** Safety gate: characterization only runs if true. */
      public static final boolean ENABLE_SYSID = false;

      /**
       * Runtime gate (SmartDashboard boolean). Both this and ENABLE_SYSID must be
       * true.
       */
      public static final String SYSID_DASH_ENABLE_KEY = "SysId/Enable";

      public static final double TURRET_RAMP_RATE_V_PER_S = 1.0;
      public static final double TURRET_STEP_V = 4.0;
      public static final double TURRET_TIMEOUT_S = 10.0;

      public static final double SHOOTER_RAMP_RATE_V_PER_S = 1.0;
      public static final double SHOOTER_STEP_V = 4.0;
      public static final double SHOOTER_TIMEOUT_S = 10.0;
      public static final double SHOOTER_SYSID_MAX_VOLTS = 6.0; // TODO: PLACEHOLDER - set a safe max voltage for
                                                                // shooter SysId testing (start conservative)

      public static final double HOOD_RAMP_RATE_V_PER_S = 1.0;
      public static final double HOOD_STEP_V = 4.0;
      public static final double HOOD_TIMEOUT_S = 10.0;

      public static final double TRANSFER_RAMP_RATE_V_PER_S = 1.0;
      public static final double TRANSFER_STEP_V = 4.0;
      public static final double TRANSFER_TIMEOUT_S = 10.0;

      public static final double SPINDEXER_RAMP_RATE_V_PER_S = 1.0;
      public static final double SPINDEXER_STEP_V = 4.0;
      public static final double SPINDEXER_TIMEOUT_S = 10.0;

      public static final double INTAKE_ROLLER_RAMP_RATE_V_PER_S = 1.0;
      public static final double INTAKE_ROLLER_STEP_V = 4.0;
      public static final double INTAKE_ROLLER_TIMEOUT_S = 10.0;

      public static final double INTAKE_PIVOT_RAMP_RATE_V_PER_S = 1.0;
      public static final double INTAKE_PIVOT_STEP_V = 4.0;
      public static final double INTAKE_PIVOT_TIMEOUT_S = 10.0;

      public static final double HOPPER_RAMP_RATE_V_PER_S = 1.0;
      public static final double HOPPER_STEP_V = 4.0;
      public static final double HOPPER_TIMEOUT_S = 10.0;

      public static final double CLIMB_RAMP_RATE_V_PER_S = 1.0;
      public static final double CLIMB_STEP_V = 4.0;
      public static final double CLIMB_TIMEOUT_S = 10.0;

    }

    public static final class IntakeConstants {
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      public static final int intakeRollerMotorId = 55;
      public static final boolean IntakeRollerInverted = true;

      public static final int intakePivotMotorId = 56;
      public static final int intakePivotFollowerMotorId = 57;
      // TODO: PLACEHOLDER - set to the actual CAN ID of the 2nd pivot Kraken
      // (follower)

      public static final boolean intakePivotFollowerOpposeLeader = true;
      // TODO: PLACEHOLDER - verify on hardware by jogging. If the motors fight, flip
      // this.

      public static final boolean intakePivotMotorInverted = true;

      // Intake roller setpoints are in RPS at the ROLLER, not motor RPS.
      public static final double ROLLER_INTAKE_RPS = 50.0;
      public static final double ROLLER_REVERSE_RPS = -20.0;

      // Gear ratios
      // Pivot: given as 1 / 26.7 arm rotations per motor rotation,
      // so motor-to-arm ratio is 26.7 motor rotations per arm rotation.
      public static final double PIVOT_MOTOR_TO_ARM_GEAR_RATIO = 26.7;

      // Roller: 2 motor rotations per 1 roller rotation.
      public static final double ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO = 2.0;

      // ---------------- Current limits (brownout protection) ----------------

      // Roller motor current limits
      public static final double ROLLER_SUPPLY_CURRENT_LIMIT_A = 25.0;
      public static final double ROLLER_SUPPLY_CURRENT_LOWER_LIMIT_A = 20.0;
      public static final double ROLLER_SUPPLY_CURRENT_LOWER_TIME_S = 0.25;
      public static final double ROLLER_STATOR_CURRENT_LIMIT_A = 60.0;

      // Pivot motors current limits (applied to BOTH leader + follower)
      public static final double PIVOT_SUPPLY_CURRENT_LIMIT_A = 30.0;
      public static final double PIVOT_SUPPLY_CURRENT_LOWER_LIMIT_A = 25.0;
      public static final double PIVOT_SUPPLY_CURRENT_LOWER_TIME_S = 0.25;
      public static final double PIVOT_SUPPLY_TIME_THRESHOLD_S = 0.10;
      public static final double PIVOT_STATOR_CURRENT_LIMIT_A = 80.0;

      public static final double PIVOT_MIN_DEG = 0; // retracted hard stop = 0 deg
      public static final double PIVOT_MAX_DEG = 53; // TODO: PLACEHOLDER - verify true max

      public static final double CAL_PIVOT_JOG_DUTY = 0.08;
      // TODO: PLACEHOLDER - start low, raise carefully if needed

      public static final double CAL_STEP_LOW_DEG = 30.0;
      public static final double CAL_STEP_HIGH_DEG = 90.0;

      public static enum IntakePositions { // arm degrees (not motor rotations)
        IntakeStowedDeg(37.0),
        IntakeRetracted(51.0),
        IntakeDeployedDeg(1.67);

        private double armDeg;

        IntakePositions(double armDeg) {
          this.armDeg = armDeg;
        }

        public double getPosition() {
          return armDeg;
        }
      }

      public static final class IntakePidConstants {
        public static class PositionDutyCycleConstants {
          public static final double intake_kP = 0.1;
          public static final double intake_kI = 0.0;
          public static final double intake_kD = 0.01;
          public static final double intake_kV = 0.12;
        }

        public static class MotionMagicDutyCycleConstants {
          public static final int slot = 0;
          public static final double intake_kP = 10; // 0.64
          public static final double intake_kI = 0.0;
          public static final double intake_kD = 0.0;
          public static final double MotionMagicCruiseVelocity = 50.0; // 75.0
          public static final double motionMagicAcceleration = 100.0; // 150.0
          public static final double motionMagicJerk = 1000.0; // 1500.0
        }

        public static class RollerVelocityVoltageConstants {
          public static final int slot = 0;

          // These are MOTOR-side gains because the Talon is controlling motor velocity.
          public static final double intake_kS = 0.25;
          public static final double intake_kV = 0.12;
          public static final double intake_kP = 0.3;
          public static final double intake_kI = 0.0;
          public static final double intake_kD = 0.0;
        }

        public static final double tolerance = 3.0; // TODO: PLACEHOLDER - your requirement
      }

      /** Simulation placeholders for SysId/Sim (tune once mechanism is built). */
      public static final double SIM_ROLLER_GEAR_RATIO = ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO;
      public static final double SIM_ROLLER_J_KGM2 = 0.002;

      public static final double SIM_PIVOT_GEAR_RATIO = PIVOT_MOTOR_TO_ARM_GEAR_RATIO;
      public static final double SIM_PIVOT_J_KGM2 = 0.01;
    }

    public static final class ClimbConstants {

      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      public static final int climbMotorLeftID = 60;
      public static final int climbMotorRightID = 61;

      /** Simulation placeholders for SysId/Sim (tune once mechanism is built). */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;

      public static class ClimbMotionMagicDutyCycleConstants {
        public static final int slot = 0;
        public static final double climb_kP = 0.64; // 0.64
        public static final double climb_kI = 0.0;
        public static final double climb_kD = 0.0;
        public static final double MotionMagicCruiseVelocity = 50.0; // 75.0
        public static final double motionMagicAcceleration = 100.0; // 150.0
        public static final double motionMagicJerk = 1000.0; // 1500.0
      }
    }

  }

  public static final class PathPlannerConstants {
    public static final boolean shouldFlipTrajectoryOnRed = true;
  }

}
