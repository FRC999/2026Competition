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
   * You indicated you'll provide these experimentally (HUB_BLUE_X/Y, HUB_RED_X/Y).
   */
  public static final class FieldTargets {
    public static final double HUB_BLUE_X = 0.0; // TODO set
    public static final double HUB_BLUE_Y = 0.0; // TODO set
    public static final double HUB_RED_X  = 0.0; // TODO set
    public static final double HUB_RED_Y  = 0.0; // TODO set
  }

public static final class EnabledSubsystems {

  public static final boolean chasis = true;
  public static final boolean ll = false;
  public static final boolean questnav = false;
  public static final boolean intake = false;
  public static final boolean shooter = false;
  public static final boolean turret = true;
  public static final boolean hood = false;
  public static final boolean hopper = false;
  public static final boolean spindexer = false;
  public static final boolean transfer = false;
  public static final boolean climber = false;
  public static final boolean supervisor = false;
}


	public static final class DebugTelemetrySubsystems {
		public static final boolean odometry = true;
		public static final boolean imu = true;
		public static final boolean chassis = true;
		public static final boolean ll = false;
		public static final boolean questnav = false;
    public static final boolean intake = false;
    public static final boolean shooter = false;
    public static final boolean turret = true;
    public static final boolean hood = false;
    public static final boolean hopper = false;
    public static final boolean spindexer = false;
    public static final boolean transfer = false;
    public static final boolean climber = false;
    public static final boolean supervisor = false;
    // Task #12: Gate SmartDashboardSubsystem output (global dashboards only).
    public static final boolean smartDashboard = false;
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
              boolean cubeControllerLeftStick, boolean cubeControllerRightStick) {}

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

      public static final double CHASSIS_POSE_HISTORY_TIME = 0.6; //seconds

      public static final boolean CTR_ODOMETRY_UPDATE_FROM_QUEST = true;

      public static final double MaxSpeed = 5.85; // m/s
      public static final double MaxAngularRate = 4.71238898038469; // rad/s
      public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
																		// rad/s^2
      public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
      public static final double DeadbandRatioLinear = 0.05; //determined by calibration method 
      public static final double DeadbandRatioAngular =  0.05; //determined by calibration method

      public static final CANBus kCANBus = new CANBus("can", "./logs/example.hoot"); // 2025
      //public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot"); // 2024 no canivore

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
      public static final double kDriveGearRatio =  5.2734375;
      public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
      public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
      
      public static final int kPigeonId = 40; // 2025
      //public static final int kPigeonId = 15; // 2024
      
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

      public static record SwerveModuleConstantsRecord(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset,
        boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInverted) {}


        // 2024 SWERVE CONSTANTS
        
        /*public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord(
          1, 
          2, 
          20, 
          -0.282470578125, 
          false, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord(
          3, 
          4, 
          21, 
          0.029541015625, 
          true, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord(
          5, 
          6, 
          22, 
          0.317138875, 
          false, 
          true, 
          false);

        public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord(
          7, 
          8, 
          23, 
          0.044677734375, 
          true, 
          true, 
          false); 
        */

          // 2026 Constants
        
        public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord( // Front Left,
						11, // driveMotorID
						12, // angleMotorID
						31, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						-0.474365, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);
       
        public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord( // Front Right
						13, // driveMotorID
						14, // angleMotorID
						21, // CanCoder ID						// 0.041015625, // angleOffset of cancoder to mark zero-position
						-0.498047, // angleOffset of cancoder to mark zero-position
						true, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);

        public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord( // Back Left
						15, // driveMotorID
						16, // angleMotorID
						34, // CanCoder ID
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						0.003174, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);
        public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord( // Back Right
						17, // driveMotorID
						18, // angleMotorID
						23, // CanCoder ID
						// 0.326171875, // angleOffset of cancoder to mark zero-position
						//0.0576171875, // angleOffset of cancoder to mark zero-position
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

      /** Absolute PWM encoder reference (PulseWidth 0-4095 equivalent). */
      public static final int ABS_TICKS_PER_REV = 4096;
      /** Absolute encoder tick value that corresponds to turret pointing forward. */
      public static final int ABS_FORWARD_TICKS = 1282;

      /**
       * Boot assumption: at robot power-on, turret is within +/- 180 degrees of forward.
       * Used to seed the software unwrapped angle.
       */
      public static final double BOOT_MAX_ABS_DEG = 180.0;

      /** Mechanical safe range relative to forward (degrees). */
      public static final double MIN_ANGLE_DEG = -340.0;
      public static final double MAX_ANGLE_DEG = 340.0;

      /**
       * "Soft" limit for auto-aiming (degrees from your turret ZERO). Your notes indicate ~±200°.
       *
       * This does NOT replace the hard umbilical safety (MIN_ANGLE_DEG/MAX_ANGLE_DEG). It is used
       * only by the auto-aim / auto-shoot logic to prefer flipping before you reach the edge.
       */
      public static final double SOFT_AIM_LIMIT_DEG = 200.0;

      /** True if your turret "ZERO" (and ABS_FORWARD_TICKS reference) points toward the ROBOT BACK. */
      public static final boolean ZERO_POINTS_ROBOT_BACK = true;
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

      /** Placeholder gains (Position control). Tune after SysId. */
      public static final double kP = 40.0;
      public static final double kI = 0.0;
      public static final double kD = 2.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      /** MotionMagic placeholders (rotations-based). */
      public static final double MM_CRUISE_VEL_RPS = 1.0/60.0; 
      public static final double MM_ACCEL_RPS2 = 2.0;

      /** Simulation placeholders. */
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_TURRET_J_KGM2 = 0.002;
      // Simulation-only: use a fixed supply so this subsystem doesn't collapse RoboRIO voltage.
      public static final double SIM_SUPPLY_VOLTS = 12.0;

      // Simulation-only “brake feel” when commanded voltage is near zero.
      public static final double SIM_BRAKE_DEADBAND_VOLTS = 0.15;
      public static final double SIM_BRAKE_KS_VOLTS = 2.0;                 // static-like braking
      public static final double SIM_BRAKE_KV_VOLTS_PER_RAD_PER_SEC = 0.25; // viscous braking
      public static final double SIM_STOP_OMEGA_EPS_RAD_PER_SEC = 0.10;     // snap-to-zero threshold

    }


    /** Hood (pitch) motor. Hardware TBD; reserved ID for future implementation. */
    public static final class Hood {
      public static final int MOTOR_ID = 42;
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /** Set true/false once the hood is installed and tested. */
      public static final boolean MOTOR_INVERTED = false;

      public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
      public static final double STATOR_CURRENT_LIMIT_A = 40.0;

      /**
       * Hood gearing: motor rotations per hood mechanism rotation.
       * Placeholder until the hood gearbox/sprocket ratio is finalized.
       */
      public static final double GEAR_RATIO_MOTOR_ROT_PER_HOOD_ROT = 1.0;

      /**
       * Conversion used by HoodSubsystem when commanding a hood physical angle (radians).
       * motorRot = hoodRad / (2π) * GEAR_RATIO_MOTOR_ROT_PER_HOOD_ROT
       */
      public static final double MOTOR_ROT_PER_RAD =
          GEAR_RATIO_MOTOR_ROT_PER_HOOD_ROT / (2.0 * Math.PI);

      /**
       * Soft limits in hood physical angle (radians). Placeholder values.
       * If you do not know yet, leave wide; tighten once mechanical range is known.
       */
      public static final double MIN_ANGLE_RAD = -0.10;
      public static final double MAX_ANGLE_RAD =  1.60;

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
      public static final double SIM_HOOD_J_KGM2 = 0.02;
    }

    /** Shooter (2x Kraken X60 on TalonFX, Phoenix 6). */
    public static final class Shooter {
      public static final int LEADER_CAN_ID = 43;
      public static final int FOLLOWER_CAN_ID = 44;
      public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

      /**
       * Shooter motors are linked by equal sprockets.
       * Set this true if the follower must spin opposite the leader due to mirrored mounting.
       */
      public static final boolean FOLLOWER_OPPOSE_MASTER = true;

      /** Shooter expels ball on negative output, so invert motor. */
      public static final boolean MOTOR_INVERTED = false;
      public static final boolean NEUTRAL_COAST = true;

                              
      public static final double MAX_DUTY_CYCLE = 0.8;
      public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
      public static final double STATOR_CURRENT_LIMIT_A = 60.0;
                              
                                        
                              

      /** Velocity control gains (placeholders). */
      public static final double kP = 0.155; //0.165
      public static final double kI = 0.0;
      public static final double kD = 0.0007; //0.0008
      public static final double kS = 0.18; //0.18
      public static final double kV = 0.121; // Try this tomorrow: 0.123
      public static final double kA = 0.004; //0.001

      /** Setpoint logic. */
      public static final double DEFAULT_RPM = 3000.0;
      public static final double RPM_STEP = 50.0;
      public static final double READY_TOLERANCE_RPM = 75.0;
      public static final double READY_MIN_TIME_S = 0.20;
      public static final double DIP_DETECT_DROP_RPM = 250.0;

      /** Simulation: motor rotations / wheel rotations. 1.0 for your 1:1 belt. */
                                            
      public static final double SIM_GEAR_RATIO = 1.0;
      public static final double SIM_J_KGM2 = 0.02;

      // ChatGPT constants for updated readiness logic
      public static final int READY_WINDOW_SAMPLES = 10;     // 200ms @ 20ms loop
      public static final double READY_RPM_TOLERANCE = 40.0;
      public static final double READY_STDDEV_MAX = 30.0;

    }

    /**
     * Auto-shoot orchestration constants.
     *
     * These are used by the one-button "shoot until empty" command.
     */
    public static final class AutoShoot {
      /** If true, turret aims at target continuously even when not shooting. */
      public static final boolean ALWAYS_AIM = true;

      /** Default RPM used if you have not yet integrated the artillery table / hood. */
      public static final double DEFAULT_SHOOT_RPM = Shooter.DEFAULT_RPM;

      /** Intake/indexer feed duty while firing (replace with your real feeder/indexer subsystem). */
      public static final double FEED_DUTY = 0.55;

      /** Minimum time between "dip" events to avoid double-counting (seconds). */
      public static final double DIP_DEBOUNCE_S = 0.10;

      /**
       * If you don't have beam breaks yet, the command can use an operator-provided
       * estimate of balls remaining. This is the default value placed on SmartDashboard.
       */
      public static final int DEFAULT_BALLS_ESTIMATE = 5;

      /**
       * Release prediction horizon (seconds). This compensates for rotation while shooting.
       * Tune by observing misses while rotating.
       */
      public static final double DT_RELEASE_SEC = 0.12;

      /** Suppress feeding/shooting for this long after a flip decision (seconds). */
      public static final double FLIP_SUPPRESS_SEC = 0.35;
    }

    /** SysId gating + default parameters. */
    

/** Spindexer motor + tuning. */
public static final class Spindexer {
  public static final int MOTOR_ID = 50; // TODO set
  public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;
  /** Low duty for circulation / keeping balls flowing. */
  public static final double BASE_DUTY = 0.25;
  /** Higher duty for supplying transfer while shooting. */
  public static final double SUPPLY_DUTY = 0.45;

  /** Simulation placeholders. */
  public static final double SIM_GEAR_RATIO = 1.0;
  public static final double SIM_J_KGM2 = 0.02;
}

/** Transfer motor + sensors + tuning. */
public static final class Transfer {
  public static final int MOTOR_ID = 51; // TODO set
  public static final int MOTOR2_ID = 52; // reserved optional second transfer motor
  public static final CANBus CANBUS_NAME = OperatorConstants.RIO_CANBUS;

  /** Sensor at transfer entry (just AFTER the spindexer handoff). */
  public static final int ENTRY_SENSOR_DIO = 0; // TODO set
  public static final boolean ENTRY_SENSOR_INVERTED = true; // common for beam breaks

  /** Sensor at shooter throat (exit of transfer). */
  public static final int THROAT_SENSOR_DIO = 1; // TODO set
  public static final boolean THROAT_SENSOR_INVERTED = true;

  /** Slow speed to keep balls staged without slamming them into the shooter. */
  public static final double STAGE_DUTY = 0.20;
  /** Fast speed to inject a ball into the shooter. */
  public static final double FEED_DUTY = 0.85;


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

  /** Weighting between matching angle vs speed in your measured table inverse lookup. */
  public static final double ANGLE_WEIGHT = 1.0;
  public static final double SPEED_WEIGHT = 0.25;
}

/** Field geometry values needed by the solver. You said you'll supply HUB X/Y experimentally. */
public static final class FieldGeometry {
  /** Height of HUB opening center above the field, meters. TODO: set from manual measurement. */
  public static final double HUB_OPENING_CENTER_Z_METERS = 0.0;
}

/** Turret geometry needed by the solver. */
public static final class TurretGeometry {
  /** Turret pivot position relative to robot origin, in the ROBOT frame (meters). */
  public static final edu.wpi.first.math.geometry.Translation2d TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS =
      new edu.wpi.first.math.geometry.Translation2d(0.0, 0.0); // TODO set

  /** Ball release height above field when leaving shooter, meters. TODO set. */
  public static final double BALL_RELEASE_HEIGHT_METERS = 0.0;
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
      public static final boolean IntakeRollerInverted = false;

      public static final int intakePivotMotorId = 56;
      public static final boolean intakePivotMotorInverted = false;
      public static final double defaultSpeed = 0.3;

      public static enum IntakePositions{ // position of the arm for the piece placement/pickup
				IntakeDown(0.0),
				IntakeUp(0.0);
        private double intakePositionSelected;
				IntakePositions(double position) {
				  this.intakePositionSelected = position;
				}
				public double getPosition() {
				  return intakePositionSelected;
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
					public static final double intake_kP = 0.64; //0.64
					public static final double intake_kI = 0.0;
					public static final double intake_kD = 0.0;
					public static final double MotionMagicCruiseVelocity = 50.0; //75.0
					public static final double motionMagicAcceleration = 100.0; //150.0
					public static final double motionMagicJerk = 1000.0; //1500.0
				}

        public static final double tolerance = 3.0;
      }

      /** Simulation placeholders for SysId/Sim (tune once mechanism is built). */
      public static final double SIM_ROLLER_GEAR_RATIO = 1.0;
      public static final double SIM_ROLLER_J_KGM2 = 0.002;

      public static final double SIM_PIVOT_GEAR_RATIO = 1.0;
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
					public static final double climb_kP = 0.64; //0.64
					public static final double climb_kI = 0.0;
					public static final double climb_kD = 0.0;
					public static final double MotionMagicCruiseVelocity = 50.0; //75.0
					public static final double motionMagicAcceleration = 100.0; //150.0
					public static final double motionMagicJerk = 1000.0; //1500.0
				}
    }

  }

  public static final class PathPlannerConstants{
    public static final boolean shouldFlipTrajectoryOnRed = true;
  }

}
