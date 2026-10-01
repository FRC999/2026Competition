package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import frc.robot.lib.TurretMotionPolicy;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// Simulation
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants.Turret;

/** TalonFX turret with a pinion CANcoder boot reference and continuous integrated position.
 * Boot requires known physical stow within half a pinion turn (+/-16.36 turret degrees).
 * +/-110 degree command limits protect the robot perimeter; zero points toward the robot rear.
 * ContinuousWrap stays disabled, and measured overshoot is never hidden by clamping.
 */
public class TurretSubsystem extends SubsystemBase {
  private static final double ABS_SEED_SAMPLE_PERIOD_SEC = 0.01;
  private static final double ABS_SEED_TIMEOUT_SEC = 0.25;
  private static final double ABS_SEED_STABLE_TOLERANCE_DEG = 1.0;
  private static final int ABS_SEED_REQUIRED_STABLE_SAMPLES = 3;

  // Turret motor controller on the specified CAN bus.
  private TalonFX turret;

  // Absolute encoder (CAN Through-Bore / CANcoder) on the same CAN bus.
  private CANcoder throughboreCANcoder = new CANcoder(Turret.CAN_ENCODER_ID, Turret.CANBUS_NAME);

  // Open-loop duty request (used for manual and SysId drive).
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  // Track what we last commanded so simulation can use true volts (not normalized output).

  // Absolute CAN Through-Bore in signed rotations (-0.5..+0.5). Wraps every revolution.
  private final StatusSignal<Angle> absPosSig = throughboreCANcoder.getAbsolutePosition();

  // Motor voltage is used for telemetry and SysId logging.
  private StatusSignal<Voltage> motorVoltageSig;

  // Integrated (relative) position from the TalonFX (multi-turn, does not wrap).
  private StatusSignal<Angle> motorPosSig;
  private StatusSignal<AngularVelocity> motorVelSig;

  // Used to guard sim-only code paths.
  private final boolean isSim = RobotBase.isSimulation();

  // If the absolute sensor only increases when turret turns CW, set +1 for CW-positive convention.
  // If you ever re-install and it flips, change to -1.
  private static final double ANGLE_SIGN = -1.0;

    /** Telemetry-only synthetic absolute scale for dashboard display. */
  private static final int ABS_TICKS_PER_REV_UI = 4096;

  // ---------------- Software unwrap tracking ----------------

  /** last wrapped absolute angle (deg) in [0, 360) */
  private double lastAbsDegWrapped = 0.0;

  /** continuous turret angle (deg), 0=robot rear, CCW positive; never clamped */
  private double continuousDeg = 0.0;

  // Unclamped multi-turn angle directly from motor position (deg).
  // This is what we use for visualization wrapping in Mechanism2d.
  private double continuousDegUnclamped = 0.0;

  /** derived velocity estimate */
  private double lastContinuousDeg = 0.0;
  private double lastUpdateTs = Timer.getFPGATimestamp();
  private double estVelDegPerSec = 0.0;
  private double rawVelDegPerSec = 0.0;
  private final LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

  /** continuous target angle (deg) inside the configured perimeter range */
  private double targetDeg = 0.0;

  /** turret ZERO reference in degrees in the absolute sensor frame */
private final double forwardDeg =
    Constants.OperatorConstants.Turret.ABS_ZERO_ROTATIONS * 360.0;
  /** Whether the most recent CANcoder-based seed fell inside the allowed boot window. */
  private boolean lastSeedWasValid = false;
  /** Whether the current zero/integrated seed came from the CANcoder absolute reading. */
  private boolean zeroCalibratedFromAbsolute = false;
  private boolean hardwareConfigured = false;
  // ---------------- Continuous wrap toggling ----------------

  // Tracks the currently-applied wrap mode (so we don't spam configs).
  // With a ±180 turret and the requirement (-170 -> +170 goes through 0),
  // continuous wrap MUST remain disabled.
  private boolean continuousWrapEnabled = false;
  private final ClosedLoopGeneralConfigs clWrapOff = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

  // ---------------- Simulation ----------------
  private Mechanism2d turretMech;
  private MechanismLigament2d turretArm;

  // Sim model used only in simulationPeriodic().
  private final frc.robot.simulation.RotaryMotorSim turretSim = RobotBase.isSimulation()
      ? new frc.robot.simulation.RotaryMotorSim(1, Turret.SIM_TURRET_J_KGM2, Turret.SIM_GEAR_RATIO) : null;

  // ---------------- SysId Characterization ----------------

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Seconds).of(Constants.OperatorConstants.SysId.TURRET_RAMP_RATE_V_PER_S),
              Volts.of(Constants.OperatorConstants.SysId.TURRET_STEP_V),
              Seconds.of(Constants.OperatorConstants.SysId.TURRET_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "turret"));

  /**
   * Runtime gating for SysId. Requires BOTH compile-time enable and dashboard enable.

   */
  private boolean isSysIdEnabled() {
    if (!hardwareConfigured || !edu.wpi.first.wpilibj.DriverStation.isTestEnabled() || frc.robot.RobotContainer.isPanicStopActive()) return false;
    // Both gates must be true to allow SysId to actually move hardware.
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public TurretSubsystem() {
    if(!EnabledSubsystems.turret){
      return;
    }
   turret = new TalonFX(Constants.OperatorConstants.Turret.MOTOR_ID,
        Constants.OperatorConstants.Turret.CANBUS_NAME);

    motorVoltageSig = turret.getMotorVoltage();

    motorPosSig = turret.getPosition();
    motorVelSig = turret.getVelocity();

    // Hardware config: motor output + current limits + gains.
    configureHardware();

    // CAN signal update rates (reduces bus load but keeps control inputs fresh).
    configureStatusSignals();

    if (isSim) {
      // Synthetic physical stow; production still obtains the actual pinion reading.
      throughboreCANcoder.getSimState().setRawPosition(
          Turret.ABS_ZERO_ROTATIONS - Turret.CANCODER_MAGNET_OFFSET_ROT);
    }
    // Seed continuous angle from absolute on boot.
    seedFromAbsoluteAtBoot();
    turret.hasResetOccurred(); // Consume startup reset; later motor resets require disabled reseeding.

    // Dashboard defaults.
    // SmartDashboard.putBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
    // SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);

    if (Constants.DebugTelemetrySubsystems.turret) {
        turretMech = new Mechanism2d(2.0, 2.0);
        MechanismRoot2d root = turretMech.getRoot("TurretRoot", 1.0, 1.0);
        turretArm = new MechanismLigament2d("TurretArm", 0.8, 0.0);
        root.append(turretArm);

        // SmartDashboard.putData("Turret/Mechanism", turretMech);
    }

  }

  private void configureStatusSignals() {
    // We want absolute angle to update quickly for unwrap math + control decisions.
    absPosSig.setUpdateFrequency(100.0);

    // Integrated motor position (used for continuous angle tracking once seeded).
    motorPosSig.setUpdateFrequency(100.0);

    // Use the Talon's own velocity estimate instead of differentiating position in Java.
    motorVelSig.setUpdateFrequency(100.0);

    // Motor voltage can be slower; mostly for telemetry and SysId.
    motorVoltageSig.setUpdateFrequency(50.0);

    // Let Phoenix reduce unnecessary CAN chatter.
    turret.optimizeBusUtilization();
  }

  private InvertedValue motorInvertedValue() {

    // Convert the project constant into Phoenix's inversion enum.
    return Constants.OperatorConstants.Turret.MOTOR_INVERTED
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
  }

  private SensorPhaseValue sensorPhaseValue() {
    // Sensor phase aligns the remote sensor's positive direction with the motor/controller convention.

    boolean inverted = Constants.OperatorConstants.Turret.SENSOR_PHASE_INVERTED;
    return inverted ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned;
  }

    private void configureHardware() {
  CANcoderConfiguration ccfg = new CANcoderConfiguration();
  ccfg.MagnetSensor.MagnetOffset =
      Constants.OperatorConstants.Turret.CANCODER_MAGNET_OFFSET_ROT;
  throughboreCANcoder.getConfigurator().apply(ccfg);

  // Motor output (brake + inversion)
  MotorOutputConfigs out = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(motorInvertedValue());

  // Current limits
  CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
  limits.SupplyCurrentLimitEnable = true;
  limits.SupplyCurrentLimit = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LIMIT_A;
  limits.SupplyCurrentLowerLimit = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LOWER_LIMIT_A;
  limits.SupplyCurrentLowerTime = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LOWER_TIME_S;
  limits.StatorCurrentLimitEnable = true;
  limits.StatorCurrentLimit = Constants.OperatorConstants.Turret.STATOR_CURRENT_LIMIT_A;

  // Slot0 gains for hardware position loop (voltage-based in Phoenix 6)
  Slot0Configs slot0 = new Slot0Configs()
      .withKP(Constants.OperatorConstants.Turret.kP)
      .withKI(Constants.OperatorConstants.Turret.kI)
      .withKD(Constants.OperatorConstants.Turret.kD)
      .withKS(Constants.OperatorConstants.Turret.kS)
      .withKV(Constants.OperatorConstants.Turret.kV)
      .withKA(Constants.OperatorConstants.Turret.kA);

  double cruiseMotorRps = (Constants.OperatorConstants.Turret.MM_CRUISE_DEG_PER_SEC / 360.0)
      * Constants.OperatorConstants.Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT;

  double accelMotorRps2 = (Constants.OperatorConstants.Turret.MM_ACCEL_DEG_PER_SEC2 / 360.0)
      * Constants.OperatorConstants.Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT;

  MotionMagicConfigs mm = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(cruiseMotorRps)
      .withMotionMagicAcceleration(accelMotorRps2);

  // IMPORTANT: Turret closed-loop uses the TalonFX integrated (relative) sensor.
  // The CANcoder/ThroughBore is used ONLY to seed the integrated sensor at boot.
  TalonFXConfiguration cfg = new TalonFXConfiguration()
    .withMotorOutput(out)
    .withCurrentLimits(limits)
    .withSlot0(slot0)
    .withMotionMagic(mm)
    .withClosedLoopGeneral(clWrapOff)
    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ANGLE_SIGN * motorRotFromTurretDeg(Turret.MIN_ANGLE_DEG))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ANGLE_SIGN * motorRotFromTurretDeg(Turret.MAX_ANGLE_DEG)));

  hardwareConfigured = turret.getConfigurator().apply(cfg).isOK();
}

  public final double getRelativePosition() {
    // Integrated/relative position from CANcoder in rotations (does not wrap in the same way).
    return throughboreCANcoder.getPosition().getValueAsDouble();
  }

    /** Absolute throughbore position for telemetry (wraps every 1 rotation at -0.5..+0.5). */
  public final double getAbsolutePosition() {
    // Returns approximately [-0.5, +0.5), wrapping across the half-turn boundary.
    return throughboreCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  // private void setContinuousWrap(boolean enable) {
  //   // If we're already in the desired wrap mode, do nothing (avoid CAN config spam).
  //   if (enable == continuousWrapEnabled) return;

  //   // Apply the wrap config to the motor controller.
  //   turret.getConfigurator().apply(enable ? clWrapOn : clWrapOff);

  //   // Record state and publish for debugging.
  //   continuousWrapEnabled = enable;
  //   SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", enable);
  // }

  /**
   * Read absolute angle (deg) in [0, 360).
   * If signal is stale, returns last known value.
   */
  private double getAbsDegWrapped(boolean refreshSignal) {
    if (refreshSignal) {
      absPosSig.refresh();
    }

    StatusCode status = absPosSig.getStatus();

    if (status != StatusCode.OK) {
      return lastAbsDegWrapped;
    }

    double rot = absPosSig.getValueAsDouble();
    rot = rot - Math.floor(rot);

    double deg = rot * 360.0;
    return wrapTo0To360(deg);
  }

  private double shortestAbsDeltaDeg(double aDeg, double bDeg) {
    return Math.abs(wrapToPlusMinus180(aDeg - bDeg));
  }

  private double computeTurretSeedDegFromAbsoluteRot(double absRot) {
    double motorRotError = MathUtil.inputModulus(
        absRot - Constants.OperatorConstants.Turret.ABS_ZERO_ROTATIONS,
        -0.5,
        0.5);

    // CANcoder absolute deltas already come back with the same sign convention we want
    // for turret angle after wrapping to the nearest offset from zero:
    // CW from zero => negative, CCW from zero => positive.
    return motorRotError
        * 360.0
        * Constants.OperatorConstants.Turret.GEAR_RATIO_TURRET_ROT_PER_MOTOR_ROT;
  }

  private void publishSeedTelemetry(
      String prefix,
      double absRot,
      double motorRotError,
      double turretSeedDeg,
      boolean validSeed) {
    if (DebugTelemetrySubsystems.turret || DebugTelemetrySubsystems.calibration) {
      SmartDashboard.putNumber(prefix + "AbsRot", absRot);
      SmartDashboard.putNumber(prefix + "MotorRotError", motorRotError);
      SmartDashboard.putNumber(prefix + "TurretDeg", turretSeedDeg);
      SmartDashboard.putBoolean(prefix + "Valid", validSeed);
    }
  }

  private double sampleAbsoluteForSeed() {
    double startTs = Timer.getFPGATimestamp();
    double lastValidAbsDeg = lastAbsDegWrapped;
    double lastSampleAbsDeg = Double.NaN;
    int stableSamples = 0;
    boolean sawValidSample = false;

    while (Timer.getFPGATimestamp() - startTs < ABS_SEED_TIMEOUT_SEC) {
      absPosSig.refresh();

      if (absPosSig.getStatus() == StatusCode.OK) {
        double absDeg = getAbsDegWrapped(false);
        lastValidAbsDeg = absDeg;
        sawValidSample = true;

        if (!Double.isFinite(lastSampleAbsDeg)
            || shortestAbsDeltaDeg(absDeg, lastSampleAbsDeg) <= ABS_SEED_STABLE_TOLERANCE_DEG) {
          stableSamples++;
        } else {
          stableSamples = 1;
        }

        lastSampleAbsDeg = absDeg;

        if (stableSamples >= ABS_SEED_REQUIRED_STABLE_SAMPLES) {
          return absDeg;
        }
      }

      Timer.delay(ABS_SEED_SAMPLE_PERIOD_SEC);
    }

    return Double.NaN; // Never seed from a stale/default angle after a failed or unstable capture.
  }

  /**
   * Seed from the pinion encoder only while physically stowed within half a pinion revolution of zero.
   * With the existing 11:1 ratio this is +/-16.36 turret degrees; turn identity is unobservable.
   */
  private void seedFromAbsoluteAtBoot() {
    double absDeg = sampleAbsoluteForSeed();
    double absRot = absDeg / 360.0;

    // Initialize last wrapped state for future delta calculations.
    lastAbsDegWrapped = absDeg;

    double motorRotError = MathUtil.inputModulus(
        absRot - Constants.OperatorConstants.Turret.ABS_ZERO_ROTATIONS,
        -0.5,
        0.5);
    double deltaDeg = computeTurretSeedDegFromAbsoluteRot(absRot);
    boolean validSeed =
        Math.abs(deltaDeg) <= Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG;
    lastSeedWasValid = validSeed;

    publishSeedTelemetry("Turret/Seed/", absRot, motorRotError, deltaDeg, validSeed);

    if (!validSeed) {
      continuousDeg = 0.0;
      continuousDegUnclamped = 0.0;
      targetDeg = 0.0;
      zeroCalibratedFromAbsolute = false;
      turret.setPosition(0.0);
      lastContinuousDeg = continuousDeg;
      lastUpdateTs = Timer.getFPGATimestamp();
      return;
    }

    // Initialize continuous turret position in your forward-relative frame.
    continuousDeg = deltaDeg;
    continuousDegUnclamped = deltaDeg;

    // Seed the TalonFX integrated position so closed-loop uses relative sensor from
    // this point.
    // TalonFX position units are rotations; we keep continuousDeg in degrees.
    // Seed TalonFX integrated position in *motor* rotations, not turret rotations.
    double motorRot = motorRotFromTurretDeg(continuousDeg);
    // alex test
    zeroCalibratedFromAbsolute = hardwareConfigured && turret.setPosition(ANGLE_SIGN * motorRot).isOK();

    // Initialize velocity bookkeeping.
    lastContinuousDeg = continuousDeg;

    // Initialize dt tracking so velocity math starts clean.
    lastUpdateTs = Timer.getFPGATimestamp();

    // Start target at current so there is no immediate step command.
    targetDeg = continuousDeg;

    // Publish seed diagnostics to dashboard.
    // SmartDashboard.putNumber("Turret/SeedAbsDeg", absDeg);
    // SmartDashboard.putNumber("Turret/SeedContinuousDeg", continuousDeg);
  }

  public void zeroTurretAngle() {
    reseedIntegratedFromAbsoluteNow();
  }

  /**
   * Update continuous angle from trusted integrated motor position without clamping.

   */
  private void updateContinuousAngle() {
    double now = Timer.getFPGATimestamp();

    BaseStatusSignal.refreshAll(motorPosSig, motorVelSig);

    if (motorPosSig.getStatus() != StatusCode.OK || motorVelSig.getStatus() != StatusCode.OK) {
      //SmartDashboard.putString("Turret/MotorPosStatus", motorPosSig.getStatus().toString());
      lastUpdateTs = now;
      return;
    }

    double motorRotSensor = motorPosSig.getValueAsDouble();

    // Undo ANGLE_SIGN so motorRot is positive in your "CCW positive" turret
    // convention.
    double motorRot = motorRotSensor / ANGLE_SIGN;

    // Convert motor rotations -> turret degrees using gear ratio.
    double nextUnclamped = turretDegFromMotorRot(motorRot);

    double motorVelRps = motorVelSig.getValueAsDouble() / ANGLE_SIGN;
    rawVelDegPerSec = turretDegFromMotorRot(motorVelRps);
    estVelDegPerSec = velocityFilter.calculate(rawVelDegPerSec);

    // Commit state.
    lastContinuousDeg = continuousDeg;          // keep last clamped value for any debugging
    continuousDegUnclamped = nextUnclamped;     // used for Mechanism wrapping / display
    continuousDeg = nextUnclamped; // Report real overtravel; clamping feedback hides a limit violation.
    lastUpdateTs = now;

    if (DebugTelemetrySubsystems.turret || DebugTelemetrySubsystems.calibration) {
      SmartDashboard.putNumber("Turret/MeasuredContinuousDegUnclamped", continuousDegUnclamped);
      SmartDashboard.putNumber("Turret/MotorSensorRot", motorRotSensor);
    }

    // Keep abs wrapped for telemetry/diagnostics
  lastAbsDegWrapped = getAbsDegWrapped(false);
  }

  // ---------------- Public API ----------------

  /** Continuous turret angle (deg), 0 = forward, CCW positive. */
  public double getAngleDeg() {
    // Unclamped measured angle; a saturated display must never conceal an overshoot.
    return continuousDeg;
  }

  public double getVelocityDegPerSec() {
    // Estimated velocity from continuous angle updates.
    return estVelDegPerSec;
  }

  public double getAppliedVolts() {
    // Refresh motor voltage signal (ensures telemetry reflects current output).
    if (motorVoltageSig == null) return Double.NaN;
    motorVoltageSig.refresh();
    return motorVoltageSig.getValueAsDouble();
  }

  /** Absolute ticks (0-4095 equivalent) from wrapped absolute sensor. */
  public int getAbsoluteTicks() {
    // Convert last wrapped absolute degrees to a ticks-per-rev representation.
    double absDeg = lastAbsDegWrapped;

    // Scale degrees -> ticks and round to nearest int.
    int ticks = (int) Math.round((absDeg / 360.0) * ABS_TICKS_PER_REV_UI);
    // Wrap into [0, ticksPerRev).
    ticks %= ABS_TICKS_PER_REV_UI;
    if (ticks < 0) ticks += ABS_TICKS_PER_REV_UI;
    return ticks;

  }

  /** Open-loop manual control with safety clamp. */
  public void setDutyCycle(double duty) {
    if (!edu.wpi.first.wpilibj.DriverStation.isEnabled() || !isPositionTrusted() || !Double.isFinite(duty) || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    // Clamp duty to avoid commanding beyond your configured safe range.
    double maxDuty = isSim
      ? Constants.OperatorConstants.Turret.SIM_MAX_DUTY_CYCLE
      : Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE;

    duty = MathUtil.clamp(duty, -maxDuty, maxDuty);

    // Send open-loop command to the motor controller.

    // alex test
    turret.setControl(dutyRequest.withOutput(duty));
  }

  public void setVoltageVolts(double volts) {
    if (!edu.wpi.first.wpilibj.DriverStation.isEnabled() || !isPositionTrusted() || !Double.isFinite(volts) || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    // Clamp request to something physically plausible.
    // In sim we’ll assume 12V supply; on real robot, you can clamp to battery if you want.
    double v = MathUtil.clamp(volts, -12.0, 12.0);

    // alex test
    turret.setControl(voltageRequest.withOutput(v));
  }

  public void stop() {
    if (!EnabledSubsystems.turret || turret == null) {
      return;
    }
    // Immediately stop output.
    turret.stopMotor();
  }

  /** Current continuous turret angle in degrees in this subsystem's reference frame (0 = "forward" per ABS_FORWARD_TICKS, CCW+). */
  public double getContinuousAngleDeg() {
    return continuousDeg;
  }

  public double getRelativeAngleFromZeroDeg() {
    return continuousDeg;
  }

  public Translation2d getTurretCenterFieldMeters() {
    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    return robotPose.getTranslation().plus(
        Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
            .rotateBy(robotPose.getRotation()));
  }

  public double getTurretAbsoluteFieldDeg() {
    double robotHeadingDeg = RobotContainer.driveSubsystem.getPose().getRotation().getDegrees();
    return wrapToPlusMinus180(
        robotHeadingDeg
            + Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG
            + getRelativeAngleFromZeroDeg());
  }

  /** Estimated turret angular velocity in deg/sec (sign matches getContinuousAngleDeg convention). */
  public double getEstimatedVelocityDegPerSec() {
    return estVelDegPerSec;
  }

  public boolean isPositionTrusted() {
    return turret != null && hardwareConfigured && zeroCalibratedFromAbsolute
        && motorPosSig.getStatus().isOK() && motorVelSig.getStatus().isOK()
        && motorPosSig.getTimestamp().getLatency() < 0.1 && motorVelSig.getTimestamp().getLatency() < 0.1
        && Double.isFinite(rawVelDegPerSec)
        && Double.isFinite(continuousDegUnclamped);
  }

  public void goToAngleDeg(double desiredDeg) {
    var target = TurretMotionPolicy.motorPositionTarget(desiredDeg, Turret.MIN_ANGLE_DEG,
        Turret.MAX_ANGLE_DEG, Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT, ANGLE_SIGN,
        isPositionTrusted() && edu.wpi.first.wpilibj.DriverStation.isEnabled() && !RobotContainer.isPanicStopActive());
    if (target.isEmpty()) { stop(); return; }
    targetDeg = MathUtil.clamp(desiredDeg, Turret.MIN_ANGLE_DEG, Turret.MAX_ANGLE_DEG);
    // Always issue the current setpoint, including small corrections and recovery from open-loop stop.
    turret.setControl(mmRequest.withPosition(target.getAsDouble()));
    Logger.recordOutput("Turret/DesiredDegInput", desiredDeg);
    Logger.recordOutput("Turret/TargetDeg", targetDeg);
    Logger.recordOutput("Turret/MotorRotTarget", target.getAsDouble());
  }

  /** true if turret is within tolerance of desired angle (deg), using best safe equivalent. */
  public boolean atAngleDeg(double desiredDeg, double toleranceDeg) {
    return TurretMotionPolicy.atTarget(continuousDegUnclamped, desiredDeg, toleranceDeg,
        Turret.MIN_ANGLE_DEG, Turret.MAX_ANGLE_DEG, isPositionTrusted());
  }

  /**
   * Convert a continuous target (deg relative to forward) into the wrapped absolute
   * rotation value in the signed CANcoder frame [-0.5, +0.5).
   */
  private double wrappedRotFromContinuousDeg(double continuousDegTarget) {
    // Convert turret angle back into the pinion CANcoder frame.
    double motorRot =
        (continuousDegTarget / 360.0)
            * Constants.OperatorConstants.Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT;
    double absRot = Constants.OperatorConstants.Turret.ABS_ZERO_ROTATIONS + (motorRot / ANGLE_SIGN);

    return MathUtil.inputModulus(absRot, -0.5, 0.5);
  }

  private double motorRotFromTurretDeg(double turretDeg) {
    return turretDeg * Constants.OperatorConstants.Turret.MOTOR_ROT_PER_TURRET_DEG;
  }

  private double turretDegFromMotorRot(double motorRot) {
    return motorRot * Constants.OperatorConstants.Turret.TURRET_DEG_PER_MOTOR_ROT;
  }

  // ============================CALIBRATION METHODS===============================

  public void reseedIntegratedFromAbsoluteNow() {
    if (!edu.wpi.first.wpilibj.DriverStation.isDisabled() || turret == null) return;
    double absRot = sampleAbsoluteForSeed() / 360.0;
    double absDeg = wrapTo0To360(absRot * 360.0);
    lastAbsDegWrapped = absDeg;

    double motorRotError = MathUtil.inputModulus(
        absRot - Constants.OperatorConstants.Turret.ABS_ZERO_ROTATIONS,
        -0.5,
        0.5);
    double deltaDeg = computeTurretSeedDegFromAbsoluteRot(absRot);
    boolean validSeed =
        Math.abs(deltaDeg) <= Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG;
    lastSeedWasValid = validSeed;

    publishSeedTelemetry("Turret/Reseed/", absRot, motorRotError, deltaDeg, validSeed);

    if (!validSeed) {
      zeroCalibratedFromAbsolute = false;
      return;
    }

    continuousDeg = deltaDeg;
    continuousDegUnclamped = deltaDeg;

    double motorRot = motorRotFromTurretDeg(continuousDeg);

    // alex test
    zeroCalibratedFromAbsolute = hardwareConfigured && turret.setPosition(ANGLE_SIGN * motorRot).isOK();

    targetDeg = continuousDeg;
    if (DebugTelemetrySubsystems.turret || DebugTelemetrySubsystems.calibration) {
      SmartDashboard.putNumber("Turret/ReseedAbsDegWrapped", absDeg);
      SmartDashboard.putNumber("Turret/ReseedContinuousDeg", continuousDeg);
    }
  }

  public void captureAbsForwardTicksCandidate() {
    int ticks = getAbsoluteTicks();
    if (DebugTelemetrySubsystems.turret || DebugTelemetrySubsystems.calibration) {
      SmartDashboard.putNumber("Turret/AbsForwardTicksCandidate", ticks);
    }
  }

  // ---------------- SysId commands ----------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return frc.robot.commands.GuardedSysId.wrap(sysIdRoutine.quasistatic(direction),
        this::isSysIdEnabled, this::stop);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return frc.robot.commands.GuardedSysId.wrap(sysIdRoutine.dynamic(direction),
        this::isSysIdEnabled, this::stop);
  }

  // ---------------- SysId callbacks ----------------

  private void sysIdVoltageDrive(Voltage volts) {
    // Safety: if SysId isn't enabled, ensure turret is stopped.
    if (!isSysIdEnabled()) {
      stop();
      return;
    }

    // Apply the requested voltage directly.
    // Clamp to something physically plausible; Phoenix will also saturate to supply.
    double v = volts.in(Volts);
    double maxV = RobotController.getBatteryVoltage();
    v = MathUtil.clamp(v, -maxV, maxV);

    setVoltageVolts(v);
  }

  private void sysIdLog(SysIdRoutineLog log) {
    // Only log when SysId is enabled.
    if (!isSysIdEnabled()) return;

    log.motor("turret")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(continuousDegUnclamped / 360.0))
        .angularVelocity(RotationsPerSecond.of(estVelDegPerSec / 360.0));
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.turret) {
      return;
    }

    // Update continuous (multi-turn) angle state every loop.
    if (turret.hasResetOccurred()) zeroCalibratedFromAbsolute = false;
    updateContinuousAngle();
    if (!isPositionTrusted() || !edu.wpi.first.wpilibj.DriverStation.isEnabled() || RobotContainer.isPanicStopActive()) stop();
    Logger.recordOutput("Turret/PositionTrusted", isPositionTrusted());
    Logger.recordOutput("Turret/MeasuredDegrees", continuousDegUnclamped);
    Logger.recordOutput("Turret/VelocityDegPerSec", rawVelDegPerSec);
    Logger.recordOutput("Turret/TargetDegrees", targetDeg);
    if (DebugTelemetrySubsystems.turret || DebugTelemetrySubsystems.calibration) {
      Translation2d turretCenterField = getTurretCenterFieldMeters();
      SmartDashboard.putNumber("Turret/RelativeAngleFromZeroDeg", getRelativeAngleFromZeroDeg());
      SmartDashboard.putNumber("Turret/CenterFieldX", turretCenterField.getX());
      SmartDashboard.putNumber("Turret/CenterFieldY", turretCenterField.getY());
      SmartDashboard.putNumber("Turret/AbsoluteFieldDeg", getTurretAbsoluteFieldDeg());
      SmartDashboard.putNumber("Turret/CANcoderAbsoluteRot", getAbsolutePosition());
      SmartDashboard.putNumber(
          "Turret/CANcoderMagnetOffsetRot",
          Constants.OperatorConstants.Turret.CANCODER_MAGNET_OFFSET_ROT);
      SmartDashboard.putBoolean("Turret/SeedValid", lastSeedWasValid);
      SmartDashboard.putBoolean("Turret/ZeroCalibratedFromAbsolute", zeroCalibratedFromAbsolute);
    }
    if(DebugTelemetrySubsystems.turret){
    // Telemetry block: expose key state for debugging and tuning.
      SmartDashboard.putNumber("Turret/VelDegPerSec", getVelocityDegPerSec());
      SmartDashboard.putNumber("Turret/VelDegPerSecRaw", rawVelDegPerSec);
      SmartDashboard.putNumber("Turret/AngleDeg_Unclamped", continuousDegUnclamped);
      SmartDashboard.putNumber("Turret/AngleDeg_Wrapped0to360", wrapTo0To360(continuousDegUnclamped));
      SmartDashboard.putNumber("Turret/AppliedVolts", getAppliedVolts());
      SmartDashboard.putNumber("Turret/AbsTicks", getAbsoluteTicks());
      SmartDashboard.putNumber("Turret/AbsDegWrapped", lastAbsDegWrapped);
      SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
      SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);
      SmartDashboard.putNumber("Turret/ErrorDeg", targetDeg - continuousDeg);
      SmartDashboard.putBoolean("Turret/At1Deg", Math.abs(targetDeg - continuousDeg) <= 1.0);
    }

    if (Constants.DebugTelemetrySubsystems.turret && turretArm != null) {
      turretArm.setAngle(wrapTo0To360(continuousDegUnclamped));
    }

  }

  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.turret) {
      return 0.0;
    }
    return turretSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim || !EnabledSubsystems.turret) return;
    var simState = turret.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    turretSim.update(simState.getMotorVoltage(), .020);
    double rotorRot = turretSim.rotorPositionRotations();
    double rotorRps = turretSim.rotorVelocityRps();
    simState.setRawRotorPosition(rotorRot);
    simState.setRotorVelocity(rotorRps);
    // CANcoder is on the pinion, with the opposite sign to the integrated motor convention.
    // Supply raw position before the real CANcoder magnet-offset configuration is applied.
    var encoderSim = throughboreCANcoder.getSimState();
    encoderSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    encoderSim.setRawPosition(Turret.ABS_ZERO_ROTATIONS - Turret.CANCODER_MAGNET_OFFSET_ROT + rotorRot / ANGLE_SIGN);
    encoderSim.setVelocity(rotorRps / ANGLE_SIGN);
  }

  // ---------------- Helpers ----------------

  private static double wrapTo0To360(double deg) {
    // Wrap any degrees into [0,360).
    double d = deg % 360.0;
    if (d < 0) d += 360.0;
    return d;
  }

    /** wrap to (-180, 180] */
  private static double wrapToPlusMinus180(double deg) {
    // Wrap degrees into (-180,180] to compute shortest signed difference.
    double d = ((deg + 180.0) % 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
  }

}
