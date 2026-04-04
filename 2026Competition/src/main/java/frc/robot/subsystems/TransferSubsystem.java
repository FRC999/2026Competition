package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.RobotContainer;

/**
 * TransferSubsystem
 *
 * Purpose:
 * - Meter balls from the spindexer into the turret/shooter with consistent
 * velocity and minimal spin.
 * - "Stage" a ball at the shooter throat (exit of transfer) so the shot can
 * happen immediately when allowed.
 *
 * Recommended sensor layout (your chosen 2-sensor setup):
 * - ENTRY sensor: placed just AFTER the spindexer handoff, inside the transfer
 * tunnel.
 * This avoids false triggers from spindexer blades.
 * - THROAT sensor: placed at the exit of transfer (right before the
 * shooter/turret throat).
 *
 * Hardware:
 * - One motor today. You reserved IDs for a second motor later.
 */
public class TransferSubsystem extends SubsystemBase {

  private TalonFX motor;
  private DutyCycleOut duty;

  // Sensors (beam breaks are typical). Wiring convention varies; we invert using
  // constants.
  // Sensors (CANrange). We read via StatusSignals so we can refresh them in
  // periodic().
  // IR beam-break sensors (DIO)
  private DigitalInput entrySensor;
  private DigitalInput throatSensor;

  // Closed-loop velocity (primary control mode)
  private VelocityDutyCycle velocityDuty = new VelocityDutyCycle(0.0);

  private StatusSignal<Angle> positionSig;
  private StatusSignal<AngularVelocity> velocitySig;
  private StatusSignal<Voltage> motorVoltageSig;
  private double posRot = 0.0;
  private double velRps = 0.0;
  private double commandedDuty = 0.0;
  private double commandedRps = 0.0;

  // --- Calibration state (for calibration bindings + AdvantageScope visibility)
  // ---
  private String calMode = "OFF";
  private double calStageRpsSet = 0.0;
  private double calFeedRpsSet = 0.0;
  private double calBlockedStageRpsSet = 0.0;
    // --- Entry -> Throat timing telemetry ---
  private boolean prevBallAtEntry = false;
  private boolean prevBallAtThroat = false;

  /** True while waiting for a ball that hit entry to later hit throat. */
  private boolean entryToThroatTimingActive = false;

  /** FPGA timestamp when the most recent entry rising edge occurred. */
  private double entryToThroatStartTs = -1.0;

  /** Most recently completed entry -> throat travel time, in seconds. */
  private double lastEntryToThroatTimeSec = -1.0;

  /** Live elapsed time while timing is active, in seconds. */
  private double currentEntryToThroatElapsedSec = 0.0;

  // ---------------- Metered eject state ----------------
  private enum EjectState {
    IDLE, EJECTING, COOLDOWN
  }

  private EjectState ejectState = EjectState.IDLE;

  /** Timestamp when current eject started (sec). */
  private double ejectStartTs = -1.0;

  /** Timestamp when last eject started (sec) - used for rate limiting. */
  private double lastEjectStartTs = -1.0;

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(Constants.OperatorConstants.SysId.TRANSFER_RAMP_RATE_V_PER_S),
          Volts.of(Constants.OperatorConstants.SysId.TRANSFER_STEP_V),
          Seconds.of(Constants.OperatorConstants.SysId.TRANSFER_TIMEOUT_S)),
      new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "transfer"));

  private boolean isSysIdEnabled() {
    if (!Constants.OperatorConstants.SysId.ENABLE_SYSID) {
      return false;
    }
    return SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  // ---------------- Simulation ----------------
  private final boolean isSim = RobotBase.isSimulation();
  private final FlywheelSim transferSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          Constants.OperatorConstants.Transfer.SIM_GEAR_RATIO,
          Constants.OperatorConstants.Transfer.SIM_J_KGM2),
      DCMotor.getKrakenX60(1));
  private double simPosRot = 0.0;

  public TransferSubsystem() {
    if (!EnabledSubsystems.transfer) {
      return;
    }

    // Initialize IR beam-break sensors
    entrySensor = new DigitalInput(
        Constants.OperatorConstants.Transfer.ENTRY_SENSOR_DIO);

    throatSensor = new DigitalInput(
        Constants.OperatorConstants.Transfer.THROAT_SENSOR_DIO);

    duty = new DutyCycleOut(0.0);
    motor = new TalonFX(
        Constants.OperatorConstants.Transfer.MOTOR_ID,
        Constants.OperatorConstants.Transfer.CANBUS_NAME);
    // ---------------- Motor configuration ----------------
    var cfg = new TalonFXConfiguration();

    // Coast requested
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits (reasonable defaults; TODO verify/tune)
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.OperatorConstants.Transfer.SUPPLY_CURRENT_LIMIT_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = Constants.OperatorConstants.Transfer.SUPPLY_CURRENT_LOWER_LIMIT_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = Constants.OperatorConstants.Transfer.SUPPLY_CURRENT_LOWER_TIME_S;

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Constants.OperatorConstants.Transfer.STATOR_CURRENT_LIMIT_A;

    // Slot0 closed-loop gains (TODO placeholders)
    cfg.Slot0.kS = Constants.OperatorConstants.Transfer.VEL_kS;
    cfg.Slot0.kV = Constants.OperatorConstants.Transfer.VEL_kV;
    cfg.Slot0.kP = Constants.OperatorConstants.Transfer.VEL_kP;
    cfg.Slot0.kI = Constants.OperatorConstants.Transfer.VEL_kI;
    cfg.Slot0.kD = Constants.OperatorConstants.Transfer.VEL_kD;

    motor.getConfigurator().apply(cfg);

    positionSig = motor.getPosition();
    velocitySig = motor.getVelocity();
    motorVoltageSig = motor.getMotorVoltage();

    positionSig.setUpdateFrequency(50.0);
    velocitySig.setUpdateFrequency(50.0);
    motorVoltageSig.setUpdateFrequency(50.0);
    motor.optimizeBusUtilization();
  }

  /** Run transfer at a raw duty cycle in [-1, +1]. */
  public void runDuty(double dutyCycle) {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    commandedDuty = dutyCycle;
    motor.setControl(duty.withOutput(dutyCycle));
  }

  /** Run transfer at a target rotor speed in RPS (closed-loop). */
  public void runVelocityRps(double targetRps) {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    commandedRps = targetRps;
    motor.setControl(velocityDuty.withVelocity(targetRps));
  }

  /** Stop transfer. */
  public void stop() {
    if (!EnabledSubsystems.transfer || motor == null) {
      return;
    }

    commandedDuty = 0.0;
    commandedRps = 0.0;
    motor.setControl(duty.withOutput(0.0));
  }

  /**
   * Run transfer at the configured staging speed (closed-loop), but do not push
   * if throat is already occupied.
   */
  public void runStage() {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    // if (hasBallAtThroat()) {
    //   motor.set(-0.4);
    //   return;
    // }
     motor.set(-0.60);
  }

  /**
   * Calibration-only: run stage using a live-tunable setpoint, with throat
   * protection.
   */
  public void runStageCal(double stageRpsSet, double blockedStageRpsSet) {
    calMode = "CAL_STAGE";
    calStageRpsSet = stageRpsSet;
    calBlockedStageRpsSet = blockedStageRpsSet;

     if (hasBallAtThroat()) {
       runVelocityRps(blockedStageRpsSet);
     } else {
      runVelocityRps(stageRpsSet);
    }
  }

  /** Calibration-only: run feed using a live-tunable setpoint. */
  public void runFeedCal(double feedRpsSet) {
    calMode = "CAL_FEED";
    calFeedRpsSet = feedRpsSet;
    runVelocityRps(feedRpsSet);
  }

  /** Calibration-only: stop and mark mode. */
  public void stopCal() {
    calMode = "CAL_STOP";
    stop();
  }

  /**
   * Run transfer at the configured feed speed (closed-loop) to inject a ball into
   * the shooter.
   */
  public void runFeed() {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    motor.set(-0.90);
    //runVelocityRps(Constants.OperatorConstants.Transfer.FEED_RPS);
  }

  public void reverseTransfer() {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    motor.set(0.80);
    //runVelocityRps(Constants.OperatorConstants.Transfer.FEED_RPS);
  }

  public void runThroat() {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    motor.set(-0.40);
    //runVelocityRps(Constants.OperatorConstants.Transfer.FEED_RPS);
  }

  /**
   * Metered feed/eject:
   * - Ejects ONE ball by running at FEED_RPS until the throat sensor clears (ball
   * leaves throat).
   * - Rate-limits how often ejection can start (EJECT_MIN_INTERVAL_S).
   * - Uses EJECT_MAX_TIME_S as a safety timeout.
   *
   * Call this repeatedly while you "want to fire" (e.g., in AutoShootSupervisor
   * FIRING state).
   */
  public void runFeedMetered() {
    if (!EnabledSubsystems.transfer || RobotContainer.isPanicStopActive()) {
      stop();
      return;
    }

    double now = Timer.getFPGATimestamp();

    // If we don't currently have a ball at the throat, just stage (but don't shove
    // if already occupied).
    if (!hasBallAtThroat()) {
      ejectState = EjectState.IDLE;
      runStage();
      return;
    }

    // Rate limiting: don't start a new eject too frequently.
    boolean intervalOk = (lastEjectStartTs < 0.0)
        || (now - lastEjectStartTs) >= Constants.OperatorConstants.Transfer.EJECT_MIN_INTERVAL_S;

    switch (ejectState) {
      case IDLE:
        if (!intervalOk) {
          // Hold: throat is full, but we're waiting for rate interval.
          runVelocityRps(0.0);
          ejectState = EjectState.COOLDOWN;
          return;
        }
        // Start an eject
        ejectState = EjectState.EJECTING;
        ejectStartTs = now;
        lastEjectStartTs = now;
        runVelocityRps(Constants.OperatorConstants.Transfer.FEED_RPS);
        return;

      case EJECTING:
        // Continue ejecting until throat clears OR safety timeout trips.
        boolean cleared = !hasBallAtThroat();
        boolean timedOut = (now - ejectStartTs) >= Constants.OperatorConstants.Transfer.EJECT_MAX_TIME_S;

        if (cleared || timedOut) {
          // Stop after one-ball ejection; next loop will stage/re-fill.
          runVelocityRps(0.0);
          ejectState = EjectState.COOLDOWN;
          return;
        }

        runVelocityRps(Constants.OperatorConstants.Transfer.FEED_RPS);
        return;

      case COOLDOWN:
      default:
        // During cooldown, keep staged but do not compress into throat.
        runStage();
        // Once interval is OK again and throat is full, allow next eject cycle.
        if (intervalOk && hasBallAtThroat()) {
          ejectState = EjectState.IDLE;
        }
        return;
    }
  }

  /** @return true if a ball is detected at transfer entry (after spindexer). */
  public boolean hasBallAtEntry() {
    if (!EnabledSubsystems.transfer || entrySensor == null) {
      return false;
    }

    boolean raw = entrySensor.get();
    return Constants.OperatorConstants.Transfer.ENTRY_SENSOR_INVERTED
        ? !raw
        : raw;
  }

  public boolean hasBallAtThroat() {
    if (!EnabledSubsystems.transfer || throatSensor == null) {
      return false;
    }
    // System.out.println("TransferSubsystem: throat sensor is NOT null");


    boolean raw = throatSensor.get();
    return Constants.OperatorConstants.Transfer.THROAT_SENSOR_INVERTED
        ? !raw
        : raw;
  }

  public double getCommandedDuty() {
    return commandedDuty;
  }

  public double getPosRot() {
    return posRot;
  }

  public double getVelRps() {
    return velRps;
  }

  // ---------------- SysId factory commands ----------------
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
    }
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
    }
    return sysIdRoutine.dynamic(direction);
  }

  // ---------------- SysId callbacks ----------------
  private void sysIdVoltageDrive(edu.wpi.first.units.measure.Voltage volts) {
    if (!isSysIdEnabled()) {
      stop();
      return;
    }
    double v = volts.in(Volts);
    double batt = RobotController.getBatteryVoltage();
    if (batt <= 1e-6) {
      stop();
      return;
    }
    double dutyOut = v / batt;
    dutyOut = MathUtil.clamp(dutyOut, -1.0, 1.0);
    runDuty(dutyOut);
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled()) {
      return;
    }
    log.motor("transfer")
        .voltage(Volts.of(motorVoltageSig.getValueAsDouble()))
        .angularPosition(Rotations.of(posRot))
        .angularVelocity(RotationsPerSecond.of(velRps));
  }

    /** Updates telemetry for time taken by a ball to travel from entry sensor to throat sensor. */
  private void updateEntryToThroatTimingTelemetry() {
    boolean ballAtEntry = hasBallAtEntry();
    boolean ballAtThroat = hasBallAtThroat();
    double now = Timer.getFPGATimestamp();

    boolean entryRisingEdge = ballAtEntry && !prevBallAtEntry;
    boolean throatRisingEdge = ballAtThroat && !prevBallAtThroat;

    // Start timing when a new ball first reaches entry.
    if (entryRisingEdge) {
      entryToThroatTimingActive = true;
      entryToThroatStartTs = now;
      currentEntryToThroatElapsedSec = 0.0;
    }

    // Update live elapsed time while timing is active.
    if (entryToThroatTimingActive && entryToThroatStartTs >= 0.0) {
      currentEntryToThroatElapsedSec = now - entryToThroatStartTs;
    } else {
      currentEntryToThroatElapsedSec = 0.0;
    }

    // Finish timing when that ball first reaches throat.
    if (entryToThroatTimingActive && throatRisingEdge && entryToThroatStartTs >= 0.0) {
      lastEntryToThroatTimeSec = now - entryToThroatStartTs;
      entryToThroatTimingActive = false;
      currentEntryToThroatElapsedSec = 0.0;
      entryToThroatStartTs = -1.0;
    }

    prevBallAtEntry = ballAtEntry;
    prevBallAtThroat = ballAtThroat;
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.transfer) {
      return;
    }
    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);
    posRot = positionSig.getValueAsDouble();
    velRps = velocitySig.getValueAsDouble();
    double rpm = velRps;

    updateEntryToThroatTimingTelemetry();

    if (!DebugTelemetrySubsystems.transfer) {
      return;
    }
    boolean ballAtEntry = hasBallAtEntry();
    boolean ballAtThroat = hasBallAtThroat();
    SmartDashboard.putNumber("Transfer/MotorVoltage", motorVoltageSig.getValueAsDouble());
    SmartDashboard.putBoolean("Transfer/BallAtEntry", ballAtEntry);
    SmartDashboard.putBoolean("Transfer/BallAtThroat", ballAtThroat);
    SmartDashboard.putNumber("Transfer/RPS", rpm);


    // --- Calibration visibility (always present; used by calibration bindings) ---
    SmartDashboard.putString("Transfer/Cal/Mode", calMode);
    SmartDashboard.putNumber("Transfer/Cal/StageRpsSet", calStageRpsSet);
    SmartDashboard.putNumber("Transfer/Cal/FeedRpsSet", calFeedRpsSet);
    SmartDashboard.putNumber("Transfer/Cal/BlockedStageRpsSet", calBlockedStageRpsSet);

    // --- Entry -> Throat timing telemetry ---
    SmartDashboard.putBoolean("Transfer/EntryToThroatTimingActive", entryToThroatTimingActive);
    SmartDashboard.putNumber("Transfer/EntryToThroatElapsedSec", currentEntryToThroatElapsedSec);
    SmartDashboard.putNumber("Transfer/LastEntryToThroatTimeSec", lastEntryToThroatTimeSec);
  }

  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.transfer) {
      return 0.0;
    }
    return transferSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim) {
      return;
    }
    if (!EnabledSubsystems.transfer) {
      return;
    }

    final double dt = 0.02;

    var simState = motor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double appliedV = simState.getMotorVoltage();

    transferSim.setInputVoltage(appliedV);
    transferSim.update(dt);

    double rps = transferSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    simPosRot += rps * dt;

    simState.setRawRotorPosition(simPosRot);
    simState.setRotorVelocity(rps);

  }

}
