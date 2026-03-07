package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;

/**
 * SpindexerSubsystem
 *
 * Purpose:
 * - Move balls at the bottom of the hopper so they present consistently to the TransferSubsystem entry.
 * - Automatically unjam by reversing briefly when stator current stays high and velocity collapses.
 *
 * Notes:
 * - This subsystem should NOT be responsible for "exactly one ball into the shooter". That job is TransferSubsystem
 *   using its throat/exit sensor and feed timing.
 */
public class SpindexerSubsystem extends SubsystemBase {

  private enum DesiredMode {
    OFF,
    BASE_FORWARD,
    SUPPLY_FORWARD,
    CAL_BASE_FORWARD,
    CAL_SUPPLY_FORWARD,
    MANUAL_VELOCITY,
    OPEN_LOOP_DUTY
  }

  private enum AntiJamState {
    NORMAL,
    UNJAM_REVERSE,
    UNJAM_SETTLE_FORWARD,
    COOLDOWN
  }

  private TalonFX motor;
  private final DutyCycleOut duty = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

  private DesiredMode desiredMode = DesiredMode.OFF;
  private AntiJamState antiJamState = AntiJamState.NORMAL;

  /** Desired forward operating target. This is what we want to resume after anti-jam. */
  private double desiredForwardRps = 0.0;

  /** Current active closed-loop velocity command actually being sent to the motor. */
  private double targetRps = 0.0;

  private boolean velocityClosedLoopEnabled = false;
  private double commandedDuty = 0.0;

  // --- Calibration state (for calibration bindings + AdvantageScope visibility) ---
  private String calMode = "OFF";
  private double calBaseRpsSet = 0.0;
  private double calSupplyRpsSet = 0.0;

  // Status signals (for telemetry + SysId logs)
  private StatusSignal<Angle> positionSig;
  private StatusSignal<AngularVelocity> velocitySig;
  private StatusSignal<Voltage> motorVoltageSig;
  private StatusSignal<Current> statorCurrentSig;

  private double posRot = 0.0;
  private double velRps = 0.0;
  private double statorCurrentA = 0.0;
  private double velocityErrorRps = 0.0;

  // Anti-jam timing
  private double jamDetectStartTs = -1.0;
  private double antiJamStateStartTs = -1.0;
  private double lastUnjamStartTs = -1.0;

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Seconds).of(Constants.OperatorConstants.SysId.SPINDEXER_RAMP_RATE_V_PER_S),
              Volts.of(Constants.OperatorConstants.SysId.SPINDEXER_STEP_V),
              Seconds.of(Constants.OperatorConstants.SysId.SPINDEXER_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "spindexer"));

  private boolean isSysIdEnabled() {
    if (!Constants.OperatorConstants.SysId.ENABLE_SYSID) {
      return false;
    }
    return SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  // ---------------- Simulation ----------------
  private final boolean isSim = RobotBase.isSimulation();
  private final FlywheelSim spindexerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              Constants.OperatorConstants.Spindexer.SIM_GEAR_RATIO,
              Constants.OperatorConstants.Spindexer.SIM_J_KGM2),
          DCMotor.getKrakenX60(1));
  private double simPosRot = 0.0;

  public SpindexerSubsystem() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    motor =
        new TalonFX(
            Constants.OperatorConstants.Spindexer.MOTOR_ID,
            Constants.OperatorConstants.Spindexer.CANBUS_NAME);

    configureHardware();

    positionSig = motor.getPosition();
    velocitySig = motor.getVelocity();
    motorVoltageSig = motor.getMotorVoltage();
    statorCurrentSig = motor.getStatorCurrent();

    positionSig.setUpdateFrequency(50.0);
    velocitySig.setUpdateFrequency(50.0);
    motorVoltageSig.setUpdateFrequency(50.0);
    statorCurrentSig.setUpdateFrequency(50.0);

    motor.optimizeBusUtilization();
  }

  private void configureHardware() {
    final var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = Constants.OperatorConstants.Spindexer.SUPPLY_CURRENT_LIMIT_A;
    limits.SupplyCurrentLowerLimit = Constants.OperatorConstants.Spindexer.SUPPLY_CURRENT_LOWER_LIMIT_A;
    limits.SupplyCurrentLowerTime = Constants.OperatorConstants.Spindexer.SUPPLY_CURRENT_LOWER_TIME_S;

    limits.StatorCurrentLimitEnable = true;
    limits.StatorCurrentLimit = Constants.OperatorConstants.Spindexer.STATOR_CURRENT_LIMIT_A;

    final var slot0 = new Slot0Configs();
    slot0.kS = Constants.OperatorConstants.Spindexer.VEL_kS;
    slot0.kV = Constants.OperatorConstants.Spindexer.VEL_kV;
    slot0.kP = Constants.OperatorConstants.Spindexer.VEL_kP;
    slot0.kI = Constants.OperatorConstants.Spindexer.VEL_kI;
    slot0.kD = Constants.OperatorConstants.Spindexer.VEL_kD;

    final var cfg = new TalonFXConfiguration();
    cfg.CurrentLimits = limits;
    cfg.Slot0 = slot0;

    motor.getConfigurator().apply(cfg);
  }

  private void resetAntiJamTimersAndState() {
    antiJamState = AntiJamState.NORMAL;
    jamDetectStartTs = -1.0;
    antiJamStateStartTs = -1.0;
    lastUnjamStartTs = -1.0;
  }

  private boolean isAntiJamEligibleMode() {
    switch (desiredMode) {
      case BASE_FORWARD:
      case SUPPLY_FORWARD:
      case CAL_BASE_FORWARD:
      case CAL_SUPPLY_FORWARD:
        return Constants.OperatorConstants.Spindexer.ANTI_JAM_ENABLED;
      default:
        return false;
    }
  }

  private void commandVelocityRpsInternal(double velocityRps) {
    velocityClosedLoopEnabled = true;
    targetRps = velocityRps;
    commandedDuty = 0.0;
    motor.setControl(velocityRequest.withVelocity(velocityRps));
  }

  private void commandDutyInternal(double dutyCycle) {
    velocityClosedLoopEnabled = false;
    targetRps = 0.0;
    commandedDuty = dutyCycle;
    motor.setControl(duty.withOutput(dutyCycle));
  }

  private void requestForwardMode(DesiredMode mode, double forwardRps) {
    desiredMode = mode;
    desiredForwardRps = forwardRps;

    // If we are not currently in an anti-jam recovery phase, apply forward command immediately.
    if (antiJamState == AntiJamState.NORMAL || antiJamState == AntiJamState.COOLDOWN) {
      commandVelocityRpsInternal(forwardRps);
    }
  }

  /** Raw open-loop duty command. Anti-jam is disabled in this mode. */
  public void runDuty(double dutyCycle) {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    desiredMode = DesiredMode.OPEN_LOOP_DUTY;
    desiredForwardRps = 0.0;
    resetAntiJamTimersAndState();
    commandDutyInternal(dutyCycle);
  }

  /** Manual closed-loop velocity command. Anti-jam is disabled in this mode. */
  public void runVelocityRps(double velocityRps) {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    desiredMode = DesiredMode.MANUAL_VELOCITY;
    desiredForwardRps = velocityRps;
    resetAntiJamTimersAndState();
    commandVelocityRpsInternal(velocityRps);
  }

  /** Stop spindexer motor. */
  public void stop() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    desiredMode = DesiredMode.OFF;
    desiredForwardRps = 0.0;
    resetAntiJamTimersAndState();
    commandDutyInternal(0.0);
  }

  /** Convenience: run at the configured "base circulation" velocity. */
  public void runBase() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    calMode = "OFF";
    requestForwardMode(DesiredMode.BASE_FORWARD, Constants.OperatorConstants.Spindexer.BASE_RPS);
  }

  /** Convenience: run at the configured "shooting supply" velocity. */
  public void runSupply() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    calMode = "OFF";
    requestForwardMode(DesiredMode.SUPPLY_FORWARD, Constants.OperatorConstants.Spindexer.SUPPLY_RPS);
  }

  /** Calibration-only: run base using a live-tunable velocity setpoint. */
  public void runBaseCal(double baseRpsSet) {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    calMode = "CAL_BASE";
    calBaseRpsSet = baseRpsSet;
    requestForwardMode(DesiredMode.CAL_BASE_FORWARD, baseRpsSet);
  }

  /** Calibration-only: run supply using a live-tunable velocity setpoint. */
  public void runSupplyCal(double supplyRpsSet) {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    calMode = "CAL_SUPPLY";
    calSupplyRpsSet = supplyRpsSet;
    requestForwardMode(DesiredMode.CAL_SUPPLY_FORWARD, supplyRpsSet);
  }

  /** Calibration-only: stop and mark mode. */
  public void stopCal() {
    calMode = "CAL_STOP";
    stop();
  }

  /** @return last duty commanded to the motor (telemetry/debug). */
  public double getCommandedDuty() {
    return commandedDuty;
  }

  public double getPosRot() {
    return posRot;
  }

  public double getVelRps() {
    return velRps;
  }

  public double getStatorCurrentA() {
    return statorCurrentA;
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

    log.motor("spindexer")
        .voltage(Volts.of(motorVoltageSig.getValueAsDouble()))
        .angularPosition(Rotations.of(posRot))
        .angularVelocity(RotationsPerSecond.of(velRps));
  }

  private boolean isJamConditionMet() {
    double absTargetRps = Math.abs(desiredForwardRps);
    double absActualRps = Math.abs(velRps);

    boolean currentHigh =
        statorCurrentA >= Constants.OperatorConstants.Spindexer.JAM_CURRENT_THRESHOLD_A;

    boolean targetFastEnough =
        absTargetRps >= Constants.OperatorConstants.Spindexer.JAM_MIN_TARGET_RPS;

    boolean velocityCollapsed =
        absActualRps <= absTargetRps * Constants.OperatorConstants.Spindexer.JAM_MIN_VELOCITY_RATIO;

    return currentHigh && targetFastEnough && velocityCollapsed;
  }

  private double getSettleForwardRps() {
    double settleAbs =
        Math.min(
            Math.abs(desiredForwardRps),
            Constants.OperatorConstants.Spindexer.UNJAM_SETTLE_FORWARD_RPS);

    if (settleAbs <= 1e-6) {
      return 0.0;
    }

    return Math.copySign(settleAbs, desiredForwardRps);
  }

  private void runAntiJamStateMachine(double nowSec) {
    switch (antiJamState) {
      case NORMAL:
        commandVelocityRpsInternal(desiredForwardRps);

        boolean cooldownExpired =
            lastUnjamStartTs < 0.0
                || (nowSec - lastUnjamStartTs)
                    >= Constants.OperatorConstants.Spindexer.UNJAM_COOLDOWN_S;

        if (!cooldownExpired) {
          jamDetectStartTs = -1.0;
          return;
        }

        if (isJamConditionMet()) {
          if (jamDetectStartTs < 0.0) {
            jamDetectStartTs = nowSec;
          } else if ((nowSec - jamDetectStartTs)
              >= Constants.OperatorConstants.Spindexer.JAM_CONFIRM_TIME_S) {
            antiJamState = AntiJamState.UNJAM_REVERSE;
            antiJamStateStartTs = nowSec;
            lastUnjamStartTs = nowSec;
            jamDetectStartTs = -1.0;
            commandDutyInternal(Constants.OperatorConstants.Spindexer.UNJAM_REVERSE_DUTY);
          }
        } else {
          jamDetectStartTs = -1.0;
        }
        return;

      case UNJAM_REVERSE:
        commandDutyInternal(Constants.OperatorConstants.Spindexer.UNJAM_REVERSE_DUTY);

        if ((nowSec - antiJamStateStartTs)
            >= Constants.OperatorConstants.Spindexer.UNJAM_REVERSE_TIME_S) {
          if (Constants.OperatorConstants.Spindexer.ENABLE_UNJAM_SETTLE_FORWARD) {
            antiJamState = AntiJamState.UNJAM_SETTLE_FORWARD;
          } else {
            antiJamState = AntiJamState.COOLDOWN;
          }
          antiJamStateStartTs = nowSec;
        }
        return;

      case UNJAM_SETTLE_FORWARD:
        commandVelocityRpsInternal(getSettleForwardRps());

        if ((nowSec - antiJamStateStartTs)
            >= Constants.OperatorConstants.Spindexer.UNJAM_SETTLE_TIME_S) {
          antiJamState = AntiJamState.COOLDOWN;
          antiJamStateStartTs = nowSec;
        }
        return;

      case COOLDOWN:
      default:
        commandVelocityRpsInternal(desiredForwardRps);
        jamDetectStartTs = -1.0;

        if ((nowSec - lastUnjamStartTs)
            >= Constants.OperatorConstants.Spindexer.UNJAM_COOLDOWN_S) {
          antiJamState = AntiJamState.NORMAL;
        }
        return;
    }
  }

    private double getJamDetectAgeMs(double nowSec) {
    if (jamDetectStartTs < 0.0) {
      return 0.0;
    }
    return (nowSec - jamDetectStartTs) * 1000.0;
  }

  private double getLastUnjamAgeMs(double nowSec) {
    if (lastUnjamStartTs < 0.0) {
      return -1.0;
    }
    return (nowSec - lastUnjamStartTs) * 1000.0;
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig, statorCurrentSig);
    posRot = positionSig.getValueAsDouble();
    velRps = velocitySig.getValueAsDouble();
    statorCurrentA = statorCurrentSig.getValueAsDouble();
    velocityErrorRps = desiredForwardRps - velRps;

    if (isAntiJamEligibleMode()) {
      runAntiJamStateMachine(Timer.getFPGATimestamp());
    } else {
      switch (desiredMode) {
        case BASE_FORWARD:
        case SUPPLY_FORWARD:
        case CAL_BASE_FORWARD:
        case CAL_SUPPLY_FORWARD:
        case MANUAL_VELOCITY:
          commandVelocityRpsInternal(desiredForwardRps);
          break;

        case OPEN_LOOP_DUTY:
        case OFF:
        default:
          // Do nothing here; output already applied when command was issued.
          break;
      }
    }

    if (DebugTelemetrySubsystems.spindexer) {
      double now = Timer.getFPGATimestamp();

      SmartDashboard.putNumber("Spindexer/DutyCmd", commandedDuty);
      SmartDashboard.putNumber("Spindexer/PosRot", posRot);
      SmartDashboard.putNumber("Spindexer/VelRps", velRps);
      SmartDashboard.putNumber("Spindexer/StatorCurrentA", statorCurrentA);

      SmartDashboard.putString("Spindexer/DesiredMode", desiredMode.name());
      SmartDashboard.putString("Spindexer/AntiJamState", antiJamState.name());

      SmartDashboard.putString("Spindexer/Cal/Mode", calMode);
      SmartDashboard.putNumber("Spindexer/Cal/BaseRpsSet", calBaseRpsSet);
      SmartDashboard.putNumber("Spindexer/Cal/SupplyRpsSet", calSupplyRpsSet);

      SmartDashboard.putBoolean("Spindexer/VelocityClosedLoopEnabled", velocityClosedLoopEnabled);
      SmartDashboard.putNumber("Spindexer/TargetRps", targetRps);
      SmartDashboard.putNumber("Spindexer/DesiredForwardRps", desiredForwardRps);
      SmartDashboard.putNumber("Spindexer/VelocityErrorRps", velocityErrorRps);

      SmartDashboard.putNumber("Spindexer/JamDetectAgeMs", getJamDetectAgeMs(now));
      SmartDashboard.putNumber("Spindexer/LastUnjamAgeMs", getLastUnjamAgeMs(now));
    }
  }

  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.spindexer) {
      return 0.0;
    }
    return spindexerSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim || !EnabledSubsystems.spindexer) {
      return;
    }

    final double dt = 0.02;

    var simState = motor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double appliedV = simState.getMotorVoltage();
    spindexerSim.setInputVoltage(appliedV);
    spindexerSim.update(dt);

    double rps = spindexerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    simPosRot += rps * dt;
    simState.setRawRotorPosition(simPosRot);
    simState.setRotorVelocity(rps);
  }
}