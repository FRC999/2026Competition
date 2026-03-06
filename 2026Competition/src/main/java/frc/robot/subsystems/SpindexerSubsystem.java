package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * SpindexerSubsystem
 *
 * Purpose:
 * - Move balls at the bottom of the hopper so they present consistently to the TransferSubsystem entry.
 *
 * Notes:
 * - This subsystem should NOT be responsible for "exactly one ball into the shooter". That job is TransferSubsystem
 *   using its throat/exit sensor and feed timing.
 */
public class SpindexerSubsystem extends SubsystemBase {

  private TalonFX motor;
  private final DutyCycleOut duty = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

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

  private double posRot = 0.0;
  private double velRps = 0.0;

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

  // @SuppressWarnings("unused")
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

    positionSig.setUpdateFrequency(50.0);
    velocitySig.setUpdateFrequency(50.0);
    motorVoltageSig.setUpdateFrequency(50.0);
    motor.optimizeBusUtilization();
  }

  public void runDuty(double dutyCycle) {
    velocityClosedLoopEnabled = false;
    commandedDuty = dutyCycle;
    motor.setControl(duty.withOutput(dutyCycle));
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
  /** Run spindexer in closed-loop velocity mode using rotor RPS. */
  public void runVelocityRps(double velocityRps) {
    velocityClosedLoopEnabled = true;
    targetRps = velocityRps;
    motor.setControl(velocityRequest.withVelocity(velocityRps));
  }

  /** Stop spindexer motor. */
  public void stop() {
    velocityClosedLoopEnabled = false;
    targetRps = 0.0;
    runDuty(0.0);
  }

  /** Convenience: run at the configured "base circulation" velocity. */
  public void runBase() {
    runVelocityRps(Constants.OperatorConstants.Spindexer.BASE_RPS);
  }

  /** Convenience: run at the configured "shooting supply" velocity. */
  public void runSupply() {
    runVelocityRps(Constants.OperatorConstants.Spindexer.SUPPLY_RPS);
  }

  /** Calibration-only: run base using a live-tunable velocity setpoint. */
  public void runBaseCal(double baseRpsSet) {
    calMode = "CAL_BASE";
    calBaseRpsSet = baseRpsSet;
    runVelocityRps(baseRpsSet);
  }

  /** Calibration-only: run supply using a live-tunable velocity setpoint. */
  public void runSupplyCal(double supplyRpsSet) {
    calMode = "CAL_SUPPLY";
    calSupplyRpsSet = supplyRpsSet;
    runVelocityRps(supplyRpsSet);
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

  @Override
  public void periodic() {
    if (!EnabledSubsystems.spindexer) {
      return;
    }

    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);
    posRot = positionSig.getValueAsDouble();
    velRps = velocitySig.getValueAsDouble();

    if (velocityClosedLoopEnabled) {
      motor.setControl(velocityRequest.withVelocity(targetRps));
    }

    if (DebugTelemetrySubsystems.spindexer) {
      SmartDashboard.putNumber("Spindexer/DutyCmd", commandedDuty);
      SmartDashboard.putNumber("Spindexer/PosRot", posRot);
      SmartDashboard.putNumber("Spindexer/VelRps", velRps);
      SmartDashboard.putString("Spindexer/Cal/Mode", calMode);
      SmartDashboard.putNumber("Spindexer/Cal/BaseRpsSet", calBaseRpsSet);
      SmartDashboard.putNumber("Spindexer/Cal/SupplyRpsSet", calSupplyRpsSet);
      SmartDashboard.putBoolean("Spindexer/VelocityClosedLoopEnabled", velocityClosedLoopEnabled);
      SmartDashboard.putNumber("Spindexer/TargetRps", targetRps);
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
