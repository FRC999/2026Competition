package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OperatorConstants.Hopper;

/**
 * HopperSubsystem
 *
 * Mechanical responsibility (as you described):
 * - Storage only + extension/contraction to hold more balls.
 *
 * Current implementation:
 * - One TalonFX motor (Kraken) placeholder for hopper actuation/roller.
 * - Placeholder extend/retract state (boolean only).
 *
 * Standards:
 * - Can be disabled via Constants.EnabledSubsystems.hopper
 * - Telemetry can be disabled via Constants.DebugTelemetrySubsystems.hopper
 * - Provides SysId + Simulation support (Task #8)
 */
public class HopperSubsystem extends SubsystemBase {
  private TalonFX hopperMotor;

  private boolean extended = false;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

  // Signals for telemetry + SysId logging
  private StatusSignal<Angle> positionSig;
  private StatusSignal<AngularVelocity> velocitySig;
  private StatusSignal<Voltage> motorVoltageSig;

  private double posRot = 0.0;
  private double velRps = 0.0;

  // ---------------- SysId ----------------
  private final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Seconds).of(Constants.OperatorConstants.SysId.HOPPER_RAMP_RATE_V_PER_S),
              Volts.of(Constants.OperatorConstants.SysId.HOPPER_STEP_V),
              Seconds.of(Constants.OperatorConstants.SysId.HOPPER_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "hopper"));

  private boolean isSysIdEnabled() {
    if (!Constants.OperatorConstants.SysId.ENABLE_SYSID) {
      return false;
    }
    return SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

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
    setDutyCycle(dutyOut);
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled()) {
      return;
    }
    log.motor("hopper")
        .voltage(Volts.of(motorVoltageSig.getValueAsDouble()))
        .angularPosition(Rotations.of(posRot))
        .angularVelocity(RotationsPerSecond.of(velRps));
  }

  // ---------------- Simulation ----------------
  private final boolean isSim = RobotBase.isSimulation();
  private final FlywheelSim hopperSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              Constants.OperatorConstants.Hopper.SIM_GEAR_RATIO,
              Constants.OperatorConstants.Hopper.SIM_J_KGM2),
          DCMotor.getKrakenX60(1));
  private double simPosRot = 0.0;

  public HopperSubsystem() {
    if (!EnabledSubsystems.hopper) {
      return;
    }

    hopperMotor = new TalonFX(Hopper.MOTOR_ID, Hopper.CANBUS_NAME);

    configureMotors();

    positionSig = hopperMotor.getPosition();
    velocitySig = hopperMotor.getVelocity();
    motorVoltageSig = hopperMotor.getMotorVoltage();

    positionSig.setUpdateFrequency(100.0);
    velocitySig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

    hopperMotor.optimizeBusUtilization();
  }

  private void configureMotors() {
    MotorOutputConfigs out =
        new MotorOutputConfigs()
            .withInverted(Constants.OperatorConstants.Hopper.MOTOR_INVERTED)
            .withNeutralMode(Constants.OperatorConstants.Hopper.NEUTRAL_COAST);

    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLowerTime(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_TIME_S)
            .withSupplyCurrentLowerLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_LIMIT_A)
            .withStatorCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
            .withStatorCurrentLimit(Constants.OperatorConstants.Hopper.STATOR_CURRENT_LIMIT_A);

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(Constants.OperatorConstants.Hopper.kP)
            .withKI(Constants.OperatorConstants.Hopper.kI)
            .withKD(Constants.OperatorConstants.Hopper.kD)
            .withKS(Constants.OperatorConstants.Hopper.kS)
            .withKV(Constants.OperatorConstants.Hopper.kV)
            .withKA(Constants.OperatorConstants.Hopper.kA);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration().withMotorOutput(out).withCurrentLimits(limits).withSlot0(slot0);

    hopperMotor.getConfigurator().apply(cfg);
  }

  public double getRelativeEncoder() {
    return hopperMotor.getPosition().getValueAsDouble();
  }

  public double getAbsoluteEncoder() {
    return hopperMotor.getRotorPosition().getValueAsDouble();
  }

  public void stop() {
    hopperMotor.setControl(dutyCycle.withOutput(0.0));
  }

  public void setDutyCycle(double percent) {
    double p = MathUtil.clamp(
            percent,
            -Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE,
            Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE);
    hopperMotor.setControl(dutyCycle.withOutput(p));
  }

 
  /** Extend the hopper (placeholder). */
  public void extend() {
    extended = true;
  }

  /** Retract the hopper (placeholder). */
  public void retract() {
    extended = false;
  }

  /** @return true if hopper is currently extended (placeholder). */
  public boolean isExtended() {
    return extended;
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim) {
      return;
    }
    if (!EnabledSubsystems.hopper) {
      return;
    }

    final double dt = 0.02;

    var simState = hopperMotor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double appliedV = simState.getMotorVoltage();

    hopperSim.setInputVoltage(appliedV);
    hopperSim.update(dt);

    double rps = hopperSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    simPosRot += rps * dt;

    simState.setRawRotorPosition(simPosRot);
    simState.setRotorVelocity(rps);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.hopper) {
      return;
    }

    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);

    posRot = positionSig.getValueAsDouble();
    velRps = velocitySig.getValueAsDouble();

    if (DebugTelemetrySubsystems.hopper) {
      SmartDashboard.putBoolean("Hopper/Extended", extended);
      SmartDashboard.putNumber("Hopper/PosRot", posRot);
      SmartDashboard.putNumber("Hopper/VelRps", velRps);
      SmartDashboard.putNumber("Hopper/MotorVoltage", motorVoltageSig.getValueAsDouble());
    }
  }
}
