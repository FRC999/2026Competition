// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.PositionDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeRollerMotor;
  private TalonFX intakePivotMotor;

  private final MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0); // MotionMagic Duty Cycle
  private double intakePivotEncoderZero = 0;

  // Status signals (telemetry + SysId logs)
  private StatusSignal<AngularVelocity> rollerVelSig;
  private StatusSignal<Voltage> rollerVoltageSig;

  private StatusSignal<Angle> pivotPosSig;
  private StatusSignal<AngularVelocity> pivotVelSig;
  private StatusSignal<Voltage> pivotVoltageSig;

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Units.Volts.per(Units.Seconds).of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_RAMP_RATE_V_PER_S),
              Units.Volts.of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_STEP_V),
              Units.Seconds.of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdRollerVoltageDrive, this::sysIdRollerLog, this, "intake-roller"));

  private final SysIdRoutine pivotSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Units.Volts.per(Units.Seconds).of(Constants.OperatorConstants.SysId.INTAKE_PIVOT_RAMP_RATE_V_PER_S),
              Units.Volts.of(Constants.OperatorConstants.SysId.INTAKE_PIVOT_STEP_V),
              Units.Seconds.of(Constants.OperatorConstants.SysId.INTAKE_PIVOT_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdPivotVoltageDrive, this::sysIdPivotLog, this, "intake-pivot"));

  private boolean isSysIdEnabled() {
    if (!Constants.OperatorConstants.SysId.ENABLE_SYSID) {
      return false;
    }
    return SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  // ---------------- Simulation ----------------
  private final boolean isSim = RobotBase.isSimulation();

  private final FlywheelSim rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              IntakeConstants.SIM_ROLLER_GEAR_RATIO,
              IntakeConstants.SIM_ROLLER_J_KGM2),
          DCMotor.getKrakenX60(1));

  private final FlywheelSim pivotSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              IntakeConstants.SIM_PIVOT_GEAR_RATIO,
              IntakeConstants.SIM_PIVOT_J_KGM2),
          DCMotor.getKrakenX60(1));

  private double simRollerPosRot = 0.0;
  private double simPivotPosRot = 0.0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if (!EnabledSubsystems.intake) {
      return;
    }

    intakeRollerMotor = new TalonFX(IntakeConstants.intakeRollerMotorId, IntakeConstants.CANBUS_NAME);
    intakePivotMotor = new TalonFX(IntakeConstants.intakePivotMotorId, IntakeConstants.CANBUS_NAME);

    configureMotors();
    configureStatusSignals();
  }

  private void configureStatusSignals() {
    rollerVelSig = intakeRollerMotor.getVelocity();
    rollerVoltageSig = intakeRollerMotor.getMotorVoltage();

    pivotPosSig = intakePivotMotor.getPosition();
    pivotVelSig = intakePivotMotor.getVelocity();
    pivotVoltageSig = intakePivotMotor.getMotorVoltage();

    rollerVelSig.setUpdateFrequency(50.0);
    rollerVoltageSig.setUpdateFrequency(20.0);

    pivotPosSig.setUpdateFrequency(100.0);
    pivotVelSig.setUpdateFrequency(50.0);
    pivotVoltageSig.setUpdateFrequency(20.0);

    intakeRollerMotor.optimizeBusUtilization();
    intakePivotMotor.optimizeBusUtilization();
  }

  private void configureMotors() {
    intakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeRollerMotor.setSafetyEnabled(false);

    var motorRollerConfig = new MotorOutputConfigs();
    motorRollerConfig.NeutralMode = NeutralModeValue.Brake;
    motorRollerConfig.Inverted =
        (IntakeConstants.IntakeRollerInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    var talonFXRollerConfigurator = intakeRollerMotor.getConfigurator();

    TalonFXConfiguration pidRollerConfig = new TalonFXConfiguration().withMotorOutput(motorRollerConfig);

    StatusCode statusRoller = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusRoller = talonFXRollerConfigurator.apply(pidRollerConfig);
      if (statusRoller.isOK()) {
        break;
      }
    }
    if (!statusRoller.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusRoller.toString());
    }

    intakePivotMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakePivotMotor.setSafetyEnabled(false);

    var motorPivotConfig = new MotorOutputConfigs();
    motorPivotConfig.NeutralMode = NeutralModeValue.Brake;
    motorPivotConfig.Inverted =
        (IntakeConstants.intakePivotMotorInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    var talonFXPivotConfigurator = intakePivotMotor.getConfigurator();

    TalonFXConfiguration pidPivotConfig = new TalonFXConfiguration().withMotorOutput(motorPivotConfig);
    configureMotionMagicDutyCycle(pidPivotConfig);

    StatusCode statusPivot = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusPivot = talonFXPivotConfigurator.apply(pidPivotConfig);
      if (statusPivot.isOK()) {
        break;
      }
    }
    if (!statusPivot.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusPivot.toString());
    }
  }

  @SuppressWarnings("unused")
  private void configurePositionDutyCycle(TalonFXConfiguration config) {
    config.Slot0.kV = PositionDutyCycleConstants.intake_kV;
    config.Slot0.kP = PositionDutyCycleConstants.intake_kP;
    config.Slot0.kI = PositionDutyCycleConstants.intake_kI;
    config.Slot0.kD = PositionDutyCycleConstants.intake_kD;
  }

  @SuppressWarnings("unused")
  private void setPositionDutyCycle(double position) {
    intakePivotMotor.setControl(new PositionDutyCycle(position));
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    // PID on Position
    config.Slot0.kP = MotionMagicDutyCycleConstants.intake_kP;
    config.Slot0.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.intake_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    motMagDutyCycle.Slot = MotionMagicDutyCycleConstants.slot;
  }

  public void setMotionMagicDutyCycle(double position) {
    intakePivotMotor.setControl(motMagDutyCycle.withPosition(position));
    System.out.println("***Pos: " + position);
  }

  /**
   * Run intake roller motor at the specified speed
   *
   * @param speed duty-cycle output [-1, 1]
   */
  public void runIntake(double speed) {
    intakeRollerMotor.set(speed);
  }

  /** Stop rotating the intake roller. */
  public void stopIntake() {
    intakeRollerMotor.set(0);
  }

  public double getIntakePivotEncoderZeroPosition() {
    return intakePivotEncoderZero;
  }

  public double getIntakePivotMotorEncoder() {
    return intakePivotMotor.getRotorPosition().getValueAsDouble();
  }

  public void setIntakePositionWithAngle(IntakePositions angle) {
    setMotionMagicDutyCycle(intakePivotEncoderZero + angle.getPosition());
  }

  public boolean isAtPosition(IntakePositions position) {
    return Math.abs(position.getPosition() - getIntakePivotMotorEncoder()) <= IntakePidConstants.tolerance;
  }

  // ---------------- SysId factory commands ----------------
  public Command sysIdRollerQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new InstantCommand();
    }
    return rollerSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdRollerDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new InstantCommand();
    }
    return rollerSysIdRoutine.dynamic(direction);
  }

  public Command sysIdPivotQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new InstantCommand();
    }
    return pivotSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdPivotDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled()) {
      return new InstantCommand();
    }
    return pivotSysIdRoutine.dynamic(direction);
  }

  // ---------------- SysId callbacks ----------------
  private void sysIdRollerVoltageDrive(edu.wpi.first.units.measure.Voltage volts) {
    if (!isSysIdEnabled()) {
      return;
    }
    double duty = volts.in(Units.Volts) / RobotController.getBatteryVoltage();
    duty = clamp(duty, -1.0, 1.0);
    intakeRollerMotor.setControl(new DutyCycleOut(duty));
  }

  private void sysIdRollerLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled()) {
      return;
    }
    BaseStatusSignal.refreshAll(rollerVelSig, rollerVoltageSig);

    log.motor("intake-roller")
        .voltage(rollerVoltageSig.getValue())
        .angularVelocity(rollerVelSig.getValue());
  }

  private void sysIdPivotVoltageDrive(edu.wpi.first.units.measure.Voltage volts) {
    if (!isSysIdEnabled()) {
      return;
    }
    double duty = volts.in(Units.Volts) / RobotController.getBatteryVoltage();
    duty = clamp(duty, -1.0, 1.0);
    intakePivotMotor.setControl(new DutyCycleOut(duty));
  }

  private void sysIdPivotLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled()) {
      return;
    }
    BaseStatusSignal.refreshAll(pivotPosSig, pivotVelSig, pivotVoltageSig);

    log.motor("intake-pivot")
        .voltage(pivotVoltageSig.getValue())
        .angularPosition(pivotPosSig.getValue())
        .angularVelocity(pivotVelSig.getValue());
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.intake) {
      return;
    }

    BaseStatusSignal.refreshAll(rollerVelSig, rollerVoltageSig, pivotPosSig, pivotVelSig, pivotVoltageSig);

    if (DebugTelemetrySubsystems.intake) {
      SmartDashboard.putNumber("Intake/RollerVelRps", rollerVelSig.getValueAsDouble());
      SmartDashboard.putNumber("Intake/RollerMotorVoltage", rollerVoltageSig.getValueAsDouble());
      SmartDashboard.putNumber("Intake/PivotPosRot", pivotPosSig.getValueAsDouble());
      SmartDashboard.putNumber("Intake/PivotVelRps", pivotVelSig.getValueAsDouble());
      SmartDashboard.putNumber("Intake/PivotMotorVoltage", pivotVoltageSig.getValueAsDouble());
      SmartDashboard.putNumber("Intake/PivotEncoderZero", intakePivotEncoderZero);
    }
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim) {
      return;
    }
    if (!EnabledSubsystems.intake) {
      return;
    }

    final double dt = 0.02;

    TalonFXSimState rollerSimState = intakeRollerMotor.getSimState();
    TalonFXSimState pivotSimState = intakePivotMotor.getSimState();

    rollerSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    pivotSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double rollerAppliedV = rollerSimState.getMotorVoltage();
    double pivotAppliedV = pivotSimState.getMotorVoltage();

    rollerSim.setInputVoltage(rollerAppliedV);
    pivotSim.setInputVoltage(pivotAppliedV);

    rollerSim.update(dt);
    pivotSim.update(dt);

    double rollerRps = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double pivotRps = pivotSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    simRollerPosRot += rollerRps * dt;
    simPivotPosRot += pivotRps * dt;

    rollerSimState.setRawRotorPosition(simRollerPosRot);
    rollerSimState.setRotorVelocity(rollerRps);

    pivotSimState.setRawRotorPosition(simPivotPosRot);
    pivotSimState.setRotorVelocity(pivotRps);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            rollerSim.getCurrentDrawAmps() + pivotSim.getCurrentDrawAmps()));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
