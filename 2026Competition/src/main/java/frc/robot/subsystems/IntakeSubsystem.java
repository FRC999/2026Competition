// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.RollerVelocityVoltageConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeRollerMotor;

  private TalonFX intakePivotMotor; // leader
  private TalonFX intakePivotFollowerMotor; // follower

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(false);
  private final VelocityVoltage rollerVelocityVoltage =
    new VelocityVoltage(0).withSlot(0);

  private double intakePivotEncoderZero = 0;
  private double targetPivotDeg = 0.0;
  private double rollerTargetRps = 0.0;
  private boolean pivotZeroed = false;

  private enum RollerDesiredMode {
  OFF,
  VELOCITY
}

  private RollerDesiredMode rollerDesiredMode = RollerDesiredMode.OFF;
  private boolean rollerVelocityClosedLoopEnabled = false;
  private double rollerCommandedMotorRps = 0.0;

  // Status signals (telemetry + SysId logs)
  private StatusSignal<AngularVelocity> rollerVelSig;
  private StatusSignal<Voltage> rollerVoltageSig;

  private StatusSignal<Angle> pivotPosSig;
  private StatusSignal<AngularVelocity> pivotVelSig;
  private StatusSignal<Voltage> pivotVoltageSig;

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Seconds).of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_RAMP_RATE_V_PER_S),
          Units.Volts.of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_STEP_V),
          Units.Seconds.of(Constants.OperatorConstants.SysId.INTAKE_ROLLER_TIMEOUT_S)),
      new SysIdRoutine.Mechanism(this::sysIdRollerVoltageDrive, this::sysIdRollerLog, this, "intake-roller"));

  private final SysIdRoutine pivotSysIdRoutine = new SysIdRoutine(
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

  private final FlywheelSim rollerSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          IntakeConstants.SIM_ROLLER_GEAR_RATIO,
          IntakeConstants.SIM_ROLLER_J_KGM2),
      DCMotor.getKrakenX60(1));

  private final FlywheelSim pivotSim = new FlywheelSim(
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
    intakePivotFollowerMotor = new TalonFX(IntakeConstants.intakePivotFollowerMotorId, IntakeConstants.CANBUS_NAME);
    // TODO: PLACEHOLDER - set correct follower CAN ID in Constants before running
    // on hardware

    configureMotors();
    //configureHardware();
    configureStatusSignals();

    // Seed pivot zero at boot (you guarantee intake starts fully retracted)
    seedZeroFromRetractedHardStop();
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
    intakePivotFollowerMotor.optimizeBusUtilization();
  }

  private void configureHardware() {
    final var rollerCurrentLimits = new CurrentLimitsConfigs();
    rollerCurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_A;
    rollerCurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LOWER_LIMIT_A;
    rollerCurrentLimits.SupplyCurrentLowerTime = IntakeConstants.ROLLER_SUPPLY_CURRENT_LOWER_TIME_S;

    rollerCurrentLimits.StatorCurrentLimitEnable = true;
    rollerCurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_A;

    final var slot0 = new Slot0Configs();
    slot0.kS =  RollerVelocityVoltageConstants.intake_kS;;
    slot0.kV = RollerVelocityVoltageConstants.intake_kV;;
    slot0.kP =  RollerVelocityVoltageConstants.intake_kP;
    slot0.kI =  RollerVelocityVoltageConstants.intake_kI;
    slot0.kD = RollerVelocityVoltageConstants.intake_kD;

    final var cfg = new TalonFXConfiguration();
    cfg.CurrentLimits = rollerCurrentLimits;
    cfg.Slot0 = slot0;


    // //alex test
    // System.out.println("*** S0: " + cfg.Slot0.kP
    //     + ", " + cfg.Slot0.kI
    //     + ", " + cfg.Slot0.kD
    //     + ", " + cfg.Slot0.kS );

    intakeRollerMotor.getConfigurator().apply(cfg);
    intakeRollerMotor.getConfigurator().apply(cfg);
    intakeRollerMotor.getConfigurator().apply(cfg);

    // alex test

    final var slot0Readback = new Slot0Configs();
var refreshStatus = intakeRollerMotor.getConfigurator().refresh(slot0Readback);
// System.out.println("slot0 refresh status = " + refreshStatus.getName() + " : " + refreshStatus.getDescription());
// System.out.println("READBACK Slot0:"
//     + " kP=" + slot0Readback.kP
//     + " kI=" + slot0Readback.kI
//     + " kD=" + slot0Readback.kD
//     + " kS=" + slot0Readback.kS
//     + " kV=" + slot0Readback.kV);

//     System.out.println("**** Configured intake roller motor.");
  }
  private void configureMotors() {

    var motorRollerConfig = new MotorOutputConfigs();
    motorRollerConfig.NeutralMode = NeutralModeValue.Brake;
    motorRollerConfig.Inverted = (IntakeConstants.IntakeRollerInverted
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive);

    var talonFXRollerConfigurator = intakeRollerMotor.getConfigurator();

    TalonFXConfiguration pidRollerConfig = new TalonFXConfiguration().withMotorOutput(motorRollerConfig);

    // ---------------- Current limits (roller) ----------------
    final var rollerCurrentLimits = new CurrentLimitsConfigs();
    rollerCurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT_A;
    rollerCurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LOWER_LIMIT_A;
    rollerCurrentLimits.SupplyCurrentLowerTime = IntakeConstants.ROLLER_SUPPLY_CURRENT_LOWER_TIME_S;

    rollerCurrentLimits.StatorCurrentLimitEnable = true;
    rollerCurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_A;

    pidRollerConfig.CurrentLimits = rollerCurrentLimits;

    // ---------------- VelocityVoltage gains (roller) ----------------
    pidRollerConfig.Slot0.kS = RollerVelocityVoltageConstants.intake_kS;
    pidRollerConfig.Slot0.kV = RollerVelocityVoltageConstants.intake_kV;
    pidRollerConfig.Slot0.kP = RollerVelocityVoltageConstants.intake_kP;
    pidRollerConfig.Slot0.kI = RollerVelocityVoltageConstants.intake_kI;
    pidRollerConfig.Slot0.kD = RollerVelocityVoltageConstants.intake_kD;

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

    intakeRollerMotor.getConfigurator().apply(pidRollerConfig);
    intakeRollerMotor.setSafetyEnabled(false);

    final MotorAlignmentValue alignment = IntakeConstants.intakePivotFollowerOpposeLeader
        ? MotorAlignmentValue.Opposed
        : MotorAlignmentValue.Aligned;

    intakePivotFollowerMotor.setControl(new Follower(IntakeConstants.intakePivotMotorId, alignment));

    var motorPivotConfig = new MotorOutputConfigs();
    motorPivotConfig.NeutralMode = NeutralModeValue.Coast;
    motorPivotConfig.Inverted = (IntakeConstants.intakePivotMotorInverted
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive);

    var talonFXPivotConfigurator = intakePivotMotor.getConfigurator();

    TalonFXConfiguration pidPivotConfig = new TalonFXConfiguration().withMotorOutput(motorPivotConfig);

    // ---------------- Current limits (pivot leader + follower) ----------------
    final var pivotCurrentLimits = new CurrentLimitsConfigs();
    pivotCurrentLimits.SupplyCurrentLimitEnable = true;
    pivotCurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT_A;
    pivotCurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_LIMIT_A;
    pivotCurrentLimits.SupplyCurrentLowerTime = IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_TIME_S;

    pivotCurrentLimits.StatorCurrentLimitEnable = true;
    pivotCurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_A;

    pidPivotConfig.CurrentLimits = pivotCurrentLimits;

    final TalonFXConfiguration pivotFollowerConfig = new TalonFXConfiguration();
    pivotFollowerConfig.CurrentLimits = pivotCurrentLimits;

    StatusCode statusPivotFollower = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusPivotFollower = intakePivotFollowerMotor.getConfigurator().apply(pivotFollowerConfig);
      if (statusPivotFollower.isOK()) {
        break;
      }
    }
    if (!statusPivotFollower.isOK()) {
      System.out.println("Could not apply follower current limits, error code: " + statusPivotFollower.toString());
    }

    final double fwdSoftLimitRot = IntakeConstants.PIVOT_MAX_DEG * IntakeConstants.PIVOT_MOTOR_TO_ARM_GEAR_RATIO
        / 360.0;

    pidPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pidPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdSoftLimitRot;

    pidPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pidPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

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

  private void commandRollerVelocityInternal(double rollerRps) {
  rollerVelocityClosedLoopEnabled = true;
  rollerTargetRps = rollerRps;
  rollerCommandedMotorRps = motorRpsFromRollerRps(rollerRps);

  // alex test
  //System.out.println("Commanding roller velocity. Roller RPS: " + rollerRps + ", Commanded motor RPS: " + rollerCommandedMotorRps);

  intakeRollerMotor.setControl(
      rollerVelocityVoltage.withVelocity(rollerCommandedMotorRps));
}


  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    // PID on Position
    config.Slot0.kP = MotionMagicDutyCycleConstants.intake_kP;
    config.Slot0.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.intake_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    motionMagicVoltage.Slot = MotionMagicDutyCycleConstants.slot;

    intakePivotMotor.getConfigurator().apply(config);
    intakePivotFollowerMotor.getConfigurator().apply(config);

  }

  private static double motorRotFromArmDeg(double armDeg) {
    return armDeg * IntakeConstants.PIVOT_MOTOR_TO_ARM_GEAR_RATIO / 360.0;
  }

  private static double armDegFromMotorRot(double motorRot) {
    return motorRot * 360.0 / IntakeConstants.PIVOT_MOTOR_TO_ARM_GEAR_RATIO;
  }

  private static double motorRpsFromRollerRps(double rollerRps) {
    System.out.println("Converting roller RPS " + rollerRps + " to motor RPS");
    return rollerRps * IntakeConstants.ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO;
  }

  private static double rollerRpsFromMotorRps(double motorRps) {
    return motorRps / IntakeConstants.ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO;
  }

  public double getPivotDeg() {
    return armDegFromMotorRot(getIntakePivotMotorEncoder() - intakePivotEncoderZero);
  }

  public double getTargetPivotDeg() {
    return targetPivotDeg;
  }

  public boolean isPivotZeroed() {
    return pivotZeroed;
  }

  public void setTargetPivotDeg(double armDeg) {
    // Clamp to configured range
    double clampedDeg = MathUtil.clamp(armDeg, IntakeConstants.PIVOT_MIN_DEG, IntakeConstants.PIVOT_MAX_DEG);
    targetPivotDeg = clampedDeg;

    // Convert to motor rotations
    double targetRot = intakePivotEncoderZero + motorRotFromArmDeg(clampedDeg);

    intakePivotMotor.setControl(motionMagicVoltage.withPosition(targetRot));
  }

  public void seedZeroFromRetractedHardStop() {
    // Because you guarantee intake starts fully retracted.
    intakePivotEncoderZero = 0.0;

    // IMPORTANT: this sets the motor sensor position to 0
    intakePivotMotor.setPosition(0.0);
    // follower is hardware-following so no need to set its position separately

    targetPivotDeg = 0.0;
    pivotZeroed = true;
  }

  public void setCalibrationPivotDutyCycle(double duty) {
    // Voltage consistency is good, but for "jog", duty is fine and simple.
    intakePivotMotor.setControl(new DutyCycleOut(MathUtil.clamp(duty, -1.0, 1.0)));
  }

  public void exitCalibrationOpenLoopHold() {
    intakePivotMotor.setControl(new DutyCycleOut(0.0));
  }

  /**
   * Run intake roller motor at the specified speed
   *
   * @param speed duty-cycle output [-1, 1]
   */
  /**
   * Run intake roller at the specified roller speed.
   *
   * @param rollerRps target roller speed in mechanism RPS
   */
public void runIntake(double rollerRps) {
  System.out.println("Running intake at " + rollerRps + " roller RPS");

  rollerDesiredMode = RollerDesiredMode.VELOCITY;
  commandRollerVelocityInternal(rollerRps);
}

  /** Stop rotating the intake roller. */
  public void stopIntake() {
  rollerDesiredMode = RollerDesiredMode.OFF;
  rollerVelocityClosedLoopEnabled = false;
  rollerTargetRps = 0.0;
  rollerCommandedMotorRps = 0.0;

  intakeRollerMotor.setControl(rollerVelocityVoltage.withVelocity(0.0));
}

  public double getRollerTargetRps() {
    return rollerTargetRps;
  }

  public double getIntakePivotEncoderZeroPosition() {
    return intakePivotEncoderZero;
  }

  public double getIntakePivotMotorEncoder() {
    return intakePivotMotor.getRotorPosition().getValueAsDouble();
  }

  public void setIntakePositionWithAngle(IntakePositions angle) {
    setTargetPivotDeg(angle.getPosition()); // now degrees
  }

  public boolean isAtPosition(IntakePositions position) {
    return Math.abs(position.getPosition() - getPivotDeg()) <= IntakePidConstants.tolerance;
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
    duty = MathUtil.clamp(duty, -1.0, 1.0);
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
    duty = MathUtil.clamp(duty, -1.0, 1.0);
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

  switch (rollerDesiredMode) {
    case VELOCITY:
      commandRollerVelocityInternal(rollerTargetRps);
      break;

    case OFF:
    default:
      break;
  }

  if (DebugTelemetrySubsystems.intake) {
    SmartDashboard.putNumber(
        "Intake/RollerVelRps",
        rollerRpsFromMotorRps(rollerVelSig.getValueAsDouble()));
    SmartDashboard.putNumber("Intake/RollerTargetRps", rollerTargetRps);
    SmartDashboard.putNumber("Intake/RollerMotorVoltage", rollerVoltageSig.getValueAsDouble());
    SmartDashboard.putBoolean("Intake/RollerVelocityClosedLoopEnabled", rollerVelocityClosedLoopEnabled);
    SmartDashboard.putString("Intake/RollerDesiredMode", rollerDesiredMode.name());
    SmartDashboard.putNumber("Intake/RollerCommandedMotorRps", rollerCommandedMotorRps);

    SmartDashboard.putNumber("Intake/PivotPosRot", pivotPosSig.getValueAsDouble());
    SmartDashboard.putNumber("Intake/PivotVelRps", pivotVelSig.getValueAsDouble());
    SmartDashboard.putNumber("Intake/PivotMotorVoltage", pivotVoltageSig.getValueAsDouble());
    SmartDashboard.putNumber("Intake/PivotEncoderZero", intakePivotEncoderZero);

    SmartDashboard.putNumber("Intake/PivotPosDeg", getPivotDeg());
    SmartDashboard.putNumber("Intake/PivotTargetDeg", getTargetPivotDeg());
    SmartDashboard.putNumber("Intake/PivotErrorDeg", getTargetPivotDeg() - getPivotDeg());
    SmartDashboard.putBoolean("Intake/PivotZeroed", isPivotZeroed());
  }

  
}

  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.intake) {
      return 0.0;
    }
    return (rollerSim.getCurrentDrawAmps() + pivotSim.getCurrentDrawAmps());
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

  }

}
