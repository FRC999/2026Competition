// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.RollerVelocityVoltageConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeRollerMotor;
  private TalonFX intakeRollerFollowerMotor;

  private TalonFX intakePivotMotor; // leader
  private TalonFX intakePivotFollowerMotor; // follower

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(false);
  private final VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private static final int PIVOT_DEPLOYED_SLOT = MotionMagicDutyCycleConstants.slot;
  private static final int PIVOT_RETRACTED_SLOT = 1;
  private static final int PIVOT_BOOSTED_SLOT = 2;

  private double intakePivotEncoderZero = 0;
  private double targetPivotDeg = 0.0;
  private double rollerTargetRps = 0.0;
  private boolean pivotZeroed = false;
  private boolean hardwareConfigured;
  private boolean homingActive;

  private enum RollerDesiredMode {
    OFF,
    VELOCITY, DUTY
  }

  private RollerDesiredMode rollerDesiredMode = RollerDesiredMode.OFF;
  private boolean rollerVelocityClosedLoopEnabled = false;
  private double rollerCommandedMotorRps = 0.0;
  private double lastRollerDutyCommand = Double.NaN;
  private double lastRollerVelocityCommandMotorRps = Double.NaN;
  private double lastPivotTargetRot = Double.NaN;
  private double lastPivotDutyCommand = Double.NaN;
  private int activePivotClosedLoopSlot = PIVOT_DEPLOYED_SLOT;
  private NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;
  private boolean pivotCurrentBoostActive = false;
  private boolean pivotClosedLoopBoostActive = false;
  private boolean stayDeployedAfterTriggerRelease =
      Constants.OperatorConstants.OIContants.INTAKE_STAY_OUT_AFTER_TRIGGER_RELEASE_DEFAULT;

  // Status signals (telemetry + SysId logs)
  private StatusSignal<AngularVelocity> rollerVelSig;
  private StatusSignal<Voltage> rollerVoltageSig;

  private StatusSignal<Angle> pivotPosSig;
  private StatusSignal<AngularVelocity> pivotVelSig;
  private StatusSignal<Voltage> pivotVoltageSig;
  private StatusSignal<edu.wpi.first.units.measure.Current> pivotLeaderStatorCurrentSig;
  private StatusSignal<edu.wpi.first.units.measure.Current> pivotFollowerStatorCurrentSig;

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
    if (!hardwareConfigured || !edu.wpi.first.wpilibj.DriverStation.isTestEnabled() || frc.robot.RobotContainer.isPanicStopActive()) return false;
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
  TalonFXConfiguration pidPivotConfigOg = new TalonFXConfiguration();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if (!EnabledSubsystems.intake) {
      return;
    }

    intakeRollerMotor = new TalonFX(IntakeConstants.intakeRollerMotorId, IntakeConstants.CANBUS_NAME);
    intakeRollerFollowerMotor = new TalonFX(IntakeConstants.intakeRollerFollowerMotorId, IntakeConstants.CANBUS_NAME);

    intakePivotMotor = new TalonFX(IntakeConstants.intakePivotMotorId, IntakeConstants.CANBUS_NAME);
    intakePivotFollowerMotor = new TalonFX(IntakeConstants.intakePivotFollowerMotorId, IntakeConstants.CANBUS_NAME);
    // TODO: PLACEHOLDER - set correct follower CAN ID in Constants before running
    // on hardware

    configureMotors();
    // configureHardware();
    configureStatusSignals();

    // Seed pivot zero at boot (you guarantee intake starts fully retracted)
    seedZeroFromRetractedHardStop();
    intakePivotMotor.hasResetOccurred(); intakePivotFollowerMotor.hasResetOccurred();
    intakeRollerMotor.hasResetOccurred(); intakeRollerFollowerMotor.hasResetOccurred();
  }

  private void configureStatusSignals() {
    rollerVelSig = intakeRollerMotor.getVelocity();
    rollerVoltageSig = intakeRollerMotor.getMotorVoltage();

    pivotPosSig = intakePivotMotor.getPosition();
    pivotVelSig = intakePivotMotor.getVelocity();
    pivotVoltageSig = intakePivotMotor.getMotorVoltage();
    pivotLeaderStatorCurrentSig = intakePivotMotor.getStatorCurrent();
    pivotFollowerStatorCurrentSig = intakePivotFollowerMotor.getStatorCurrent();

    rollerVelSig.setUpdateFrequency(50.0);
    rollerVoltageSig.setUpdateFrequency(20.0);

    pivotPosSig.setUpdateFrequency(100.0);
    pivotVelSig.setUpdateFrequency(50.0);
    pivotVoltageSig.setUpdateFrequency(20.0);
    pivotLeaderStatorCurrentSig.setUpdateFrequency(50.0);
    pivotFollowerStatorCurrentSig.setUpdateFrequency(50.0);

    intakeRollerMotor.optimizeBusUtilization();
    intakeRollerFollowerMotor.optimizeBusUtilization();
    intakePivotMotor.optimizeBusUtilization();
    intakePivotFollowerMotor.optimizeBusUtilization();
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
      statusRoller = intakeRollerMotor.getConfigurator().apply(pidRollerConfig);
      if (statusRoller.isOK()) {
        break;
      }
    }
    if (!statusRoller.isOK()) {
      // System.out.println("Could not apply configs, error code: " +
      // statusRoller.toString());
    }

    hardwareConfigured = statusRoller.isOK();
    hardwareConfigured &= intakeRollerFollowerMotor.getConfigurator().apply(pidRollerConfig).isOK();
    intakeRollerMotor.setSafetyEnabled(false);
    intakeRollerFollowerMotor.setSafetyEnabled(false);

    final MotorAlignmentValue rollerAlignment = IntakeConstants.intakeRollerFollowerOpposeLeader
        ? MotorAlignmentValue.Opposed
        : MotorAlignmentValue.Aligned;

    intakeRollerFollowerMotor.setControl(
        new Follower(IntakeConstants.intakeRollerMotorId, rollerAlignment));

    applyPivotFollowerControl();

    var motorPivotConfig = new MotorOutputConfigs();
    motorPivotConfig.NeutralMode = NeutralModeValue.Brake;
    motorPivotConfig.Inverted = (IntakeConstants.intakePivotMotorInverted
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive);

    var talonFXPivotConfigurator = intakePivotMotor.getConfigurator();

    pidPivotConfigOg.withMotorOutput(motorPivotConfig);

    // ---------------- Current limits (pivot leader + follower) ----------------
    final var pivotCurrentLimits = new CurrentLimitsConfigs();
    pivotCurrentLimits.SupplyCurrentLimitEnable = true;
    pivotCurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT_A;
    pivotCurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_LIMIT_A;
    pivotCurrentLimits.SupplyCurrentLowerTime = IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_TIME_S;

    pivotCurrentLimits.StatorCurrentLimitEnable = true;
    pivotCurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_A;

    pidPivotConfigOg.CurrentLimits = pivotCurrentLimits;

    final TalonFXConfiguration pivotFollowerConfig = new TalonFXConfiguration();
    pivotFollowerConfig.CurrentLimits = pivotCurrentLimits;
    pivotFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    StatusCode statusPivotFollower = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusPivotFollower = intakePivotFollowerMotor.getConfigurator().apply(pivotFollowerConfig);
      if (statusPivotFollower.isOK()) {
        break;
      }
    }
    if (!statusPivotFollower.isOK()) {
      // System.out.println("Could not apply follower current limits, error code: " +
      // statusPivotFollower.toString());
    }

    hardwareConfigured &= statusPivotFollower.isOK();
    // Phoenix thresholds use mechanism rotations because SensorToMechanismRatio is configured.
    pidPivotConfigOg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pidPivotConfigOg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_MAX_DEG / 360.0;
    pidPivotConfigOg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pidPivotConfigOg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_MIN_DEG / 360.0;

    configureMotionMagicDutyCycle(pidPivotConfigOg);

    StatusCode statusPivot = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusPivot = talonFXPivotConfigurator.apply(pidPivotConfigOg);
      if (statusPivot.isOK()) {
        break;
      }
    }
    hardwareConfigured &= statusPivot.isOK();
    applyPivotFollowerControl();
    if (!statusPivot.isOK()) {
      // System.out.println("Could not apply configs, error code: " +
      // statusPivot.toString());
    }
  }

  private void commandRollerVelocityInternal(double rollerRps) {
    if (!hardwareConfigured || !Double.isFinite(rollerRps) || !DriverStation.isEnabled()
        || RobotContainer.isPanicStopActive()) { stopIntake(); return; }
    rollerVelocityClosedLoopEnabled = true;
    rollerTargetRps = rollerRps;
    rollerCommandedMotorRps = motorRpsFromRollerRps(rollerRps);

    if (Double.isFinite(lastRollerVelocityCommandMotorRps)
        && Math.abs(lastRollerVelocityCommandMotorRps - rollerCommandedMotorRps) < 1e-6) {
      return;
    }

    // alex test
    // System.out.println("Commanding roller velocity. Roller RPS: " + rollerRps +
    // ", Commanded motor RPS: " + rollerCommandedMotorRps);

    intakeRollerMotor.setControl(
        rollerVelocityVoltage.withVelocity(rollerCommandedMotorRps));
    lastRollerVelocityCommandMotorRps = rollerCommandedMotorRps;
    lastRollerDutyCommand = Double.NaN;
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    config.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_MOTOR_TO_ARM_GEAR_RATIO;

    applyPivotMotionMagicGains(config);

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    motionMagicVoltage.Slot = PIVOT_DEPLOYED_SLOT;

  }

  private void applyPivotMotionMagicGains(TalonFXConfiguration config) {
    // Slot 0 keeps the existing deployed tuning unchanged.
    config.Slot0.kP = MotionMagicDutyCycleConstants.intake_kP_Deployed;
    config.Slot0.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.intake_kD_Deployed;
    config.Slot0.kS = MotionMagicDutyCycleConstants.intake_kS;
    config.Slot0.kV = MotionMagicDutyCycleConstants.intake_kV;
    config.Slot0.kA = MotionMagicDutyCycleConstants.intake_kA;
    config.Slot0.kG = MotionMagicDutyCycleConstants.intake_kG_Deployed;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.GravityArmPositionOffset = MotionMagicDutyCycleConstants.gravityArmPositionOffsetRot;

    // Slot 1 is dedicated to retract tuning.
    config.Slot1.kP = MotionMagicDutyCycleConstants.intake_kP_Retracted;
    config.Slot1.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot1.kD = MotionMagicDutyCycleConstants.intake_kD_Retracted;
    config.Slot1.kS = MotionMagicDutyCycleConstants.intake_kS;
    config.Slot1.kV = MotionMagicDutyCycleConstants.intake_kV;
    config.Slot1.kA = MotionMagicDutyCycleConstants.intake_kA;
    config.Slot1.kG = MotionMagicDutyCycleConstants.intake_kG_Retracted;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot1.GravityArmPositionOffset = MotionMagicDutyCycleConstants.gravityArmPositionOffsetRot;

    double boostMultiplier = 1.0 + IntakeConstants.TELEOP_PIVOT_CLOSED_LOOP_BOOST_PERCENT;
    config.Slot2.kP = Math.max(
        MotionMagicDutyCycleConstants.intake_kP_Deployed,
        MotionMagicDutyCycleConstants.intake_kP_Retracted) * boostMultiplier;
    config.Slot2.kI = MotionMagicDutyCycleConstants.intake_kI;
    config.Slot2.kD = Math.max(
        MotionMagicDutyCycleConstants.intake_kD_Deployed,
        MotionMagicDutyCycleConstants.intake_kD_Retracted) * boostMultiplier;
    config.Slot2.kS = MotionMagicDutyCycleConstants.intake_kS * boostMultiplier;
    config.Slot2.kV = MotionMagicDutyCycleConstants.intake_kV * boostMultiplier;
    config.Slot2.kA = MotionMagicDutyCycleConstants.intake_kA * boostMultiplier;
    config.Slot2.kG = 0.0;
    config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot2.GravityArmPositionOffset = MotionMagicDutyCycleConstants.gravityArmPositionOffsetRot;
  }

  private void applyPivotFollowerControl() {
    final MotorAlignmentValue alignment = IntakeConstants.intakePivotFollowerOpposeLeader
        ? MotorAlignmentValue.Opposed
        : MotorAlignmentValue.Aligned;
    intakePivotFollowerMotor.setControl(new Follower(IntakeConstants.intakePivotMotorId, alignment));
  }

  private int choosePivotClosedLoopSlot(double targetDeg) {
    double currentDeg = getPivotDeg();
    double deltaDeg = targetDeg - currentDeg;
    if (pivotClosedLoopBoostActive && Math.abs(deltaDeg) > 1e-3) {
      return PIVOT_BOOSTED_SLOT;
    }
    if (deltaDeg > 1e-3) {
      return PIVOT_DEPLOYED_SLOT;
    }
    if (deltaDeg < -1e-3) {
      return PIVOT_RETRACTED_SLOT;
    }
    return activePivotClosedLoopSlot;
  }

  private static double mechanismRotFromArmDeg(double armDeg) {
    return armDeg / 360.0;
  }

  private static double armDegFromMechanismRot(double mechanismRot) {
    return mechanismRot * 360.0;
  }

  private static double motorRpsFromRollerRps(double rollerRps) {
    // System.out.println("Converting roller RPS " + rollerRps + " to motor RPS");
    return rollerRps * IntakeConstants.ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO;
  }

  private static double rollerRpsFromMotorRps(double motorRps) {
    return motorRps / IntakeConstants.ROLLER_MOTOR_TO_ROLLER_GEAR_RATIO;
  }

  public double getPivotDeg() {
    return armDegFromMechanismRot(getIntakePivotMotorEncoder() - intakePivotEncoderZero);
  }

  public double getTargetPivotDeg() {
    return targetPivotDeg;
  }

  public boolean isPivotZeroed() {
    return hardwareConfigured && pivotZeroed && pivotPosSig != null && pivotPosSig.getStatus().isOK()
        && pivotPosSig.getTimestamp().getLatency() < .1;
  }

  public void setStayDeployedAfterTriggerRelease(boolean stayDeployed) {
    stayDeployedAfterTriggerRelease = stayDeployed;
  }

  public boolean shouldStayDeployedAfterTriggerRelease() {
    return stayDeployedAfterTriggerRelease;
  }

  private double getPivotPositionToleranceDeg() {
    return IntakePidConstants.PIVOT_AT_TARGET_TOLERANCE_DEG;
  }

  public void setTargetPivotDeg(double armDeg) {
    setTargetPivotDeg(armDeg, 0.0);
  }

  public void setTargetPivotDeg(double armDeg, double feedForwardVolts) {
    if (!isPivotZeroed() || !Double.isFinite(armDeg) || !Double.isFinite(feedForwardVolts)
        || !DriverStation.isEnabled() || RobotContainer.isPanicStopActive()) { stopPivotOutput(); return; }
    setPivotNeutralMode(NeutralModeValue.Brake);
    double clampedDeg = MathUtil.clamp(armDeg, IntakeConstants.PIVOT_MIN_DEG,
        IntakeConstants.PIVOT_MAX_DEG);
    targetPivotDeg = clampedDeg;

    double targetRot = intakePivotEncoderZero + mechanismRotFromArmDeg(clampedDeg);
    int desiredSlot = choosePivotClosedLoopSlot(clampedDeg);
    double clampedFeedForwardVolts = MathUtil.clamp(feedForwardVolts, -12.0, 12.0);
    if (Double.isFinite(lastPivotTargetRot)
        && Math.abs(lastPivotTargetRot - targetRot) < 1e-6
        && activePivotClosedLoopSlot == desiredSlot
        && Math.abs(motionMagicVoltage.FeedForward - clampedFeedForwardVolts) < 1e-6) {
      return;
    }
    intakePivotMotor.setControl(
        motionMagicVoltage
            .withSlot(desiredSlot)
            .withFeedForward(clampedFeedForwardVolts)
            .withPosition(targetRot));
    activePivotClosedLoopSlot = desiredSlot;
    lastPivotTargetRot = targetRot;
    lastPivotDutyCommand = Double.NaN;
  }

  private void setPivotNeutralMode(NeutralModeValue neutralMode) {
    if (pivotNeutralMode == neutralMode) {
      return;
    }

    if (!EnabledSubsystems.intake || intakePivotMotor == null) return;
    // Update neutral mode only: applying a fresh MotorOutputConfigs also resets inversion.
    hardwareConfigured &= intakePivotMotor.setNeutralMode(neutralMode).isOK();
    hardwareConfigured &= intakePivotFollowerMotor.setNeutralMode(neutralMode).isOK();
    pivotNeutralMode = neutralMode;
  }

  public void releaseDeployHoldToCoast() {
    targetPivotDeg = getPivotDeg();
    setPivotDutyCycle(0.0);
    setPivotNeutralMode(NeutralModeValue.Coast);
  }

  public void beginInitialAutoDeploy(double duty) {
    targetPivotDeg = IntakePositions.IntakeDeployedDeg.getPosition();
    setPivotNeutralMode(NeutralModeValue.Brake);
    setPivotDutyCycle(Math.abs(duty));
  }

  public void stopPivotInBrake() {
    targetPivotDeg = getPivotDeg();
    setPivotDutyCycle(0.0);
    setPivotNeutralMode(NeutralModeValue.Brake);
  }

  /** Disabled manual reseed requires physical retracted placement. */
  public void seedZeroFromRetractedHardStop() {
    if (!DriverStation.isDisabled()) return;
    seedVerifiedRetractedPosition();
  }

  /** Called only after the bounded homing command confirms fresh sustained stall evidence. */
  public void seedAfterConfirmedHoming() { seedVerifiedRetractedPosition(); }

  private void seedVerifiedRetractedPosition() {
    if (!EnabledSubsystems.intake || !hardwareConfigured) return;
    stopPivotOutput();
    intakePivotEncoderZero = 0;
    pivotZeroed = intakePivotMotor.setPosition(0).isOK();
    targetPivotDeg = 0;
    lastPivotTargetRot = Double.NaN;
  }

  private void stopPivotOutput() {
    homingActive = false;
    if (intakePivotMotor != null) intakePivotMotor.stopMotor();
    lastPivotDutyCommand = Double.NaN;
    lastPivotTargetRot = Double.NaN;
  }

  public boolean hasFreshHomingCurrents() {
    return hardwareConfigured && pivotLeaderStatorCurrentSig != null && pivotLeaderStatorCurrentSig.getStatus().isOK()
        && pivotFollowerStatorCurrentSig.getStatus().isOK()
        && pivotLeaderStatorCurrentSig.getTimestamp().getLatency() < .1
        && pivotFollowerStatorCurrentSig.getTimestamp().getLatency() < .1;
  }

  /** Only the bounded homing command uses this negative, power-limited soft-limit override. */
  public void runRetractedHoming() {
    if (!DriverStation.isEnabled() || !hasFreshHomingCurrents() || RobotContainer.isPanicStopActive()) {
      stopPivotOutput(); return;
    }
    homingActive = true;
    lastPivotTargetRot = lastPivotDutyCommand = Double.NaN;
    intakePivotMotor.setControl(new DutyCycleOut(-Math.abs(IntakeConstants.INTAKE_PIVOT_REZERO_RETRACT_DUTY))
        .withIgnoreSoftwareLimits(true));
  }

  public void setPivotDutyCycle(double duty) {
    homingActive = false;
    if (!Double.isFinite(duty) || !isPivotZeroed() || !DriverStation.isEnabled()
        || RobotContainer.isPanicStopActive()) { stopPivotOutput(); return; }
    // Voltage consistency is good, but for "jog", duty is fine and simple.
    double clampedDuty = MathUtil.clamp(duty, -1.0, 1.0);
    if (Double.isFinite(lastPivotDutyCommand) && Math.abs(lastPivotDutyCommand - clampedDuty) < 1e-6) {
      return;
    }
    intakePivotMotor.setControl(new DutyCycleOut(clampedDuty));
    lastPivotTargetRot = Double.NaN;
    lastPivotDutyCommand = clampedDuty;
  }

  public double getPivotLeaderStatorCurrentAmps() {
    return intakePivotMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getPivotFollowerStatorCurrentAmps() {
    return intakePivotFollowerMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getPivotAverageStatorCurrentAmps() {
    return (getPivotLeaderStatorCurrentAmps() + getPivotFollowerStatorCurrentAmps()) * 0.5;
  }

  public double getPivotVelocityDegPerSec() {
    return pivotVelSig.getValueAsDouble() * 360.0;
  }

  private void applyPivotCurrentLimits(
      double supplyCurrentLimitAmps,
      double supplyCurrentLowerLimitAmps,
      double supplyCurrentLowerTimeSec,
      double statorCurrentLimitAmps) {
    final var currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
    currentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimitAmps;
    currentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTimeSec;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = statorCurrentLimitAmps;

    intakePivotMotor.getConfigurator().apply(currentLimits);
    intakePivotFollowerMotor.getConfigurator().apply(currentLimits);
  }

  private void restoreDefaultPivotCurrentLimits() {
    applyPivotCurrentLimits(
        IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT_A,
        IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_LIMIT_A,
        IntakeConstants.PIVOT_SUPPLY_CURRENT_LOWER_TIME_S,
        IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_A);
  }

  private void enableTeleopPivotPowerBoost() {
    if (!DriverStation.isTeleopEnabled() || pivotClosedLoopBoostActive) {
      return;
    }

    pivotClosedLoopBoostActive = true;
  }

  public void enableTeleopDeployPivotPowerBoost() {
    enableTeleopPivotPowerBoost();
  }

  public void enableTeleopRetractPivotPowerBoost() {
    enableTeleopPivotPowerBoost();
  }

  public void disableTeleopPivotPowerBoost() {
    if (!pivotClosedLoopBoostActive) {
      return;
    }

    pivotClosedLoopBoostActive = false;
  }

  public void enableInitialAutoDeployCurrentBoost() {
    if (pivotCurrentBoostActive) {
      return;
    }

    applyPivotCurrentLimits(
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_BOOST_SUPPLY_CURRENT_LIMIT_A,
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_BOOST_SUPPLY_CURRENT_LOWER_LIMIT_A,
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_BOOST_SUPPLY_CURRENT_LOWER_TIME_S,
        IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_BOOST_STATOR_CURRENT_LIMIT_A);
    pivotCurrentBoostActive = true;
  }

  public void disableInitialAutoDeployCurrentBoost() {
    if (!pivotCurrentBoostActive) {
      return;
    }

    restoreDefaultPivotCurrentLimits();
    pivotCurrentBoostActive = false;
  }

  public void exitOpenLoopHold() {
    //intakePivotMotor.setControl(new DutyCycleOut(0.0));
    setTargetPivotDeg(targetPivotDeg);
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
    rollerDesiredMode = RollerDesiredMode.VELOCITY;
    rollerTargetRps = rollerRps;
    commandRollerVelocityInternal(rollerRps);
  }

  public void runIntakeNoPid(double duty){
    if (!hardwareConfigured || !Double.isFinite(duty) || !DriverStation.isEnabled()
        || RobotContainer.isPanicStopActive()) { stopIntake(); return; }
    rollerDesiredMode = RollerDesiredMode.DUTY; rollerVelocityClosedLoopEnabled = false;
    rollerTargetRps = rollerCommandedMotorRps = 0;
    // alex text
    // System.out.println("RUNNING INTAKE ROLLER");
    double clampedDuty = MathUtil.clamp(-duty, -1.0, 1.0);
    if (Double.isFinite(lastRollerDutyCommand) && Math.abs(lastRollerDutyCommand - clampedDuty) < 1e-6) {
      return;
    }
    intakeRollerMotor.setControl(new DutyCycleOut(clampedDuty));
    lastRollerDutyCommand = clampedDuty;
    lastRollerVelocityCommandMotorRps = Double.NaN;
  }

  public void runIntakeReverse() {
    rollerDesiredMode = RollerDesiredMode.VELOCITY;
    rollerTargetRps = IntakeConstants.ROLLER_REVERSE_RPS;
    commandRollerVelocityInternal(IntakeConstants.ROLLER_REVERSE_RPS);
  }

  public void runIntakeReverseNoPid(double duty) { runIntakeNoPid(duty); }

  /** Stop rotating the intake roller. */
  public void stopIntake() {
    rollerDesiredMode = RollerDesiredMode.OFF;
    rollerVelocityClosedLoopEnabled = false;
    rollerTargetRps = rollerCommandedMotorRps = 0;
    if (intakeRollerMotor != null) intakeRollerMotor.stopMotor();
    lastRollerVelocityCommandMotorRps = lastRollerDutyCommand = Double.NaN;
  }
  public void stopIntakeNoPid() { stopIntake(); }

  public void applyPanicStop() {
    pivotClosedLoopBoostActive = false;
    disableInitialAutoDeployCurrentBoost();
    stopIntake();
    setPivotDutyCycle(0.0);
    lastPivotTargetRot = Double.NaN;
  }

  public double getRollerTargetRps() {
    return rollerTargetRps;
  }

  public double getIntakePivotEncoderZeroPosition() {
    return intakePivotEncoderZero;
  }

  public double getIntakePivotMotorEncoder() {
    return pivotPosSig == null ? Double.NaN : pivotPosSig.getValueAsDouble();
  }

  public void setIntakePositionWithAngle(IntakePositions angle) {
    setTargetPivotDeg(angle.getPosition()); // now degrees
  }

  public void setIntakePositionWithAngle(IntakePositions angle, double feedForwardVolts) {
    setTargetPivotDeg(angle.getPosition(), feedForwardVolts); // now degrees
  }

  public boolean isAtPosition(IntakePositions position) {
    return isPivotZeroed() && Math.abs(position.getPosition() - getPivotDeg()) <= getPivotPositionToleranceDeg();
  }

  public boolean isAtPositionDeg(double targetDeg) {
    return isPivotZeroed() && Math.abs(targetDeg - getPivotDeg()) <= getPivotPositionToleranceDeg();
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
    runIntakeNoPid(-duty);
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
    setPivotDutyCycle(duty);
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

    if (RobotContainer.isPanicStopActive()) {
      applyPanicStop();
      return;
    }

    BaseStatusSignal.refreshAll(rollerVelSig, rollerVoltageSig, pivotPosSig, pivotVelSig, pivotVoltageSig,
        pivotLeaderStatorCurrentSig, pivotFollowerStatorCurrentSig);
    if (intakePivotMotor.hasResetOccurred() | intakePivotFollowerMotor.hasResetOccurred()) {
      pivotZeroed = false; stopPivotOutput(); applyPivotFollowerControl();
    }
    if (intakeRollerMotor.hasResetOccurred() | intakeRollerFollowerMotor.hasResetOccurred()) {
      stopIntake();
      intakeRollerFollowerMotor.setControl(new Follower(IntakeConstants.intakeRollerMotorId,
          IntakeConstants.intakeRollerFollowerOpposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }
    if (!DriverStation.isEnabled() || !hardwareConfigured) { stopIntake(); stopPivotOutput(); }
    if (!isPivotZeroed() && !homingActive) stopPivotOutput();
    org.littletonrobotics.junction.Logger.recordOutput("Intake/PivotTrusted", isPivotZeroed());
    org.littletonrobotics.junction.Logger.recordOutput("Intake/HardwareConfigured", hardwareConfigured);
    org.littletonrobotics.junction.Logger.recordOutput("Intake/PivotDegrees", getPivotDeg());
    org.littletonrobotics.junction.Logger.recordOutput("Intake/TargetDegrees", targetPivotDeg);
    org.littletonrobotics.junction.Logger.recordOutput("Intake/RollerMode", rollerDesiredMode.toString());
    org.littletonrobotics.junction.Logger.recordOutput("Intake/RollerDuty", lastRollerDutyCommand);

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
      SmartDashboard.putNumber(
          "Intake/PivotGravityOffsetRot",
          MotionMagicDutyCycleConstants.gravityArmPositionOffsetRot);

      SmartDashboard.putNumber("Intake/PivotPosDeg", getPivotDeg());
      SmartDashboard.putNumber("Intake/PivotTargetDeg", getTargetPivotDeg());
      SmartDashboard.putNumber("Intake/PivotErrorDeg", getTargetPivotDeg() - getPivotDeg());
      SmartDashboard.putBoolean("Intake/PivotZeroed", isPivotZeroed());

      SmartDashboard.putNumber("Intake/PivotToleranceDeg", getPivotPositionToleranceDeg());
      SmartDashboard.putNumber("Intake/PivotActiveClosedLoopSlot", activePivotClosedLoopSlot);
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
