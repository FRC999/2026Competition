package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;

/**
 * Hood subsystem (pitch axis) driven by a single TalonFX (Kraken).
 *
 * <p>Primary API is setTargetAngleRad(), where "angle" is the hood physical angle in radians.
 * Internally we convert that to TalonFX integrated-sensor rotations using constants.
 *
 * <p>At the beginning of the match, the team will start the hood fully down and we seed
 * the motor encoder to zero (position = 0 rotations).
 */
public class HoodSubsystem extends SubsystemBase {

  private final boolean isSim = RobotBase.isSimulation();

  private TalonFX hoodMotor;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  private StatusSignal<Angle> positionSig;
  private StatusSignal<AngularVelocity> velocitySig;
  private StatusSignal<Voltage> motorVoltageSig;

  private double positionRot = 0.0;
  private double velocityRps = 0.0;
  private double targetRot = 0.0;
  private double targetAngleRad = 0.0;

  // ---------------- Simulation ----------------
  private final FlywheelSim hoodSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          Constants.OperatorConstants.Hood.SIM_GEAR_RATIO,
          Constants.OperatorConstants.Hood.SIM_HOOD_J_KGM2),
      DCMotor.getKrakenX60(1));
  private double simPosRot = 0.0;

  // ---------------- SysId Characterization ----------------
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(Constants.OperatorConstants.SysId.HOOD_RAMP_RATE_V_PER_S),
          Volts.of(Constants.OperatorConstants.SysId.HOOD_STEP_V),
          Seconds.of(Constants.OperatorConstants.SysId.HOOD_TIMEOUT_S)),
      new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "hood"));

  /** Runtime gating for SysId. */
  private boolean isSysIdEnabled() {
    if (!Constants.OperatorConstants.SysId.ENABLE_SYSID) {
      return false;
    }
    return SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public HoodSubsystem() {
    if (!EnabledSubsystems.hood) {
      return;
    }

    hoodMotor = new TalonFX(
        Constants.OperatorConstants.Hood.MOTOR_ID,
        Constants.OperatorConstants.Hood.CANBUS_NAME);

    configureHardware();

    positionSig = hoodMotor.getPosition();
    velocitySig = hoodMotor.getVelocity();
    motorVoltageSig = hoodMotor.getMotorVoltage();
    configureStatusSignals();

    // Seed: hood starts fully down at beginning of match
    hoodMotor.setPosition(0.0);
    positionRot = 0.0;
    targetRot = 0.0;
    targetAngleRad = 0.0;
  }

  private void configureHardware() {
    MotorOutputConfigs out = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(Constants.OperatorConstants.Hood.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.OperatorConstants.Hood.SUPPLY_CURRENT_LIMIT_A)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.OperatorConstants.Hood.STATOR_CURRENT_LIMIT_A);

    Slot0Configs slot0 = new Slot0Configs()
        .withKP(Constants.OperatorConstants.Hood.kP)
        .withKI(Constants.OperatorConstants.Hood.kI)
        .withKD(Constants.OperatorConstants.Hood.kD)
        .withKS(Constants.OperatorConstants.Hood.kS)
        .withKV(Constants.OperatorConstants.Hood.kV)
        .withKA(Constants.OperatorConstants.Hood.kA);

    ClosedLoopGeneralConfigs cl = new ClosedLoopGeneralConfigs().withContinuousWrap(false);
								   

    TalonFXConfiguration cfg = new TalonFXConfiguration()
        .withMotorOutput(out)
        .withCurrentLimits(limits)
        .withSlot0(slot0)
        .withClosedLoopGeneral(cl);

    hoodMotor.getConfigurator().apply(cfg);
  }

  private void configureStatusSignals() {
										  
												  

    positionSig.setUpdateFrequency(100.0);
    velocitySig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

    hoodMotor.optimizeBusUtilization();
  }

  // ---------------- Public API ----------------

  public double getPositionRot() {
    return positionRot;
  }

  public double getVelocityRps() {
    return velocityRps;
  }

  public double getTargetAngleRad() {
    return targetAngleRad;
  }

  public double getAppliedVolts() {
    return motorVoltageSig.getValueAsDouble();
  }

  /** Command hood using a physical hood angle (radians). */
														
																														 
	 
  public void setTargetAngleRad(double angleRad) {
								
    double clampedRad = clamp(
        angleRad,
        Constants.OperatorConstants.Hood.MIN_ANGLE_RAD,
        Constants.OperatorConstants.Hood.MAX_ANGLE_RAD);

    targetAngleRad = clampedRad;
    targetRot = clampedRad * Constants.OperatorConstants.Hood.MOTOR_ROT_PER_RAD;
  }

  /** Direct motor-rotation target (kept for testing). */
  public void setTargetRot(double rot) {
    targetRot = rot;
														   
    targetAngleRad = rot / Constants.OperatorConstants.Hood.MOTOR_ROT_PER_RAD;
  }

  /** Open-loop duty-cycle (for quick tests). */
  public void setDutyCycle(double duty) {
								  
			 
	 
    hoodMotor.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    hoodMotor.setControl(dutyRequest.withOutput(0.0));
  }

  // ---------------- SysId factory commands ----------------
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled())
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    if (!isSysIdEnabled())
      return new edu.wpi.first.wpilibj2.command.InstantCommand();
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

    double duty = v / batt;
    hoodMotor.setControl(dutyRequest.withOutput(duty));
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled())
      return;
    log.motor("hood")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(getPositionRot()))
        .angularVelocity(RotationsPerSecond.of(getVelocityRps()));
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.hood) {
      return;
    }

    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);
    positionRot = positionSig.getValueAsDouble();
    velocityRps = velocitySig.getValueAsDouble();

    hoodMotor.setControl(positionRequest.withPosition(targetRot));

    if (DebugTelemetrySubsystems.hood) {
      SmartDashboard.putNumber("Hood/PosRot", positionRot);
      SmartDashboard.putNumber("Hood/VelRps", velocityRps);
      SmartDashboard.putNumber("Hood/TargetRot", targetRot);
      SmartDashboard.putNumber("Hood/TargetRad", targetAngleRad);
      SmartDashboard.putNumber("Hood/MotorVoltage", motorVoltageSig.getValueAsDouble());
    }
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim)
      return;
    if (!EnabledSubsystems.hood)
      return;

    final double dt = 0.02;

    var simState = hoodMotor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double appliedV = simState.getMotorVoltage();

    hoodSim.setInputVoltage(appliedV);
    hoodSim.update(dt);

    double rps = hoodSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    simPosRot += rps * dt;

    simState.setRawRotorPosition(simPosRot);
    simState.setRotorVelocity(rps);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.getCurrentDrawAmps()));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
