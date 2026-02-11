package frc.robot.subsystems;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;

/**
 * Hood subsystem (pitch axis) driven by a single TalonFX (Kraken).
 *
 * <p>Primary API is setTargetAngleRad(), where "angle" is the hood physical angle in radians.
 * Internally we convert that to TalonFX integrated-sensor rotations using constants.
 *
 * <p>At the beginning of the match, the team will start the hood fully down and we will seed
 * the motor encoder to zero (position = 0 rotations).
 */
public class HoodSubsystem extends SubsystemBase {

  private TalonFX hoodMotor;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  private StatusSignal<Angle> positionSig;
  private StatusSignal<Voltage> motorVoltageSig;

  private double positionRot = 0.0;
  private double targetRot = 0.0;
  private double targetAngleRad = 0.0;

  public HoodSubsystem() {
    if (!EnabledSubsystems.hood) {
      return;
    }

    hoodMotor = new TalonFX(
        Constants.OperatorConstants.Hood.MOTOR_ID,
        Constants.OperatorConstants.Hood.CANBUS_NAME);

    configureHardware();
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

    ClosedLoopGeneralConfigs cl = new ClosedLoopGeneralConfigs()
        .withContinuousWrap(false);

    TalonFXConfiguration cfg = new TalonFXConfiguration()
        .withMotorOutput(out)
        .withCurrentLimits(limits)
        .withSlot0(slot0)
        .withClosedLoopGeneral(cl);

    hoodMotor.getConfigurator().apply(cfg);
  }

  private void configureStatusSignals() {
    positionSig = hoodMotor.getPosition();
    motorVoltageSig = hoodMotor.getMotorVoltage();

    positionSig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

    hoodMotor.optimizeBusUtilization();
  }

  public double getPositionRot() {
    return positionRot;
  }

  public double getTargetAngleRad() {
    return targetAngleRad;
  }

  /**
   * Command hood using a physical hood angle (radians).
   * Internally converts to TalonFX integrated sensor rotations using Constants.OperatorConstants.Hood.MOTOR_ROT_PER_RAD.
   */
  public void setTargetAngleRad(double angleRad) {
    // Clamp in hood-angle space
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
    // Keep angle telemetry consistent with rotation target
    targetAngleRad = rot / Constants.OperatorConstants.Hood.MOTOR_ROT_PER_RAD;
  }

  public void setDutyCycle(double duty) {
    if (!EnabledSubsystems.hood) {
      return;
    }
    hoodMotor.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    if (!EnabledSubsystems.hood) {
      return;
    }
    hoodMotor.setControl(dutyRequest.withOutput(0.0));
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.hood) {
      return;
    }

    positionRot = positionSig.getValueAsDouble();

    hoodMotor.setControl(positionRequest.withPosition(targetRot));

    if (DebugTelemetrySubsystems.hood) {
      SmartDashboard.putNumber("Hood/PosRot", getPositionRot());
      SmartDashboard.putNumber("Hood/TargetRot", targetRot);
      SmartDashboard.putNumber("Hood/TargetRad", targetAngleRad);
      SmartDashboard.putNumber("Hood/MotorVoltage", motorVoltageSig.getValueAsDouble());
    }
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
