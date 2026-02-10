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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;

/**
 * Hood mechanism driven by a single Kraken (TalonFX).
 *
 * <p>For now, "angle" is expressed in TalonFX internal rotations (same as the motor encoder),
 * because you said you will calibrate zero at the start of a match with the hood fully down.
 *
 * <p>Future upgrade: add a CANcoder/Through-bore absolute sensor to seed/verify position.
 */
public class HoodSubsystem extends SubsystemBase {

  private TalonFX hoodMotor;

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private StatusSignal<Angle> positionSig;

  // Stored target in rotations (motor units)
  private double targetRot = 0.0;

  public HoodSubsystem() {
    if (!EnabledSubsystems.hood) {
      return;
    }

    hoodMotor = new TalonFX(Constants.OperatorConstants.Hood.MOTOR_ID, Constants.OperatorConstants.Hood.CANBUS_NAME);

    configureHardware();

    positionSig = hoodMotor.getPosition();
    positionSig.setUpdateFrequency(100.0);
    hoodMotor.optimizeBusUtilization();

    seedZeroAtBoot();
  }

  private void configureHardware() {
    MotorOutputConfigs out = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(Constants.OperatorConstants.Hood.MOTOR_INVERTED
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

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

    // Relative sensor => no wrap.
    ClosedLoopGeneralConfigs cl = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

    TalonFXConfiguration cfg = new TalonFXConfiguration()
        .withMotorOutput(out)
        .withCurrentLimits(limits)
        .withClosedLoopGeneral(cl)
        .withSlot0(slot0);

    hoodMotor.getConfigurator().apply(cfg);
  }

  /**
   * Seed hood position to 0 rotations at boot.
   *
   * <p>This matches your stated procedure: at the beginning of a match, hood is fully down,
   * so we can treat that as an accurate zero reference.
   *
   * <p>If you later add an absolute sensor, this should be replaced with a seed-from-absolute routine.
   */
  private void seedZeroAtBoot() {
    Timer.delay(0.05);
    hoodMotor.setPosition(0.0);
    targetRot = 0.0;

    SmartDashboard.putNumber("Hood/SeedPosRot", 0.0);
  }

  // ---------------- Public API ----------------

  /** Current hood position in motor rotations (relative). */
  public double getPositionRot() {
    if (!EnabledSubsystems.hood) return 0.0;
    positionSig.refresh();
    return positionSig.getValue().in(edu.wpi.first.units.Units.Rotations);
  }

  /** Set hood target in motor rotations (relative). */
  public void setTargetRot(double rot) {
    if (!EnabledSubsystems.hood) return;

    // Optional soft limits (if you have constants for them later).
    // For now we just clamp to something reasonable if constants exist; else pass-through.
    targetRot = rot;
    hoodMotor.setControl(positionRequest.withPosition(targetRot));
  }

  /** Open-loop duty-cycle (manual tests only). */
  public void setDutyCycle(double duty) {
    if (!EnabledSubsystems.hood) return;
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    hoodMotor.setControl(dutyRequest.withOutput(duty));
  }

  public void stop() {
    if (!EnabledSubsystems.hood) return;
    hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (!EnabledSubsystems.hood) return;

    BaseStatusSignal.refreshAll(positionSig);

    SmartDashboard.putNumber("Hood/PosRot", getPositionRot());
    SmartDashboard.putNumber("Hood/TargetRot", targetRot);
  }
}
