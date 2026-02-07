// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants.ClimbConstants;
import frc.robot.Constants.OperatorConstants.ClimbConstants.ClimbMotionMagicDutyCycleConstants;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX climbMotorLeft;
  private final TalonFX climbMotorRight;

  // Motion Magic request (position is in rotations per Phoenix 6)
  private final MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0);

  // Percent output request (manual)
  private final DutyCycleOut percentOut = new DutyCycleOut(0);

  // Stop request
  private final DutyCycleOut stopOut = new DutyCycleOut(0);

  // Last requested target (for telemetry + hold)
  private double lastSetpointRot = 0.0;

  // Only for dashboard readability; Phoenix 6 position is already rotations.
  private static final double TICKS_PER_ROT = 2048.0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotorLeft = new TalonFX(ClimbConstants.climbMotorLeftID);
    climbMotorRight = new TalonFX(ClimbConstants.climbMotorRightID);

    climbMotorLeft.setSafetyEnabled(false);
    climbMotorRight.setSafetyEnabled(false);

    publishDashboardDefaults();
    configMotors();
  }

  // ---------------------------------------------------------------------------
  // CONFIG
  // ---------------------------------------------------------------------------

  private void configMotors() {
    // IMPORTANT FIX:
    // Build the config fully FIRST (PID + MotionMagic + CurrentLimits), THEN apply.
    // Your original code applied config before setting PID/MM values, so they were never sent.

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Brake mode (climb should not free-fall)
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;

    leftConfig.withMotorOutput(motorOutput);
    rightConfig.withMotorOutput(motorOutput);

    // PID + Motion Magic
    configureMotionMagicDutyCycle(leftConfig);
    configureMotionMagicDutyCycle(rightConfig);

    // Current limits (MANDATORY)
    applyCurrentLimits(leftConfig);
    applyCurrentLimits(rightConfig);

    // Apply configs with retry (CAN hiccups happen)
    applyWithRetry(climbMotorLeft, leftConfig, "Climb Left");
    applyWithRetry(climbMotorRight, rightConfig, "Climb Right");

    // Follower: right follows left. OpposeMasterDirection is a BOOLEAN in Phoenix 6.
    // false = same direction (most common when mechanically linked),
    // true  = opposite direction (mirrored mounting, etc.)
    boolean oppose = SmartDashboard.getBoolean("Climb/FollowerOpposeMaster", false);
    climbMotorRight.setControl(new Follower(ClimbConstants.climbMotorLeftID, MotorAlignmentValue.Opposed));
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    // PID on Position (Slot0)
    config.Slot0.kP = ClimbMotionMagicDutyCycleConstants.climb_kP;
    config.Slot0.kI = ClimbMotionMagicDutyCycleConstants.climb_kI;
    config.Slot0.kD = ClimbMotionMagicDutyCycleConstants.climb_kD;

    // Motion Magic constraints
    config.MotionMagic.MotionMagicCruiseVelocity = ClimbMotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ClimbMotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = ClimbMotionMagicDutyCycleConstants.motionMagicJerk;

    // Which slot the control request uses
    motMagDutyCycle.Slot = ClimbMotionMagicDutyCycleConstants.slot;
  }

  private void applyCurrentLimits(TalonFXConfiguration config) {
    // "Typical FRC" style: enable supply + stator current limits via CTRE config.
    // Supply limit helps protect battery/PDH and reduce brownout risk.
    // Stator limit caps torque spikes and protects the motor/controller.
    //
    // Values are dashboard-tunable so you can dial them in without touching Constants.java.

    boolean enable = SmartDashboard.getBoolean("Climb/EnableCurrentLimits", true);

    double supplyLimitA = SmartDashboard.getNumber("Climb/SupplyCurrentLimitA", 60.0);
    double supplyThresholdA = SmartDashboard.getNumber("Climb/SupplyCurrentThresholdA", 70.0);
    double supplyThresholdTimeS = SmartDashboard.getNumber("Climb/SupplyCurrentThresholdTimeS", 0.10);

    double statorLimitA = SmartDashboard.getNumber("Climb/StatorCurrentLimitA", 120.0);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimitEnable = enable;
    limits.SupplyCurrentLimit = supplyLimitA;
    limits.SupplyCurrentLimit = supplyThresholdA;
    limits.SupplyCurrentLowerTime = supplyThresholdTimeS;

    limits.StatorCurrentLimitEnable = enable;
    limits.StatorCurrentLimit = statorLimitA;

    config.withCurrentLimits(limits);
  }

  private void applyWithRetry(TalonFX motor, TalonFXConfiguration config, String name) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }
    System.out.println("Could not apply configs for " + name + ", error: " + status.toString());
  }

  private void publishDashboardDefaults() {
    // Current limits (tune at practice)
    SmartDashboard.putBoolean("Climb/EnableCurrentLimits", true);
    SmartDashboard.putNumber("Climb/SupplyCurrentLimitA", 60.0);
    SmartDashboard.putNumber("Climb/SupplyCurrentThresholdA", 70.0);
    SmartDashboard.putNumber("Climb/SupplyCurrentThresholdTimeS", 0.10);
    SmartDashboard.putNumber("Climb/StatorCurrentLimitA", 120.0);

    // Follower direction toggle (so you can flip it later without code changes)
    SmartDashboard.putBoolean("Climb/FollowerOpposeMaster", false);

    // "At setpoint" tolerance for commands (rotations)
    SmartDashboard.putNumber("Climb/AtSetpointTolRot", 0.05);

    // Manual duty cycle magnitude (0..1)
    SmartDashboard.putNumber("Climb/ManualDuty", 0.25);
  }

  // ---------------------------------------------------------------------------
  // BASIC CONTROL METHODS (Subsystem API)
  // ---------------------------------------------------------------------------

  /** Motion Magic position setpoint (units: rotations). */
  public void setMotionMagicDutyCycle(double positionRot) {
    lastSetpointRot = positionRot;
    climbMotorLeft.setControl(motMagDutyCycle.withPosition(positionRot));
  }

  /** Manual open-loop output (-1..1). Useful for testing / emergency moves. */
  public void setDutyCycle(double dutyCycle) {
    // Clamp for safety (commentary: avoids accidental >1 inputs)
    double dc = Math.max(-1.0, Math.min(1.0, dutyCycle));
    climbMotorLeft.setControl(percentOut.withOutput(dc));
  }

  /** Stop the climber (leader). Follower stops because it follows leader output. */
  public void stopMotors() {
    climbMotorLeft.setControl(stopOut);
  }

  /** Leader rotor position (rotations). */
  public double getClimbMotorEncoder() {
    return climbMotorLeft.getRotorPosition().getValueAsDouble();
  }

  /** Follower rotor position (rotations). */
  public double getClimbMotorRightEncoder() {
    return climbMotorRight.getRotorPosition().getValueAsDouble();
  }

  /** Returns absolute position error vs last setpoint (rotations). */
  public double getPositionErrorRot() {
    return Math.abs(getClimbMotorEncoder() - lastSetpointRot);
  }

  /** Simple "at setpoint" check based on last setpoint and dashboard tolerance. */
  public boolean atSetpoint() {
    double tol = SmartDashboard.getNumber("Climb/AtSetpointTolRot", 0.05);
    return getPositionErrorRot() <= tol;
  }

  // ---------------------------------------------------------------------------
  // COMMAND FACTORIES (so RobotContainer can bind buttons cleanly)
  // ---------------------------------------------------------------------------

  /**
   * Command: Move to a Motion Magic target and finish when within tolerance.
   * Commentary: This is your "go to climb position" command.
   */
  public Command cmdMoveToPosition(double positionRot) {
    return Commands.runOnce(() -> setMotionMagicDutyCycle(positionRot), this)
        .andThen(Commands.waitUntil(this::atSetpoint));
  }

  /**
   * Command: Hold current position (captures current rotor position and commands it).
   * Commentary: This is a software "hold" using Motion Magic at the current spot.
   */
  public Command cmdHoldCurrentPosition() {
    return Commands.runOnce(() -> setMotionMagicDutyCycle(getClimbMotorEncoder()), this);
  }

  /**
   * Command: Stop motors (ends immediately).
   */
  public Command cmdStop() {
    return Commands.runOnce(this::stopMotors, this);
  }

  /**
   * Command: Manual up (runs until interrupted).
   * Commentary: Use for testing or emergency only.
   */
  public Command cmdManualUp() {
    return Commands.runEnd(
        () -> setDutyCycle(Math.abs(SmartDashboard.getNumber("Climb/ManualDuty", 0.25))),
        this::stopMotors,
        this);
  }

  /**
   * Command: Manual down (runs until interrupted).
   * Commentary: Use for testing or emergency only.
   */
  public Command cmdManualDown() {
    return Commands.runEnd(
        () -> setDutyCycle(-Math.abs(SmartDashboard.getNumber("Climb/ManualDuty", 0.25))),
        this::stopMotors,
        this);
  }

  // ---------------------------------------------------------------------------
  // TELEMETRY
  // ---------------------------------------------------------------------------

  private void publishTelemetry() {
    double leftRot = getClimbMotorEncoder();
    double rightRot = getClimbMotorRightEncoder();

    SmartDashboard.putNumber("Climb/LeftPosRot", leftRot);
    SmartDashboard.putNumber("Climb/RightPosRot", rightRot);

    // Derived "ticks" for human readability/logging (Phoenix 6 returns rotations natively)
    SmartDashboard.putNumber("Climb/LeftPosTicks", leftRot * TICKS_PER_ROT);
    SmartDashboard.putNumber("Climb/RightPosTicks", rightRot * TICKS_PER_ROT);

    SmartDashboard.putNumber("Climb/LastSetpointRot", lastSetpointRot);
    SmartDashboard.putNumber("Climb/PosErrorRot", getPositionErrorRot());
    SmartDashboard.putBoolean("Climb/AtSetpoint", atSetpoint());

    SmartDashboard.putNumber("Climb/LeftSupplyA", climbMotorLeft.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climb/RightSupplyA", climbMotorRight.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Climb/LeftStatorA", climbMotorLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climb/RightStatorA", climbMotorRight.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Climb/FollowerOpposeMaster", SmartDashboard.getBoolean("Climb/FollowerOpposeMaster", false));
  }

  @Override
  public void periodic() {
    publishTelemetry();
  }
}
