
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.IntakeConstants;

/**
 * HopperSubsystem
 *
 * Mechanical responsibility (as you described):
 * - Storage only + extension/contraction to hold more balls.
 *
 * This subsystem is intentionally light right now. It provides placeholders for:
 * - extend / retract commands
 * - "isExtended" telemetry
 *
 * TODO: implement the actuator hardware (motor/pneumatic) once finalized.
 */
public class HopperSubsystem extends SubsystemBase {

  private boolean extended = false;

  private static TalonFX hopperExtendMotor;

  public HopperSubsystem() {
    hopperExtendMotor = new TalonFX(0);
  }

  private void configureMotors() { 
    hopperExtendMotor.getConfigurator().apply(new TalonFXConfiguration());
    hopperExtendMotor.setSafetyEnabled(false);

    var hopperMotorConfig = new MotorOutputConfigs();
    hopperMotorConfig.NeutralMode = NeutralModeValue.Brake;
    // hopperMotorConfig.Inverted = (HopperConstants.hopperMotorInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
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
  public void periodic() {
    SmartDashboard.putBoolean("Hopper/Extended", extended);
  }
}
