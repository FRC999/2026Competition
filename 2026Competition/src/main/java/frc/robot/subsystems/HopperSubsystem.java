
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OperatorConstants.Hopper;
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
  private TalonFX hopperMotor;
  
  private boolean extended = false;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);


  

  public HopperSubsystem() {
    if(!Constants.EnabledSubsystems.hopper){
      return;
    }
    hopperMotor = new TalonFX(Hopper.MOTOR_ID, Hopper.CANBUS_NAME);

    configureMotors();
  }

  private void configureMotors() { 
    MotorOutputConfigs out = new MotorOutputConfigs()
      .withInverted(Constants.OperatorConstants.Hopper.MOTOR_INVERTED)
      .withNeutralMode(Constants.OperatorConstants.Hopper.NEUTRAL_COAST);

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LIMIT_A)
      .withSupplyCurrentLowerTime(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_TIME_S)
      .withSupplyCurrentLowerLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_LIMIT_A)
      .withStatorCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
      .withStatorCurrentLimit(Constants.OperatorConstants.Hopper.STATOR_CURRENT_LIMIT_A);

    Slot0Configs slot0 = new Slot0Configs()
      .withKP(Constants.OperatorConstants.Hopper.kP)
      .withKI(Constants.OperatorConstants.Hopper.kI)
      .withKD(Constants.OperatorConstants.Hopper.kD)
      .withKS(Constants.OperatorConstants.Hopper.kS)
      .withKV(Constants.OperatorConstants.Hopper.kV)
      .withKA(Constants.OperatorConstants.Hopper.kA);

    TalonFXConfiguration cfg = new TalonFXConfiguration()
      .withMotorOutput(out)
      .withCurrentLimits(limits)
      .withSlot0(slot0);

     hopperMotor.getConfigurator().apply(cfg);

  
    // hopperExtendMotor.getConfigurator().apply(new TalonFXConfiguration());
    // hopperExtendMotor.setSafetyEnabled(false);

    // var hopperMotorConfig = new MotorOutputConfigs();
    // hopperMotorConfig.NeutralMode = NeutralModeValue.Brake;
    // hopperMotorConfig.Inverted = (Hopper.MOTOR_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    // hopperMotorConfig.Inverted = (IntakeConstants.IntakeRollerInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    // var talonFXRollerConfigurator =  MotorRollerconfig.getConfigurator();
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
    double p = clamp(percent, -Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE, Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE);
    hopperMotor.setControl(dutyCycle.withOutput(p));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
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
    if (!Constants.EnabledSubsystems.hopper) {
      return;
    }
    var sim = hopperMotor.getSimState();

    double batteryV = RoboRioSim.getVInVoltage();
    sim.setSupplyVoltage(batteryV);

    double motorVolts = sim.getMotorVoltage();

  }

   
  public void periodic() {
    if (!Constants.EnabledSubsystems.hopper) {
      return;
    }
    if (DebugTelemetrySubsystems.hopper) {
    SmartDashboard.putBoolean("Hopper/Extended", extended);
    }
  }
}
