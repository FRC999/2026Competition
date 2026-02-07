// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.ClimbConstants;
import frc.robot.Constants.OperatorConstants.ClimbConstants.ClimbMotionMagicDutyCycleConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePidConstants.MotionMagicDutyCycleConstants;

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX climbMotorLeft;
  private TalonFX climbMotorRight;

  private MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0); // for MotionMagic Duty Cycle

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotorLeft = new TalonFX(ClimbConstants.climbMotorLeftID);
    climbMotorRight = new TalonFX(ClimbConstants.climbMotorRightID);

    configMotors();
  }

  private void configMotors() {
    climbMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    climbMotorLeft.setSafetyEnabled(false);

    var climbMotorLeftConfig = new MotorOutputConfigs();
    climbMotorLeftConfig.NeutralMode = NeutralModeValue.Brake;

    var climbMotorLeftConfigurator = climbMotorLeft.getConfigurator();

    TalonFXConfiguration TalonFXClimbMotorLeftConfig = new TalonFXConfiguration().withMotorOutput(climbMotorLeftConfig);
    
    StatusCode statusClimbLeft = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusClimbLeft = climbMotorLeftConfigurator.apply(TalonFXClimbMotorLeftConfig);
      if (statusClimbLeft.isOK())
        break;
    }
    if (!statusClimbLeft.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusClimbLeft.toString());
    }

    configureMotionMagicDutyCycle(TalonFXClimbMotorLeftConfig);


    climbMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    climbMotorRight.setSafetyEnabled(false);

    var climbMotorRightConfig = new MotorOutputConfigs();
    climbMotorRightConfig.NeutralMode = NeutralModeValue.Brake;
    climbMotorRight.setControl(new Follower(ClimbConstants.climbMotorLeftID, null));

    var climbMotorRightConfigurator = climbMotorRight.getConfigurator();

    TalonFXConfiguration TalonFXClimbMotorRightConfig = new TalonFXConfiguration().withMotorOutput(climbMotorRightConfig);
    
    StatusCode statusClimbRight = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusClimbRight = climbMotorRightConfigurator.apply(TalonFXClimbMotorRightConfig);
      if (statusClimbRight.isOK())
        break;
    }
    if (!statusClimbRight.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusClimbRight.toString());
    }
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    //config.Slot0.kS = 0.24; // add 0.24 V to overcome friction
    //config.Slot0.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = ClimbMotionMagicDutyCycleConstants.climb_kP;
    config.Slot0.kI = ClimbMotionMagicDutyCycleConstants.climb_kI;
    config.Slot0.kD = ClimbMotionMagicDutyCycleConstants.climb_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = ClimbMotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ClimbMotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = ClimbMotionMagicDutyCycleConstants.motionMagicJerk;

    motMagDutyCycle.Slot = ClimbMotionMagicDutyCycleConstants.slot;
  }

  public void setMotionMagicDutyCycle(double position){
    climbMotorLeft.setControl(motMagDutyCycle.withPosition(position));
    System.out.println("***Pos: " + position);
  }

  public double getClimbMotorEncoder() {
    return climbMotorLeft.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
