// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // --- AdvantageKit setup (NO robot behavior changes) ---
    // Logger.recordMetadata("Project", "2026Competition");
    // Logger.recordMetadata("Mode", "Competition");

    if (isReal()) { 
      // Real robot logging
      // Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      // Logger.addDataReceiver(new NT4Publisher());
    } else {
      // NORMAL SIMULATION (no replay, real timing)
      // Logger.addDataReceiver(new NT4Publisher());
      // Logger.addDataReceiver(new WPILOGWriter("logs/sim"));
    }

    //Logger.start();
    m_robotContainer = new RobotContainer();
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void robotInit() {
    RobotContainer.setIfAllianceRed();

    // The YAW should be set by autos and not really here
    //RobotContainer.driveSubsystem.zeroYaw(); //Sets Yaw to 180 if on Red Alliance, or 0 on Blue (theoretically)
    // RobotContainer.driveSubsystem.zeroYawInitial();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //m_robotContainer.publishPoseToAdvantageScope();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.setIfAllianceRed();
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // One shared battery voltage for the whole robot simulation.
    // Each subsystem should set its motor controller SimState supply voltage from RoboRioSim.getVInVoltage().

    double totalCurrentAmps = 0.0;

    // Sum current draw from subsystems that simulate loads.
    // (Each subsystem returns 0 if disabled or not sim.)
    totalCurrentAmps += RobotContainer.turretSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.m_kraken.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.shooterSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.intakeSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.transferSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.spindexerSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.hoodSubsystem.getSimCurrentDrawAmps();
    totalCurrentAmps += RobotContainer.climbSubsystem.getSimCurrentDrawAmps(); // if present/enabled

    // Convert current draw -> loaded battery voltage and apply to RoboRIO (shared for all devices).
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps));
  }

}
