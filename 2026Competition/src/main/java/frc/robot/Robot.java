// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("Project", "2026Competition/OffSeason-195");
    try (var in = Robot.class.getResourceAsStream("/build-info.properties")) {
      if (in != null) {
        var metadata = new java.util.Properties();
        metadata.load(in);
        for (String key : metadata.stringPropertyNames()) Logger.recordMetadata(key, metadata.getProperty(key));
      }
    } catch (java.io.IOException ex) {
      DriverStation.reportWarning("Build metadata unavailable: " + ex.getMessage(), false);
    }
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(isReal() ? new WPILOGWriter() : new WPILOGWriter("logs/sim"));
    Logger.start();

    m_robotContainer = new RobotContainer();

  }

@Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

@Override
  public void disabledPeriodic() {

  }

@Override
  public void autonomousInit() {
    RobotContainer.driveSubsystem.clearAutonomousPrecisionFailure();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

@Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

@Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

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
