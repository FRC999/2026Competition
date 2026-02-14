// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.RobotContainer;
import frc.robot.lib.ElasticHelpers;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  // Global / cross-cutting telemetry (allowed to remain here).
  private void systemsCheckTelemetry() {
    SmartDashboard.putNumber("Battery-Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putString("Alliance-Side", ElasticHelpers.getAllianceSide());
    SmartDashboard.putString("Auto-Selected", ElasticHelpers.getAutoSelectedColor());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  private void teleopTelemetry() {
    SmartDashboard.putData("Field", ElasticHelpers.getRobotonfield());
    SmartDashboard.putData("Auto Field", ElasticHelpers.getAutoDisplayField());
    SmartDashboard.putString("Lock in to End Game", ElasticHelpers.shouldEndGameColor());
  }

  @Override
  public void periodic() {
    // Task #12: Gate SmartDashboardSubsystem output.
    if (!DebugTelemetrySubsystems.smartDashboard) {
      return;
    }

    // Keep pose -> field object updates here (global display), gated with the rest.
    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    ElasticHelpers.updateRobotPose(robotPose);

    systemsCheckTelemetry();
    teleopTelemetry();

    // Task #12: Subsystem-specific telemetry (LL visibility, turret sensors, etc.)
    // should live inside their respective subsystems, not here.
  }
}
