// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

public class ReverseShooterTemporary extends Command {
  private static boolean pendingSnapshotValid = false;
  private static boolean pendingShootRequested = false;
  private static AutoShootSupervisorSubsystem.ShotMode pendingShotMode =
      AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO;

  private double previousTargetRpm = 0.0;
  private boolean previousShootRequested = false;
  private AutoShootSupervisorSubsystem.ShotMode previousShotMode =
      AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO;

  public static void captureCurrentShooterControlState() {
    pendingShootRequested = RobotContainer.autoShootSupervisorSubsystem.isShootRequested();
    pendingShotMode = RobotContainer.autoShootSupervisorSubsystem.getShotMode();
    pendingSnapshotValid = true;
  }

  public ReverseShooterTemporary() {
    addRequirements(RobotContainer.shooterSubsystem);
  }

  @Override
  public void initialize() {
    previousTargetRpm = RobotContainer.shooterSubsystem.getTargetRpm();

    if (pendingSnapshotValid) {
      previousShootRequested = pendingShootRequested;
      previousShotMode = pendingShotMode;
      pendingSnapshotValid = false;
    } else {
      previousShootRequested = RobotContainer.autoShootSupervisorSubsystem.isShootRequested();
      previousShotMode = RobotContainer.autoShootSupervisorSubsystem.getShotMode();
    }

    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
    RobotContainer.shooterSubsystem.setReverseTargetRpm(
        Constants.OperatorConstants.Shooter.REVERSE_CLEAR_RPM);
  }

  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.setReverseTargetRpm(
        Constants.OperatorConstants.Shooter.REVERSE_CLEAR_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.autoShootSupervisorSubsystem.setShotMode(previousShotMode);
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(previousShootRequested);

    if (previousShootRequested) {
      return;
    }

    if (previousTargetRpm > 1.0) {
      RobotContainer.shooterSubsystem.setTargetRpm(previousTargetRpm);
    } else {
      RobotContainer.shooterSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
