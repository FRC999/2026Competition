// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.TurretHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestTurretAngleCommand extends Command {
  /** Creates a new TestTurretAngleCommand. */
  public TestTurretAngleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.autoShootSupervisorSubsystem, RobotContainer.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Hardcoded test inputs (replace with real values for testing)
        Pose2d robotPose = new Pose2d(5.0, 10.0, new Rotation2d(Math.toRadians(45)));
        Translation2d targetPosition = new Translation2d(20.0, 15.0);
        double vx = 0, vy = 0, omega = 0;
        double readinessTimeMs = 500.0;
        Twist2d turretOffset = new Twist2d(0.5, 0.5, 0.0); // Offset from robot center to turret

        // Calculate the turret angle
        double turretAngle = TurretHelpers.Solution.computeTurretYawAngleRelativeToRobotDeg(robotPose, targetPosition, vx, vy, omega, readinessTimeMs, turretOffset);

        // Print the result (in a real application, you could send this to SmartDashboard or a logger)
        System.out.println("Calculated Turret Angle: " + turretAngle + " degrees");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
