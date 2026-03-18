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
import frc.robot.Constants.FieldTargets;
import frc.robot.Constants.OperatorConstants.TurretGeometry;
import frc.robot.lib.TurretHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestTurretAngleCommand extends Command {
  int counter = 0;
  /** Creates a new TestTurretAngleCommand. */
  public TestTurretAngleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Hardcoded test inputs (replace with real values for testing)
    Pose2d robotPose = new Pose2d(0.0, 4.021328, Rotation2d.fromDegrees(0.0));
    Translation2d targetPosition = new Translation2d(FieldTargets.HUB_BLUE_X, FieldTargets.HUB_BLUE_Y);
    double vx = 0.0, vy = 0.0, omega = 0.0;
    double readinessTimeMs = 0.0;

    // Calculate the turret angle
    double turretAngle = TurretHelpers.Solution.computeTurretYawAngleRelativeToRobotDeg(
        robotPose, targetPosition, vx, vy, omega, readinessTimeMs);

    // Print the result (in a real application, you could send this to SmartDashboard or a logger)
        System.out.println("Calculated Turret Angle: " + turretAngle + " degrees");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Pose2d robotPose = new Pose2d(1.542, 6.021, Rotation2d.fromDegrees(0));
    Pose2d robotPose =  RobotContainer.driveSubsystem.getPose();
    Translation2d targetPosition = new Translation2d(FieldTargets.HUB_BLUE_X, FieldTargets.HUB_BLUE_Y);
    double vx = 0.0, vy = 0.0, omega = 0.0;
    double readinessTimeMs = 0.0;

    // Calculate the turret angle
    double turretAngle = TurretHelpers.Solution.computeTurretYawAngleRelativeToRobotDeg(
        robotPose, targetPosition, vx, vy, omega, readinessTimeMs);

    // Print the result (in a real application, you could send this to SmartDashboard or a logger)
        System.out.println("Calculated Turret Angle: " + turretAngle + " degrees");
        counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 20;
  }
}
