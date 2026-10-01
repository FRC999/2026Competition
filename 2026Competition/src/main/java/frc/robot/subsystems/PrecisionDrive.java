package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Measured motion/control boundary; allows controller tests without constructing CAN hardware. */
public interface PrecisionDrive extends Subsystem {
  Pose2d getPose();
  ChassisSpeeds getRobotRelativeSpeeds();
  ChassisSpeeds getFieldRelativeSpeeds();
  double getGyroYawRateRadiansPerSecond();
  SwerveModuleState[] getModuleStates();
  void driveRobotRelativeVelocity(ChassisSpeeds speeds);
  void holdPrecisionModuleAngles();
  void stop();
}
