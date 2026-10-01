package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Measured motion/control boundary for endpoint commands and hardware-free controller tests.
 * Poses always retain the blue field origin. Velocities use m/s and rad/s; module angles are radians.
 * Call control methods from the scheduler thread. Implementations must distinguish measured motion
 * from requested motion: a zero request alone is not evidence that the robot has stopped.
 */
public interface PrecisionDrive extends Subsystem {
  Pose2d getPose();
  ChassisSpeeds getRobotRelativeSpeeds();
  ChassisSpeeds getFieldRelativeSpeeds();
  double getGyroYawRateRadiansPerSecond();
  SwerveModuleState[] getModuleStates();
  /** Takes motion ownership with robot-frame +X forward, +Y left and positive counterclockwise omega. */
  void driveRobotRelativeVelocity(ChassisSpeeds speeds);
  /** Commands zero drive while retaining the module angles captured when entering this hold. */
  void holdPrecisionModuleAngles();
  /** Stops commanded motion; does not assert that measured speed has already reached zero. */
  void stop();
}
