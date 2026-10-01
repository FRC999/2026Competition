package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.PrecisionDrive;
import org.junit.jupiter.api.*;

class PrecisionControllerIntegrationTest {
  static class Plant implements PrecisionDrive {
    Pose2d pose = new Pose2d();
    ChassisSpeeds speed = new ChassisSpeeds(), request = new ChassisSpeeds();
    boolean blocked;
    public Pose2d getPose() { return pose; }
    public ChassisSpeeds getRobotRelativeSpeeds() { return speed; }
    public ChassisSpeeds getFieldRelativeSpeeds() { return ChassisSpeeds.fromRobotRelativeSpeeds(speed, pose.getRotation()); }
    public double getGyroYawRateRadiansPerSecond() { return speed.omegaRadiansPerSecond; }
    public SwerveModuleState[] getModuleStates() {
      double max = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) + Math.abs(speed.omegaRadiansPerSecond) * .372;
      return new SwerveModuleState[] {new SwerveModuleState(max, Rotation2d.kZero)};
    }
    public void driveRobotRelativeVelocity(ChassisSpeeds value) { request = value; }
    public void holdPrecisionModuleAngles() { request = new ChassisSpeeds(); }
    public void stop() { holdPrecisionModuleAngles(); }
    void step() {
      // Deliberate 80ms first-order velocity lag; no estimator-to-truth feedback.
      speed = blocked ? new ChassisSpeeds() : new ChassisSpeeds(
          speed.vxMetersPerSecond + .25 * (request.vxMetersPerSecond - speed.vxMetersPerSecond),
          speed.vyMetersPerSecond + .25 * (request.vyMetersPerSecond - speed.vyMetersPerSecond),
          speed.omegaRadiansPerSecond + .25 * (request.omegaRadiansPerSecond - speed.omegaRadiansPerSecond));
      pose = pose.exp(new Twist2d(speed.vxMetersPerSecond * .02, speed.vyMetersPerSecond * .02, speed.omegaRadiansPerSecond * .02));
      SimHooks.stepTiming(.02);
    }
  }
  @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
  @BeforeEach void pause() { SimHooks.pauseTiming(); }
  @AfterEach void resume() { SimHooks.resumeTiming(); }

  void run(DriveToPosePrecisionCommand command, Plant plant) {
    command.initialize();
    for (int i = 0; i < 260 && !command.isFinished(); i++) { command.execute(); plant.step(); }
    assertTrue(command.isFinished());
    command.end(false);
  }
  @Test void translatesAndRotatesToSettledPose() {
    var plant = new Plant();
    var target = new Pose2d(1, .4, Rotation2d.fromDegrees(40));
    var command = new DriveToPosePrecisionCommand(plant, target);
    run(command, plant);
    assertTrue(command.succeeded(), command.getCompletionReason().toString());
    assertTrue(plant.pose.getTranslation().getDistance(target.getTranslation()) <= .04);
    assertTrue(Math.abs(plant.pose.getRotation().minus(target.getRotation()).getDegrees()) <= 1.5);
    assertEquals(0, plant.request.vxMetersPerSecond);
  }
  @Test void blockedMotionTimesOutWithoutClaimingArrival() {
    var plant = new Plant(); plant.blocked = true;
    var command = new DriveToPosePrecisionCommand(plant, new Pose2d(1, 0, Rotation2d.kZero));
    run(command, plant);
    assertEquals(DriveToPosePrecisionCommand.CompletionReason.TIMED_OUT, command.getCompletionReason());
  }
  @Test void poseOnlyCannotFinishWithoutVisionPermission() {
    var plant = new Plant();
    var command = new DriveToPosePrecisionCommand(plant, new Pose2d()).withFinishPermission(() -> false);
    run(command, plant);
    assertFalse(command.succeeded());
  }
  @Test void invalidInitialPoseStopsBeforeAnyControllerOutput() {
    var plant = new Plant(); plant.pose = new Pose2d(Double.NaN, 0, Rotation2d.kZero);
    var command = new DriveToPosePrecisionCommand(plant, new Pose2d());
    command.initialize(); assertTrue(command.isFinished()); command.end(false);
    assertEquals(DriveToPosePrecisionCommand.CompletionReason.INVALID_STATE, command.getCompletionReason());
    assertEquals(0, plant.request.vxMetersPerSecond);
  }
}
