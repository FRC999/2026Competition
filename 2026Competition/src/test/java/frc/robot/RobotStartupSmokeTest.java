package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.ShootWhileHeld;
import frc.robot.commands.ReverseShooterTemporary;
import frc.robot.commands.ReverseTransfer;
import frc.robot.commands.ReverseSpindexer;
import frc.robot.lib.ShotPlanner;
import org.junit.jupiter.api.*;

@Tag("robotSmoke")
class RobotStartupSmokeTest {
  @Test @Timeout(45)
  void bootsAndChecksLocalizationModesCommandOwnershipAndRepeatedAuto() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    assertTrue(RobotBase.isSimulation());
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setJoystickAxisCount(Constants.OperatorConstants.OIContants.BUTTON_BOX, 6);
    DriverStationSim.notifyNewData();
    var error = new AtomicReference<Throwable>();
    var actions = new ConcurrentLinkedQueue<Runnable>();
    try (var robot = new Robot() {
      @Override public void robotPeriodic() {
        Runnable action;
        while ((action = actions.poll()) != null) action.run();
        super.robotPeriodic();
      }
    }) {
      Thread loop = new Thread(() -> {
        try { robot.startCompetition(); } catch (Throwable ex) { error.set(ex); }
      }, "offseason-desktop-smoke");
      loop.start();
      try {
        pump(150);
        assertNull(error.get(), () -> String.valueOf(error.get()));
        assertTrue(SmartDashboard.getBoolean("Vision/ConfigValid", false));
        assertTrue(RobotContainer.vision.hasRecentMeasurement(), "Expected simulated PhotonVision fusion");
        assertTrue(RobotContainer.vision.isLocalizationReady(), "Stable disabled MultiTag must establish a field reference");
        assertTrue(RobotContainer.driveSubsystem.getPose().getTranslation()
            .getDistance(RobotContainer.driveSubsystem.getSimulationTruthPose().getTranslation()) < .20);
        var before = RobotContainer.driveSubsystem.getSimulationTruthPose();
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        pump(12);
        assertTrue(Math.abs(RobotContainer.driveSubsystem.getPose().getRotation()
            .minus(before.getRotation()).getDegrees()) < 1, "Late alliance must not rewrite field yaw");
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        pump(65);
        assertNull(error.get(), () -> String.valueOf(error.get()));
        assertTrue(RobotContainer.driveSubsystem.getSimulationTruthPose().getTranslation()
            .getDistance(before.getTranslation()) < .02, "Default auto must not drive");
        onLoop(actions, () -> RobotContainer.driveSubsystem.resetPoseFromVision(
            new Pose2d(10, 5, Rotation2d.fromDegrees(90))));
        assertTrue(RobotContainer.driveSubsystem.getPose().getTranslation()
            .getDistance(before.getTranslation()) < .20, "Manual vision seed must be ignored while enabled");

        DriverStationSim.setAutonomous(false);
        pump(10);
        onLoop(actions, RobotContainer.driveSubsystem::orientDriverForwardToCurrentHeading);
        assertTrue(Math.abs(RobotContainer.driveSubsystem.getPose().getRotation()
            .minus(before.getRotation()).getDegrees()) < 1);
        assertTrue(Math.abs(RobotContainer.driveSubsystem.getOperatorForwardDirection()
            .minus(before.getRotation()).getDegrees()) < 1, "Driver reset changes perspective only");

        var shoot = new ShootWhileHeld(ShotPlanner.Mode.MOVING_AUTO, false);
        var reverse = new ReverseShooterTemporary().alongWith(new ReverseTransfer(), new ReverseSpindexer());
        onLoop(actions, shoot::schedule); pump(5);
        assertTrue(RobotContainer.autoShootSupervisorSubsystem.isShootRequested());
        onLoop(actions, reverse::schedule); pump(8);
        assertFalse(shoot.isScheduled());
        assertFalse(RobotContainer.autoShootSupervisorSubsystem.isShootRequested());
        assertEquals(frc.robot.subsystems.AutoShootSupervisorSubsystem.VolleyState.EXTERNAL_CONTROL,
            RobotContainer.autoShootSupervisorSubsystem.getVolleyState());
        assertTrue(RobotContainer.shooterSubsystem.getTargetRpm() <= 0, "Supervisor must not overwrite reverse");
        onLoop(actions, () -> {
          var state = RobotContainer.autoShootSupervisorSubsystem.getVolleyState();
          RobotContainer.autoShootSupervisorSubsystem.calculateDiagnosticSolution();
          RobotContainer.autoShootSupervisorSubsystem.getHubCommandRelativeAngleDeg();
          assertEquals(state, RobotContainer.autoShootSupervisorSubsystem.getVolleyState());
          assertFalse(RobotContainer.autoShootSupervisorSubsystem.isShootRequested());
        });
        onLoop(actions, reverse::cancel); pump(5);
        assertFalse(RobotContainer.autoShootSupervisorSubsystem.isShootRequested(), "Jam clear must not revive old intent");
        onLoop(actions, shoot::schedule); pump(5);
        assertTrue(RobotContainer.autoShootSupervisorSubsystem.isShootRequested(), "A fresh request can rearm");
        onLoop(actions, shoot::cancel);

        DriverStationSim.setEnabled(false); pump(10);
        DriverStationSim.setJoystickAxis(Constants.OperatorConstants.OIContants.BUTTON_BOX,
            Constants.OperatorConstants.OIContants.BB_PANIC_STOP_AXIS, 1);
        pump(5);
        assertTrue(RobotContainer.isPanicStopActive(), "Panic changes must be observed while disabled");
        DriverStationSim.setEnabled(true); pump(5);
        assertEquals(0, RobotContainer.shooterSubsystem.getTargetRpm(), 1e-9);
        DriverStationSim.setEnabled(false);
        DriverStationSim.setJoystickAxis(Constants.OperatorConstants.OIContants.BUTTON_BOX,
            Constants.OperatorConstants.OIContants.BB_PANIC_STOP_AXIS, 0);
        pump(10);
        DriverStationSim.setAutonomous(true); DriverStationSim.setEnabled(true); pump(15);
        assertNull(error.get(), "Repeated auto enable must not re-compose an already composed command");
        assertTrue(RobotContainer.driveSubsystem.getSimulationTruthPose().getTranslation()
            .getDistance(before.getTranslation()) < .02);
      } finally {
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        robot.endCompetition();
        loop.join(3000);
        assertFalse(loop.isAlive(), "Robot simulation loop did not shut down");
      }
    }
  }
  private static void onLoop(ConcurrentLinkedQueue<Runnable> actions, Runnable action) throws Exception {
    var done = new CountDownLatch(1);
    var failure = new AtomicReference<Throwable>();
    actions.add(() -> {
      try { action.run(); } catch (Throwable ex) { failure.set(ex); } finally { done.countDown(); }
    });
    assertTrue(done.await(3, TimeUnit.SECONDS), "Robot loop did not process the test action");
    if (failure.get() != null) throw new AssertionError(failure.get());
  }
  private static void pump(int loops) throws InterruptedException {
    for (int i = 0; i < loops; i++) { DriverStationSim.notifyNewData(); Thread.sleep(20); }
  }
}
