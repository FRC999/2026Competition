package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.*;

@Tag("robotSmoke")
class RobotStartupSmokeTest {
  @Test @Timeout(45)
  void bootsFullRobotAndReceivesVisionWhileDefaultAutoRemainsStopped() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    assertTrue(RobotBase.isSimulation());
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    var error = new AtomicReference<Throwable>();
    try (var robot = new Robot()) {
      Thread loop = new Thread(() -> {
        try { robot.startCompetition(); } catch (Throwable ex) { error.set(ex); }
      }, "offseason-desktop-smoke");
      loop.start();
      try {
        pump(150);
        assertNull(error.get(), () -> String.valueOf(error.get()));
        assertTrue(SmartDashboard.getBoolean("Vision/ConfigValid", false));
        assertTrue(RobotContainer.vision.hasRecentMeasurement(), "Expected simulated PhotonVision fusion");
        assertTrue(RobotContainer.driveSubsystem.getPose().getTranslation()
            .getDistance(RobotContainer.driveSubsystem.getSimulationTruthPose().getTranslation()) < .20);
        var before = RobotContainer.driveSubsystem.getSimulationTruthPose();
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        pump(65);
        assertNull(error.get(), () -> String.valueOf(error.get()));
        assertTrue(RobotContainer.driveSubsystem.getSimulationTruthPose().getTranslation()
            .getDistance(before.getTranslation()) < .02, "Default auto must not drive");
      } finally {
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        robot.endCompetition();
        loop.join(3000);
        assertFalse(loop.isAlive(), "Robot simulation loop did not shut down");
      }
    }
  }
  private static void pump(int loops) throws InterruptedException {
    for (int i = 0; i < loops; i++) { DriverStationSim.notifyNewData(); Thread.sleep(20); }
  }
}
