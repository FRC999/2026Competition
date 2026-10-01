package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.*;

class StopAtRouteEndTest {
  @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
  @BeforeEach void pause() { SimHooks.pauseTiming(); }
  @AfterEach void resume() { SimHooks.resumeTiming(); }

  @Test void noisyPoseNearRouteEndpointDoesNotGenerateCorrectiveMotion() {
    var plant = new PrecisionControllerIntegrationTest.Plant();
    var stop = new StopAtRouteEnd(plant, new Pose2d(), () -> true);
    stop.initialize();
    for (int i = 0; i < 12 && !stop.isFinished(); i++) {
      plant.pose = new Pose2d(i % 2 == 0 ? .07 : -.08, .02, Rotation2d.fromDegrees(2));
      stop.execute(); plant.step();
      assertEquals(0, plant.request.vxMetersPerSecond);
      assertEquals(0, plant.request.omegaRadiansPerSecond);
    }
    assertTrue(stop.isFinished()); stop.end(false); assertTrue(stop.succeeded());
  }

  @Test void missedEndpointAndLostVisionFailWithoutClaimingArrival() {
    for (boolean vision : new boolean[] {false, true}) {
      var plant = new PrecisionControllerIntegrationTest.Plant();
      plant.pose = vision ? new Pose2d(1, 0, Rotation2d.kZero) : new Pose2d();
      var stop = new StopAtRouteEnd(plant, new Pose2d(), () -> vision);
      stop.initialize();
      for (int i = 0; i < 30 && !stop.isFinished(); i++) { stop.execute(); plant.step(); }
      assertTrue(stop.isFinished()); stop.end(false); assertFalse(stop.succeeded());
    }
  }

  @Test void interruptionAndReschedulingClearPriorSuccess() {
    var plant = new PrecisionControllerIntegrationTest.Plant();
    var stop = new StopAtRouteEnd(plant, new Pose2d(), () -> true);
    stop.initialize();
    for (int i = 0; i < 8; i++) { stop.execute(); plant.step(); }
    stop.end(false); assertTrue(stop.succeeded());
    stop.initialize(); assertFalse(stop.succeeded()); stop.end(true); assertFalse(stop.succeeded());
    assertEquals(0, plant.request.vxMetersPerSecond);
  }
}
