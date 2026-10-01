package frc.robot.subsystems.vision;
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.config.*;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class PhotonSimulationIntegrationTest {
  @Test void syntheticMultiTagFramesPassThroughProductionDecoder() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    var config = OffseasonVisionConfig.load(Path.of("simulation/vision.json"));
    VisionConstants.configure(config);
    var truth = new Pose2d(3, 3, Rotation2d.kZero);
    var io = new VisionIOPhotonVisionSim("integration-rear", config.cameras().get(0).robotToCamera(), () -> truth);
    var inputs = new VisionIO.VisionIOInputs();
    int decoded = 0;
    double minError = Double.POSITIVE_INFINITY;
    SimHooks.pauseTiming();
    try {
      for (int i = 0; i < 150; i++) {
        SimHooks.stepTiming(.02);
        io.updateInputs(inputs);
        for (var observation : inputs.poseObservations) {
          if (observation.tagCount() >= 2) {
            decoded++;
            minError = Math.min(minError, observation.pose().toPose2d().getTranslation().getDistance(truth.getTranslation()));
            assertEquals(9, inputs.rawFieldToCamera.length);
          }
        }
        Thread.sleep(5); // Let NT transport deliver camera publications.
      }
      assertTrue(decoded >= 3, "Expected multiple production-decoded MultiTag frames, got " + decoded);
      assertTrue(minError < .10, "Synthetic pose error: " + minError);
    } finally {
      io.close();
      SimHooks.resumeTiming();
      VisionConstants.configure(OffseasonVisionConfig.load(Path.of("src/main/deploy/vision/cameras.json")));
    }
  }
}
