package frc.robot.subsystems.vision;
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.config.*;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;

class VisionFusionIntegrationTest {
  @Test void fusionNeedsCalibrationAndFreshUniquePostResetFrames() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    var original = OffseasonVisionConfig.load(Path.of("src/main/deploy/vision/cameras.json"));
    var fixture = OffseasonVisionConfig.load(Path.of("simulation/vision.json"));
    var camera = fixture.cameras().get(0);
    var config = new OffseasonVisionConfig("simulation", fixture.layoutSha256(), fixture.layout(), true, List.of(camera));
    VisionConstants.configure(config);
    var frames = new VisionIO.PoseObservation[1];
    var fused = new ArrayList<Double>();
    double[] reset = {Double.NEGATIVE_INFINITY};
    var vision = new Vision((pose, timestamp, std) -> fused.add(timestamp), () -> new Pose2d(3,3,Rotation2d.kZero),
        () -> reset[0], timestamp -> Optional.of(Rotation2d.kZero), new VisionIO() {
          @Override public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true; inputs.poseObservations = frames.clone();
          }
        });
    try {
      vision.configureCameras(new OffseasonVisionConfig("simulation", fixture.layoutSha256(), fixture.layout(), false, List.of(camera)), () -> true);
      frames[0] = frame(Timer.getFPGATimestamp()); vision.periodic(); assertTrue(fused.isEmpty());
      vision.configureCameras(config, () -> true);
      SimHooks.stepTiming(.02); frames[0] = frame(Timer.getFPGATimestamp()); vision.periodic();
      assertEquals(1, fused.size()); assertTrue(vision.hasRecentMeasurement());
      vision.periodic(); assertEquals(1, fused.size()); // Duplicate cannot count again.
      reset[0] = Timer.getFPGATimestamp(); assertFalse(vision.hasRecentMeasurement());
      SimHooks.stepTiming(.02); frames[0] = frame(Timer.getFPGATimestamp()); vision.periodic();
      assertEquals(1, fused.size()); // Post-reset quarantine.
      SimHooks.stepTiming(.4); frames[0] = frame(Timer.getFPGATimestamp()); vision.periodic();
      assertEquals(2, fused.size());
      SimHooks.stepTiming(.6); assertFalse(vision.hasRecentMeasurement());
      frames[0] = frame(Timer.getFPGATimestamp() + .2); vision.periodic(); assertEquals(2, fused.size());
    } finally {
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(vision);
      VisionConstants.configure(original); SimHooks.resumeTiming();
    }
  }
  static VisionIO.PoseObservation frame(double timestamp) {
    return new VisionIO.PoseObservation(timestamp, new Pose3d(3,3,0,new Rotation3d()), 0, 2, 2, 1);
  }
}
