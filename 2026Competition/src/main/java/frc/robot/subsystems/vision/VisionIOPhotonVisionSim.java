package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.config.VisionConstants;

/** Synthetic camera frames traverse the production PhotonVision ingestion path.
 * Camera FOV/noise defaults are simulation assumptions, not measured OV9782 lens properties.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  // A camera owns its world. Static worlds retained old layouts/cameras across reconstruction,
  // and rendering one shared world from each camera's update advanced it more than once per loop.
  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;

  private final Supplier<Pose2d> poseSupplier;

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(requireSimulation(name), robotToCamera);
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim(name);
    visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);

    // Synthetic lens model; replace with measured intrinsics for predictive simulation.
    // Idea: 3467 configures SimCameraProperties to match the real sensor so the sim is honest about
    // resolution, frame rate, latency, and calibration noise instead of assuming a perfect camera.
    var props = new SimCameraProperties();
    props.setCalibration(
        VisionConstants.SIM_CAMERA_WIDTH_PX,
        VisionConstants.SIM_CAMERA_HEIGHT_PX,
        Rotation2d.fromDegrees(VisionConstants.SIM_CAMERA_DIAGONAL_FOV_DEGREES));
    props.setCalibError(
        VisionConstants.SIM_CAMERA_AVG_PX_ERROR, VisionConstants.SIM_CAMERA_PX_ERROR_STD_DEV);
    props.setFPS(VisionConstants.SIM_CAMERA_FPS);
    props.setAvgLatencyMs(VisionConstants.SIM_CAMERA_AVG_LATENCY_MS);
    props.setLatencyStdDevMs(VisionConstants.SIM_CAMERA_LATENCY_STD_DEV_MS);

    cameraSim = new PhotonCameraSim(camera, props, VisionConstants.FIELD_LAYOUT);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  private static String requireSimulation(String name) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation())
      throw new IllegalStateException("Synthetic camera on real robot");
    return name;
  }

  @Override public void close() {
    visionSim.removeCamera(cameraSim);
    cameraSim.close();
    super.close();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Render the world from the current true robot pose, then let the real ingestion code read it.
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
