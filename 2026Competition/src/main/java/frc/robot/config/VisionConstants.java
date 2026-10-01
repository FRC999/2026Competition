package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Algorithm defaults ported from d20594a; per-camera measurements are deliberately NOT ported. */
public final class VisionConstants {
  private VisionConstants() {}
  public static AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
  public static double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();
  public static double[] CAMERA_STD_DEV_FACTORS = {1, 1, 1, 1};
  public static double[] CAMERA_ANGULAR_STD_DEV_FACTORS = {1, 1, 1, 1};
  public static boolean[] CAMERA_ROTATION_TRUST_ENABLED = {true, true, true, true};
  public static Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
      new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d()};
  public static final double MAX_FRAME_AGE_SECONDS = 0.5;
  public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.02;
  public static final double MIN_LINEAR_STD_DEV_METERS = 0.02;
  public static final double MIN_ANGULAR_STD_DEV_RADIANS = Math.toRadians(2.0);

  /** Called once before constructing the vision subsystem; no enabled field switching. */
  public static void configure(OffseasonVisionConfig config) {
    FIELD_LAYOUT = config.layout();
    FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
    FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();
    com.pathplanner.lib.util.FlippingUtil.fieldSizeX = FIELD_LENGTH_METERS;
    com.pathplanner.lib.util.FlippingUtil.fieldSizeY = FIELD_WIDTH_METERS;
    com.pathplanner.lib.util.FlippingUtil.symmetryType = com.pathplanner.lib.util.FlippingUtil.FieldSymmetry.kRotational;
    var cameras = config.cameras();
    CAMERA_STD_DEV_FACTORS = cameras.stream().mapToDouble(OffseasonVisionConfig.Camera::xyStdDevFactor).toArray();
    CAMERA_ANGULAR_STD_DEV_FACTORS = cameras.stream().mapToDouble(OffseasonVisionConfig.Camera::thetaStdDevFactor).toArray();
    ROBOT_TO_CAMERA_TRANSFORMS = cameras.stream().map(OffseasonVisionConfig.Camera::robotToCamera).toArray(Transform3d[]::new);
    CAMERA_ROTATION_TRUST_ENABLED = new boolean[cameras.size()];
    for (int i=0; i<cameras.size(); i++) CAMERA_ROTATION_TRUST_ENABLED[i] = cameras.get(i).trustRotation();
  }
  public static final double MAX_ACCEPTED_Z_METERS = 0.25;
  public static final double FIELD_BORDER_MARGIN_METERS = 0.50;
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
  public static final double MAX_AVERAGE_TAG_DISTANCE_METERS = 5.0;
  public static final double LINEAR_STD_DEV_BASELINE = 0.06;
  public static final double ANGULAR_STD_DEV_BASELINE = 0.08;
  public static final boolean FUSE_VISION_ROTATION_WHILE_ENABLED = false;
  public static final int CAMERA_JITTER_CAPTURE_SAMPLES = 100;
  public static final double ANISO_SINGLE_TAG_PARALLEL_COEFF = 0.060;
  public static final double ANISO_SINGLE_TAG_PARALLEL_EXP = 2.0;
  public static final double ANISO_SINGLE_TAG_PERP_COEFF = 0.030;
  public static final double ANISO_SINGLE_TAG_PERP_EXP = 2.0;
  public static final double ANISO_MULTI_TAG_PARALLEL_COEFF = 0.015;
  public static final double ANISO_MULTI_TAG_PARALLEL_EXP = 2.0;
  public static final double ANISO_MULTI_TAG_PERP_COEFF = 0.0075;
  public static final double ANISO_MULTI_TAG_PERP_EXP = 2.0;
  public static final double AUTO_VISION_IGNORE_SECONDS = 0.3;
  public static final double TARGET_OBSERVATION_MAX_STALENESS_SECONDS = 0.25;
  public static final double VISION_SEED_MAX_STALENESS_SECONDS = 0.25;
  public static final double RESET_QUARANTINE_SECONDS = 0.35;
  public static final int SIM_CAMERA_WIDTH_PX = 1280;
  public static final int SIM_CAMERA_HEIGHT_PX = 800;
  public static final double SIM_CAMERA_DIAGONAL_FOV_DEGREES = 84.0;
  public static final double SIM_CAMERA_AVG_PX_ERROR = 0.25;
  public static final double SIM_CAMERA_PX_ERROR_STD_DEV = 0.08;
  public static final double SIM_CAMERA_FPS = 50.0;
  public static final double SIM_CAMERA_AVG_LATENCY_MS = 30.0;
  public static final double SIM_CAMERA_LATENCY_STD_DEV_MS = 8.0;
}
