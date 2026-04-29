package frc.robot.OdometryUpdates;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class LLAprilTagConstants {
    public static final class LLVisionConstants {

    // public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltAndymark; // Field Layout - changes year-to-year
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded; // Field Layout - changes year-to-year
	
	/** Seed LL4 internal IMU from robot yaw while waiting for first reliable field anchor. */
	public static final int LL_IMU_MODE_SEED = 1;
	/** Use LL4 internal IMU only for MT2 after initial anchoring. */
	public static final int LL_IMU_MODE_TRACKING_INTERNAL = 2;
	/** Allow MT1 to gently correct LL4 internal IMU, but only while reliable multi-tag MT1 is visible. */
	public static final int LL_IMU_MODE_TRACKING_MT1_ASSIST = 3;
	public static final double LL_IMU_ASSIST_ALPHA = 0.001;
		
	public static enum LLCamera {

			LLMIDDLE(
				"limelight-middle"
			),

			LLRIGHT(
				"limelight-right"
			);
			
			private String cameraname;
			private boolean prevCleared; // Set to true if nothing was seen last time
			public boolean isPrevCleared() {
				return prevCleared;
			}
			public void setPrevCleared(boolean prevCleared) {
				this.prevCleared = prevCleared;
			}
			LLCamera(String cn) {
				this.cameraname = cn;
				this.prevCleared = false;
			}
			public String getCameraName() {
				return cameraname;
			}
			
		}

        public static final double kMaxSingleTagAmbiguity = 0.20; // Maximum ambiguity when seeing a single AprilTag
        public static final double kMaxCameraToTargetDistance = 3.0; // Maximum distance from camera to AprilTag during normal fusion
        public static final double kMaxInitialSeedCameraToTargetDistance = 4.0; // Allow a longer range for the first field anchor
	}

    public static final class VisionHelperConstants {
		public static final double distanceBetweenReefPoles = Units.inchesToMeters(12.5); // page 162 https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
		public static final double bumperWidth = Units.inchesToMeters(2.5);
		public static class RobotPoseConstants {
			public static Map<String, Pose2d> visionRobotPoses = new HashMap<String, Pose2d>();
			public static Map<Integer, String> tagNumberToKey = new HashMap<Integer, String>();
			public static Map<Pose2d, Integer> reefTagPoses = new HashMap<Pose2d, Integer>();
			public static Map<Pose2d, Integer> redReefTagPoses = new HashMap<Pose2d, Integer>();
			public static Map<Pose2d, Integer> blueReefTagPoses = new HashMap<Pose2d, Integer>();
		}
	}
}
