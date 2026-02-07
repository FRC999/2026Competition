package frc.robot.lib;

import java.util.ArrayList;

import javax.xml.crypto.dsig.spec.ExcC14NParameterSpec;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PathPlannerConstants;

public class TrajectoryHelper {

    public static final double FIELD_LENGTH_BLUE = 8.2296; // 27 feet

    public TrajectoryHelper() {}

    /**
     * When running a trajectory, PP flips it automatically (unless disabled by a flag)
     * around field center. To recalibrate Quest to the flipped trajectory, this method
     * may be used to flip the initial pose of the trajectory.
     * This is only needed if odometry reset to the starting pose is needed (e.g. if cameras
     * could not calibrate quest at the beginning of the game for some reason, so we
     * have to assume starting pose of the bot)
     * @param pose
     * @return pose flipped around center of the field
     */
    public static Pose2d flipQuestPoseRed(Pose2d pose) {
        return (PathPlannerConstants.shouldFlipTrajectoryOnRed) ? 
            FlippingUtil.flipFieldPose(pose) :
            pose;
    }

    public static final class AutoDesiredPoses {
        public static final Pose2d BlueOutpost = new Pose2d(0.50, 0.65, new Rotation2d(90));
        public static final Pose2d BlueDepot = new Pose2d(0.55, 5.95, new Rotation2d(0));
        public static final Pose2d BlueNeutralRight = new Pose2d(7.85, 2.6, new Rotation2d(0));
        public static final Pose2d BlueNeutralLeft = new Pose2d(7.85, 5.95, new Rotation2d(0));
        public static final Pose2d BlueNeurtralMiddle = new Pose2d(7.85, 4.02, new Rotation2d(0));
        public static final Pose2d BlueBumpRight = new Pose2d(4.466, 2.372, new Rotation2d());
        public static final Pose2d BlueBumpLeft = new Pose2d(4.466, 5.568, new Rotation2d());
        public static final Pose2d BlueTower = new Pose2d(1.425, 3.75, new Rotation2d(0));
    }
}
