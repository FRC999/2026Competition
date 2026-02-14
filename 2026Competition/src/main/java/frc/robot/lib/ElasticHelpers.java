package frc.robot.lib;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringArraySubscriber;
import frc.robot.RobotContainer;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;


public class ElasticHelpers {

    private static final Field2d robotOnField = new Field2d();
    private static Pose2d currentPose = new Pose2d();

    private static final Field2d autoDisplayField = new Field2d();
    private static String autoSelected = "";
    private static Trajectory currentTrajectory = new Trajectory();

    // Autos selection is written by Elastic. Options are published by robot code.
    private static final NetworkTable AUTO_NT = NetworkTableInstance.getDefault().getTable("Autos");
    private static final String FIRST_DEST_KEY = "FirstDestination";
    private static final String CHAIN_KEY      = "Chain"; // "FirstDest;Path1;Path2;..."

    // NT subscribers must be created once; do NOT create entries/subscribers in periodic loops.
    private static final StringSubscriber SUB_FIRST_DEST = AUTO_NT.getStringTopic(FIRST_DEST_KEY).subscribe("");
    private static final StringSubscriber SUB_CHAIN      = AUTO_NT.getStringTopic(CHAIN_KEY).subscribe("");
    private static final StringArraySubscriber SUB_SELECTED_SEGMENTS = AUTO_NT.getStringArrayTopic("SelectedSegments").subscribe(new String[0]);


    // Cached to avoid rebuilding preview every loop when nothing changed
    private static String lastAutoChain = "";
    private static String lastFirstDest = "";

    public static String questStatesColors(String state) {
        switch (state) {
            case "INITIALIZE":
                return "#4E5FFF";
            case "SEEKING_TAGS_Q":
                return "#CF4EFF";
            case "SEEKING_TAGS_NO_Q":
                return "#E22222";
            case "CALIBRATED_Q":
                return "#43D567";
            case "CALIBRATED_NO_Q":
                return "#EDFF4E";
            default:
                return "#000000";
        }
    }

    public static String LLAnyVisibleColors(boolean visible) {
        if (visible) {
            return "#43D567";
        }
        return "#E22222";
    }

    public static String LLBestAmbiguityColors(double ambiguity, double maxAmbiguity) {
        if (ambiguity <= maxAmbiguity) {
            return "#43D567";
        }
        return "#EDFF4E";
    }

    public static String getAllianceSide() {
        return DriverStation.getAlliance().map(alliance -> {
            switch (alliance) {
                case Red:
                    return "#FF0000";
                case Blue:
                    return "#0000FF";
                default:
                    return "Invalid";
            }
        }).orElse("Invalid");
    }


    public static void updateAutoSelected() {
        try {
            if (RobotContainer.autoChooser != null && RobotContainer.autoChooser.getSelected() != null) {
                autoSelected = RobotContainer.autoChooser.getSelected().toString();
            } else {
                autoSelected = "";
            }
        } catch (Exception e) {
            autoSelected = "";
        }
    }
    

    public static String getAutoSelectedColor() {
        try {
            updateAutoSelected();
            // System.out.println("Auto Selected: " + autoSelected);
            if (autoSelected.contains("RED")) {
                return "#FF0000";
            } else if (autoSelected.contains("BLUE")) {
                return "#0000FF";
            } else {
                return "#00FF00"; // Green for other selections
            }
        } catch (Exception e) {
            return "Uh Oh oops: " + e;
        }
        
    }

    public static boolean shouldEndGame() { 
        double matchTime = DriverStation.getMatchTime();
        return matchTime <= 30.0;
    }

    public static String shouldEndGameColor() {
        if (shouldEndGame()) {
            return "#FF00d0"; // Red
        } else {
            return "#00FF00"; // Green
        }
    }

    // Returns the global Field2d instance 
    public static Field2d getRobotonfield() {
        return robotOnField;
    }

    // Updates the robot pose displayed on the field 
    public static void updateRobotPose(Pose2d pose) {
        currentPose = pose;
        robotOnField.setRobotPose(pose);
    }

    // Returns the Field2d used for displaying auto trajectories
    public static Field2d getAutoDisplayField() {
        return autoDisplayField;
    }

    // Displays a PathPlanner path on the auto field as a polyline
    public static void setAutoPathSingle(PathPlannerPath path) {
        Pose2d[] poses = path.getAllPathPoints()
            .stream()
            .map(p -> new Pose2d(
                    p.position.getX(),
                    p.position.getY(),
                    new Rotation2d()))
            .toArray(Pose2d[]::new);

        autoDisplayField.getObject("Trajectory").setPoses(poses);
    }

    public static void setAutoPathMultiple(List<PathPlannerPath> paths) {
    System.out.println("Setting auto path with multiple paths, count: " + paths.size());
    List<Pose2d> allPoses = new ArrayList<>();

    for (PathPlannerPath path : paths) {
        path.getAllPathPoints().forEach(p -> 
            allPoses.add(new Pose2d(
                p.position.getX(),
                p.position.getY(),
                new Rotation2d() // heading not needed for drawing the line
            ))
        );
    }

    Pose2d[] poseArray = allPoses.toArray(new Pose2d[0]);
    autoDisplayField.getObject("Trajectory").setPoses(poseArray);
}


    // Returns the most recently stored robot pose 
    public static Pose2d getCurrentPose() {
        return currentPose;
    }

    // Displays a trajectory on the field 
    public static void setTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        autoDisplayField.getObject("Trajectory").setTrajectory(trajectory);
    }

    // Clears any displayed trajectory 
    public static void clearTrajectory() {
        autoDisplayField.getObject("Trajectory").setPoses();
    }

    /**
     * Real-time preview of the stitched auto in the "Auto Field" Field2d.
     *
     * - Reads Autos/FirstDestination and Autos/Chain (written by Elastic)
     * - Draws a line from current robot pose -> first destination
     * - Then appends every selected PathPlanner path's poses
     *
     * Call this from a periodic loop (SmartDashboardSubsystem.periodic is a good spot).
     */
    public static void updateAutoPreviewRealtime() {
        final String firstDest = SUB_FIRST_DEST.get().trim();
        final String[] selected = SUB_SELECTED_SEGMENTS.get();
        final String chainFallback = SUB_CHAIN.get().trim(); // legacy fallback

        // Build a cache key based on firstDest + selected segments (or legacy chain if provided)
        final String selectionKey = firstDest + "|" + java.util.Arrays.toString(selected) + "|" + chainFallback;

        // Nothing selected => clear preview
        if (firstDest.isBlank() && selected.length == 0 && chainFallback.isBlank()) {
            if (!lastAutoChain.isBlank() || !lastFirstDest.isBlank()) {
                autoDisplayField.getObject("Trajectory").setPoses();
            }
            lastAutoChain = "";
            lastFirstDest = "";
            return;
        }

        // If unchanged, skip work
        if (Objects.equals(selectionKey, lastAutoChain) && Objects.equals(firstDest, lastFirstDest)) {
            return;
        }
        lastAutoChain = selectionKey;
        lastFirstDest = firstDest;

        String effectiveFirstDest = firstDest;
        List<String> segments = new ArrayList<>();

        if (selected.length > 0) {
            for (String s : selected) {
                if (s == null) break;
                String t = s.trim();
                if (!t.isEmpty()) segments.add(t);
            }
        } else if (!chainFallback.isBlank()) {
            // Backwards compatibility: parse chain string if SelectedSegments isn't being used.
            String[] parts = chainFallback.split(";", -1);
            if (parts.length >= 1 && !parts[0].trim().isBlank()) {
                effectiveFirstDest = parts[0].trim();
            }
            for (int i = 1; i < parts.length; i++) {
                String p = parts[i].trim();
                if (p.isEmpty()) break;
                segments.add(p);
            }
        }

        if (effectiveFirstDest.isBlank()) {
            autoDisplayField.getObject("Trajectory").setPoses();
            return;
        }

        Pose2d destPoseBlue = lookupFirstDestinationPoseBlue(effectiveFirstDest);
        if (destPoseBlue == null) {
            autoDisplayField.getObject("Trajectory").setPoses();
            return;
        }

        boolean flipNow = shouldFlipNow();
        Pose2d destPose = flipNow ? FlippingUtil.flipFieldPose(destPoseBlue) : destPoseBlue;

        List<Pose2d> preview = new ArrayList<>();

        // On-the-fly segment preview (straight line)
        preview.add(currentPose);
        preview.add(destPose);

        // Append selected PP paths
        for (String pathName : segments) {
            PathPlannerPath path = loadPathPlannerPath(pathName);
            if (path == null) break;

            List<Pose2d> poses = path.getPathPoses();
            if (poses == null || poses.isEmpty()) continue;

            if (flipNow && !pathPreventsFlipping(path)) {
                poses = poses.stream().map(FlippingUtil::flipFieldPose).toList();
            }

            preview.addAll(poses);
        }

        autoDisplayField.getObject("Trajectory").setPoses(preview);
    }

    private static Pose2d lookupFirstDestinationPoseBlue(String name) {
        // Names should match TrajectoryHelper.getAnchors()
        return switch (name) {
            case "BlueOutpost" -> TrajectoryHelper.AutoDesiredPoses.BlueOutpost;
            case "BlueDepot" -> TrajectoryHelper.AutoDesiredPoses.BlueDepot;
            case "BlueTower" -> TrajectoryHelper.AutoDesiredPoses.BlueTower;
            case "BlueNeutralRight" -> TrajectoryHelper.AutoDesiredPoses.BlueNeutralRight;
            case "BlueNeutralLeft" -> TrajectoryHelper.AutoDesiredPoses.BlueNeutralLeft;
            case "BlueNeutralMiddle" -> TrajectoryHelper.AutoDesiredPoses.BlueNeurtralMiddle;
            case "BlueBumpRight" -> TrajectoryHelper.AutoDesiredPoses.BlueBumpRight;
            case "BlueBumpLeft" -> TrajectoryHelper.AutoDesiredPoses.BlueBumpLeft;
            default -> null;
        };
    }

    private static boolean shouldFlipNow() {
        try {
            if (!frc.robot.Constants.PathPlannerConstants.shouldFlipTrajectoryOnRed) return false;
        } catch (Exception ignored) {
            return false;
        }
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    private static PathPlannerPath loadPathPlannerPath(String name) {
        try {
            return PathPlannerPath.fromPathFile(name);
        } catch (Exception ex) {
            DriverStation.reportError("[Elastic] Failed to load PathPlanner path '" + name + "': " + ex.getMessage(), ex.getStackTrace());
            return null;
        }
    }

    /**
     * Some PathPlanner versions expose a public boolean field named "preventFlipping".
     * If it exists, respect it; if not, assume false.
     */
    private static boolean pathPreventsFlipping(PathPlannerPath path) {
        try {
            Field f = path.getClass().getField("preventFlipping");
            if (f.getType() == boolean.class) {
                return f.getBoolean(path);
            }
        } catch (Exception ignored) {
        }
        return false;
    }

}