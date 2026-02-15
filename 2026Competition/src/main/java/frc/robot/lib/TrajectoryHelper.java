package frc.robot.lib;

import java.util.ArrayList;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.PathPlannerConstants;
import frc.robot.RobotContainer;

public class TrajectoryHelper {

    public static final double FIELD_LENGTH_BLUE = 8.2296; // 27 feet

    public TrajectoryHelper() {
    }

    /**
     * When running a trajectory, PP flips it automatically (unless disabled by a
     * flag)
     * around field center. To recalibrate Quest to the flipped trajectory, this
     * method
     * may be used to flip the initial pose of the trajectory.
     * This is only needed if odometry reset to the starting pose is needed (e.g. if
     * cameras
     * could not calibrate quest at the beginning of the game for some reason, so we
     * have to assume starting pose of the bot)
     * 
     * @param pose
     * @return pose flipped around center of the field
     */
    public static Pose2d flipQuestPoseRed(Pose2d pose) {
        return (PathPlannerConstants.shouldFlipTrajectoryOnRed) ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static final class AutoDesiredPoses {
        public static final Pose2d BlueOutpost = new Pose2d(0.50, 0.65, new Rotation2d(Math.toRadians(90)));
        public static final Pose2d BlueDepot = new Pose2d(0.55, 5.95, new Rotation2d(0));
        public static final Pose2d BlueTower = new Pose2d(1.425, 3.75, new Rotation2d(0));

        public static final Pose2d BlueNeutralRight = new Pose2d(7.8, 1.1, new Rotation2d(90));
        public static final Pose2d BlueNeutralLeft = new Pose2d(7.9, 5.9, new Rotation2d());
        public static final Pose2d BlueNeurtralMiddle = new Pose2d(7.85, 4.02, new Rotation2d(0));

        public static final Pose2d BlueBumpRight = new Pose2d(4.466, 2.372, new Rotation2d());
        public static final Pose2d BlueBumpLeft = new Pose2d(4.466, 5.568, new Rotation2d());
    }

    /*
     * =========================================================
     * Auto stitching for Elastic (pose-based, no filename parsing)
     *
     * Driver workflow:
     * 1) Driver selects FirstDestination (one of AutoDesiredPoses anchors)
     * 2) Robot runs on-the-fly trajectory (runTrajectory2Poses) to that destination
     * 3) From that destination, robot publishes compatible next PP paths
     * 4) Driver builds chain: "FirstDest;Path1;Path2;Path3"
     * =========================================================
     */

    private static final NetworkTable AUTO_NT = NetworkTableInstance.getDefault().getTable("Autos");

    // NT subscribers/publishers must be created once; do NOT create entries/publishers in periodic loops.
    private static final StringSubscriber SUB_FIRST_DEST = AUTO_NT.getStringTopic("FirstDestination").subscribe("");
    private static final StringSubscriber SUB_CHAIN = AUTO_NT.getStringTopic("Chain").subscribe(""); // legacy
    private static final edu.wpi.first.networktables.StringArraySubscriber SUB_SELECTED_SEGMENTS = AUTO_NT.getStringArrayTopic("SelectedSegments").subscribe(new String[0]);

    private static final StringArrayPublisher PUB_FIRST_DEST_OPTIONS = AUTO_NT.getStringArrayTopic("FirstDestinationOptions").publish();
    private static final StringArrayPublisher PUB_NEXT_OPTIONS = AUTO_NT.getStringArrayTopic("NextOptions").publish();


    // Tuning: how close a path start/end must be to an anchor pose to be considered
    // “connected”
    private static final double SNAP_POS_TOL_M = 0.60;
    private static final double SNAP_ROT_TOL_DEG = 35.0;

    private record Anchor(String name, Pose2d bluePose) {
    }

    private record Segment(String pathName, String startAnchor, String endAnchor) {
    }

    private static boolean sInit = false;
    private static String sLastChain = "";

    // startAnchor -> list of paths that start there
    private static final Map<String, List<Segment>> sByStart = new HashMap<>();
    private static final Map<String, PathPlannerPath> sPathByName = new HashMap<>();

    /** Call in disabledPeriodic to keep Elastic dropdowns updated */
    public static void autoStitchUpdate() {
        initAutoStitchIfNeeded();

        final String firstDest = SUB_FIRST_DEST.get().trim();
        final String[] selected = SUB_SELECTED_SEGMENTS.get();

        // cache key to avoid recompute spam
        final String cacheKey = firstDest + "|" + java.util.Arrays.toString(selected);
        if (cacheKey.equals(sLastChain)) {
            if (!firstDest.isBlank() && selected.length == 0) {
                publishNextOptionsFromAnchor(firstDest);
            }
            return;
        }
        sLastChain = cacheKey;

        if (firstDest.isBlank()) {
            publishNextOptions(List.of());
            return;
        }

        String currentAnchor = firstDest;

        for (String pathNameRaw : selected) {
            if (pathNameRaw == null) break;
            String pathName = pathNameRaw.trim();
            if (pathName.isBlank()) break;

            Segment seg = findSegmentByNameFromAnchor(currentAnchor, pathName);
            if (seg == null) break;

            currentAnchor = seg.endAnchor;
        }

        publishNextOptionsFromAnchor(currentAnchor);
    }

    /**
/**
     * Build the actual auto:
     * 1) on-the-fly segment to first destination (drivers can place robot anywhere)
     * 2) follow chosen PP segments in order
     */
    public static Command buildStitchedAutoCommand() {
        initAutoStitchIfNeeded();

        final String firstDest = SUB_FIRST_DEST.get().trim();
        final String[] selected = SUB_SELECTED_SEGMENTS.get();

        if (firstDest.isBlank()) {
            return Commands.none();
        }

        Anchor destAnchor = getAnchors().stream()
                .filter(a -> a.name().equals(firstDest))
                .findFirst()
                .orElse(null);

        if (destAnchor == null) {
            return Commands.none();
        }

        Pose2d destPose = anchorPoseForAlliance(destAnchor.bluePose);

        // Validate selected path list by walking the graph
        String currentAnchor = firstDest;
        List<String> selectedPaths = new java.util.ArrayList<>();

        for (String pathNameRaw : selected) {
            if (pathNameRaw == null) break;
            String pathName = pathNameRaw.trim();
            if (pathName.isBlank()) break;

            Segment seg = findSegmentByNameFromAnchor(currentAnchor, pathName);
            if (seg == null) break;

            selectedPaths.add(seg.pathName);
            currentAnchor = seg.endAnchor;
        }

        // FIRST SEGMENT: on-the-fly path from *current pose* to the chosen first destination
        Command toFirstDest = Commands.defer(
                () -> RobotContainer.runTrajectory2Poses(
                        false,
                        RobotContainer.driveSubsystem.getPose(),
                        destPose),
                java.util.Set.of(RobotContainer.driveSubsystem));

        // FOLLOW stitched PP segments
        List<Command> follow = selectedPaths.stream()
                .map(sPathByName::get)
                .filter(Objects::nonNull)
                .map(AutoBuilder::followPath)
                .collect(Collectors.toList());

        return Commands.sequence(
                toFirstDest,
                Commands.sequence(follow.toArray(Command[]::new)))
                .withName("StitchedAuto");
    }

// ---------------- internal ----------------

    private static void initAutoStitchIfNeeded() {
        if (sInit)
            return;
        sInit = true;

        List<Anchor> anchors = getAnchors();
        // Publish the first destination dropdown options (these are the anchors)
        PUB_FIRST_DEST_OPTIONS.set(anchors.stream().map(Anchor::name).sorted().toArray(String[]::new));

        buildGraphFromPaths(anchors);

        // initial publish
        autoStitchUpdate();
    }

    private static List<Anchor> getAnchors() {
        // These are YOUR preset poses already defined in this file.
        return List.of(
                new Anchor("BlueOutpost", AutoDesiredPoses.BlueOutpost),
                new Anchor("BlueDepot", AutoDesiredPoses.BlueDepot),
                new Anchor("BlueTower", AutoDesiredPoses.BlueTower),
                new Anchor("BlueNeutralRight", AutoDesiredPoses.BlueNeutralRight),
                new Anchor("BlueNeutralLeft", AutoDesiredPoses.BlueNeutralLeft),
                new Anchor("BlueNeutralMiddle", AutoDesiredPoses.BlueNeurtralMiddle),
                new Anchor("BlueBumpRight", AutoDesiredPoses.BlueBumpRight),
                new Anchor("BlueBumpLeft", AutoDesiredPoses.BlueBumpLeft));
    }

    private static void buildGraphFromPaths(List<Anchor> anchors) {
        sByStart.clear();
        sPathByName.clear();

        File dir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        File[] files = dir.listFiles((d, name) -> name.endsWith(".path"));
        if (files == null)
            return;

        boolean flip = shouldFlipNow();

        for (File f : files) {
            String pathName = f.getName().replace(".path", "");

            final PathPlannerPath path;
            try {
                path = PathPlannerPath.fromPathFile(pathName);
            } catch (Exception ex) {
                DriverStation.reportError(
                        "[AutoStitch] Failed to load PathPlanner path '" + pathName + "': " + ex.getMessage(),
                        ex.getStackTrace());
                continue;
            }
            sPathByName.put(pathName, path);

            Pose2d start = path.getStartingHolonomicPose().orElseGet(path::getStartingDifferentialPose);

            var poses = path.getPathPoses();
            if (poses.isEmpty()) {
                DriverStation.reportWarning("[AutoStitch] Path '" + pathName + "' has no poses; skipping", false);
                continue;
            }
            Pose2d end = poses.get(poses.size() - 1);

            if (flip) {
                start = FlippingUtil.flipFieldPose(start);
                end = FlippingUtil.flipFieldPose(end);
            }

            String startAnchor = snapToAnchor(anchors, start, flip);
            String endAnchor = snapToAnchor(anchors, end, flip);
            if (startAnchor == null || endAnchor == null)
                continue;

            Segment seg = new Segment(pathName, startAnchor, endAnchor);
            sByStart.computeIfAbsent(startAnchor, k -> new ArrayList<>()).add(seg);
        }

        // stable ordering
        for (var list : sByStart.values()) {
            list.sort(java.util.Comparator.comparing(s -> s.pathName));
        }
    }

    private static boolean shouldFlipNow() {
        if (!PathPlannerConstants.shouldFlipTrajectoryOnRed)
            return false;
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    private static Pose2d anchorPoseForAlliance(Pose2d bluePose) {
        return shouldFlipNow() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    private static String snapToAnchor(List<Anchor> anchors, Pose2d pose, boolean flip) {
        Anchor best = null;
        double bestDist = Double.POSITIVE_INFINITY;

        for (Anchor a : anchors) {
            Pose2d ap = flip ? FlippingUtil.flipFieldPose(a.bluePose) : a.bluePose;
            double dist = pose.getTranslation().getDistance(ap.getTranslation());
            if (dist < bestDist) {
                bestDist = dist;
                best = a;
            }
        }

        if (best == null || bestDist > SNAP_POS_TOL_M)
            return null;

        Pose2d bestPose = flip ? FlippingUtil.flipFieldPose(best.bluePose) : best.bluePose;
        double rotErr = Math.abs(pose.getRotation().minus(bestPose.getRotation()).getDegrees());
        if (rotErr > SNAP_ROT_TOL_DEG)
            return null;

        return best.name;
    }

    private static Segment findSegmentByNameFromAnchor(String anchor, String pathName) {
        List<Segment> opts = sByStart.getOrDefault(anchor, List.of());
        for (Segment s : opts) {
            if (s.pathName.equals(pathName))
                return s;
        }
        return null;
    }

    private static void publishNextOptionsFromAnchor(String anchor) {
        List<Segment> opts = sByStart.getOrDefault(anchor, List.of());
        publishNextOptions(opts.stream().map(s -> s.pathName).toList());
    }

    private static void publishNextOptions(List<String> options) {
        PUB_NEXT_OPTIONS.set(options.toArray(String[]::new));
    }

}
