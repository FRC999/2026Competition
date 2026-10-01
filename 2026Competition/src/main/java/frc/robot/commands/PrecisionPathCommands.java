package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Resolve the field frame once, then finish stopping paths using measured pose and motion. */
public final class PrecisionPathCommands {
  private PrecisionPathCommands() {}

  public static PathPlannerPath inFieldFrame(PathPlannerPath source, boolean forceRedFlip, boolean redAlliance) {
    // fromPathFile caches objects. Never mutate that shared source or the next alliance can inherit
    // preventFlipping=true from an earlier schedule.
    PathPlannerPath resolved = new PathPlannerPath(source.getWaypoints(), source.getRotationTargets(),
        source.getPointTowardsZones(), source.getConstraintZones(), source.getEventMarkers(),
        source.getGlobalConstraints(), source.getIdealStartingState(), source.getGoalEndState(), source.isReversed());
    if (forceRedFlip || (!source.preventFlipping && redAlliance)) resolved = resolved.flipPath();
    // AutoBuilder must not flip a path whose coordinates have already been resolved.
    resolved.preventFlipping = true;
    return resolved;
  }

  public static Pose2d endpoint(PathPlannerPath path) {
    var points = path.getAllPathPoints();
    if (points.isEmpty()) throw new IllegalArgumentException("Path has no points");
    return new Pose2d(points.get(points.size() - 1).position, path.getGoalEndState().rotation());
  }

  public static Command followResolved(DriveSubsystem drive, PathPlannerPath path, boolean resetToStart,
      BooleanSupplier recentVision) {
    Command coarse = AutoBuilder.followPath(path);
    if (resetToStart) {
      Pose2d start = path.getStartingHolonomicPose().orElseThrow(
          () -> new IllegalArgumentException("Path has no explicit starting holonomic pose"));
      coarse = Commands.runOnce(() -> drive.resetCTREPose(start), drive).andThen(coarse);
    }
    if (Math.abs(path.getGoalEndState().velocityMPS()) > 1e-3) return coarse;
    // Retain the complete planned route and all event markers. Spatial early handoffs are explicit
    // opt-ins using DriveToPosePrecisionCommand.handoffFrom on a separately validated final corridor.
    return coarse.andThen(finishAt(drive, endpoint(path), recentVision));
  }

  public static Command finishAt(DriveSubsystem drive, Pose2d target, BooleanSupplier recentVision) {
    var precise = new DriveToPosePrecisionCommand(drive, target).withFinishPermission(recentVision);
    return precise.andThen(Commands.either(Commands.none(),
        failedHold(drive, "Precision endpoint did not qualify; autonomous sequence held"), precise::succeeded));
  }

  public static Command failedHold(DriveSubsystem drive, String reason) {
    return Commands.runOnce(() -> {
      drive.latchAutonomousPrecisionFailure();
      DriverStation.reportError(reason, false);
      Logger.recordOutput("DriveToPose/Failure", reason);
    }).andThen(Commands.run(drive::stop, drive));
  }
}
