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
  public enum FieldFrame { ALLIANCE, FORCE_RED, ABSOLUTE }
  public enum FinishPolicy { PRECISION_ALIGNMENT, ROUTE_STOP }

  public static PathPlannerPath inFieldFrame(PathPlannerPath source, FieldFrame frame, boolean redAlliance) {
    // fromPathFile caches objects. Never mutate that shared source or the next alliance can inherit
    // preventFlipping=true from an earlier schedule.
    PathPlannerPath resolved = new PathPlannerPath(source.getWaypoints(), source.getRotationTargets(),
        source.getPointTowardsZones(), source.getConstraintZones(), source.getEventMarkers(),
        source.getGlobalConstraints(), source.getIdealStartingState(), source.getGoalEndState(), source.isReversed());
    if (frame == FieldFrame.FORCE_RED
        || (frame == FieldFrame.ALLIANCE && !source.preventFlipping && redAlliance)) {
      resolved = resolved.flipPath();
    }
    // AutoBuilder must not flip a path whose coordinates have already been resolved.
    resolved.preventFlipping = true;
    resolved.name = source.name;
    return resolved;
  }

  public static Pose2d endpoint(PathPlannerPath path) {
    var points = path.getAllPathPoints();
    if (points.isEmpty()) throw new IllegalArgumentException("Path has no points");
    return new Pose2d(points.get(points.size() - 1).position, path.getGoalEndState().rotation());
  }

  public static Command followResolved(DriveSubsystem drive, PathPlannerPath path, boolean resetToStart,
      BooleanSupplier recentVision) {
    return followResolved(drive, path, resetToStart, recentVision, FinishPolicy.PRECISION_ALIGNMENT);
  }

  public static Command followResolved(DriveSubsystem drive, PathPlannerPath path, boolean resetToStart,
      BooleanSupplier recentVision, FinishPolicy policy) {
    Command coarse = AutoBuilder.followPath(path).finallyDo(interrupted -> {
      // PathPlanner intentionally retains its last request on interruption for alignment handoffs.
      // Our wrapper can be canceled independently, so it must release that request immediately.
      if (interrupted) drive.stop();
    });
    if (resetToStart) {
      Pose2d start = path.getStartingHolonomicPose().orElseThrow(
          () -> new IllegalArgumentException("Path has no explicit starting holonomic pose"));
      coarse = Commands.runOnce(() -> drive.resetKnownFieldPose(start), drive).andThen(coarse);
    }
    // Retain the complete planned route and all event markers. Spatial early handoffs are explicit
    // opt-ins using DriveToPosePrecisionCommand.handoffFrom on a separately validated final corridor.
    Command movement = Math.abs(path.getGoalEndState().velocityMPS()) > 1e-3 ? coarse
        : coarse.andThen(finishAt(drive, endpoint(path), recentVision, policy));
    return Commands.either(movement,
        failedHold(drive, "Path start requires an established field pose and fresh vision"),
        () -> resetToStart || (drive.hasFieldReference() && recentVision.getAsBoolean()));
  }

  /** A routine with absolute alliance-specific waypoints cannot run on the opposite alliance. */
  public static Command requireAlliance(DriveSubsystem drive, DriverStation.Alliance expected) {
    return Commands.either(Commands.none(), failedHold(drive,
        "Autonomous selection requires " + expected + " alliance"),
        () -> DriverStation.getAlliance().filter(expected::equals).isPresent());
  }

  public static Command finishAt(DriveSubsystem drive, Pose2d target, BooleanSupplier recentVision) {
    return finishAt(drive, target, recentVision, FinishPolicy.PRECISION_ALIGNMENT);
  }

  public static Command finishAt(DriveSubsystem drive, Pose2d target, BooleanSupplier recentVision,
      FinishPolicy policy) {
    if (policy == FinishPolicy.ROUTE_STOP) {
      var stop = new StopAtRouteEnd(drive, target, recentVision);
      return stop.andThen(Commands.either(Commands.none(),
          failedHold(drive, "Route stop outside acceptance or still moving; autonomous held"), stop::succeeded));
    }
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
