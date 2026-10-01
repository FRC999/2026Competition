package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.FieldRules;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class AutoRouteAuditTest {
  static final String[] MAIN = {"BlueTrenchRight2_BlueNeutralRight", "BlueNeutralRight_BlueNeutralRightMiddle",
      "BlueNeutralRightMiddle_BlueNeutralHubRight", "BlueNeutralHubRight_BlueNearBump",
      "BlueNearBump_BlueTrenchRight", "BlueTrenchRight_BlueTrenchRight2",
      "BlueTrenchRight2_BlueNeutralHubRight", "BlueNeutralHubRight2_BlueNearBump"};
  static final String[] WORLDS = {"BlueTrenchRight2_BlueNeutralHubRightMore", "BlueNeutralHubRightMore_BlueOffCenter",
      "BlueOffCenter_BlueNearTower", "BlueNearTower_BlueDepotThrough", "BlueDepotThrough_BlueLeftLine"};

  @Test void auditNominalTrajectoryBudgetsAndContinuousRouteStates() throws Exception {
    var config = RobotConfig.fromGUISettings();
    var report = new StringBuilder("# Nominal PathPlanner route audit\n\n")
        .append("Generated from the checked-in paths and robot config. Excludes starting approach, intake waits, shooting and stop qualification; not physical timing.\n\n")
        .append("| Path | Nominal seconds | End m/s | Max blue X m |\n|---|---:|---:|---:|\n");
    List<String> names = new ArrayList<>(List.of(MAIN)); names.addAll(List.of(WORLDS));
    names.addAll(List.of("BlueMiddle_BlueOutpost", "BlueTrenchRight_BlueOutpost", "BlueHubMiddle_BlueAllianceMiddle"));
    for (String name : names) {
      var path = PathPlannerPath.fromPathFile(name);
      var trajectory = path.getIdealTrajectory(config).orElseThrow();
      assertTrue(Double.isFinite(trajectory.getTotalTimeSeconds()) && trajectory.getTotalTimeSeconds() > 0, name);
      double maxX = path.getAllPathPoints().stream().mapToDouble(p -> p.position.getX()).max().orElseThrow();
      report.append(String.format(java.util.Locale.ROOT, "| %s | %.3f | %.2f | %.3f |%n",
          name, trajectory.getTotalTimeSeconds(), path.getGoalEndState().velocityMPS(), maxX));
      for (boolean red : new boolean[] {false, true}) {
        var resolved = PrecisionPathCommands.inFieldFrame(path, PrecisionPathCommands.FieldFrame.ALLIANCE, red);
        for (var point : resolved.getAllPathPoints()) assertTrue(FieldRules.onOwnAutoHalf(
            new Pose2d(point.position, Rotation2d.kZero), red), name + " crosses G403 conservative center limit");
      }
    }
    for (var route : List.of(MAIN, WORLDS)) {
      PathPlannerPath previous = null;
      double total = 0;
      for (String name : route) {
        var path = PathPlannerPath.fromPathFile(name);
        total += path.getIdealTrajectory(config).orElseThrow().getTotalTimeSeconds();
        if (previous != null) {
          assertEquals(previous.getGoalEndState().velocityMPS(), path.getIdealStartingState().velocityMPS(), .001, name + " speed jump");
          assertTrue(Math.abs(previous.getGoalEndState().rotation().minus(path.getIdealStartingState().rotation()).getDegrees()) < .01,
              name + " holonomic heading jump");
          if (path.getIdealStartingState().velocityMPS() > .01) {
            var prevWaypoints = previous.getWaypoints();
            var last = prevWaypoints.get(prevWaypoints.size()-1);
            var first = path.getWaypoints().get(0);
            Translation2d incoming = last.anchor().minus(last.prevControl());
            Translation2d outgoing = first.nextControl().minus(first.anchor());
            assertTrue(Math.abs(incoming.getAngle().minus(outgoing.getAngle()).getDegrees()) < 2,
                name + " instantaneous velocity direction change");
          }
        }
        previous = path;
      }
      report.append(String.format(java.util.Locale.ROOT, "%n%s path time: %.3f s%n", route == MAIN ? "Main" : "Worlds Blue", total));
    }
    Path output = Path.of("build/reports/auto-route-audit.md");
    Files.createDirectories(output.getParent()); Files.writeString(output, report);
  }
}
