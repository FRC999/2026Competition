package frc.robot.lib;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotContainer;

/** Shared dashboard displays; all values are read-only views of control state. */
public final class ElasticHelpers {
  private static final Field2d robotOnField = new Field2d();
  private static final Field2d autoDisplayField = new Field2d();
  private ElasticHelpers() {}
  public static String getAllianceSide() {
    return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? "#FF0000" : "#0000FF")
        .orElse("#777777");
  }
  public static String getAutoSelectedColor() {
    var selected = RobotContainer.autoChooser.getSelected();
    String name = selected == null ? "" : selected.getName().toUpperCase(java.util.Locale.ROOT);
    return name.contains("RED") ? "#FF0000" : name.contains("BLUE") ? "#0000FF" : "#00FF00";
  }
  public static String shouldEndGameColor() {
    double remaining = DriverStation.getMatchTime();
    return DriverStation.isTeleopEnabled() && remaining >= 0 && remaining <= 30 ? "#FF00D0" : "#00FF00";
  }
  public static Field2d getRobotonfield() { return robotOnField; }
  public static Field2d getAutoDisplayField() { return autoDisplayField; }
  public static void updateRobotPose(Pose2d pose) { robotOnField.setRobotPose(pose); }
  public static void setAutoPathSingle(PathPlannerPath path) {
    autoDisplayField.getObject("Trajectory").setPoses(path.getAllPathPoints().stream()
        .map(p -> new Pose2d(p.position, Rotation2d.kZero)).toArray(Pose2d[]::new));
  }
}
