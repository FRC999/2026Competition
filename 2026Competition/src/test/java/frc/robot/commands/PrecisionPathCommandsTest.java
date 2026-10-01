package frc.robot.commands;
import static org.junit.jupiter.api.Assertions.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

class PrecisionPathCommandsTest {
  @Test void repeatAllianceResolutionNeverMutatesCachedSourceOrFlipsTwice() {
    var source = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        new Pose2d(1, 2, Rotation2d.kZero), new Pose2d(3, 2, Rotation2d.kZero)),
        new PathConstraints(2, 2, 3, 3), new IdealStartingState(0, Rotation2d.kZero),
        new GoalEndState(0, Rotation2d.fromDegrees(35)));
    var blue = PrecisionPathCommands.inFieldFrame(source, false, false);
    var red = PrecisionPathCommands.inFieldFrame(source, false, true);
    var redAgain = PrecisionPathCommands.inFieldFrame(source, true, true);
    assertFalse(source.preventFlipping);
    assertEquals(new Pose2d(3, 2, Rotation2d.fromDegrees(35)), PrecisionPathCommands.endpoint(blue));
    assertEquals(PrecisionPathCommands.endpoint(source.flipPath()), PrecisionPathCommands.endpoint(red));
    assertEquals(PrecisionPathCommands.endpoint(red), PrecisionPathCommands.endpoint(redAgain));
    assertEquals(PrecisionPathCommands.endpoint(blue),
        PrecisionPathCommands.endpoint(PrecisionPathCommands.inFieldFrame(source, false, false)));
    assertTrue(red.preventFlipping);
  }
}
