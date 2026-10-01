package frc.robot.commands;
import static org.junit.jupiter.api.Assertions.*;
import com.pathplanner.lib.path.*;
import frc.robot.commands.PrecisionPathCommands.FieldFrame;
import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

class PrecisionPathCommandsTest {
  @Test void explicitFrameAndPreventFlagMatrixPreservesVelocityHeadingAndCache() {
    for (boolean prevent : new boolean[] {false, true}) {
      var source = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
          new Pose2d(1, 2, Rotation2d.kZero), new Pose2d(3, 2, Rotation2d.kZero)),
          new PathConstraints(2, 2, 3, 3), new IdealStartingState(.7, Rotation2d.fromDegrees(20)),
          new GoalEndState(1.1, Rotation2d.fromDegrees(35)));
      source.preventFlipping = prevent;
      for (boolean red : new boolean[] {false, true}) for (FieldFrame frame : FieldFrame.values()) {
        boolean flipped = frame == FieldFrame.FORCE_RED || (frame == FieldFrame.ALLIANCE && red && !prevent);
        var expected = flipped ? source.flipPath() : source;
        var actual = PrecisionPathCommands.inFieldFrame(source, frame, red);
        assertEquals(PrecisionPathCommands.endpoint(expected), PrecisionPathCommands.endpoint(actual));
        assertEquals(expected.getStartingHolonomicPose(), actual.getStartingHolonomicPose());
        assertEquals(.7, actual.getIdealStartingState().velocityMPS(), 1e-9);
        assertEquals(1.1, actual.getGoalEndState().velocityMPS(), 1e-9);
        assertEquals(source.getEventMarkers().size(), actual.getEventMarkers().size());
        assertTrue(actual.preventFlipping);
        assertEquals(prevent, source.preventFlipping);
      }
    }
  }

  @Test void shippedMultiPathRoutesJoinOnBothAlliances() throws Exception {
    String[][] routes = {
      {"BlueTrenchRight2_BlueNeutralRight", "BlueNeutralRight_BlueNeutralRightMiddle",
       "BlueNeutralRightMiddle_BlueNeutralHubRight", "BlueNeutralHubRight_BlueNearBump",
       "BlueNearBump_BlueTrenchRight", "BlueTrenchRight_BlueTrenchRight2",
       "BlueTrenchRight2_BlueNeutralHubRight", "BlueNeutralHubRight2_BlueNearBump"},
      {"BlueTrenchRight2_BlueNeutralHubRightMore", "BlueNeutralHubRightMore_BlueOffCenter",
       "BlueOffCenter_BlueNearTower", "BlueNearTower_BlueDepotThrough", "BlueDepotThrough_BlueLeftLine"}
    };
    for (boolean red : new boolean[] {false, true}) for (String[] route : routes) {
      Translation2d previous = null;
      for (String name : route) {
        var path = PrecisionPathCommands.inFieldFrame(PathPlannerPath.fromPathFile(name), FieldFrame.ALLIANCE, red);
        Translation2d start = path.getStartingHolonomicPose().orElseThrow().getTranslation();
        if (previous != null) assertTrue(previous.getDistance(start) < .002, name + " has a route gap");
        previous = PrecisionPathCommands.endpoint(path).getTranslation();
      }
    }
  }

  @Test void repeatAllianceResolutionNeverMutatesCachedSourceOrFlipsTwice() {
    var source = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        new Pose2d(1, 2, Rotation2d.kZero), new Pose2d(3, 2, Rotation2d.kZero)),
        new PathConstraints(2, 2, 3, 3), new IdealStartingState(0, Rotation2d.kZero),
        new GoalEndState(0, Rotation2d.fromDegrees(35)));
    var blue = PrecisionPathCommands.inFieldFrame(source, FieldFrame.ALLIANCE, false);
    var red = PrecisionPathCommands.inFieldFrame(source, FieldFrame.ALLIANCE, true);
    var redAgain = PrecisionPathCommands.inFieldFrame(source, FieldFrame.FORCE_RED, true);
    assertFalse(source.preventFlipping);
    assertEquals(new Pose2d(3, 2, Rotation2d.fromDegrees(35)), PrecisionPathCommands.endpoint(blue));
    assertEquals(PrecisionPathCommands.endpoint(source.flipPath()), PrecisionPathCommands.endpoint(red));
    assertEquals(PrecisionPathCommands.endpoint(red), PrecisionPathCommands.endpoint(redAgain));
    assertEquals(PrecisionPathCommands.endpoint(blue),
        PrecisionPathCommands.endpoint(PrecisionPathCommands.inFieldFrame(source, FieldFrame.ALLIANCE, false)));
    assertTrue(red.preventFlipping);
  }
}
