package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.VisionConstants;
import org.junit.jupiter.api.Test;

class TrenchGuardTest {
  @Test void coversFullStructureAndApproachOnBothEnds() {
    assertTrue(FieldTargeting.inTrench(new Pose2d(4.5, 1.2, Rotation2d.kZero)), "Old narrow strip missed this");
    for (boolean red : new boolean[] {false, true}) {
      double x = red ? VisionConstants.FIELD_LAYOUT.getFieldLength() - 3.3 : 3.3;
      double y = red ? VisionConstants.FIELD_LAYOUT.getFieldWidth() - .7 : .7;
      Pose2d pose = new Pose2d(x, y, Rotation2d.kZero);
      assertFalse(FieldTargeting.trenchInhibit(pose, new ChassisSpeeds()));
      assertTrue(FieldTargeting.trenchInhibit(pose, new ChassisSpeeds(red ? -2 : 2, 0, 0)));
      assertFalse(FieldTargeting.trenchInhibit(pose, new ChassisSpeeds(red ? 2 : -2, 0, 0)));
    }
  }
  @Test void fastCrossingCannotSkipRegionAndClearCenterRemainsAvailable() {
    assertTrue(FieldTargeting.trenchInhibit(new Pose2d(3, .7, Rotation2d.kZero), new ChassisSpeeds(10, 0, 0)));
    assertFalse(FieldTargeting.trenchInhibit(new Pose2d(4.5, 4, Rotation2d.kZero), new ChassisSpeeds(3, 0, 0)));
  }
}
