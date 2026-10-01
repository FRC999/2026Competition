package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.VisionConstants;
import org.junit.jupiter.api.Test;

class FieldRulesTest {
  @Test void bothAlliancesCheckNowAndReleasePositionIndependentlyOfTargetHysteresis() {
    double length = VisionConstants.FIELD_LAYOUT.getFieldLength();
    for (boolean red : new boolean[] {false, true}) {
      var pose = new Pose2d(red ? length-3.8 : 3.8, 2, Rotation2d.kZero);
      assertTrue(FieldRules.hubZoneConfirmed(pose, new ChassisSpeeds(), red, .12));
      assertFalse(FieldRules.hubZoneConfirmed(pose, new ChassisSpeeds(red ? -3 : 3,0,0), red, .12));
      var neutral = new Pose2d(red ? length-4.5 : 4.5, 2, Rotation2d.kZero);
      assertFalse(FieldRules.hubZoneConfirmed(neutral, new ChassisSpeeds(), red, .12));
      assertTrue(FieldRules.onOwnAutoHalf(neutral, red));
      assertFalse(FieldRules.onOwnAutoHalf(new Pose2d(red ? 2 : length-2, 2, Rotation2d.kZero), red));
    }
  }
  @Test void badInputsCannotQualifyAZone() {
    assertFalse(FieldRules.hubZoneConfirmed(new Pose2d(Double.NaN,2,Rotation2d.kZero), new ChassisSpeeds(),false,.12));
    assertFalse(FieldRules.hubZoneConfirmed(new Pose2d(2,-1,Rotation2d.kZero), new ChassisSpeeds(),false,.12));
    assertFalse(FieldRules.hubZoneConfirmed(new Pose2d(2,2,Rotation2d.kZero), new ChassisSpeeds(Double.NaN,0,0),false,.12));
  }
}
