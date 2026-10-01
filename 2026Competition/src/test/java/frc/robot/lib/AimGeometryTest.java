package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldTargets.AimTarget;
import frc.robot.config.VisionConstants;
import org.junit.jupiter.api.Test;
class AimGeometryTest {
  @Test void negativePivotIsBehindAndRightAtAllCardinalHeadings() {
    double[][] expected = {{-.18,-.06},{.06,-.18},{.18,.06},{-.06,.18}};
    for(int i=0;i<4;i++) {
      var pivot=AimGeometry.pivot(new Pose2d(5,4,Rotation2d.fromDegrees(90*i)));
      assertEquals(5+expected[i][0],pivot.getX(),1e-9);
      assertEquals(4+expected[i][1],pivot.getY(),1e-9);
    }
    var pose=new Pose2d(5,4,Rotation2d.kZero);
    assertEquals(0,AimGeometry.turretDegrees(pose,new Translation2d(2,3.94)),1e-8);
    assertEquals(-90,AimGeometry.turretDegrees(pose,new Translation2d(4.82,7)),1e-8);
  }
  @Test void simultaneousFieldRotationPreservesTurretBearingAndMovingIntercept() {
    var pose=new Pose2d(4,3,Rotation2d.fromDegrees(23));
    var target=new Translation2d(1,5);
    var speeds=new ChassisSpeeds(.6,-.4,.3);
    var offset=new Translation2d(-.18,-.06);
    var original=MovingAimModel.predict(pose,speeds,offset,target,.12,.5,.5,180).orElseThrow();
    for (int degrees : new int[]{90,180,270}) {
      var rotation=Rotation2d.fromDegrees(degrees);
      var transformed=new Pose2d(pose.getTranslation().rotateBy(rotation),pose.getRotation().plus(rotation));
      var result=MovingAimModel.predict(transformed,speeds,offset,target.rotateBy(rotation),.12,.5,.5,180).orElseThrow();
      assertEquals(original.turretDegrees(),result.turretDegrees(),1e-8);
      assertEquals(original.effectiveDistanceMeters(),result.effectiveDistanceMeters(),1e-8);
    }
    var projectileRelativeVelocity = new Translation2d(original.effectiveDistanceMeters()/.5,
        new Rotation2d(original.fieldYawRadians()));
    var landing=original.releasePosition().plus(original.releaseVelocity().plus(projectileRelativeVelocity).times(.5));
    assertEquals(target.getX(),landing.getX(),1e-8);
    assertEquals(target.getY(),landing.getY(),1e-8);
  }
  @Test void redAndBlueSelectionUseOneRotationalFieldFrameAndStaticAlwaysUsesHub() {
    double length=VisionConstants.FIELD_LAYOUT.getFieldLength(), width=VisionConstants.FIELD_LAYOUT.getFieldWidth();
    var blue=new Pose2d(6,2,Rotation2d.kZero);
    var red=new Pose2d(length-6,width-2,Rotation2d.k180deg);
    assertEquals(AimTarget.NEUTRAL_LOW,FieldTargeting.select(blue,false,true,AimTarget.HUB));
    assertEquals(AimTarget.NEUTRAL_LOW,FieldTargeting.select(red,true,true,AimTarget.HUB));
    assertEquals(AimTarget.HUB,FieldTargeting.select(red,true,false,AimTarget.NEUTRAL_LOW));
    assertTrue(FieldTargeting.inTrench(new Pose2d(4.5,7.4,Rotation2d.kZero)));
    assertTrue(FieldTargeting.inTrench(new Pose2d(length-4.5,width-7.4,Rotation2d.kZero)));
    assertFalse(FieldTargeting.inTrench(new Pose2d(length/2,width/2,Rotation2d.kZero)));
    assertEquals(AimTarget.HUB,FieldTargeting.select(new Pose2d(4.70,2,Rotation2d.kZero),false,true,AimTarget.HUB));
    assertEquals(AimTarget.NEUTRAL_LOW,FieldTargeting.select(new Pose2d(4.70,2,Rotation2d.kZero),false,true,AimTarget.NEUTRAL_LOW));
  }
}
