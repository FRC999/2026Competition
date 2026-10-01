package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class MovingAimModelTest {
  @Test void stationaryBackwardShotHasZeroTurretAngle() {
    var aim = MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(), new Translation2d(-0.18, -0.06),
        new Translation2d(-4.18, -0.06), .12, .45, .45, 180).orElseThrow();
    assertEquals(0, aim.turretDegrees(), 1e-9);
    assertEquals(4, aim.effectiveDistanceMeters(), 1e-9);
  }
  @Test void strafeLeadCancelsInheritedBallVelocityAtImpact() {
    var aim = MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(0, 1, 0), new Translation2d(),
        new Translation2d(4, 0), .12, .5, .5, 0).orElseThrow();
    Translation2d launchTravel = new Translation2d(aim.effectiveDistanceMeters(), new Rotation2d(aim.fieldYawRadians()));
    Translation2d impact = aim.releasePosition().plus(launchTravel).plus(aim.releaseVelocity().times(.5));
    assertEquals(4, impact.getX(), 1e-9);
    assertEquals(0, impact.getY(), 1e-9);
    assertTrue(aim.fieldYawRadians() < 0);
  }
  @Test void rotationAddsPivotVelocityAndUsesReleaseHeadingNotImpactHeading() {
    var aim = MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(0, 0, 1), new Translation2d(.2, 0),
        new Translation2d(4, 0), .12, .5, .5, 0).orElseThrow();
    assertEquals(.2, aim.releaseVelocity().getNorm(), 1e-9);
    assertEquals(Math.toDegrees(aim.fieldYawRadians() - .12), aim.turretDegrees(), 1e-9);
  }
  @Test void frameRotationDoesNotChangeRelativeTurretSolution() {
    var a = MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(.4, .2, .3), new Translation2d(-.18, -.06),
        new Translation2d(-4, .3), .12, .45, .45, 180).orElseThrow();
    Rotation2d r = Rotation2d.fromDegrees(113);
    var b = MovingAimModel.predict(new Pose2d(0, 0, r), new ChassisSpeeds(.4, .2, .3), new Translation2d(-.18, -.06),
        new Translation2d(-4, .3).rotateBy(r), .12, .45, .45, 180).orElseThrow();
    assertEquals(a.turretDegrees(), b.turretDegrees(), 1e-9);
    assertEquals(a.effectiveDistanceMeters(), b.effectiveDistanceMeters(), 1e-9);
  }
  @Test void rejectsBadInputsAndTargetAtReleasePoint() {
    assertTrue(MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(), new Translation2d(),
        new Translation2d(), .12, .5, .5, 0).isEmpty());
    assertTrue(MovingAimModel.predict(new Pose2d(), new ChassisSpeeds(Double.NaN, 0, 0), new Translation2d(),
        new Translation2d(4, 0), .12, .5, .5, 0).isEmpty());
  }
}
