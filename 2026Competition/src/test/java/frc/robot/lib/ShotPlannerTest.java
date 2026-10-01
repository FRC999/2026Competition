package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldTargets.AimTarget;
import org.junit.jupiter.api.Test;
class ShotPlannerTest {
  @Test void plannerUsesMeasuredHoodAndRejectsUnmeasuredPassOrDistance() {
    var table = new ShotTable();
    table.addSample(2,0,2000,.05,12);
    table.addSample(4,0,2400,.10,12);
    var planner=new ShotPlanner(table,new ShotTable(),new ShotFlightTimeTable());
    var pose=new Pose2d(5,4,Rotation2d.kZero);
    var target=new Translation2d(1.82,3.94);
    var shot=planner.solve(ShotPlanner.Mode.MOVING_AUTO,pose,new ChassisSpeeds(),target,AimTarget.HUB,2200,0,0);
    assertTrue(shot.valid()); assertEquals(2200,shot.shooterRpmCommand(),1e-8);
    assertEquals(.075,shot.hoodCommandAngleRad(),1e-8); assertEquals(0,shot.turretDegrees(),1e-8);
    var pass=planner.solve(ShotPlanner.Mode.MOVING_AUTO,pose,new ChassisSpeeds(),target,AimTarget.NEUTRAL_LOW,2200,0,0);
    assertFalse(pass.valid()); assertEquals("NO_MEASURED_PASS_SOLUTION",pass.leadSource());
    assertFalse(planner.solve(ShotPlanner.Mode.MOVING_AUTO,pose,new ChassisSpeeds(),new Translation2d(-5,4),
        AimTarget.HUB,2200,0,0).valid());
  }
  @Test void diagnosticCalculationIsRepeatableAndDoesNotChangeSettings() {
    var table = new ShotTable();
    table.addSample(2,0,2000,0,12); table.addSample(4,0,2400,.1,12);
    var planner=new ShotPlanner(table,new ShotTable(),new ShotFlightTimeTable());
    var pose=new Pose2d(5,4,Rotation2d.kZero); var target=new Translation2d(1.82,3.94);
    var first=planner.solve(ShotPlanner.Mode.MANUAL_PRESET_3M,pose,new ChassisSpeeds(),target,AimTarget.HUB,2200,0,.02);
    for(int i=0;i<20;i++) assertEquals(first,
        planner.solve(ShotPlanner.Mode.MANUAL_PRESET_3M,pose,new ChassisSpeeds(),target,AimTarget.HUB,2200,0,.02));
    assertEquals(2244, first.shooterRpmCommand(),1e-8);
  }
}
