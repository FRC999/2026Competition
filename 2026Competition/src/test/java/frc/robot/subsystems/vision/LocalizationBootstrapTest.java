package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import java.util.List;
import org.junit.jupiter.api.Test;

class LocalizationBootstrapTest {
  private final Pose2d truth = new Pose2d(12, 7, Rotation2d.fromDegrees(173));
  private LocalizationBootstrap.Sample sample(int camera, double t, Pose2d pose) {
    return new LocalizationBootstrap.Sample(camera, t, pose);
  }

  @Test void initializesFarFromOriginWithoutAllianceOrInnovationLockout() {
    var boot = new LocalizationBootstrap();
    for (int i=0; i<5; i++) assertTrue(boot.update(1+i*.02, true, true, false, new Pose2d(),
        List.of(sample(0, 1+i*.02, truth))).isEmpty());
    assertEquals(truth, boot.update(1.12, true, true, false, new Pose2d(),
        List.of(sample(0, 1.12, truth))).orElseThrow());
  }

  @Test void duplicateFramesCannotEstablishReference() {
    var boot = new LocalizationBootstrap();
    for (int i=0; i<20; i++) assertTrue(boot.update(1+i*.02, true, true, false, new Pose2d(),
        List.of(sample(0, 1, truth))).isEmpty());
  }

  @Test void staggeredCamerasDoNotRestartTheWindowAndConflictsBlockReset() {
    var boot = new LocalizationBootstrap();
    for (int i=0; i<5; i++) boot.update(1+i*.02, true, true, false, new Pose2d(),
        List.of(sample(0, 1+i*.02, truth), sample(1, 1+i*.02-.005, truth)));
    assertEquals(truth, boot.update(1.13, true, true, false, new Pose2d(),
        List.of(sample(0, 1.12, truth), sample(1, 1.125, truth))).orElseThrow());
    var conflict = truth.transformBy(new Transform2d(.4,0,Rotation2d.kZero));
    for (int i=0; i<10; i++) assertTrue(boot.update(2+i*.02,true,true,false,new Pose2d(),
        List.of(sample(0,2+i*.02,truth),sample(1,2+i*.02,conflict))).isEmpty());
    assertEquals("CAMERAS_DISAGREE", boot.status());
  }

  @Test void neverResetsEnabledOrWhileMovingAndCanRecoverAfterPlacement() {
    var boot = new LocalizationBootstrap();
    for (int i=0; i<10; i++) {
      assertTrue(boot.update(1+i*.02,false,true,false,new Pose2d(),List.of(sample(0,1+i*.02,truth))).isEmpty());
      assertTrue(boot.update(1+i*.02,true,false,false,new Pose2d(),List.of(sample(0,1+i*.02,truth))).isEmpty());
    }
    for (int i=0; i<5; i++) assertTrue(boot.update(2+i*.02,true,true,true,new Pose2d(),
        List.of(sample(0,2+i*.02,truth))).isEmpty());
    assertEquals(truth,boot.update(2.12,true,true,true,new Pose2d(),List.of(sample(0,2.12,truth))).orElseThrow());
    for (int i=0; i<20; i++) assertTrue(boot.update(3+i*.02,true,true,true,truth,
        List.of(sample(0,3+i*.02,truth))).isEmpty());
    assertEquals("REFERENCED",boot.status());
  }
}
