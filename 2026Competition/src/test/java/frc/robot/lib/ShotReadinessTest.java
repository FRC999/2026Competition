package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
class ShotReadinessTest {
  private ShotReadiness.Inputs inputs(boolean[] r) {
    return new ShotReadiness.Inputs(r[0],r[1],r[2],r[3],!r[4],r[5],r[6],r[7],r[8],r[9],!r[10],r[11]);
  }
  @Test void everyGateMustRemainTrueDuringFiring() {
    boolean[] all = new boolean[12]; java.util.Arrays.fill(all,true);
    assertEquals(ShotReadiness.Reason.READY, ShotReadiness.evaluate(inputs(all)));
    for (int i=0;i<all.length;i++) {
      all[i]=false;
      assertNotEquals(ShotReadiness.Reason.READY, ShotReadiness.evaluate(inputs(all)), "gate "+i);
      all[i]=true;
    }
  }
  @Test void jamClearAndDisableCannotRestoreOldIntent() {
    var intent = new ShotIntent();
    intent.request(true,false); assertTrue(intent.requested());
    intent.externalControl(true); assertFalse(intent.requested());
    intent.request(true,false); assertFalse(intent.requested());
    intent.externalControl(false); assertFalse(intent.requested());
    intent.request(true,false); assertTrue(intent.requested());
    intent.observe(false,false); intent.observe(true,false); assertFalse(intent.requested());
  }
  @Test void trenchRequiresExitAndFreshRequestAndCannotRearmInside() {
    var intent = new ShotIntent();
    intent.request(true,false); intent.observe(true,true);
    assertTrue(intent.trenchLocked());
    intent.request(false,true); intent.request(true,true);
    assertTrue(intent.trenchLocked());
    intent.observe(true,false); assertTrue(intent.trenchLocked());
    intent.request(false,false); intent.request(true,false);
    assertFalse(intent.trenchLocked());
  }
}
