package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
class HardStopHomingTest {
  @Test void delayedLoopCannotConfirmHomingAfterTheDeadline() {
    var homing = new HardStopHoming(.2, .1, 1); homing.start(0);
    assertEquals(HardStopHoming.Result.MOVING, homing.update(.95, true));
    assertEquals(HardStopHoming.Result.TIMED_OUT, homing.update(1.12, true));
  }
  @Test void timeoutNeverEstablishesZero() {
    var homing=new HardStopHoming(.2,.1,1); homing.start(0);
    assertEquals(HardStopHoming.Result.MOVING,homing.update(.5,false));
    assertEquals(HardStopHoming.Result.TIMED_OUT,homing.update(1.1,false));
    assertEquals(HardStopHoming.Result.TIMED_OUT,homing.update(2,true));
  }
  @Test void minimumTravelAndContinuousFreshStallEvidenceAreBothRequired() {
    var homing=new HardStopHoming(.2,.1,1); homing.start(0);
    homing.update(.1,true); homing.update(.21,true); homing.update(.25,false);
    assertEquals(HardStopHoming.Result.MOVING,homing.update(.32,true));
    assertEquals(HardStopHoming.Result.MOVING,homing.update(.39,true));
    assertEquals(HardStopHoming.Result.CONFIRMED_STALL,homing.update(.44,true));
  }
}
