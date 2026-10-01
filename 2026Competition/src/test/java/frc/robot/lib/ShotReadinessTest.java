package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class ShotReadinessTest {
  @Test void everyReadinessConditionIsRequiredEvenAfterFiringStarts() {
    assertTrue(ShotReadiness.canFeed(true, true, true, true, true, true, true, false, false));
    // Loss of each independent permission stops feed; elapsed time cannot override any gate.
    for (int missing = 0; missing < 7; missing++) {
      boolean[] ready = {true, true, true, true, true, true, true};
      ready[missing] = false;
      assertFalse(ShotReadiness.canFeed(ready[0], ready[1], ready[2], ready[3], ready[4], ready[5], ready[6], false, false));
    }
    assertFalse(ShotReadiness.canFeed(true, true, true, true, true, true, true, true, false));
    assertFalse(ShotReadiness.canFeed(true, true, true, true, true, true, true, false, true));
  }
}
