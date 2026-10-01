package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class FreshPressTest {
  @Test void heldInputRequiresReleaseAfterModeChangeOrPanic() {
    var gate = new FreshPress();
    assertFalse(gate.update(false,true));
    assertFalse(gate.update(true,true));
    assertFalse(gate.update(true,false));
    assertTrue(gate.update(true,true));
    assertFalse(gate.update(false,true));
    assertFalse(gate.update(true,true));
    assertFalse(gate.update(true,false));
    assertTrue(gate.update(true,true));
  }
}
