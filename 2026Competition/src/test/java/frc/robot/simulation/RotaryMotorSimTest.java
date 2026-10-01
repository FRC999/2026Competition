package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.*;

class RotaryMotorSimTest {
  @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
  @Test void rotorFreeSpeedDoesNotChangeWithReduction() {
    double expected = DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * 6 / (2*Math.PI);
    for (double reduction : new double[] {1,2,11,26.7}) {
      var model = new RotaryMotorSim(1, .002, reduction);
      for (int i=0;i<250;i++) model.update(6,.02);
      assertEquals(expected, model.rotorVelocityRps(), expected*.001);
      assertTrue(model.rotorPositionRotations() > 0);
    }
  }
  @Test void inertiaChangesAccelerationAndReverseStillLoadsBattery() {
    var light = new RotaryMotorSim(1,.002,1);
    var heavy = new RotaryMotorSim(1,.2,1);
    light.update(-6,.02); heavy.update(-6,.02);
    assertTrue(Math.abs(light.rotorVelocityRps()) > 5*Math.abs(heavy.rotorVelocityRps()));
    assertTrue(light.getCurrentDrawAmps() > 0);
    assertTrue(heavy.getCurrentDrawAmps() > 0);
  }
}
