package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.*;

class GuardedSysIdTest {
  @BeforeEach void enable() {
    assertTrue(HAL.initialize(500,0));
    DriverStationSim.setDsAttached(true); DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); DriverStation.refreshData();
  }
  @AfterEach void reset() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData(); DriverStation.refreshData();
  }
  @Test void constructionWhileDisallowedDoesNotBakeInANoopAndRevocationStopsOutput() {
    var allowed = new AtomicBoolean(false);
    var output = new AtomicInteger();
    var command = GuardedSysId.wrap(Commands.run(() -> output.set(6)), allowed::get, () -> output.set(0));
    allowed.set(true);
    CommandScheduler.getInstance().schedule(command); CommandScheduler.getInstance().run();
    assertEquals(6,output.get());
    allowed.set(false); CommandScheduler.getInstance().run();
    assertFalse(command.isScheduled()); assertEquals(0,output.get());
    allowed.set(true); CommandScheduler.getInstance().schedule(command); CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().cancel(command); assertEquals(0,output.get());
  }
}
