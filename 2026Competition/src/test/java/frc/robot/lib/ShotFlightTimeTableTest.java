package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
class ShotFlightTimeTableTest {
  @TempDir Path dir;
  @Test void interpolatesOnlyWithinMeasuredRange() throws Exception {
    var file = dir.resolve("flight.csv");
    Files.writeString(file, "distance_m,flight_time_s\n2,0.5\n4,0.9\n");
    var table = ShotFlightTimeTable.load(file);
    assertEquals(.7, table.atDistance(3).orElseThrow(), 1e-9);
    assertTrue(table.atDistance(1).isEmpty());
    assertTrue(table.atDistance(Double.NaN).isEmpty());
  }
  @Test void malformedAndDuplicateSamplesFail() throws Exception {
    var file = dir.resolve("flight.csv");
    for (String text : new String[] {"2,NaN", "2,-1", "2,.5\n2,.6", "Infinity,.5"}) {
      Files.writeString(file, text);
      assertThrows(java.io.IOException.class, () -> ShotFlightTimeTable.load(file));
    }
  }
}
