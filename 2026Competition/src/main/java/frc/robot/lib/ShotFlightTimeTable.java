package frc.robot.lib;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.OptionalDouble;
import java.util.TreeMap;

/** Optional measured distance -> flight time table. Empty/uncovered ranges use labeled legacy lead. */
public final class ShotFlightTimeTable {
  private final TreeMap<Double, Double> samples = new TreeMap<>();

  public static ShotFlightTimeTable load(Path file) throws IOException {
    ShotFlightTimeTable result = new ShotFlightTimeTable();
    if (!Files.exists(file)) return result;
    for (String line : Files.readAllLines(file)) {
      if (line.isBlank() || line.startsWith("#") || line.startsWith("distance_m,")) continue;
      String[] columns = line.split(",");
      try {
        if (columns.length != 2) throw new NumberFormatException();
        double distance = Double.parseDouble(columns[0]);
        double seconds = Double.parseDouble(columns[1]);
        if (!Double.isFinite(distance) || !Double.isFinite(seconds) || distance <= 0 || seconds <= 0
            || result.samples.putIfAbsent(distance, seconds) != null) throw new NumberFormatException();
      } catch (NumberFormatException ex) {
        throw new IOException("Invalid/duplicate flight-time sample: " + line, ex);
      }
    }
    return result;
  }

  public OptionalDouble atDistance(double distance) {
    if (!Double.isFinite(distance) || samples.size() < 2
        || distance < samples.firstKey() || distance > samples.lastKey()) return OptionalDouble.empty();
    var low = samples.floorEntry(distance);
    var high = samples.ceilingEntry(distance);
    if (low.getKey().equals(high.getKey())) return OptionalDouble.of(low.getValue());
    double fraction = (distance - low.getKey()) / (high.getKey() - low.getKey());
    return OptionalDouble.of(low.getValue() + fraction * (high.getValue() - low.getValue()));
  }
}
