package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ShotTableTest {
  @TempDir Path directory;
  @Test void corruptCalibrationCannotLeaveAPartialUsableTable() throws Exception {
    Path file = directory.resolve("shots.csv");
    Files.writeString(file, "distance,shooterRpm,hoodAngleCommanded,turretAngle,batteryVoltage\n"
        + "2,2200,3,0,12\n3,NaN,6,0,12\n");
    var table = ShotTable.loadFromCsv(file);
    assertFalse(table.hasAnyData());
    assertEquals(0, table.sampleCount());
    assertTrue(table.loadStatus().startsWith("REJECTED"));
    assertEquals(64, table.sourceSha256().length());
  }
  @Test void interpolationPreservesMeasuredSettingsAndNeverExtrapolatesDistance() throws Exception {
    Path file = directory.resolve("shots.csv");
    Files.writeString(file, "# measured settings\n2,2000,0,0,12\n4,2400,6,0,12\n");
    var table = ShotTable.loadFromCsv(file);
    var result = table.findInterpolatedShot(3, 45, 2200);
    assertEquals("LOADED", table.loadStatus());
    assertEquals(2, table.sampleCount());
    assertEquals(2200, result.shooterRpmCommand, 1e-9);
    assertEquals(Math.toRadians(3), result.hoodCommandAngleRad, 1e-9);
    assertFalse(table.findInterpolatedShot(4.01, 0, 2200).valid);
    assertFalse(table.findInterpolatedShot(1.99, 0, 2200).valid);
    assertFalse(table.findInterpolatedShot(Double.NaN, 0, 2200).valid);
  }
}
