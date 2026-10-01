package frc.robot.config;
import static org.junit.jupiter.api.Assertions.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.nio.file.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;

class OffseasonVisionConfigTest {
  @TempDir Path dir;
  final ObjectMapper mapper = new ObjectMapper();
  Path config;
  ObjectNode root;
  @BeforeEach void setup() throws Exception {
    Files.copy(Path.of("simulation/two-tag-field.json"), dir.resolve("field.json"));
    root = (ObjectNode) mapper.readTree(Path.of("simulation/vision.json").toFile());
    root.put("fieldLayout", "field.json");
    config = dir.resolve("vision.json");
  }
  OffseasonVisionConfig load() throws Exception { mapper.writeValue(config.toFile(), root); return OffseasonVisionConfig.load(config); }
  @Test void hashAcknowledgmentMustMatchExactFile() throws Exception {
    root.put("confirmedCoprocessorLayoutSha256", "wrong");
    assertFalse(load().layoutConfirmedOnCoprocessors());
    root.put("confirmedCoprocessorLayoutSha256", OffseasonVisionConfig.sha256(Files.readAllBytes(dir.resolve("field.json"))));
    assertTrue(load().layoutConfirmedOnCoprocessors());
  }
  @Test void calibratedCameraCannotOmitOffsets() throws Exception {
    ((ObjectNode) root.get("cameras").get(0)).putNull("translationMeters");
    assertThrows(java.io.IOException.class, this::load);
    ((ObjectNode) root.get("cameras").get(0)).put("calibrated", false);
    assertFalse(load().cameras().get(0).calibrated());
  }
  @Test void rejectsUnknownProfileAndEscapingLayout() {
    root.put("profile", "unknown"); assertThrows(java.io.IOException.class, this::load);
    root.put("profile", "calibration").put("fieldLayout", "../field.json");
    assertThrows(java.io.IOException.class, this::load);
  }
  @Test void calibrationLayoutCannotMasqueradeAsCompetitionField() {
    root.put("profile", "competition-welded");
    assertThrows(java.io.IOException.class, this::load);
  }
}
