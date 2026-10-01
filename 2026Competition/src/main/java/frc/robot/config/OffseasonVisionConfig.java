package frc.robot.config;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.HexFormat;
import java.util.List;
import java.util.Set;

/** Startup-only configuration. No camera extrinsic or field identity is silently guessed. */
public record OffseasonVisionConfig(
    String profile, String layoutSha256, AprilTagFieldLayout layout,
    boolean layoutConfirmedOnCoprocessors, List<Camera> cameras) {

  public record Camera(String name, boolean calibrated, Transform3d robotToCamera,
      double xyStdDevFactor, double thetaStdDevFactor, boolean trustRotation) {}

  public OffseasonVisionConfig {
    cameras = List.copyOf(cameras);
  }

  public static OffseasonVisionConfig load(Path configFile) throws IOException {
    JsonNode root = new ObjectMapper().readTree(configFile.toFile());
    if (root.path("schemaVersion").asInt(-1) != 1) {
      throw new IOException("vision configuration schemaVersion must be 1");
    }
    String profile = requiredText(root, "profile");
    if (!Set.of("competition-welded", "competition-andymark", "calibration", "simulation")
        .contains(profile)) {
      throw new IOException("Unknown vision profile: " + profile);
    }
    Path base = configFile.toAbsolutePath().normalize().getParent();
    Path layoutPath = base.resolve(requiredText(root, "fieldLayout")).normalize();
    if (!layoutPath.startsWith(base)) {
      throw new IOException("fieldLayout must stay within the vision configuration directory");
    }
    AprilTagFieldLayout layout = new AprilTagFieldLayout(layoutPath);
    if (profile.startsWith("competition-")) {
      var official = AprilTagFieldLayout.loadField(profile.equals("competition-welded")
          ? AprilTagFields.k2026RebuiltWelded : AprilTagFields.k2026RebuiltAndymark);
      if (layout.getFieldLength() != official.getFieldLength()
          || layout.getFieldWidth() != official.getFieldWidth()
          || layout.getTags().size() != official.getTags().size()
          || layout.getTags().stream().anyMatch(tag -> !official.getTagPose(tag.ID).map(tag.pose::equals).orElse(false))) {
        throw new IOException("Competition profile does not match its WPILib 2026 field layout");
      }
    }
    if (!(Double.isFinite(layout.getFieldLength()) && layout.getFieldLength() > 0
        && Double.isFinite(layout.getFieldWidth()) && layout.getFieldWidth() > 0)
        || layout.getTags().isEmpty()) {
      throw new IOException("Field layout needs positive dimensions and surveyed tags");
    }
    Set<Integer> tagIds = new HashSet<>();
    for (var tag : layout.getTags()) {
      var p = tag.pose;
      if (tag.ID < 0 || !tagIds.add(tag.ID)
          || !finite(p.getX(), p.getY(), p.getZ(), p.getRotation().getX(),
              p.getRotation().getY(), p.getRotation().getZ())) {
        throw new IOException("Invalid or duplicate tag in layout");
      }
    }
    String hash = sha256(Files.readAllBytes(layoutPath));
    // The acknowledgment is an operator check, not remote proof of the Pi's configuration.
    boolean confirmed = hash.equals(root.path("confirmedCoprocessorLayoutSha256").asText());
    JsonNode cameraNodes = root.path("cameras");
    if (!cameraNodes.isArray() || cameraNodes.size() < 1 || cameraNodes.size() > 4) {
      throw new IOException("Configure one to four cameras");
    }
    List<Camera> cameras = new ArrayList<>();
    Set<String> names = new HashSet<>();
    for (JsonNode node : cameraNodes) {
      String name = requiredText(node, "name");
      if (!name.matches("[A-Za-z0-9_-]+") || !names.add(name)) {
        throw new IOException("Camera names must be unique simple NetworkTables names");
      }
      if (!node.path("enabled").asBoolean(false)) continue;
      boolean calibrated = node.path("calibrated").asBoolean(false);
      Transform3d transform = new Transform3d();
      JsonNode xyz = node.path("translationMeters");
      JsonNode rpy = node.path("rotationDegrees");
      if (!xyz.isNull() && !xyz.isMissingNode() && !rpy.isNull() && !rpy.isMissingNode()) {
        double[] t = vector3(xyz, "translationMeters");
        double[] r = vector3(rpy, "rotationDegrees (roll, pitch, yaw)");
        transform = new Transform3d(new Translation3d(t[0], t[1], t[2]),
            new Rotation3d(Math.toRadians(r[0]), Math.toRadians(r[1]), Math.toRadians(r[2])));
      } else if (calibrated) {
        throw new IOException("Calibrated camera " + name + " needs translation and rotation");
      }
      cameras.add(new Camera(name, calibrated, transform,
          positive(node, "xyStdDevFactor"), positive(node, "thetaStdDevFactor"),
          node.path("trustRotation").asBoolean(false)));
    }
    if (cameras.isEmpty()) throw new IOException("At least one camera must be enabled");
    return new OffseasonVisionConfig(profile, hash, layout, confirmed, cameras);
  }

  public static String sha256(byte[] bytes) {
    try {
      return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  private static String requiredText(JsonNode node, String key) throws IOException {
    JsonNode value = node.path(key);
    if (!value.isTextual() || value.asText().isBlank()) throw new IOException("Missing " + key);
    return value.asText();
  }

  private static double[] vector3(JsonNode node, String description) throws IOException {
    if (!node.isArray() || node.size() != 3) throw new IOException("Expected three " + description);
    double[] values = new double[3];
    for (int i = 0; i < 3; i++) {
      if (!node.get(i).isNumber()) throw new IOException("Non-numeric " + description);
      values[i] = node.get(i).asDouble();
      if (!Double.isFinite(values[i])) throw new IOException("Non-finite " + description);
    }
    return values;
  }

  private static double positive(JsonNode node, String key) throws IOException {
    if (!node.path(key).isNumber()) throw new IOException("Missing numeric " + key);
    double value = node.get(key).asDouble();
    if (!Double.isFinite(value) || value <= 0) throw new IOException(key + " must be positive");
    return value;
  }

  private static boolean finite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }
}
