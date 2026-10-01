package frc.robot.lib;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import edu.wpi.first.wpilibj.Filesystem;

/** Measured shot settings, interpolated only within the measured distance range.
 * Turret-angle endpoint reuse retains the season's distance-only (all angle=0) calibration assumption.
 */
public final class ShotTable {
    private static final double LOWEST_HOOD_DISTANCE_THRESHOLD_METERS = 3.5;
    public static class Setting {
        public final boolean valid;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double batteryVoltage;

        public Setting(
                boolean valid,
                double shooterRpmCommand,
                double hoodCommandAngleRad,
                double batteryVoltage) {
            this.valid = valid;
            this.shooterRpmCommand = shooterRpmCommand;
            this.hoodCommandAngleRad = hoodCommandAngleRad;
            this.batteryVoltage = batteryVoltage;
        }
    }

    public static class Sample {
        public final double distanceMeters;
        public final double turretAngleDeg;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double batteryVoltage;

        public Sample(
                double distanceMeters,
                double turretAngleDeg,
                double shooterRpmCommand,
                double hoodCommandAngleRad,
                double batteryVoltage) {
            this.distanceMeters = distanceMeters;
            this.turretAngleDeg = turretAngleDeg;
            this.shooterRpmCommand = shooterRpmCommand;
            this.hoodCommandAngleRad = hoodCommandAngleRad;
            this.batteryVoltage = batteryVoltage;
        }
    }

    private final TreeMap<Double, TreeMap<Double, ArrayList<Sample>>> table =
            new TreeMap<>();

    public void addSample(
            double distanceMeters,
            double turretAngleDeg,
            double shooterRpmCommand,
            double hoodCommandAngleRad,
            double batteryVoltage) {
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0
                || !Double.isFinite(turretAngleDeg) || !Double.isFinite(shooterRpmCommand) || shooterRpmCommand <= 0
                || !Double.isFinite(hoodCommandAngleRad)) {
            throw new IllegalArgumentException("Moving-shot sample must contain finite positive distance/RPM and finite angles");
        }
        sampleCount++;
        table.computeIfAbsent(distanceMeters, k -> new TreeMap<>())
                .computeIfAbsent(turretAngleDeg, k -> new ArrayList<>())
                .add(
                        new Sample(
                                distanceMeters,
                                turretAngleDeg,
                                shooterRpmCommand,
                                hoodCommandAngleRad,
                                batteryVoltage));
    }

    public boolean hasAnyData() {
        return !table.isEmpty();
    }

    public double getMinDistanceMeters() {
        return table.isEmpty() ? Double.NaN : table.firstKey();
    }

    public double getMaxDistanceMeters() {
        return table.isEmpty() ? Double.NaN : table.lastKey();
    }

    public static ShotTable loadFromDeployCsv(String deployRelativePath) {
        Path file = Filesystem.getDeployDirectory().toPath().resolve(deployRelativePath);
        return loadFromCsv(file);
    }

    private String loadStatus = "IN_MEMORY";
    private String sourceSha256 = "";
    private int sampleCount;
    public String loadStatus() { return loadStatus; }
    public String sourceSha256() { return sourceSha256; }
    public int sampleCount() { return sampleCount; }

    /** A corrupt calibration file is rejected as a unit; never use a silent partial table. */
    public static ShotTable loadFromCsv(Path csvPath) {
        ShotTable out = new ShotTable();
        try {
            if (csvPath == null || !Files.isRegularFile(csvPath)) {
                out.loadStatus = "MISSING"; return out;
            }
            byte[] bytes = Files.readAllBytes(csvPath);
            out.sourceSha256 = java.util.HexFormat.of().formatHex(
                    java.security.MessageDigest.getInstance("SHA-256").digest(bytes));
            int lineNumber = 0;
            for (String raw : new String(bytes, StandardCharsets.UTF_8).split("\\R")) {
                lineNumber++;
                String line = raw.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                if (line.equals("distance,shooterRpm,hoodAngleCommanded,turretAngle,batteryVoltage")) continue;
                String[] parts = line.split(",", -1);
                if (parts.length != 4 && parts.length != 5) throw new IllegalArgumentException("columns at line " + lineNumber);
                double distance = Double.parseDouble(parts[0].trim());
                double rpm = Double.parseDouble(parts[1].trim());
                double hood = Double.parseDouble(parts[2].trim());
                double turret = Double.parseDouble(parts[3].trim());
                double battery = parts.length == 5 && !parts[4].isBlank()
                        ? Double.parseDouble(parts[4].trim()) : Double.NaN;
                if (parts.length == 5 && !parts[4].isBlank() && (!Double.isFinite(battery) || battery <= 0))
                    throw new IllegalArgumentException("battery voltage at line " + lineNumber);
                out.addSample(distance, turret, rpm, Math.toRadians(hood), battery);
            }
            out.loadStatus = out.hasAnyData() ? "LOADED" : "EMPTY_UNCALIBRATED";
        } catch (IOException | IllegalArgumentException | java.security.NoSuchAlgorithmException ex) {
            out.table.clear(); out.sampleCount = 0;
            out.loadStatus = "REJECTED: " + ex.getMessage();
        }
        return out;
    }

    public Setting findInterpolatedShot(
            double distanceMeters,
            double turretAngleDeg,
            double preferredShooterRpm) {
        if (table.isEmpty()
                || !Double.isFinite(distanceMeters)
                || !Double.isFinite(turretAngleDeg)
                || distanceMeters < table.firstKey() || distanceMeters > table.lastKey()) {
            return makeInvalidSetting();
        }

        double distanceLow = floorKeyOrUseFirstKey(table, distanceMeters);
        double distanceHigh = ceilKeyOrUseLastKey(table, distanceMeters);

        if (nearlyEqual(distanceLow, distanceHigh)) {
            return interpolateAcrossTurretAngleForSingleDistance(
                    table.get(distanceLow),
                    turretAngleDeg,
                    preferredShooterRpm,
                    distanceMeters);
        }

        Setting low =
                interpolateAcrossTurretAngleForSingleDistance(
                        table.get(distanceLow),
                        turretAngleDeg,
                        preferredShooterRpm,
                        distanceMeters);
        Setting high =
                interpolateAcrossTurretAngleForSingleDistance(
                        table.get(distanceHigh),
                        turretAngleDeg,
                        preferredShooterRpm,
                        distanceMeters);

        if (!isFiniteSetting(low) || !isFiniteSetting(high)) {
            return makeInvalidSetting();
        }

        double t = fraction(distanceMeters, distanceLow, distanceHigh);

        return new Setting(
                true,
                lerp(low.shooterRpmCommand, high.shooterRpmCommand, t),
                lerp(low.hoodCommandAngleRad, high.hoodCommandAngleRad, t),
                lerp(low.batteryVoltage, high.batteryVoltage, t));
    }

    private static Setting interpolateAcrossTurretAngleForSingleDistance(
            TreeMap<Double, ArrayList<Sample>> angleMap,
            double turretAngleDeg,
            double preferredShooterRpm,
            double distanceMeters) {
        if (angleMap == null || angleMap.isEmpty()) {
            return makeInvalidSetting();
        }

        double angleLow = floorKeyOrUseFirstKey(angleMap, turretAngleDeg);
        double angleHigh = ceilKeyOrUseLastKey(angleMap, turretAngleDeg);

        if (nearlyEqual(angleLow, angleHigh)) {
            Sample s =
                    chooseClosestRpmSample(angleMap.get(angleLow), preferredShooterRpm, distanceMeters);
            if (s == null) {
                return makeInvalidSetting();
            }
            return new Setting(
                    true,
                    s.shooterRpmCommand,
                    s.hoodCommandAngleRad,
                    s.batteryVoltage);
        }

        Sample a =
                chooseClosestRpmSample(angleMap.get(angleLow), preferredShooterRpm, distanceMeters);
        Sample b =
                chooseClosestRpmSample(angleMap.get(angleHigh), preferredShooterRpm, distanceMeters);

        if (a == null || b == null) {
            return makeInvalidSetting();
        }

        double t = fraction(turretAngleDeg, angleLow, angleHigh);

        return new Setting(
                true,
                lerp(a.shooterRpmCommand, b.shooterRpmCommand, t),
                lerp(a.hoodCommandAngleRad, b.hoodCommandAngleRad, t),
                lerp(a.batteryVoltage, b.batteryVoltage, t));
    }

    private static Sample chooseClosestRpmSample(
            List<Sample> candidates,
            double preferredShooterRpm,
            double distanceMeters) {
        if (candidates == null || candidates.isEmpty()) {
            return null;
        }

        Sample best = null;
        double bestDelta = Double.POSITIVE_INFINITY;
        boolean preferLowestHood = shouldPreferLowestHoodForDistance(distanceMeters);

        for (Sample s : candidates) {
            if (s == null || !Double.isFinite(s.shooterRpmCommand)) {
                continue;
            }

            double delta = Math.abs(s.shooterRpmCommand - preferredShooterRpm);

            if (best == null || isSamplePreferred(
                    s,
                    delta,
                    best,
                    bestDelta,
                    preferLowestHood)) {
                bestDelta = delta;
                best = s;
            }
        }

        return best;
    }

    private static boolean isFiniteSetting(Setting c) {
        return c != null
                && c.valid
                && Double.isFinite(c.shooterRpmCommand)
                && Double.isFinite(c.hoodCommandAngleRad);
    }

    private static Setting makeInvalidSetting() {
        return new Setting(false, Double.NaN, Double.NaN, Double.NaN);
    }

    private static boolean shouldPreferLowestHoodForDistance(double distanceMeters) {
        return Double.isFinite(distanceMeters)
                && distanceMeters < LOWEST_HOOD_DISTANCE_THRESHOLD_METERS;
    }

    private static boolean isSamplePreferred(
            Sample candidate,
            double candidateRpmDelta,
            Sample currentBest,
            double currentBestRpmDelta,
            boolean preferLowestHood) {
        if (preferLowestHood) {
            if (candidate.hoodCommandAngleRad < currentBest.hoodCommandAngleRad - 1e-12) {
                return true;
            }
            if (candidate.hoodCommandAngleRad > currentBest.hoodCommandAngleRad + 1e-12) {
                return false;
            }
        }

        if (candidateRpmDelta < currentBestRpmDelta - 1e-12) {
            return true;
        }
        if (candidateRpmDelta > currentBestRpmDelta + 1e-12) {
            return false;
        }

        if (candidate.shooterRpmCommand < currentBest.shooterRpmCommand - 1e-9) {
            return true;
        }
        if (candidate.shooterRpmCommand > currentBest.shooterRpmCommand + 1e-9) {
            return false;
        }

        return candidate.hoodCommandAngleRad < currentBest.hoodCommandAngleRad - 1e-12;
    }

    private static boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double fraction(double x, double lo, double hi) {
        if (nearlyEqual(lo, hi))
            return 0.0;
        double t = (x - lo) / (hi - lo);
        if (t < 0.0)
            return 0.0;
        if (t > 1.0)
            return 1.0;
        return t;
    }

    private static <V> double floorKeyOrUseFirstKey(NavigableMap<Double, V> map, double key) {
        Double k = map.floorKey(key);
        return (k != null) ? k : map.firstKey();
    }

    private static <V> double ceilKeyOrUseLastKey(NavigableMap<Double, V> map, double key) {
        Double k = map.ceilingKey(key);
        return (k != null) ? k : map.lastKey();
    }
}
