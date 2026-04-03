package frc.robot.lib;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers.Solution;



/**
 * TurretHelpers
 *
 * Static math library: numbers in -> numbers out.
 *
 * What this solver does:
 * 1) Predict robot pose + velocity at the projectile release time.
 * 2) Compute the required initial ball velocity relative to the robot
 * (expressed in the FIELD frame).
 * 3) Convert that velocity into a desired yaw (field), desired pitch, and
 * desired speed.
 * 4) Use an artillery table built from measured data to choose the shooter RPM
 * and hood angle commands.
 *
 * Important table reality:
 * - Ball output angle depends on BOTH hood angle AND shooter RPM.
 * - The artillery table here is therefore indexed by (RPM, HoodAngle) and
 * stores the measured outcome:
 * (BallOutputAngle, BallExitSpeed)
 * - The solver performs an inverse-lookup: given (desiredBallAngle,
 * desiredBallSpeed),
 * find the (RPM, HoodAngle) pair that best matches.
 *
 * Forbidden terms note:
 * - This file intentionally does not use certain mechanical terms.
 */
public final class TurretHelpers {
    private static final double LOWEST_HOOD_DISTANCE_THRESHOLD_METERS = 3.5;

    // ---------------------------------------------------------------------------
    // Artillery table (what you MEASURE) indexed by (RPM, HoodAngle)
    // ---------------------------------------------------------------------------
    /** Measured outcome at a specific (RPM, HoodAngle). */
    public static class MeasuredBallOutcome {
        /** True ball output angle above horizontal (rad). */
        public final double ballOutputAngleRad;
        /** Ball exit speed relative to robot (m/s). */
        public final double ballExitSpeedMps;
        /** Optional: RPM drop observed when firing one ball at this setting (RPM). */
        public final double measuredRpmDrop;
        /** Optional: time to recover back within your RPM-ready tolerance (sec). */
        public final double measuredRecoveryTimeSec;

        public MeasuredBallOutcome(double ballOutputAngleRad, double ballExitSpeedMps) {
            this(ballOutputAngleRad, ballExitSpeedMps, Double.NaN, Double.NaN);
        }

        public MeasuredBallOutcome(double ballOutputAngleRad, double ballExitSpeedMps,
                double measuredRpmDrop, double measuredRecoveryTimeSec) {
            this.ballOutputAngleRad = ballOutputAngleRad;
            this.ballExitSpeedMps = ballExitSpeedMps;
            this.measuredRpmDrop = measuredRpmDrop;
            this.measuredRecoveryTimeSec = measuredRecoveryTimeSec;
        }
    }

    /**
     * A single command point (what you COMMAND to hardware) plus the measured
     * outcome.
     *
     * This is convenient because inverse-lookup returns commands, but we also want
     * to know
     * what measured outcome those commands represent (for debugging and scoring).
     */
    public static class ShooterCommandsAndMeasuredOutcome {
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double measuredBallOutputAngleRad;
        public final double measuredBallExitSpeedMps;

        public ShooterCommandsAndMeasuredOutcome(
                double shooterRpmCommand,
                double hoodCommandAngleRad,
                double measuredBallOutputAngleRad,
                double measuredBallExitSpeedMps) {
            this.shooterRpmCommand = shooterRpmCommand;
            this.hoodCommandAngleRad = hoodCommandAngleRad;
            this.measuredBallOutputAngleRad = measuredBallOutputAngleRad;
            this.measuredBallExitSpeedMps = measuredBallExitSpeedMps;
        }
    }

    /**
     * Artillery table indexed by:
     * RPM -> (HoodAngle -> MeasuredBallOutcome)
     *
     * This matches how you collect data:
     * 1) Pick an RPM
     * 2) Sweep hood angle in increments and record ball output angle + speed
     * 3) Increase RPM and repeat
     *
     * This class provides:
     * - adding measured points
     * - optional forward interpolation to estimate outcomes between measured points
     * - inverse lookup: choose (RPM, HoodAngle) commands that best match a desired
     * ball vector
     */
    public static class ArtilleryTableIndexedByShooterRpmAndHoodAngle {
        private final TreeMap<Double, TreeMap<Double, MeasuredBallOutcome>> table = new TreeMap<>();

        /** Add one measured sample: (rpm, hoodAngle) -> (ballAngle, ballSpeed). */
        public void addMeasuredSample(
                double shooterRpm,
                double hoodCommandAngleRad,
                double measuredBallOutputAngleRad,
                double measuredBallExitSpeedMps) {
            table.computeIfAbsent(shooterRpm, k -> new TreeMap<>())
                    .put(hoodCommandAngleRad,
                            new MeasuredBallOutcome(measuredBallOutputAngleRad, measuredBallExitSpeedMps));
        }

        public boolean hasAnyData() {
            return !table.isEmpty();
        }

        /**
         * Load an artillery table from a CSV on the roboRIO.
         *
         * <p>
         * Recommended location: put the file under src/main/deploy and load it with
         * {@link #loadFromDeployCsv(String)}.
         *
         * <h3>CSV format</h3>
         * <ul>
         * <li>Header row is optional. Lines starting with '#' are ignored.</li>
         * <li>Delimiter: comma</li>
         * <li>Required columns (either order):
         * <ul>
         * <li>shooter_rpm (double)</li>
         * <li>hood_deg (double) - hood command angle in degrees</li>
         * <li>measured_ball_angle_deg (double) - measured exit angle above horizontal,
         * degrees</li>
         * <li>measured_ball_speed_mps (double)</li>
         * </ul>
         * </li>
         * <li>Optional columns:
         * <ul>
         * <li>measured_rpm_drop (double) - RPM dip when firing one ball</li>
         * <li>measured_recovery_time_sec (double) - time to recover to "ready"</li>
         * </ul>
         * </li>
         * </ul>
         *
         * <p>
         * Units are explicit to avoid confusion. If you prefer radians in the file,
         * convert before writing.
         *
         * <p>
         * Error handling policy:
         * <ul>
         * <li>Bad lines are skipped (and counted).</li>
         * <li>If <b>no valid samples</b> are parsed, {@code hasAnyData()} will be
         * false.</li>
         * </ul>
         */
        public static ArtilleryTableIndexedByShooterRpmAndHoodAngle loadFromDeployCsv(String deployRelativePath) {
            Path file = Filesystem.getDeployDirectory().toPath().resolve(deployRelativePath);
            return loadFromCsv(file);
        }

        /**
         * Same as {@link #loadFromDeployCsv(String)} but takes an absolute/relative
         * {@link Path}.
         */
        public static ArtilleryTableIndexedByShooterRpmAndHoodAngle loadFromCsv(Path csvPath) {
            ArtilleryTableIndexedByShooterRpmAndHoodAngle out = new ArtilleryTableIndexedByShooterRpmAndHoodAngle();
            if (csvPath == null)
                return out;
            if (!Files.exists(csvPath)) {
                return out;
            }

            int badLines = 0;
            int goodLines = 0;

            try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
                String line;
                while ((line = br.readLine()) != null) {
                    line = line.trim();
                    if (line.isEmpty() || line.startsWith("#"))
                        continue;
                    // Allow a header by skipping any line that contains non-numeric tokens in the
                    // first 2 columns.
                    String[] parts = line.split(",");
                    if (parts.length < 4) {
                        badLines++;
                        continue;
                    }
                    Double rpm = tryParse(parts[0]);
                    Double hoodDeg = tryParse(parts[1]);
                    Double ballAngDeg = tryParse(parts[2]);
                    Double ballSpd = tryParse(parts[3]);
                    if (rpm == null || hoodDeg == null || ballAngDeg == null || ballSpd == null) {
                        // Header or malformed line
                        badLines++;
                        continue;
                    }
                    Double rpmDrop = (parts.length >= 5) ? tryParse(parts[4]) : null;
                    Double rec = (parts.length >= 6) ? tryParse(parts[5]) : null;

                    double hoodRad = Math.toRadians(hoodDeg);
                    double ballAngRad = Math.toRadians(ballAngDeg);
                    out.table.computeIfAbsent(rpm, k -> new TreeMap<>())
                            .put(hoodRad, new MeasuredBallOutcome(ballAngRad, ballSpd,
                                    (rpmDrop != null) ? rpmDrop : Double.NaN,
                                    (rec != null) ? rec : Double.NaN));
                    goodLines++;
                }
            } catch (IOException e) {
                // Leave table empty; caller can detect via hasAnyData()
                return out;
            }

            // Note: We intentionally do not throw if all lines were bad; empty table =>
            // invalid solution.
            return out;
        }

        private static Double tryParse(String s) {
            if (s == null)
                return null;
            s = s.trim();
            if (s.isEmpty())
                return null;
            try {
                return Double.parseDouble(s);
            } catch (NumberFormatException ex) {
                return null;
            }
        }

        /**
         * Inverse lookup:
         * Given a desired ball output angle (rad) and desired ball exit speed (m/s),
         * find the RPM + hood angle commands that best match.
         *
         * This returns a concrete object always.
         * If no data exists, it returns an object filled with NaN and valid=false will
         * be handled by caller.
         *
         * Matching rule:
         * error = angleWeight * |measuredAngle - desiredAngle| + speedWeight *
         * |measuredSpeed - desiredSpeed|
         *
         * Tie-breaking rule (when errors are extremely close):
         * - prefer lower RPM (usually easier shot / less stress)
         * - then prefer smaller hood angle change magnitude (caller can change this
         * later if needed)
         */
        public ShooterCommandsAndMeasuredOutcome findShooterRpmAndHoodAngleCommandsThatBestMatchDesiredBallAngleAndSpeed(
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                double angleWeight,
                double speedWeight) {
            if (table.isEmpty()) {
                return makeNotARealCommandsAndOutcome();
            }

            ShooterCommandsAndMeasuredOutcome coarseBest = findBestMeasuredGridPointMatchingDesiredBallAngleAndSpeed(
                    desiredBallOutputAngleRad,
                    desiredBallExitSpeedMps,
                    angleWeight,
                    speedWeight);

            if (!isFiniteCommandsAndOutcome(coarseBest)) {
                return makeNotARealCommandsAndOutcome();
            }

            ShooterCommandsAndMeasuredOutcome refinedBest = refineBestMeasuredGridPointUsingInterpolation(
                    coarseBest,
                    desiredBallOutputAngleRad,
                    desiredBallExitSpeedMps,
                    angleWeight,
                    speedWeight);

            return isFiniteCommandsAndOutcome(refinedBest) ? refinedBest : coarseBest;
        }

        /**
         * Optional forward interpolation:
         * Estimate (ballAngle, ballSpeed) for any (rpm, hood) by bilinear-like
         * interpolation.
         *
         * You do NOT need this to be correct, but it can smooth your table.
         * This returns NaNs if table is empty or missing rows.
         */
        public MeasuredBallOutcome estimateMeasuredBallOutcomeForShooterRpmAndHoodAngleUsingInterpolation(
                double shooterRpm,
                double hoodCommandAngleRad) {
            if (table.isEmpty())
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double rpmLow = floorKeyOrUseFirstKey(table, shooterRpm);
            double rpmHigh = ceilKeyOrUseLastKey(table, shooterRpm);
            if (nearlyEqual(rpmLow, rpmHigh)) {
                return interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmLow), hoodCommandAngleRad);
            }
            MeasuredBallOutcome low = interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmLow),
                    hoodCommandAngleRad);
            MeasuredBallOutcome high = interpolateOutcomeAcrossHoodAngleForSingleRpm(table.get(rpmHigh),
                    hoodCommandAngleRad);
            if (!isFiniteOutcome(low) || !isFiniteOutcome(high))
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double t = fraction(shooterRpm, rpmLow, rpmHigh);
            double ang = lerpAngle(low.ballOutputAngleRad, high.ballOutputAngleRad, t);
            double spd = lerp(low.ballExitSpeedMps, high.ballExitSpeedMps, t);
            return new MeasuredBallOutcome(ang, spd);
        }

        private static MeasuredBallOutcome interpolateOutcomeAcrossHoodAngleForSingleRpm(
                TreeMap<Double, MeasuredBallOutcome> hoodMap,
                double hoodCommandAngleRad) {
            if (hoodMap == null || hoodMap.isEmpty())
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double hoodLow = floorKeyOrUseFirstKey(hoodMap, hoodCommandAngleRad);
            double hoodHigh = ceilKeyOrUseLastKey(hoodMap, hoodCommandAngleRad);
            if (nearlyEqual(hoodLow, hoodHigh)) {
                MeasuredBallOutcome out = hoodMap.get(hoodLow);
                return (out != null) ? out : new MeasuredBallOutcome(Double.NaN, Double.NaN);
            }
            MeasuredBallOutcome a = hoodMap.get(hoodLow);
            MeasuredBallOutcome b = hoodMap.get(hoodHigh);
            if (a == null || b == null)
                return new MeasuredBallOutcome(Double.NaN, Double.NaN);
            double t = fraction(hoodCommandAngleRad, hoodLow, hoodHigh);
            double ang = lerpAngle(a.ballOutputAngleRad, b.ballOutputAngleRad, t);
            double spd = lerp(a.ballExitSpeedMps, b.ballExitSpeedMps, t);
            return new MeasuredBallOutcome(ang, spd);
        }

        private ShooterCommandsAndMeasuredOutcome findBestMeasuredGridPointMatchingDesiredBallAngleAndSpeed(
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                double angleWeight,
                double speedWeight) {
            ShooterCommandsAndMeasuredOutcome best = null;
            double bestError = Double.POSITIVE_INFINITY;

            for (Map.Entry<Double, TreeMap<Double, MeasuredBallOutcome>> rpmRow : table.entrySet()) {
                double rpm = rpmRow.getKey();
                TreeMap<Double, MeasuredBallOutcome> hoodMap = rpmRow.getValue();
                if (hoodMap == null || hoodMap.isEmpty()) {
                    continue;
                }

                for (Map.Entry<Double, MeasuredBallOutcome> hoodEntry : hoodMap.entrySet()) {
                    double hood = hoodEntry.getKey();
                    MeasuredBallOutcome out = hoodEntry.getValue();
                    if (out == null) {
                        continue;
                    }

                    double error = computeDesiredBallMatchError(
                            out.ballOutputAngleRad,
                            out.ballExitSpeedMps,
                            desiredBallOutputAngleRad,
                            desiredBallExitSpeedMps,
                            angleWeight,
                            speedWeight);

                    if (error < bestError - 1e-12) {
                        bestError = error;
                        best = new ShooterCommandsAndMeasuredOutcome(
                                rpm,
                                hood,
                                out.ballOutputAngleRad,
                                out.ballExitSpeedMps);
                    } else if (Math.abs(error - bestError) <= 1e-12 && best != null) {
                        if (rpm < best.shooterRpmCommand - 1e-9) {
                            best = new ShooterCommandsAndMeasuredOutcome(
                                    rpm,
                                    hood,
                                    out.ballOutputAngleRad,
                                    out.ballExitSpeedMps);
                        }
                    }
                }
            }

            return (best != null) ? best : makeNotARealCommandsAndOutcome();
        }

        private ShooterCommandsAndMeasuredOutcome refineBestMeasuredGridPointUsingInterpolation(
                ShooterCommandsAndMeasuredOutcome coarseBest,
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                double angleWeight,
                double speedWeight) {
            if (!isFiniteCommandsAndOutcome(coarseBest)) {
                return makeNotARealCommandsAndOutcome();
            }

            double coarseRpm = coarseBest.shooterRpmCommand;
            double coarseHood = coarseBest.hoodCommandAngleRad;

            Double rpmLowObj = table.lowerKey(coarseRpm);
            Double rpmHighObj = table.higherKey(coarseRpm);

            double rpmMin = (rpmLowObj != null) ? rpmLowObj : coarseRpm;
            double rpmMax = (rpmHighObj != null) ? rpmHighObj : coarseRpm;

            TreeMap<Double, MeasuredBallOutcome> hoodMapAtCoarseRpm = table.get(coarseRpm);
            if (hoodMapAtCoarseRpm == null || hoodMapAtCoarseRpm.isEmpty()) {
                return coarseBest;
            }

            Double hoodLowObj = hoodMapAtCoarseRpm.lowerKey(coarseHood);
            Double hoodHighObj = hoodMapAtCoarseRpm.higherKey(coarseHood);

            double hoodMin = (hoodLowObj != null) ? hoodLowObj : coarseHood;
            double hoodMax = (hoodHighObj != null) ? hoodHighObj : coarseHood;

            int rpmSubdivisions = nearlyEqual(rpmMin, rpmMax) ? 1 : 10;
            int hoodSubdivisions = nearlyEqual(hoodMin, hoodMax) ? 1 : 10;

            ShooterCommandsAndMeasuredOutcome best = coarseBest;
            double bestError = computeDesiredBallMatchError(
                    coarseBest.measuredBallOutputAngleRad,
                    coarseBest.measuredBallExitSpeedMps,
                    desiredBallOutputAngleRad,
                    desiredBallExitSpeedMps,
                    angleWeight,
                    speedWeight);

            for (int i = 0; i <= rpmSubdivisions; i++) {
                double rpm = (rpmSubdivisions == 1)
                        ? coarseRpm
                        : lerp(rpmMin, rpmMax, i / (double) rpmSubdivisions);

                for (int j = 0; j <= hoodSubdivisions; j++) {
                    double hood = (hoodSubdivisions == 1)
                            ? coarseHood
                            : lerp(hoodMin, hoodMax, j / (double) hoodSubdivisions);

                    MeasuredBallOutcome out = estimateMeasuredBallOutcomeForShooterRpmAndHoodAngleUsingInterpolation(
                            rpm, hood);

                    if (!isFiniteOutcome(out)) {
                        continue;
                    }

                    double error = computeDesiredBallMatchError(
                            out.ballOutputAngleRad,
                            out.ballExitSpeedMps,
                            desiredBallOutputAngleRad,
                            desiredBallExitSpeedMps,
                            angleWeight,
                            speedWeight);

                    if (error < bestError - 1e-12) {
                        bestError = error;
                        best = new ShooterCommandsAndMeasuredOutcome(
                                rpm,
                                hood,
                                out.ballOutputAngleRad,
                                out.ballExitSpeedMps);
                    } else if (Math.abs(error - bestError) <= 1e-12) {
                        if (rpm < best.shooterRpmCommand - 1e-9) {
                            best = new ShooterCommandsAndMeasuredOutcome(
                                    rpm,
                                    hood,
                                    out.ballOutputAngleRad,
                                    out.ballExitSpeedMps);
                        }
                    }
                }
            }

            return best;
        }

        private static double computeDesiredBallMatchError(
                double actualBallOutputAngleRad,
                double actualBallExitSpeedMps,
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                double angleWeight,
                double speedWeight) {
            double angleErr = Math.abs(wrapToPi(actualBallOutputAngleRad - desiredBallOutputAngleRad));
            double speedErr = Math.abs(actualBallExitSpeedMps - desiredBallExitSpeedMps);
            return angleWeight * angleErr + speedWeight * speedErr;
        }
    }

        // ---------------------------------------------------------------------------
    // Moving-auto shot lookup table keyed by (distance, turret angle)
    // ---------------------------------------------------------------------------
    public static class MovingAutoShotCommand {
        public final boolean valid;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double batteryVoltage;

        public MovingAutoShotCommand(
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

    public static class MovingAutoShotSample {
        public final double distanceMeters;
        public final double turretAngleDeg;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double batteryVoltage;

        public MovingAutoShotSample(
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

    public static class MovingAutoShotTable {
    private final TreeMap<Double, TreeMap<Double, ArrayList<MovingAutoShotSample>>> table =
            new TreeMap<>();

    public void addSample(
            double distanceMeters,
            double turretAngleDeg,
            double shooterRpmCommand,
            double hoodCommandAngleRad,
            double batteryVoltage) {
        table.computeIfAbsent(distanceMeters, k -> new TreeMap<>())
                .computeIfAbsent(turretAngleDeg, k -> new ArrayList<>())
                .add(
                        new MovingAutoShotSample(
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

    public static MovingAutoShotTable loadFromDeployCsv(String deployRelativePath) {
        Path file = Filesystem.getDeployDirectory().toPath().resolve(deployRelativePath);
        return loadFromCsv(file);
    }

    public static MovingAutoShotTable loadFromCsv(Path csvPath) {
        MovingAutoShotTable out = new MovingAutoShotTable();
        if (csvPath == null || !Files.exists(csvPath)) {
            return out;
        }

        try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }

                String[] parts = line.split(",");
                if (parts.length < 4) {
                    continue;
                }

                Double distanceMeters = tryParseMovingAuto(parts[0]);
                Double shooterRpm = tryParseMovingAuto(parts[1]);
                Double hoodAngleDeg = tryParseMovingAuto(parts[2]);
                Double turretAngleDeg = tryParseMovingAuto(parts[3]);
                Double batteryVoltage = (parts.length >= 5) ? tryParseMovingAuto(parts[4]) : null;

                if (distanceMeters == null
                        || shooterRpm == null
                        || hoodAngleDeg == null
                        || turretAngleDeg == null) {
                    continue;
                }

                out.addSample(
                        distanceMeters,
                        turretAngleDeg,
                        shooterRpm,
                        Math.toRadians(hoodAngleDeg),
                        batteryVoltage != null ? batteryVoltage : Double.NaN);
            }
        } catch (IOException e) {
            return out;
        }

        return out;
    }

    public MovingAutoShotCommand findInterpolatedShot(
            double distanceMeters,
            double turretAngleDeg,
            double preferredShooterRpm) {
        if (table.isEmpty()
                || !Double.isFinite(distanceMeters)
                || !Double.isFinite(turretAngleDeg)) {
            return makeInvalidMovingAutoShotCommand();
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

        MovingAutoShotCommand low =
                interpolateAcrossTurretAngleForSingleDistance(
                        table.get(distanceLow),
                        turretAngleDeg,
                        preferredShooterRpm,
                        distanceMeters);
        MovingAutoShotCommand high =
                interpolateAcrossTurretAngleForSingleDistance(
                        table.get(distanceHigh),
                        turretAngleDeg,
                        preferredShooterRpm,
                        distanceMeters);

        if (!isFiniteMovingAutoShotCommand(low) || !isFiniteMovingAutoShotCommand(high)) {
            return makeInvalidMovingAutoShotCommand();
        }

        double t = fraction(distanceMeters, distanceLow, distanceHigh);

        return new MovingAutoShotCommand(
                true,
                lerp(low.shooterRpmCommand, high.shooterRpmCommand, t),
                lerp(low.hoodCommandAngleRad, high.hoodCommandAngleRad, t),
                lerp(low.batteryVoltage, high.batteryVoltage, t));
    }

    private static MovingAutoShotCommand interpolateAcrossTurretAngleForSingleDistance(
            TreeMap<Double, ArrayList<MovingAutoShotSample>> angleMap,
            double turretAngleDeg,
            double preferredShooterRpm,
            double distanceMeters) {
        if (angleMap == null || angleMap.isEmpty()) {
            return makeInvalidMovingAutoShotCommand();
        }

        double angleLow = floorKeyOrUseFirstKey(angleMap, turretAngleDeg);
        double angleHigh = ceilKeyOrUseLastKey(angleMap, turretAngleDeg);

        if (nearlyEqual(angleLow, angleHigh)) {
            MovingAutoShotSample s =
                    chooseClosestRpmSample(angleMap.get(angleLow), preferredShooterRpm, distanceMeters);
            if (s == null) {
                return makeInvalidMovingAutoShotCommand();
            }
            return new MovingAutoShotCommand(
                    true,
                    s.shooterRpmCommand,
                    s.hoodCommandAngleRad,
                    s.batteryVoltage);
        }

        MovingAutoShotSample a =
                chooseClosestRpmSample(angleMap.get(angleLow), preferredShooterRpm, distanceMeters);
        MovingAutoShotSample b =
                chooseClosestRpmSample(angleMap.get(angleHigh), preferredShooterRpm, distanceMeters);

        if (a == null || b == null) {
            return makeInvalidMovingAutoShotCommand();
        }

        double t = fraction(turretAngleDeg, angleLow, angleHigh);

        return new MovingAutoShotCommand(
                true,
                lerp(a.shooterRpmCommand, b.shooterRpmCommand, t),
                lerp(a.hoodCommandAngleRad, b.hoodCommandAngleRad, t),
                lerp(a.batteryVoltage, b.batteryVoltage, t));
    }

    private static MovingAutoShotSample chooseClosestRpmSample(
            List<MovingAutoShotSample> candidates,
            double preferredShooterRpm,
            double distanceMeters) {
        if (candidates == null || candidates.isEmpty()) {
            return null;
        }

        MovingAutoShotSample best = null;
        double bestDelta = Double.POSITIVE_INFINITY;
        boolean preferLowestHood = shouldPreferLowestHoodForDistance(distanceMeters);

        for (MovingAutoShotSample s : candidates) {
            if (s == null || !Double.isFinite(s.shooterRpmCommand)) {
                continue;
            }

            double delta = Math.abs(s.shooterRpmCommand - preferredShooterRpm);

            if (best == null || isMovingAutoShotSamplePreferred(
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

    private static boolean isFiniteMovingAutoShotCommand(MovingAutoShotCommand c) {
        return c != null
                && c.valid
                && Double.isFinite(c.shooterRpmCommand)
                && Double.isFinite(c.hoodCommandAngleRad);
    }

    private static MovingAutoShotCommand makeInvalidMovingAutoShotCommand() {
        return new MovingAutoShotCommand(false, Double.NaN, Double.NaN, Double.NaN);
    }

    private static Double tryParseMovingAuto(String s) {
        if (s == null) {
            return null;
        }
        s = s.trim();
        if (s.isEmpty()) {
            return null;
        }
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException ex) {
            return null;
        }
    }
}

    // ---------------------------------------------------------------------------
    // Solver output (similar style to your sim code)
    // ---------------------------------------------------------------------------
    /**
     * Solver output.
     *
     * valid:
     * - true if a feasible solution was found using table data
     *
     * timeOfFlightS:
     * - chosen time (sec)
     *
     * yawFieldRad:
     * - field yaw direction of the required ball exit-relative velocity (rad)
     *
     * desiredBallOutputAngleRad:
     * - pitch (rad) from the computed vector (this is what physics asked for)
     *
     * desiredBallExitSpeedMps:
     * - speed (m/s) from the computed vector (this is what physics asked for)
     *
     * ballExitVelocityRelativeToRobotExpressedInFieldFrame:
     * - the computed exit-relative velocity vector u, expressed in field
     * coordinates
     *
     * shooterRpmCommand / hoodCommandAngleRad:
     * - commands selected from the artillery table
     *
     * tableMatchedBallOutputAngleRad / tableMatchedBallExitSpeedMps:
     * - measured outcome at the chosen command point
     */
    public static class Solution {
        public final boolean valid;
        public final double timeOfFlightS;
        public final double yawFieldRad;
        public final double desiredBallOutputAngleRad;
        public final double desiredBallExitSpeedMps;
        public final Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame;
        public final double shooterRpmCommand;
        public final double hoodCommandAngleRad;
        public final double tableMatchedBallOutputAngleRad;
        public final double tableMatchedBallExitSpeedMps;

        public Solution(
                boolean valid,
                double timeOfFlightS,
                double yawFieldRad,
                double desiredBallOutputAngleRad,
                double desiredBallExitSpeedMps,
                Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame,
                double shooterRpm,
                double hoodAngleRad,
                double tableMatchedBallOutputAngleRad,
                double tableMatchedBallExitSpeedMps) {
            this.valid = valid;
            this.timeOfFlightS = timeOfFlightS;
            this.yawFieldRad = yawFieldRad;
            this.desiredBallOutputAngleRad = desiredBallOutputAngleRad;
            this.desiredBallExitSpeedMps = desiredBallExitSpeedMps;
            this.ballExitVelocityRelativeToRobotExpressedInFieldFrame = ballExitVelocityRelativeToRobotExpressedInFieldFrame;
            this.shooterRpmCommand = shooterRpm;
            this.hoodCommandAngleRad = hoodAngleRad;
            this.tableMatchedBallOutputAngleRad = tableMatchedBallOutputAngleRad;
            this.tableMatchedBallExitSpeedMps = tableMatchedBallExitSpeedMps;
        }

        /**
         * Compute the turret command angle in degrees, relative to turret zero.
         *
         * Conventions:
         * - Robot frame: +X forward, +Y left.
         * - Field yaw: CCW positive from field +X.
         * - Returned angle is relative to turret zero, not robot forward.
         * - Turret zero direction is defined by
         * Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG.
         *
         * This method handles:
         * - constant-velocity prediction over readiness time
         * - turret pivot translation offset from robot origin
         * - turret zero-direction offset from robot forward
         *
         * This method does NOT decide what to do if the angle is outside legal turret
         * rotation limits. That decision should stay separate.
         */
        public static double computeTurretYawAngleRelativeToRobotDeg(
                Pose2d robotPose,
                Translation2d targetPosition,
                double vx,
                double vy,
                double omega,
                double readinessTimeMs) {

            double readinessTimeSec = readinessTimeMs / 1000.0;

            // Predict robot origin in the field frame at readiness time.
            Translation2d predictedRobotOriginField = robotPose.getTranslation().plus(
                    new Translation2d(vx * readinessTimeSec, vy * readinessTimeSec));

            // Predict robot heading at readiness time.
            Rotation2d predictedRobotHeading = robotPose.getRotation().plus(
                    Rotation2d.fromRadians(omega * readinessTimeSec));

            // Rotate turret pivot offset from robot frame into field frame, then add it to
            // the predicted robot origin to get the predicted turret pivot position.
            Translation2d turretPivotField = predictedRobotOriginField.plus(
                    Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
                            .rotateBy(predictedRobotHeading));

            // Vector from turret pivot to target in the field frame.
            Translation2d pivotToTargetField = targetPosition.minus(turretPivotField);

            // Field yaw from turret pivot to target.
            Rotation2d desiredYawField = pivotToTargetField.getAngle();

            // Convert field yaw into robot-relative yaw.
            Rotation2d robotRelativeYaw = desiredYawField.minus(predictedRobotHeading);

            // Convert robot-forward-relative yaw into turret-zero-relative yaw.
            Rotation2d turretZeroOffset = Rotation2d.fromDegrees(
                    Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG);

            Rotation2d turretRelativeYaw = robotRelativeYaw.minus(turretZeroOffset);

            // Returns (-180, 180], so an exact "behind" result will be +180.
            return turretRelativeYaw.getDegrees();
        }

        /**
         * Convenience list in the format you asked for: [ShooterRPM, HoodPosition,
         * BallVelocity, BallAngle]
         */
        public ArrayList<Double> toShooterRpmHoodBallSpeedBallAngleList() {
            ArrayList<Double> out = new ArrayList<>(4);
            out.add(shooterRpmCommand);
            out.add(hoodCommandAngleRad);
            out.add(desiredBallExitSpeedMps);
            out.add(desiredBallOutputAngleRad);
            return out;
        }
    }

    /**
     * Convert an aim target enum into a field-frame Translation2d.
     */
    public static Translation2d aimTargetToFieldTranslation(
            Constants.FieldTargets.AimTarget aimTarget,
            boolean isRedAlliance) {

        // System.out.println("Hub X: " + aimTarget.getX(isRedAlliance));

        return new Translation2d(
                aimTarget.getX(isRedAlliance),
                aimTarget.getY(isRedAlliance));
    }

    /**
     * Stationary/raw turret angle helper:
     * - readiness time = 0
     * - vx = 0
     * - vy = 0
     * - omega = 0
     *
     * Returned angle is relative to turret zero.
     */
    public static double computeStationaryRawTurretYawDeg(
            Pose2d robotPose,
            Translation2d targetPositionField) {
        return Solution.computeTurretYawAngleRelativeToRobotDeg(
                robotPose,
                targetPositionField,
                0.0,
                0.0,
                0.0,
                0.0);
    }

    /**
     * Inclusive window test for turret angles in degrees.
     */
    public static boolean isTurretAngleWithinWindowDeg(
            double turretDeg,
            double minDeg,
            double maxDeg) {
        return Double.isFinite(turretDeg)
                && turretDeg >= minDeg
                && turretDeg <= maxDeg;
    }

    /**
     * Compute the signed robot heading change (degrees) needed to move a turret
     * solution into the requested window.
     *
     * Sign convention:
     * - positive result => rotate robot CCW
     * - negative result => rotate robot CW
     *
     * If already in the window, returns 0.
     */
    public static double computeRobotHeadingDeltaDegToEnterTurretWindowDeg(
            double turretDeg,
            double minDeg,
            double maxDeg) {
        if (!Double.isFinite(turretDeg) || minDeg > maxDeg) {
            return Double.NaN;
        }

        double targetTurretDeg = MathUtil.clamp(turretDeg, minDeg, maxDeg);
        return turretDeg - targetTurretDeg;
    }

    /**
     * One-call convenience helper for the stationary illegal-shot auto-turn assist.
     *
     * Returns:
     * - 0 if the current stationary shot is already inside the comfort window
     * - otherwise a signed normalized omega command in [-1, +1]
     */
    public static double computeStationaryRobotAutoTurnCommandToEnterLegalShotWindow(
            Pose2d robotPose,
            Translation2d targetPositionField,
            double comfortMarginDeg,
            double fixedAbsTurnCmd) {

        double rawTurretDeg = computeStationaryRawTurretYawDeg(robotPose, targetPositionField);

        double comfortMinDeg = Constants.OperatorConstants.Turret.MIN_ANGLE_DEG + comfortMarginDeg;
        double comfortMaxDeg = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG - comfortMarginDeg;

        if (!Double.isFinite(rawTurretDeg) || comfortMinDeg > comfortMaxDeg) {
            return 0.0;
        }

        if (isTurretAngleWithinWindowDeg(rawTurretDeg, comfortMinDeg, comfortMaxDeg)) {
            return 0.0;
        }

        double robotHeadingDeltaDeg = computeRobotHeadingDeltaDegToEnterTurretWindowDeg(
                rawTurretDeg,
                comfortMinDeg,
                comfortMaxDeg);

        if (!Double.isFinite(robotHeadingDeltaDeg) || Math.abs(robotHeadingDeltaDeg) < 1e-9) {
            return 0.0;
        }

        return Math.copySign(Math.abs(fixedAbsTurnCmd), robotHeadingDeltaDeg);
    }

    // ---------------------------------------------------------------------------
    // Public top-level method with descriptive name
    // ---------------------------------------------------------------------------
    /**
     * Compute shooter RPM + hood angle commands to hit the hub while the robot is
     * moving.
     *
     * This method:
     * - predicts robot pose and velocity at release time using velocity +
     * acceleration (no constant velocity assumption)
     * - tries candidate times-of-flight and computes the required ball
     * exit-relative velocity vector for each
     * - converts that vector to yaw/pitch/speed
     * - uses the artillery table inverse lookup to pick commands that best match
     * pitch/speed
     * - returns the best solution found
     *
     * Inputs:
     * robotPoseAtDecisionTimeField: pose at time t0
     * robotVelocityAtDecisionTimeFieldMps: velocity at time t0
     * robotAccelerationAtDecisionTimeFieldMps2: acceleration at time t0
     * robotYawRateAtDecisionTimeRadPerSec: yaw rate at time t0 (if unknown, pass 0)
     * feedDelayFromDecisionToReleaseSec: delay from t0 to release time
     * turretPivotOffsetFromRobotOriginRobotFrameMeters: turret pivot offset in
     * robot frame (x forward, y left)
     * ballReleaseHeightMeters: release height z0
     * hubTargetPositionFieldMeters: target position (xh,yh,zh)
     * artilleryTable: measured table indexed by (rpm, hood) -> (ballAngle,
     * ballSpeed)
     * timeOfFlightSearchMinSec / Max / Step: search parameters
     * gravityMetersPerSec2: typically 9.81
     * angleWeight / speedWeight: how strongly to prioritize matching angle vs speed
     * in inverse lookup
     */
    public static Solution solveForShooterRpmAndHoodAngleCommandsWhileRobotIsMovingUsingMeasuredTableIndexedByRpmAndHood(
            Pose2d robotPoseAtDecisionTimeField,
            Translation2d robotVelocityAtDecisionTimeFieldMps,
            Translation2d robotAccelerationAtDecisionTimeFieldMps2,
            double robotYawRateAtDecisionTimeRadPerSec,
            double feedDelayFromDecisionToReleaseSec,
            Translation2d turretPivotOffsetFromRobotOriginRobotFrameMeters,
            double ballReleaseHeightMeters,
            Translation3d hubTargetPositionFieldMeters,
            ArtilleryTableIndexedByShooterRpmAndHoodAngle artilleryTable,
            double timeOfFlightSearchMinSec,
            double timeOfFlightSearchMaxSec,
            double timeOfFlightSearchStepSec,
            double gravityMetersPerSec2,
            double angleWeight,
            double speedWeight) {
        if (artilleryTable == null || !artilleryTable.hasAnyData()) {
            return makeInvalidSolution();
        }
        PredictedRobotPoseAndVelocityAtReleaseTime predicted = predictRobotPoseAndVelocityAtReleaseTimeUsingVelocityAndAcceleration(
                robotPoseAtDecisionTimeField,
                robotVelocityAtDecisionTimeFieldMps,
                robotAccelerationAtDecisionTimeFieldMps2,
                robotYawRateAtDecisionTimeRadPerSec,
                feedDelayFromDecisionToReleaseSec);
        Translation2d turretPivotPositionFieldMeters = computeTurretPivotPositionInFieldFrameAtReleaseTimeFromPredictedRobotPose(
                predicted.predictedPoseField,
                turretPivotOffsetFromRobotOriginRobotFrameMeters);
        Translation3d startPositionFieldMeters = new Translation3d(turretPivotPositionFieldMeters.getX(),
                turretPivotPositionFieldMeters.getY(), ballReleaseHeightMeters);
        Translation3d robotVelocityFieldAtReleaseMps = new Translation3d(predicted.predictedVxFieldMps,
                predicted.predictedVyFieldMps, 0.0);
        double shotDistanceMeters = turretPivotPositionFieldMeters.getDistance(
                new Translation2d(hubTargetPositionFieldMeters.getX(), hubTargetPositionFieldMeters.getY()));
        boolean preferLowestHood = shouldPreferLowestHoodForDistance(shotDistanceMeters);
        Solution best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (double T = timeOfFlightSearchMinSec; T <= timeOfFlightSearchMaxSec
                + 1e-9; T += timeOfFlightSearchStepSec) {
            if (T <= 1e-6)
                continue;
            Translation3d requiredBallExitVelocityRelativeToRobotFieldCoords = computeRequiredBallExitVelocityRelativeToRobotExpressedInFieldFrameForGivenTimeOfFlight(
                    startPositionFieldMeters,
                    hubTargetPositionFieldMeters,
                    robotVelocityFieldAtReleaseMps,
                    gravityMetersPerSec2,
                    T);
            YawPitchSpeed computedYawPitchSpeed = computeFieldYawAndBallOutputAngleAndBallSpeedFrom3dExitVelocityVector(
                    requiredBallExitVelocityRelativeToRobotFieldCoords);
            ShooterCommandsAndMeasuredOutcome commandsFromTable = artilleryTable
                    .findShooterRpmAndHoodAngleCommandsThatBestMatchDesiredBallAngleAndSpeed(
                            computedYawPitchSpeed.ballOutputAngleRad,
                            computedYawPitchSpeed.ballExitSpeedMps,
                            angleWeight,
                            speedWeight);
            if (!isFiniteCommandsAndOutcome(commandsFromTable))
                continue;
            // Define overall cost based on how well the chosen commands match what physics
            // asked for.
            double chosenAngleErr = Math.abs(
                    wrapToPi(commandsFromTable.measuredBallOutputAngleRad - computedYawPitchSpeed.ballOutputAngleRad));
            double chosenSpeedErr = Math
                    .abs(commandsFromTable.measuredBallExitSpeedMps - computedYawPitchSpeed.ballExitSpeedMps);
            double cost = angleWeight * chosenAngleErr + speedWeight * chosenSpeedErr;
            // Small bias toward lower speed (optional stability)
            cost += 0.02 * computedYawPitchSpeed.ballExitSpeedMps;
            Solution candidate = new Solution(
                    true,
                    T,
                    computedYawPitchSpeed.yawFieldRad,
                    computedYawPitchSpeed.ballOutputAngleRad,
                    computedYawPitchSpeed.ballExitSpeedMps,
                    requiredBallExitVelocityRelativeToRobotFieldCoords,
                    commandsFromTable.shooterRpmCommand,
                    commandsFromTable.hoodCommandAngleRad,
                    commandsFromTable.measuredBallOutputAngleRad,
                    commandsFromTable.measuredBallExitSpeedMps);
            if (best == null || isSolutionCandidatePreferred(
                    candidate,
                    cost,
                    best,
                    bestCost,
                    preferLowestHood)) {
                bestCost = cost;
                best = candidate;
            }
        }
        return (best != null) ? best : makeInvalidSolution();
    }

    // ---------------------------------------------------------------------------
    // Helper methods (descriptive names)
    // ---------------------------------------------------------------------------
    private static class PredictedRobotPoseAndVelocityAtReleaseTime {
        public final Pose2d predictedPoseField;
        public final double predictedVxFieldMps;
        public final double predictedVyFieldMps;

        public PredictedRobotPoseAndVelocityAtReleaseTime(Pose2d predictedPoseField, double predictedVxFieldMps,
                double predictedVyFieldMps) {
            this.predictedPoseField = predictedPoseField;
            this.predictedVxFieldMps = predictedVxFieldMps;
            this.predictedVyFieldMps = predictedVyFieldMps;
        }
    }

    private static PredictedRobotPoseAndVelocityAtReleaseTime predictRobotPoseAndVelocityAtReleaseTimeUsingVelocityAndAcceleration(
            Pose2d robotPoseAtDecisionTimeField,
            Translation2d robotVelocityAtDecisionTimeFieldMps,
            Translation2d robotAccelerationAtDecisionTimeFieldMps2,
            double robotYawRateAtDecisionTimeRadPerSec,
            double delayFromDecisionToReleaseSec) {
        double dt = delayFromDecisionToReleaseSec;
        double predictedX = robotPoseAtDecisionTimeField.getX()
                + robotVelocityAtDecisionTimeFieldMps.getX() * dt
                + 0.5 * robotAccelerationAtDecisionTimeFieldMps2.getX() * dt * dt;
        double predictedY = robotPoseAtDecisionTimeField.getY()
                + robotVelocityAtDecisionTimeFieldMps.getY() * dt
                + 0.5 * robotAccelerationAtDecisionTimeFieldMps2.getY() * dt * dt;
        double predictedVx = robotVelocityAtDecisionTimeFieldMps.getX()
                + robotAccelerationAtDecisionTimeFieldMps2.getX() * dt;
        double predictedVy = robotVelocityAtDecisionTimeFieldMps.getY()
                + robotAccelerationAtDecisionTimeFieldMps2.getY() * dt;
        double predictedYawRad = robotPoseAtDecisionTimeField.getRotation().getRadians()
                + robotYawRateAtDecisionTimeRadPerSec * dt;
        Pose2d predictedPoseField = new Pose2d(
                predictedX,
                predictedY,
                new edu.wpi.first.math.geometry.Rotation2d(predictedYawRad));
        return new PredictedRobotPoseAndVelocityAtReleaseTime(predictedPoseField, predictedVx, predictedVy);
    }

    private static Translation2d computeTurretPivotPositionInFieldFrameAtReleaseTimeFromPredictedRobotPose(
            Pose2d predictedRobotPoseField,
            Translation2d turretPivotOffsetFromRobotOriginRobotFrameMeters) {
        double yaw = predictedRobotPoseField.getRotation().getRadians();
        double c = Math.cos(yaw);
        double s = Math.sin(yaw);
        double offsetXField = c * turretPivotOffsetFromRobotOriginRobotFrameMeters.getX()
                - s * turretPivotOffsetFromRobotOriginRobotFrameMeters.getY();
        double offsetYField = s * turretPivotOffsetFromRobotOriginRobotFrameMeters.getX()
                + c * turretPivotOffsetFromRobotOriginRobotFrameMeters.getY();
        return new Translation2d(
                predictedRobotPoseField.getX() + offsetXField,
                predictedRobotPoseField.getY() + offsetYField);
    }

    /**
     * Compute required ball exit-relative velocity (u) in FIELD coordinates for a
     * chosen time-of-flight T.
     *
     * Model:
     * pt = p0 + (vRobot + u)*T + 0.5*gVec*T^2
     * where gVec = (0,0,-g).
     *
     * Solve:
     * u = (pt - p0 - vRobot*T - 0.5*gVec*T^2)/T
     *
     * Note: because gVec is downward, the z term becomes "+ 0.5*g*T^2" in the
     * numerator.
     */
    private static Translation3d computeRequiredBallExitVelocityRelativeToRobotExpressedInFieldFrameForGivenTimeOfFlight(
            Translation3d startPositionFieldMeters,
            Translation3d targetPositionFieldMeters,
            Translation3d robotVelocityFieldAtReleaseMps,
            double gravityMetersPerSec2,
            double timeOfFlightSec) {
        double T = timeOfFlightSec;
        double g = gravityMetersPerSec2;
        double dx = targetPositionFieldMeters.getX() - startPositionFieldMeters.getX();
        double dy = targetPositionFieldMeters.getY() - startPositionFieldMeters.getY();
        double dz = targetPositionFieldMeters.getZ() - startPositionFieldMeters.getZ();
        double ux = (dx - robotVelocityFieldAtReleaseMps.getX() * T) / T;
        double uy = (dy - robotVelocityFieldAtReleaseMps.getY() * T) / T;
        double uz = (dz - robotVelocityFieldAtReleaseMps.getZ() * T + 0.5 * g * T * T) / T;
        return new Translation3d(ux, uy, uz);
    }

    private static class YawPitchSpeed {
        public final double yawFieldRad;
        public final double ballOutputAngleRad;
        public final double ballExitSpeedMps;

        public YawPitchSpeed(double yawFieldRad, double ballOutputAngleRad, double ballExitSpeedMps) {
            this.yawFieldRad = yawFieldRad;
            this.ballOutputAngleRad = ballOutputAngleRad;
            this.ballExitSpeedMps = ballExitSpeedMps;
        }
    }

    /**
     * Convert a 3D velocity vector into field yaw + ball output angle (pitch) +
     * speed.
     */
    private static YawPitchSpeed computeFieldYawAndBallOutputAngleAndBallSpeedFrom3dExitVelocityVector(
            Translation3d ballExitVelocityRelativeToRobotExpressedInFieldFrame) {
        double vx = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getX();
        double vy = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getY();
        double vz = ballExitVelocityRelativeToRobotExpressedInFieldFrame.getZ();
        double yawField = Math.atan2(vy, vx);
        double horizontalSpeed = Math.hypot(vx, vy);
        double ballAngle = Math.atan2(vz, horizontalSpeed);
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        return new YawPitchSpeed(yawField, ballAngle, speed);
    }

    // ---------------------------------------------------------------------------
    // Validation / utility
    // ---------------------------------------------------------------------------
    private static boolean isFiniteOutcome(MeasuredBallOutcome o) {
        return o != null
                && Double.isFinite(o.ballOutputAngleRad)
                && Double.isFinite(o.ballExitSpeedMps);
    }

    private static boolean isFiniteCommandsAndOutcome(ShooterCommandsAndMeasuredOutcome c) {
        return c != null
                && Double.isFinite(c.shooterRpmCommand)
                && Double.isFinite(c.hoodCommandAngleRad)
                && Double.isFinite(c.measuredBallOutputAngleRad)
                && Double.isFinite(c.measuredBallExitSpeedMps);
    }

    private static boolean shouldPreferLowestHoodForDistance(double distanceMeters) {
        return Double.isFinite(distanceMeters)
                && distanceMeters < LOWEST_HOOD_DISTANCE_THRESHOLD_METERS;
    }

    private static boolean isMovingAutoShotSamplePreferred(
            MovingAutoShotSample candidate,
            double candidateRpmDelta,
            MovingAutoShotSample currentBest,
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

    private static boolean isSolutionCandidatePreferred(
            Solution candidate,
            double candidateCost,
            Solution currentBest,
            double currentBestCost,
            boolean preferLowestHood) {
        if (preferLowestHood) {
            if (candidate.hoodCommandAngleRad < currentBest.hoodCommandAngleRad - 1e-12) {
                return true;
            }
            if (candidate.hoodCommandAngleRad > currentBest.hoodCommandAngleRad + 1e-12) {
                return false;
            }
        }

        if (candidateCost < currentBestCost - 1e-12) {
            return true;
        }
        if (candidateCost > currentBestCost + 1e-12) {
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

    private static ShooterCommandsAndMeasuredOutcome makeNotARealCommandsAndOutcome() {
        return new ShooterCommandsAndMeasuredOutcome(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    public static Solution makeInvalidSolution() {
        return new Solution(
                false,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                new Translation3d(Double.NaN, Double.NaN, Double.NaN),
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    private static boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }

    /** Wrap angle to [-pi, pi]. */
    public static double wrapToPi(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /** Interpolate angles safely by wrapping the delta. */
    private static double lerpAngle(double aRad, double bRad, double t) {
        double d = wrapToPi(bRad - aRad);
        return aRad + d * t;
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
