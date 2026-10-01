package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.OffseasonVisionConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.VisionConstants;
import frc.robot.subsystems.vision.VisionPolicy.CovarianceModel;
import frc.robot.subsystems.vision.VisionPolicy.RejectionReason;
import frc.robot.subsystems.vision.VisionPolicy.SingleTagStrategy;

/**
 * AprilTag localization front end. Owns frame ingestion (via {@link VisionIO}), pose validation,
 * covariance selection, timestamped fusion, and structured logging. The drivetrain only receives
 * accepted, weighted, timestamped observations through a {@link VisionConsumer}.
 *
 * <p>All fusion <em>decisions</em> (gates, covariance math, timing rules) live in the pure
 * {@link VisionPolicy}; this class orchestrates them per loop and logs the outcome. This rewrite is
 * based on the official AdvantageKit PhotonVision template (also shipped by 1768 Nashoba) for the
 * IO-layer + accepted/rejected logging shape, with several deliberate upgrades that encode this
 * project's whole thesis -- "fix the measurement discipline that hurt us in 2025/2026":
 *
 * <ul>
 *   <li><b>Single-tag heading is never trusted</b>: angular std-dev is {@code +Infinity} for one-tag
 *       solves. Idea: 6328 {@code Vision.java} and the v2 strategy doc rule 7.4.
 *   <li><b>NaN / non-finite rejection</b>, <b>per-camera std-dev factors</b> (6328),
 *       <b>innovation logging</b> (pragmatic 3467), <b>structured rejection reasons</b> (3467),
 *       <b>early-auto vision ignore</b> (6328) -- see {@link VisionPolicy}.
 *   <li><b>Selectable single-tag strategy</b> (2026-07-16): {@link SingleTagStrategy#TRIG_SOLVE}
 *       recomputes single-tag XY from the camera-to-tag translation + the odometry-buffer heading at
 *       the frame timestamp ({@link SingleTagTrigSolver}; idea: 6328 via PhotonVision
 *       {@code PNP_DISTANCE_TRIG_SOLVE}, 1678 C2026 production). Default remains the validated
 *       {@link SingleTagStrategy#PNP}; A/B autos in {@code RobotContainer} switch modes per run.
 *   <li><b>Selectable covariance model</b> (2026-07-16): {@link CovarianceModel#ANISOTROPIC} weights
 *       X/Y by the camera->tag ray direction (idea: 5940). Default remains
 *       {@link CovarianceModel#ISOTROPIC} until coefficients are fitted from robot logs.
 * </ul>
 */
public class Vision extends SubsystemBase {
  /** Sink for accepted observations. {@code RobotContainer} wires this to CTRE's estimator. */
  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  /**
   * Source of the robot heading at a given FPGA timestamp, for the trig-solve strategy.
   * {@code RobotContainer} wires this to the CTRE odometry pose-history buffer
   * ({@code DriveSubsystem.sampleHeadingAt}), so each frame gets the heading the robot actually had
   * when the frame was captured -- the same latency compensation the estimator itself uses. Empty when
   * the buffer cannot answer (e.g., right after boot); the caller then falls back to the PnP pose.
   */
  @FunctionalInterface
  public static interface HeadingSampler {
    Optional<Rotation2d> headingAt(double fpgaTimestampSeconds);
  }

  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final DoubleSupplier lastResetTimeSupplier;
  private final HeadingSampler headingSampler;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final double[] lastFrameTimestamps;
  private final boolean[] cameraFusionEnabled;
  private final String[] cameraNames;
  private BooleanSupplier stationarySupplier = () -> false;
  private double lastFusedTimestamp = Double.NEGATIVE_INFINITY;
  private final Alert[] disconnectedAlerts;
  private final PoseJitterAccumulator[] jitterAccumulators;
  private boolean jitterCaptureActive;

  // Freshest accepted observation with a trustworthy heading (MultiTag only). This is intentionally
  // separate from the fused drivetrain pose so an operator can explicitly re-anchor the estimator from
  // camera geometry during disabled bring-up.
  private Pose2d latestTrustedPose;
  private double latestTrustedPoseTimestamp = Double.NEGATIVE_INFINITY;

  // All tag poses in the layout, precomputed. Logged every loop so AdvantageScope can always draw the
  // whole board, not just the tags a camera happens to see this loop (Vision/Summary/TagPoses).
  private final Pose3d[] layoutTagPoses;

  // Restarted whenever we are NOT in enabled autonomous, so it measures "seconds since auto start".
  private final Timer autoTimer = new Timer();

  // Experiment toggles (2026-07-16). Defaults are the validated 2026-06-30 baseline; the A/B autos in
  // RobotContainer set these explicitly at the start of every run so a leftover mode from a previous
  // test can never contaminate a comparison run.
  private SingleTagStrategy singleTagStrategy = SingleTagStrategy.PNP;
  private CovarianceModel covarianceModel = CovarianceModel.ISOTROPIC;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier lastResetTimeSupplier,
      HeadingSampler headingSampler,
      VisionIO... io) {
    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.lastResetTimeSupplier = lastResetTimeSupplier;
    this.headingSampler = headingSampler;
    this.io = io;
    layoutTagPoses =
        VisionConstants.FIELD_LAYOUT.getTags().stream()
            .map(tag -> tag.pose)
            .toArray(Pose3d[]::new);

    inputs = new VisionIOInputsAutoLogged[io.length];
    lastFrameTimestamps = new double[io.length];
    Arrays.fill(lastFrameTimestamps, Double.NEGATIVE_INFINITY);
    cameraFusionEnabled = new boolean[io.length];
    Arrays.fill(cameraFusionEnabled, true);
    cameraNames = new String[io.length];
    disconnectedAlerts = new Alert[io.length];
    jitterAccumulators = new PoseJitterAccumulator[io.length];
    for (int i = 0; i < io.length; i++) {
      cameraNames[i] = "camera-" + i;
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
      jitterAccumulators[i] =
          new PoseJitterAccumulator(VisionConstants.CAMERA_JITTER_CAPTURE_SAMPLES);
    }
    autoTimer.start();
  }

  public void configureCameras(OffseasonVisionConfig config, BooleanSupplier stationarySupplier) {
    if (config.cameras().size() != io.length) throw new IllegalArgumentException("Camera IO/config count mismatch");
    this.stationarySupplier = stationarySupplier;
    competitionAimFrame = config.profile().equals("competition-welded");
    for (int i = 0; i < io.length; i++) {
      cameraNames[i] = config.cameras().get(i).name();
      cameraFusionEnabled[i] = config.cameras().get(i).calibrated()
          && config.layoutConfirmedOnCoprocessors();
      Logger.recordOutput("Vision/Camera" + i + "/Name", cameraNames[i]);
      Logger.recordOutput("Vision/Camera" + i + "/FusionConfigured", cameraFusionEnabled[i]);
      Logger.recordOutput("Vision/Camera" + i + "/RobotToCamera", new Pose3d().transformBy(config.cameras().get(i).robotToCamera()));
      Logger.recordOutput("Vision/Camera" + i + "/XYStdDevFactor", config.cameras().get(i).xyStdDevFactor());
      Logger.recordOutput("Vision/Camera" + i + "/ThetaStdDevFactor", config.cameras().get(i).thetaStdDevFactor());
    }
  }

  private boolean competitionAimFrame;

  public boolean hasCompetitionAimFrame() { return competitionAimFrame; }

  public boolean hasRecentMeasurement() {
    double age = Timer.getFPGATimestamp() - lastFusedTimestamp;
    return lastFusedTimestamp > lastResetTimeSupplier.getAsDouble()
        && age >= -VisionConstants.MAX_FUTURE_TIMESTAMP_SECONDS
        && age <= VisionConstants.MAX_FRAME_AGE_SECONDS;
  }

  /** Selects how single-tag frames are solved. Set by the A/B autos; safe to call at any time. */
  public void setSingleTagStrategy(SingleTagStrategy strategy) {
    this.singleTagStrategy = strategy;
  }

  public SingleTagStrategy getSingleTagStrategy() {
    return singleTagStrategy;
  }

  /** Selects the measurement-noise model. Set by the A/B autos; safe to call at any time. */
  public void setCovarianceModel(CovarianceModel model) {
    this.covarianceModel = model;
  }

  public CovarianceModel getCovarianceModel() {
    return covarianceModel;
  }

  /**
   * Clears and starts a fixed 100-sample accepted-MultiTag pose capture for every camera. The robot
   * must stay disabled and physically stationary; the method rejects an enabled request so drivetrain
   * motion cannot be mislabeled as camera jitter.
   */
  public void startCameraJitterCapture() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning(
          "Camera jitter capture rejected: disable the robot and keep it stationary.", false);
      return;
    }
    for (PoseJitterAccumulator accumulator : jitterAccumulators) {
      accumulator.reset();
    }
    jitterCaptureActive = true;
    DriverStation.reportWarning(
        "Camera jitter capture started: keep the disabled robot stationary with both tags visible.",
        false);
  }

  /** Freezes the current camera-jitter sample sets. Safe while disabled only. */
  public void stopCameraJitterCapture() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Camera jitter capture stop rejected while enabled.", false);
      return;
    }
    jitterCaptureActive = false;
  }

  /**
   * Camera-relative yaw to the best target on the given camera, or empty when the index is invalid, the
   * camera sees no target, or the bearing is stale (no fresh frame within
   * {@code TARGET_OBSERVATION_MAX_STALENESS_SECONDS}). The hook a future boresight loop would servo on.
   *
   * <p>Returns {@link Optional} (not a bare angle) so a caller cannot mistake "no/stale target" for
   * "target dead ahead (0 deg)"; {@code hasTarget} + the frame timestamp carry that distinction.
   */
  public Optional<Rotation2d> getTargetX(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= inputs.length) {
      return Optional.empty();
    }
    var obs = inputs[cameraIndex].latestTargetObservation;
    return VisionPolicy.freshTargetX(obs, Timer.getTimestamp());
  }

  /**
   * Returns the freshest accepted MultiTag robot pose when it is recent enough for a manual estimator
   * seed. Single-tag observations are never returned because this project deliberately assigns their
   * heading infinite uncertainty.
   */
  public Optional<Pose2d> getFreshTrustedSeedPose() {
    if (latestTrustedPose == null
        || latestTrustedPoseTimestamp < lastResetTimeSupplier.getAsDouble()
        || Math.abs(Timer.getTimestamp() - latestTrustedPoseTimestamp)
            > VisionConstants.VISION_SEED_MAX_STALENESS_SECONDS) {
      return Optional.empty();
    }
    return Optional.of(latestTrustedPose);
  }

  @Override
  public void periodic() {
    double periodicStartSeconds = Timer.getFPGATimestamp();
    boolean capturePermitted = DriverStation.isDisabled() && stationarySupplier.getAsBoolean();
    SmartDashboard.putNumber("Calibration/RobotTimestamp", periodicStartSeconds);
    SmartDashboard.putBoolean("Calibration/CapturePermitted", capturePermitted);
    if (jitterCaptureActive && !DriverStation.isDisabled()) {
      jitterCaptureActive = false;
      DriverStation.reportWarning(
          "Camera jitter capture stopped because the robot left disabled mode.", false);
    }
    for (int i = 0; i < io.length; i++) {
      double ioStartSeconds = Timer.getFPGATimestamp();
      io[i].updateInputs(inputs[i]);
      Logger.recordOutput(
          "Vision/Timing/Camera" + i + "IoUpdateMs",
          (Timer.getFPGATimestamp() - ioStartSeconds) * 1000.0);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      SmartDashboard.putNumberArray("Calibration/" + cameraNames[i] + "/FieldToCamera",
          capturePermitted ? inputs[i].rawFieldToCamera : new double[0]);
    }

    // Idea: 6328 -- restart the timer whenever not in enabled auto; suppress vision for the first
    // AUTO_VISION_IGNORE_SECONDS of autonomous so a stray frame cannot yank the known start pose.
    if (!DriverStation.isAutonomousEnabled()) {
      autoTimer.restart();
    }
    boolean acceptDuringAuto =
        VisionPolicy.shouldAcceptDuringAuto(DriverStation.isAutonomousEnabled(), autoTimer.get());

    List<Pose3d> allAccepted = new LinkedList<>();
    List<Pose3d> allTrigSolved = new LinkedList<>();
    List<Pose3d> allSuppressed = new LinkedList<>();
    List<Pose3d> allResetSuppressed = new LinkedList<>();
    List<Pose3d> allRejected = new LinkedList<>();
    List<Pose3d> allTagPoses = new LinkedList<>();
    Pose2d currentEstimate = robotPoseSupplier.get();
    double lastResetTime = lastResetTimeSupplier.getAsDouble();
    double now = Timer.getTimestamp();

    for (int cam = 0; cam < io.length; cam++) {
      disconnectedAlerts[cam].set(!inputs[cam].connected);

      for (int tagId : inputs[cam].tagIds) {
        VisionConstants.FIELD_LAYOUT.getTagPose(tagId).ifPresent(allTagPoses::add);
      }

      int accepted = 0;
      int rejected = 0;
      double fusionDurationMs = 0.0;
      for (var obs : inputs[cam].poseObservations) {
        RejectionReason reason = VisionPolicy.timestampRejectionReason(
            obs.timestamp(), now, lastFrameTimestamps[cam]);
        if (reason == RejectionReason.ACCEPTED) {
          lastFrameTimestamps[cam] = obs.timestamp();
          reason = cameraFusionEnabled[cam] ? VisionPolicy.rejectionReason(obs)
              : RejectionReason.UNCALIBRATED_OR_LAYOUT_UNCONFIRMED;
        }
        if (reason != RejectionReason.ACCEPTED) {
          allRejected.add(obs.pose());
          rejected++;
          Logger.recordOutput("Vision/Camera" + cam + "/LastRejectionReason", reason.toString());
          continue;
        }

        // Reset gate: discard a frame captured before the last reset (timestamp check) OR any frame during
        // a short quarantine window right after a reset. The 2026-07-01 sim log showed queued/sim-delayed
        // frames whose timestamp slipped just past the reset still bouncing the fresh pose back; the
        // quarantine catches those. Both parts are self-limiting once the vision stream tracks the new pose.
        if (VisionPolicy.isResetSuppressed(
            obs.timestamp(), lastResetTime, now, VisionConstants.RESET_QUARANTINE_SECONDS)) {
          allResetSuppressed.add(obs.pose());
          continue;
        }

        // Early-auto gate (6328): a validated frame is NOT fused during the first
        // AUTO_VISION_IGNORE_SECONDS of autonomous, so a stray early frame cannot move the known start
        // pose. Suppressed poses are logged on their OWN channel (not AcceptedPoses) so a log reader can
        // tell "validated but withheld" from "actually fused."
        if (!acceptDuringAuto) {
          allSuppressed.add(obs.pose());
          continue;
        }

        // Single-tag strategy (2026-07-16): in TRIG_SOLVE mode, replace the single-tag PnP pose's XY
        // with the trig solution (camera-to-tag translation + odometry-buffer heading at the frame
        // timestamp). Falls back to the PnP pose when the heading buffer or tag lookup cannot answer.
        // The gates above ran on the PnP pose, so both modes fuse the SAME frames -- a fair A/B.
        Pose3d fusedPose = obs.pose();
        boolean usedTrigSolve = false;
        if (obs.tagCount() == 1 && singleTagStrategy == SingleTagStrategy.TRIG_SOLVE) {
          Optional<Pose2d> solved = trigSolve(cam, obs);
          if (solved.isPresent()) {
            fusedPose = new Pose3d(solved.get());
            usedTrigSolve = true;
            allTrigSolved.add(fusedPose);
          }
        }
        // Recheck reconstructed XY; passing the original PnP gate does not validate a trig result.
        if (usedTrigSolve && VisionPolicy.rejectionReason(new VisionIO.PoseObservation(
            obs.timestamp(), fusedPose, obs.ambiguity(), obs.tagCount(),
            obs.averageTagDistance(), obs.primaryTagId())) != RejectionReason.ACCEPTED) {
          allRejected.add(fusedPose);
          rejected++;
          continue;
        }

        boolean multiTag = obs.tagCount() >= 2;
        boolean seedRotationEligible =
            multiTag && VisionPolicy.cameraRotationTrustEnabled(cam);
        boolean trustRotation =
            VisionPolicy.shouldFuseRotation(cam, obs.tagCount(), DriverStation.isEnabled());
        Matrix<N3, N1> stdDevs = selectStandardDeviations(cam, obs, fusedPose, trustRotation);

        // Deliberate static calibration capture: accepted MultiTag observations only. Never infer
        // camera jitter from a moving trajectory because real robot motion and latency would inflate
        // the result. The dashboard start command is disabled-only and the operator holds the robot
        // stationary until each fixed-count accumulator freezes.
        if (jitterCaptureActive && DriverStation.isDisabled() && multiTag) {
          jitterAccumulators[cam].add(fusedPose.toPose2d());
        }

        // Preserve an eligible MultiTag pose for the operator's explicit manual seed even when
        // running estimator fusion is configured XY-only while enabled.
        if (seedRotationEligible && obs.timestamp() >= latestTrustedPoseTimestamp) {
          latestTrustedPose = fusedPose.toPose2d();
          latestTrustedPoseTimestamp = obs.timestamp();
        }

        double fusionStartSeconds = Timer.getFPGATimestamp();
        consumer.accept(fusedPose.toPose2d(), obs.timestamp(), stdDevs);
        lastFusedTimestamp = Math.max(lastFusedTimestamp, obs.timestamp());
        fusionDurationMs += (Timer.getFPGATimestamp() - fusionStartSeconds) * 1000.0;
        accepted++;
        // AcceptedPoses == frames actually fused (matches the AcceptedFrames count below).
        allAccepted.add(fusedPose);

        // Pragmatic 3467-style innovation signal: how far this accepted frame pulled us.
        double innovationMeters =
            fusedPose.toPose2d().getTranslation().getDistance(currentEstimate.getTranslation());
        Logger.recordOutput("Vision/Camera" + cam + "/LastInnovationMeters", innovationMeters);
        Logger.recordOutput("Vision/Camera" + cam + "/LastAcceptedPose", fusedPose.toPose2d());
        Logger.recordOutput("Vision/Camera" + cam + "/LastTrustedRotation", trustRotation);
        Logger.recordOutput("Vision/Camera" + cam + "/LastUsedTrigSolve", usedTrigSolve);
      }

      Logger.recordOutput("Vision/Camera" + cam + "/AcceptedFrames", accepted);
      Logger.recordOutput("Vision/Camera" + cam + "/RejectedFrames", rejected);
      Logger.recordOutput("Vision/Camera" + cam + "/Connected", inputs[cam].connected);
      Logger.recordOutput("Vision/Timing/Camera" + cam + "FusionMs", fusionDurationMs);
      logJitterOutputs(cam);
    }

    boolean comparisonReady =
        jitterAccumulators.length >= 2
            && jitterAccumulators[0].isReady()
            && jitterAccumulators[1].isReady();
    // Compare the first two configured cameras. Jitter is repeatability, not absolute calibration.
    if (jitterCaptureActive && comparisonReady) {
      jitterCaptureActive = false;
    }
    double meanPoseDifferenceMeters = 0.0;
    double meanXDifferenceMeters = 0.0;
    double meanYDifferenceMeters = 0.0;
    double meanYawDifferenceDegrees = 0.0;
    if (comparisonReady) {
      Pose2d camera0Mean = jitterAccumulators[0].getMeanPose();
      Pose2d camera1Mean = jitterAccumulators[1].getMeanPose();
      meanXDifferenceMeters = camera0Mean.getX() - camera1Mean.getX();
      meanYDifferenceMeters = camera0Mean.getY() - camera1Mean.getY();
      meanPoseDifferenceMeters =
          camera0Mean.getTranslation().getDistance(camera1Mean.getTranslation());
      meanYawDifferenceDegrees =
          camera0Mean.getRotation().minus(camera1Mean.getRotation()).getDegrees();
    }
    Logger.recordOutput("Vision/JitterCapture/Active", jitterCaptureActive);
    Logger.recordOutput(
        "Vision/JitterCapture/TargetSamples", VisionConstants.CAMERA_JITTER_CAPTURE_SAMPLES);
    Logger.recordOutput("Vision/JitterCapture/ComparisonReady", comparisonReady);
    Logger.recordOutput(
        "Vision/JitterCapture/Camera0MinusCamera1MeanPoseDifferenceMeters",
        meanPoseDifferenceMeters);
    Logger.recordOutput(
        "Vision/JitterCapture/Camera0MinusCamera1MeanXMeters", meanXDifferenceMeters);
    Logger.recordOutput(
        "Vision/JitterCapture/Camera0MinusCamera1MeanYMeters", meanYDifferenceMeters);
    Logger.recordOutput(
        "Vision/JitterCapture/Camera0MinusCamera1MeanYawDegrees",
        meanYawDifferenceDegrees);

    Logger.recordOutput("Vision/Summary/AcceptedPoses", allAccepted.toArray(Pose3d[]::new));
    // Subset of AcceptedPoses whose XY came from the trig solver -- lets AdvantageScope overlay the
    // two single-tag strategies directly during A/B runs.
    Logger.recordOutput("Vision/Summary/TrigSolvedPoses", allTrigSolved.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/AutoSuppressedPoses", allSuppressed.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/ResetSuppressedPoses", allResetSuppressed.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RejectedPoses", allRejected.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/AcceptingDuringAuto", acceptDuringAuto);
    // The active experiment modes, logged every loop so every A/B log names its configuration.
    Logger.recordOutput("Vision/Modes/SingleTagStrategy", singleTagStrategy.toString());
    Logger.recordOutput("Vision/Modes/CovarianceModel", covarianceModel.toString());
    Logger.recordOutput(
        "Vision/Modes/FuseRotationWhileEnabled",
        VisionConstants.FUSE_VISION_ROTATION_WHILE_ENABLED);
    // Every tag in the layout, always -- so AdvantageScope can render the whole board even when no
    // camera currently sees a tag. Add /RealOutputs/Vision/Layout/TagPoses in file replay.
    Logger.recordOutput("Vision/Layout/TagPoses", layoutTagPoses);
    Logger.recordOutput(
        "Vision/Timing/PeriodicMs",
        (Timer.getFPGATimestamp() - periodicStartSeconds) * 1000.0);
  }

  private void logJitterOutputs(int cameraIndex) {
    PoseJitterAccumulator accumulator = jitterAccumulators[cameraIndex];
    String prefix = "Vision/Camera" + cameraIndex + "/Jitter/";
    Logger.recordOutput(prefix + "SampleCount", accumulator.getCount());
    Logger.recordOutput(prefix + "Ready", accumulator.isReady());
    Logger.recordOutput(prefix + "MeanPose", accumulator.getMeanPose());
    Logger.recordOutput(prefix + "StdDevXMeters", accumulator.getStdDevX());
    Logger.recordOutput(prefix + "StdDevYMeters", accumulator.getStdDevY());
    Logger.recordOutput(
        prefix + "StdDevTranslationMeters", accumulator.getStdDevTranslation());
    Logger.recordOutput(prefix + "StdDevYawDegrees", accumulator.getStdDevYawDegrees());
    Logger.recordOutput(prefix + "PeakToPeakXMeters", accumulator.getPeakToPeakX());
    Logger.recordOutput(prefix + "PeakToPeakYMeters", accumulator.getPeakToPeakY());
    Logger.recordOutput(
        prefix + "PeakToPeakYawDegrees", accumulator.getPeakToPeakYawDegrees());
    Logger.recordOutput(
        prefix + "ConfiguredXyStdDevFactor", VisionPolicy.cameraFactor(cameraIndex));
    Logger.recordOutput(
        prefix + "ConfiguredAngularStdDevFactor",
        VisionPolicy.angularCameraFactor(cameraIndex));
    Logger.recordOutput(
        prefix + "ConfiguredRotationTrustEnabled",
        VisionPolicy.cameraRotationTrustEnabled(cameraIndex));
  }

  /**
   * Attempts the trig solve for a single-tag observation: reconstruct the camera-to-tag transform from
   * the logged PnP pose ({@link SingleTagTrigSolver#reconstructCameraToTag}), sample the heading the
   * robot had at the frame timestamp, and re-anchor XY on the known tag pose. Empty (-> PnP fallback)
   * when the tag is not in the layout, the camera index has no configured transform, or the heading
   * buffer cannot answer.
   */
  private Optional<Pose2d> trigSolve(int cameraIndex, VisionIO.PoseObservation obs) {
    if (cameraIndex >= VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS.length) {
      return Optional.empty();
    }
    Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(obs.primaryTagId());
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }
    Optional<Rotation2d> heading = headingSampler.headingAt(obs.timestamp());
    if (heading.isEmpty()) {
      return Optional.empty();
    }
    Transform3d robotToCamera = VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS[cameraIndex];
    Transform3d cameraToTag =
        SingleTagTrigSolver.reconstructCameraToTag(obs.pose(), robotToCamera, tagPose.get());
    return Optional.of(
        SingleTagTrigSolver.solve(tagPose.get(), robotToCamera, cameraToTag, heading.get()));
  }

  /**
   * Picks the measurement noise for an accepted frame from the active {@link CovarianceModel}. The
   * anisotropic model needs the field-frame robot->tag ray angle; if the primary tag cannot be found in
   * the layout it degrades gracefully to the isotropic baseline.
   */
  private Matrix<N3, N1> selectStandardDeviations(
      int cameraIndex, VisionIO.PoseObservation obs, Pose3d fusedPose, boolean trustRotation) {
    if (covarianceModel == CovarianceModel.ANISOTROPIC) {
      Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(obs.primaryTagId());
      if (tagPose.isPresent()) {
        double rayAngle =
            Math.atan2(
                tagPose.get().getY() - fusedPose.getY(),
                tagPose.get().getX() - fusedPose.getX());
        return VisionPolicy.anisotropicStandardDeviations(
            cameraIndex, obs.averageTagDistance(), obs.tagCount(), trustRotation, rayAngle);
      }
    }
    return VisionPolicy.standardDeviations(
        cameraIndex, obs.averageTagDistance(), obs.tagCount(), trustRotation);
  }
}
