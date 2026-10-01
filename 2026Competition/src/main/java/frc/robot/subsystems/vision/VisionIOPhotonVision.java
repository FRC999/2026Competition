package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.config.VisionConstants;

/**
 * Real-hardware PhotonVision camera IO.
 *
 * <p>Idea traceability: this is the official AdvantageKit PhotonVision template implementation (shipped
 * by 1768 Nashoba as {@code VisionIOPhotonVision}), adapted to Team 999's custom two-tag layout.
 *
 * <p>Key points the template gets right and Codex's first pass also did:
 *
 * <p>- {@code getAllUnreadResults()} drains <em>every</em> queued frame, not just the latest. PhotonVision
 * can buffer several frames between 50 Hz robot loops. Robot evidence from {@code 716d7881} showed
 * that retaining a persistent FIFO can leave the estimator hundreds of milliseconds behind and is
 * unsafe. The IO therefore drains the complete NetworkTables queue but presents only the newest
 * solvable pose in that burst to the estimator, while logging how many intermediate poses were
 * superseded.
 *
 * <p>- Multi-tag uses the coprocessor's combined PnP solve directly; single-tag is reconstructed from the
 * known tag pose. The robot pose is obtained by composing the field->camera transform with the inverse of
 * the robot->camera transform.
 *
 * <p>What was intentionally removed vs the 1768 template: 1768 hard-codes a blocklist of game-specific
 * REEFSCAPE tag IDs to ignore for single-tag solves. We have no such game tags, so the blocklist is gone;
 * single-tag quality is instead governed by the ambiguity gate in {@link Vision}.
 */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.rawFieldToCamera = new double[0];

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    var unreadResults = camera.getAllUnreadResults();
    inputs.unreadResultCount = unreadResults.size();

    for (var result : unreadResults) {
      // Latest simple target bearing (for future boresight aiming, not for fusion).
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
                true,
                result.getTimestampSeconds());
      } else {
        inputs.latestTargetObservation =
            new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, false, result.getTimestampSeconds());
      }

      if (result.multitagResult.isPresent() && !result.targets.isEmpty()) {
        // Multi-tag: coprocessor solved a combined field->camera transform (!isEmpty guards the divide).
        var multitagResult = result.multitagResult.get();
        // A coprocessor using another layout must not introduce unknown tag IDs into fusion.
        if (multitagResult.fiducialIDsUsed.size() < 2
            || multitagResult.fiducialIDsUsed.stream()
                .anyMatch(id -> VisionConstants.FIELD_LAYOUT.getTagPose(id).isEmpty())) continue;
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        if (inputs.rawFieldToCamera.length == 0
            || result.getTimestampSeconds() > inputs.rawFieldToCamera[0]) {
          var q = fieldToCamera.getRotation().getQuaternion();
          inputs.rawFieldToCamera = new double[] {result.getTimestampSeconds(),
              fieldToCamera.getX(), fieldToCamera.getY(), fieldToCamera.getZ(),
              q.getW(), q.getX(), q.getY(), q.getZ(), multitagResult.fiducialIDsUsed.size()};
        }
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalTagDistance = 0.0;
        for (var id : multitagResult.fiducialIDsUsed) {
          totalTagDistance += VisionConstants.FIELD_LAYOUT.getTagPose(id).orElseThrow()
              .getTranslation().getDistance(fieldToCamera.getTranslation());
        }
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / multitagResult.fiducialIDsUsed.size(),
                // Primary tag = first tag of the combined solve (anchors the anisotropic-covariance
                // ray angle; on the 0.5 m two-tag board the per-tag ray angles are nearly identical).
                multitagResult.fiducialIDsUsed.isEmpty()
                    ? -1
                    : multitagResult.fiducialIDsUsed.get(0)));

      } else if (!result.targets.isEmpty()) {
        // Single-tag: reconstruct robot pose from the known tag pose in our custom layout.
        var target = result.targets.get(0);
        var tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          tagIds.add((short) target.fiducialId);
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm(),
                  // Primary tag = THE tag: lets Vision reconstruct cameraToTarget for the trig-solve
                  // strategy without logging a whole extra Transform3d per observation.
                  target.fiducialId));
        }
      }
    }

    if (poseObservations.isEmpty()) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.supersededPoseObservationCount = 0;
    } else {
      PoseObservation newestObservation = poseObservations.get(0);
      for (PoseObservation candidate : poseObservations) {
        if (candidate.timestamp() > newestObservation.timestamp()) {
          newestObservation = candidate;
        }
      }
      inputs.poseObservations = new PoseObservation[] {newestObservation};
      inputs.supersededPoseObservationCount = poseObservations.size() - 1;
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
