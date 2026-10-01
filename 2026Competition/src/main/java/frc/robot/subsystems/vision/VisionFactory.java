package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.OffseasonVisionConfig;
import frc.robot.config.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.nio.file.Path;
import org.littletonrobotics.junction.Logger;

public final class VisionFactory {
  private VisionFactory() {}

  public static Vision create(DriveSubsystem drive) {
    try {
      Path configPath = RobotBase.isSimulation() ? Path.of("simulation/vision.json")
          : Filesystem.getDeployDirectory().toPath().resolve("vision/cameras.json");
      OffseasonVisionConfig config = OffseasonVisionConfig.load(configPath);
      if (RobotBase.isReal() && config.profile().equals("simulation")) {
        throw new IllegalArgumentException("Synthetic simulation calibration cannot run on the robot");
      }
      VisionConstants.configure(config);
      VisionIO[] io = config.cameras().stream().map(camera -> RobotBase.isSimulation()
          ? new VisionIOPhotonVisionSim(camera.name(), camera.robotToCamera(), drive::getSimulationTruthPose)
          : new VisionIOPhotonVision(camera.name(), camera.robotToCamera())).toArray(VisionIO[]::new);
      Vision vision = createWithIO(drive, io);
      vision.configureCameras(config, drive::isStationaryForLocalization);
      SmartDashboard.putBoolean("Vision/ConfigValid", true);
      SmartDashboard.putString("Vision/Profile", config.profile());
      SmartDashboard.putString("Vision/LayoutSHA256", config.layoutSha256());
      SmartDashboard.putBoolean("Vision/LayoutConfirmed", config.layoutConfirmedOnCoprocessors());
      Logger.recordOutput("Vision/Configuration/Profile", config.profile());
      Logger.recordOutput("Vision/Configuration/LayoutSHA256", config.layoutSha256());
      Logger.recordOutput("Vision/Configuration/ConfigSHA256", OffseasonVisionConfig.sha256(java.nio.file.Files.readAllBytes(configPath)));
      Logger.recordOutput("Vision/Configuration/CompetitionAimFrame", vision.hasCompetitionAimFrame());
      if (!config.layoutConfirmedOnCoprocessors()
          || config.cameras().stream().anyMatch(camera -> !camera.calibrated())) {
        DriverStation.reportWarning("Vision requires measured camera extrinsics and matching Pi field layouts; unconfigured cameras cannot fuse.", false);
      }
      return vision;
    } catch (Exception ex) {
      DriverStation.reportError("PhotonVision configuration rejected: " + ex.getMessage(), false);
      SmartDashboard.putBoolean("Vision/ConfigValid", false);
      SmartDashboard.putString("Vision/ConfigurationError", ex.toString());
      // Preserve manual drivetrain operation when a configuration file is missing or invalid.
      return createWithIO(drive, new VisionIO[0]);
    }
  }

  private static Vision createWithIO(DriveSubsystem drive, VisionIO[] io) {
    Vision vision = new Vision(drive::addVisionMeasurement, drive::getPose, drive::getLastPoseResetSeconds,
        timestamp -> drive.getSample(timestamp).map(pose -> pose.getRotation()), io);
    vision.configureLocalization(drive::hasFieldReference, drive::resetPoseFromVision);
    return vision;
    // DriveSubsystem.addVisionMeasurement performs FPGA -> CTRE time conversion exactly once.
  }
}
