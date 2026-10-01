
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.AimGeometry;
import frc.robot.lib.FieldTargeting;

public class DriveManuallyCommand extends Command {
  private final DoubleSupplier mVxSupplier;
  private final DoubleSupplier mVySupplier;
  private final DoubleSupplier mOmegaSupplier;
  private final BooleanSupplier mStationaryShotAutoTurnSupplier;

  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand(
      DoubleSupplier vxSupplier,
      DoubleSupplier vySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier stationaryShotAutoTurnSupplier) {
    addRequirements(RobotContainer.driveSubsystem);

    mVxSupplier = vxSupplier;
    mVySupplier = vySupplier;
    mOmegaSupplier = omegaSupplier;
    mStationaryShotAutoTurnSupplier = stationaryShotAutoTurnSupplier;
  }

  /**
   * This method man be used when troubleshooting controller inputs
   *
   * @param dx
   * @param dy
   * @param dm
   */
  @SuppressWarnings("unused")
  private void driveControlTelemetry(double dx, double dy, double dm) {
  }
  @Override
  public void initialize() {
  }
  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled() || RobotContainer.isPanicStopActive()) {
      RobotContainer.driveSubsystem.stop();
      return;
    }
    double xInput = mVxSupplier.getAsDouble();
    double yInput = mVySupplier.getAsDouble();
    double omegaInput = mOmegaSupplier.getAsDouble();

    boolean stationaryAutoTurnRequested = mStationaryShotAutoTurnSupplier.getAsBoolean();
    double autoTurnRawTurretDeg = Double.NaN;
    double autoTurnRobotHeadingDeltaDeg = Double.NaN;
    double autoTurnOmegaCmd = 0.0; // normalized command [-1, +1]
    double autoTurnOmegaRadPerSec = 0.0; // actual requested chassis omega
    boolean autoTurnActive = false;

    var measured = RobotContainer.driveSubsystem.getRobotRelativeSpeeds();
    if (stationaryAutoTurnRequested && Math.abs(omegaInput) <= 1e-9
        && Math.hypot(xInput, yInput) <= 1e-9
        && Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) < .15
        && DriverStation.getAlliance().isPresent()
        && !RobotContainer.isHubTrackingDisabledByButtonBox()
        && RobotContainer.vision.hasCompetitionAimFrame() && RobotContainer.vision.isLocalizationReady()) {

      Pose2d robotPoseField = RobotContainer.driveSubsystem.getPose();

      Translation2d targetPositionField = FieldTargeting.target(
          RobotContainer.autoShootSupervisorSubsystem.getCurrentAimTarget(),
          DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red);

      autoTurnRawTurretDeg = AimGeometry.turretDegrees(
          robotPoseField,
          targetPositionField);

      double comfort = Constants.OperatorConstants.AutoShoot.STATIONARY_ILLEGAL_SHOT_COMFORT_MARGIN_DEG;
      autoTurnRawTurretDeg += Constants.OperatorConstants.Turret.AUTO_AIM_TRIM_DEG;
      autoTurnRobotHeadingDeltaDeg = AimGeometry.chassisTurnToWindow(autoTurnRawTurretDeg,
          Constants.OperatorConstants.Turret.MIN_ANGLE_DEG + comfort,
          Constants.OperatorConstants.Turret.MAX_ANGLE_DEG - comfort);
      autoTurnOmegaCmd = Double.isFinite(autoTurnRobotHeadingDeltaDeg)
          ? Math.signum(autoTurnRobotHeadingDeltaDeg) : 0;
      autoTurnOmegaRadPerSec =
          autoTurnOmegaCmd
              * Constants.OperatorConstants.AutoShoot.STATIONARY_ILLEGAL_SHOT_FIXED_AUTO_TURN_RAD_PER_SEC;

      if (Math.abs(autoTurnOmegaRadPerSec) > 1e-9) {
        omegaInput = autoTurnOmegaRadPerSec / SwerveConstants.MaxAngularRate;
        autoTurnActive = true;
      }
    }

    if(DebugTelemetrySubsystems.chassis){
      SmartDashboard.putNumber("Drive/StationaryAutoTurnOmegaRadPerSec", autoTurnOmegaRadPerSec);
      SmartDashboard.putBoolean("Drive/StationaryAutoTurnRequested", stationaryAutoTurnRequested);
      SmartDashboard.putBoolean("Drive/StationaryAutoTurnActive", autoTurnActive);
      SmartDashboard.putNumber("Drive/StationaryAutoTurnRawTurretDeg", autoTurnRawTurretDeg);
      SmartDashboard.putNumber("Drive/StationaryAutoTurnRobotDeltaDeg", autoTurnRobotHeadingDeltaDeg);
      SmartDashboard.putNumber("Drive/StationaryAutoTurnOmegaCmd", autoTurnOmegaCmd);
    }

    if (Math.hypot(xInput, yInput) <= 1e-9
        && Math.abs(omegaInput) <= 1e-9) {
      RobotContainer.driveSubsystem.stop();
      return;
    }
    if (!RobotContainer.driveSubsystem.getRobotCentric()) {
      RobotContainer.driveSubsystem.drive(
          xInput * SwerveConstants.MaxSpeed,
          yInput * SwerveConstants.MaxSpeed,
          omegaInput * SwerveConstants.MaxAngularRate);
    } else {
      RobotContainer.driveSubsystem.driveRobotCentric(
          xInput * SwerveConstants.MaxSpeed,
          yInput * SwerveConstants.MaxSpeed,
          omegaInput * SwerveConstants.MaxAngularRate);
    }
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.stop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
