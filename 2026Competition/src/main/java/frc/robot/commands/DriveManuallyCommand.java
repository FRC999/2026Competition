// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.TurretHelpers;

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
    //System.out.print("DX " + dx);
    //System.out.print(" DY " + dy);
    //System.out.println(" Dm " + dm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("stationary bool: **** " + mStationaryShotAutoTurnSupplier.getAsBoolean());
    double xInput = mVxSupplier.getAsDouble();
    double yInput = mVySupplier.getAsDouble();
    double omegaInput = mOmegaSupplier.getAsDouble();

    boolean stationaryAutoTurnRequested = mStationaryShotAutoTurnSupplier.getAsBoolean();

    // getDriverOmegaAxis() scales the right-stick X by 0.6, so match that scale
    // here.
    double omegaDeadband = Constants.OperatorConstants.AutoShoot.STATIONARY_ASSIST_OMEGA_DEADBAND;
    double autoTurnRawTurretDeg = Double.NaN;
    double autoTurnRobotHeadingDeltaDeg = Double.NaN;
    double autoTurnOmegaCmd = 0.0; // normalized command [-1, +1]
    double autoTurnOmegaRadPerSec = 0.0; // actual requested chassis omega
    boolean autoTurnActive = false;

    //suchita test
    //System.out.println("o1: " + omegaInput);

    if (stationaryAutoTurnRequested && Math.abs(omegaInput) <= omegaDeadband) {

      Pose2d robotPoseField = RobotContainer.driveSubsystem.getPose();

      Translation2d targetPositionField = TurretHelpers.aimTargetToFieldTranslation(
          RobotContainer.autoShootSupervisorSubsystem.getCurrentAimTarget(),
          RobotContainer.isAllianceRed);

          //   System.out.println("AimTarget=" + RobotContainer.autoShootSupervisorSubsystem.getCurrentAimTarget()
          // + " allianceRed=" + RobotContainer.isAllianceRed);

      autoTurnRawTurretDeg = TurretHelpers.computeStationaryRawTurretYawDeg(
          robotPoseField,
          targetPositionField);

      // System.out.println("AD:"+autoTurnRawTurretDeg + " RP:"+robotPoseField.toString()+ "TP:"+targetPositionField.toString());

      double thresholdDeg = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG;

      if (autoTurnRawTurretDeg < 0 && autoTurnRawTurretDeg < -thresholdDeg) {
        autoTurnOmegaRadPerSec =
            -Constants.OperatorConstants.AutoShoot.STATIONARY_ILLEGAL_SHOT_FIXED_AUTO_TURN_RAD_PER_SEC;
      } else if (autoTurnRawTurretDeg >= 0 && autoTurnRawTurretDeg > thresholdDeg) {
        autoTurnOmegaRadPerSec =
            Constants.OperatorConstants.AutoShoot.STATIONARY_ILLEGAL_SHOT_FIXED_AUTO_TURN_RAD_PER_SEC;
      } else {
        autoTurnOmegaRadPerSec = 0.0;
      }

      autoTurnOmegaCmd = autoTurnOmegaRadPerSec / SwerveConstants.MaxAngularRate;
      autoTurnRobotHeadingDeltaDeg = autoTurnRawTurretDeg;

      // alex test
      // System.out.println(
      //     "[StationaryAutoTurn] rawTurretDeg=" + autoTurnRawTurretDeg
      //         + " thresholdDeg=" + thresholdDeg
      //         + " omegaRadPerSec=" + autoTurnOmegaRadPerSec
      //         + " omegaNormalized=" + autoTurnOmegaCmd
      //         + " driverOmegaInput=" + omegaInput
      //         + " omegaDeadband=" + omegaDeadband);

      if (Math.abs(autoTurnOmegaRadPerSec) > 1e-9) {
        omegaInput = autoTurnOmegaCmd;
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

    if (!RobotContainer.driveSubsystem.getRobotCentric()) {
      RobotContainer.driveSubsystem.drive(
          xInput * SwerveConstants.MaxSpeed,
          yInput * SwerveConstants.MaxSpeed,
          omegaInput * SwerveConstants.MaxAngularRate);
    } else {
      RobotContainer.driveSubsystem.driveRobotCentric(
          xInput * SwerveConstants.MaxSpeed,
          0,
          omegaInput * SwerveConstants.MaxAngularRate);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
