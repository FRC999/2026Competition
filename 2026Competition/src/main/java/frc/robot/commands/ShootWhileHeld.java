package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Generic "shoot while held" command.
 *
 * While scheduled:
 * - sets supervisor shot mode
 * - sets supervisor shootRequested(true)
 * - optionally holds drivetrain still (vx=vy=0) and holds heading captured at
 * press time
 *
 * When the button is released:
 * - shootRequested(false)
 * - restores supervisor mode to MOVING_AUTO
 * - stops drivetrain output if it was holding
 */
public class ShootWhileHeld extends Command {
  private final AutoShootSupervisorSubsystem.ShotMode mode;
  private final boolean holdDriveHeading;

  private final PIDController headingPid;
  private double headingSetpointDeg = 0.0;


  public ShootWhileHeld(
      AutoShootSupervisorSubsystem.ShotMode mode,
      boolean holdDriveHeading) {
    this.mode = mode;
    this.holdDriveHeading = holdDriveHeading;

    // These are the subsystems involved in shooting. Requiring them prevents other
    // commands
    // from fighting shooter/hood/transfer/spindexer/turret while this command is
    // held.
    addRequirements(
        RobotContainer.shooterSubsystem,
        RobotContainer.hoodSubsystem,
        RobotContainer.transferSubsystem,
        RobotContainer.spindexerSubsystem,
        RobotContainer.turretSubsystem);

    // Static modes also "own" the drivetrain so it can be held still.
    if (holdDriveHeading) {
      addRequirements(RobotContainer.driveSubsystem);
    }

    headingPid = new PIDController(
        Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kP,
        Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kI,
        Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kD);
    headingPid.enableContinuousInput(-180.0, 180.0);
  }

  private void applyInvalidShotRumble() {
    var validity = RobotContainer.autoShootSupervisorSubsystem.getSolutionValidity();

    double left = 0.0;
    double right = 0.0;

    if (validity == AutoShootSupervisorSubsystem.SolutionValidity.TURRET_ONLY_INVALID) {
      left = Constants.OperatorConstants.AutoShoot.TURRET_ONLY_INVALID_LEFT_RUMBLE;
    } else if (validity == AutoShootSupervisorSubsystem.SolutionValidity.GLOBAL_INVALID) {
      double phase = Timer.getFPGATimestamp()
          / Constants.OperatorConstants.AutoShoot.GLOBAL_INVALID_PULSE_PERIOD_S;
      boolean pulseOn = (((int) Math.floor(phase)) % 2) == 0;
      right = pulseOn ? Constants.OperatorConstants.AutoShoot.GLOBAL_INVALID_RIGHT_RUMBLE : 0.0;
    }

    RobotContainer.getDriveController().setRumble(RumbleType.kLeftRumble, left);
    RobotContainer.getDriveController().setRumble(RumbleType.kRightRumble, right);
  }

  @Override
  public void initialize() {
    System.out.println("Shoot while held called");
    RobotContainer.autoShootSupervisorSubsystem.setShotMode(mode);
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);

    // Print diagnostics once when manual fixed shot begins
    if (mode == AutoShootSupervisorSubsystem.ShotMode.MANUAL_FIXED) {

      var driveState = RobotContainer.driveSubsystem.getState();
      var pose = driveState.Pose;

      Translation2d hub = RobotContainer.autoShootSupervisorSubsystem
          .getAllianceAwareAimTarget(Constants.FieldTargets.AimTarget.HUB);

      Translation2d turretCenter = pose.getTranslation().plus(
          Constants.OperatorConstants.TurretGeometry.TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
              .rotateBy(pose.getRotation()));

      double distance = turretCenter.getDistance(hub);

      double axis3 = MathUtil.clamp(RobotContainer.getTurretStick().getRawAxis(3), -1.0, 1.0);

      double rpm = Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_BASE_RPM
          + axis3 * Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_RPM_TRIM_RANGE;

      double hood = Constants.OperatorConstants.AutoShoot.MANUAL_FIXED_SHOT_HOOD_DEG;

      // System.out.println("========================================");
      // System.out.println("MANUAL FIXED SHOT");
      // System.out.printf("Distance turret->hub: %.3f m%n", distance);
      // System.out.printf("Hood angle: %.2f deg%n", hood);
      // System.out.printf("Shooter RPM: %.1f%n", rpm);
      // System.out.println("========================================");
      SmartDashboard.putString("Shoot While Held Parameters",
          "Distance to Hub: " + String.format("%.3f", distance) + " Manual Fixed Shot Hood Deg: "
              + String.format("%.2f", hood) + " Manual Fixed Shot Shooter RPM: " + String.format("%.1f", rpm)
              + "Turret Angle: " + String.format("%.2f", RobotContainer.turretSubsystem.getAngleDeg()));
    }
  }

  @Override
  public void execute() {
    applyInvalidShotRumble();

    if (!holdDriveHeading) {
      return;
    }

    final double currentDeg = RobotContainer.driveSubsystem.getYaw();
    double omegaDegPerSec = headingPid.calculate(currentDeg);

    omegaDegPerSec = MathUtil.clamp(
        omegaDegPerSec,
        -Constants.OperatorConstants.AutoShoot.STATIC_HOLD_MAX_OMEGA_DEG_PER_S,
        +Constants.OperatorConstants.AutoShoot.STATIC_HOLD_MAX_OMEGA_DEG_PER_S);

    RobotContainer.driveSubsystem.drive(
        0.0,
        0.0,
        Math.toRadians(omegaDegPerSec));
  }

  @Override
  public void end(boolean interrupted) {
    // Immediately stop feeding by clearing shoot request
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);

    // Return to default moving mode so next RT press behaves normally
    RobotContainer.autoShootSupervisorSubsystem.setShotMode(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO);

    if (holdDriveHeading) {
      RobotContainer.driveSubsystem.drive(0.0, 0.0, 0.0);
    }

    RobotContainer.getDriveController().setRumble(RumbleType.kLeftRumble, 0.0);
    RobotContainer.getDriveController().setRumble(RumbleType.kRightRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false; // while-held
  }
}