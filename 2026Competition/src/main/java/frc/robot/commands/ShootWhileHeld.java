package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

/**
 * Generic "shoot while held" command.
 *
 * While scheduled:
 *  - sets supervisor shot mode
 *  - sets supervisor shootRequested(true)
 *  - optionally holds drivetrain still (vx=vy=0) and holds heading captured at press time
 *
 * When the button is released:
 *  - shootRequested(false)
 *  - restores supervisor mode to MOVING_AUTO
 *  - stops drivetrain output if it was holding
 */
public class ShootWhileHeld extends Command {
  private final AutoShootSupervisorSubsystem.ShotMode mode;
  private final boolean holdDriveHeading;

  private final PIDController headingPid;
  private double headingSetpointDeg = 0.0;

  public ShootWhileHeld(
      AutoShootSupervisorSubsystem.ShotMode mode,
      boolean holdDriveHeading
  ) {
    this.mode = mode;
    this.holdDriveHeading = holdDriveHeading;

    // These are the subsystems involved in shooting. Requiring them prevents other commands
    // from fighting shooter/hood/transfer/spindexer/turret while this command is held.
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
    RobotContainer.autoShootSupervisorSubsystem.setShotMode(mode);
    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);

    if (holdDriveHeading) {
      // Capture heading at the moment the button is pressed
      headingSetpointDeg = RobotContainer.driveSubsystem.getYaw();
      headingPid.reset();
      headingPid.setSetpoint(headingSetpointDeg);
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