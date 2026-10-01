
package frc.robot;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.commands.AutoMainOneRightBlue;
import frc.robot.commands.AutoMainOneRightRed;
import frc.robot.commands.AutoShootOnly;
import frc.robot.commands.AutoWorldsHubSweepBlue;
import frc.robot.commands.AutoWorldsHubSweepRed;
import frc.robot.commands.AutoBlueHubSimpleMoveAndShoot;
import frc.robot.commands.AutoBlueMiddleToOutpostAndShoot;
import frc.robot.commands.AutoBlueTrenchToOutpostAndShoot;
import frc.robot.commands.DeployIntakeSequence;
import frc.robot.commands.DeployAndRunIntakeWhileHeld;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.InitialAutoDeployWhileHeld;
import frc.robot.commands.IntakeRezeroFromRetractedHardStop;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseShooterTemporary;
import frc.robot.commands.ShootWhileHeld;
import frc.robot.commands.StopIntake;
import frc.robot.commands.TurretGoToZeroCommand;
import frc.robot.commands.TurretJogCommand;
import frc.robot.lib.ElasticHelpers;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.PrintTurretShotDiagnosticsCommand;
import frc.robot.commands.RetractIntakeSequence;

/**
 * One process-wide robot assembly: hardware, vision, command factories and operator bindings.
 * Commands express intent; subsystem guards still apply to every output. Deferred auto factories
 * resolve alliance/pose when scheduled, and the cached outer command supplies the 20-second deadline.
 * Operator release actions check teleop/panic before taking requirements. Static CAN/logger ownership
 * means a full robot integration test belongs in a separate JVM.
 */
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */

  private static Controller xboxDriveController = new Controller(OIContants.XBOX_CONTROLLER);

  private static Joystick turretStick = null;

  public static final Joystick bb = new Joystick(OIContants.BUTTON_BOX);
  private static boolean panicStopLatched = false;

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static final frc.robot.subsystems.vision.Vision vision =
      frc.robot.subsystems.vision.VisionFactory.create(driveSubsystem);
  public static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static TransferSubsystem transferSubsystem = new TransferSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public static AutoShootSupervisorSubsystem autoShootSupervisorSubsystem = new AutoShootSupervisorSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final java.util.Map<Command, Command> boundedAutos = new java.util.IdentityHashMap<>();

  public static void seedPoseFromPhotonVision() {
    if (!DriverStation.isDisabled()) return;
    vision.getFreshTrustedSeedPose().ifPresentOrElse(driveSubsystem::resetPoseFromVision,
        () -> DriverStation.reportWarning("No fresh calibrated MultiTag pose available for seeding.", false));
  }

  public RobotContainer() {
    configureBindings();

        driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis(),
            () -> xboxDriveController.getRawAxis(3) > 0.3));
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    AutonomousConfigure();
    frc.robot.commands.DriveToPosePrecisionCommand.primeTelemetrySchema();
    autoChooser.setDefaultOption("OffSeason: Do nothing", Commands.run(driveSubsystem::stop, driveSubsystem));
    autoChooser.addOption("OffSeason: Precision 1m forward (clear area)", Commands.defer(() -> {
      if (!vision.isLocalizationReady()) return frc.robot.commands.PrecisionPathCommands.failedHold(
          driveSubsystem, "Precision test requires an established field pose and fresh vision");
      Pose2d start = driveSubsystem.getPose();
      Pose2d goal = start.transformBy(new edu.wpi.first.math.geometry.Transform2d(1, 0, Rotation2d.kZero));
      return new frc.robot.commands.DriveToPosePrecisionCommand(driveSubsystem, goal)
          .withFinishPermission(vision::isLocalizationReady);
    }, java.util.Set.of(driveSubsystem)));
    SmartDashboard.putData("Diagnostics/Shot plan", new PrintTurretShotDiagnosticsCommand().ignoringDisable(true));
    SmartDashboard.putData("Vision/Seed pose (disabled)",
        Commands.runOnce(RobotContainer::seedPoseFromPhotonVision).ignoringDisable(true));
    SmartDashboard.putData("Vision/Capture camera jitter (disabled)",
        Commands.runOnce(vision::startCameraJitterCapture).ignoringDisable(true));
    SmartDashboard.putData("Vision/Stop camera jitter capture",
        Commands.runOnce(vision::stopCameraJitterCapture).ignoringDisable(true));
    SmartDashboard.putData("Calibration/Reseed turret ONLY when physically stowed (disabled)",
        Commands.runOnce(turretSubsystem::reseedIntegratedFromAbsoluteNow).ignoringDisable(true));
    SmartDashboard.putData("Calibration/Zero hood ONLY when fully down (disabled)",
        Commands.runOnce(hoodSubsystem::seedZeroFromDownHardStop).ignoringDisable(true));
    if (RobotBase.isSimulation()) {
      configureSimulation();
    }
  }

  private static void configureSimulation() {
    driveSubsystem.placeSimulationRobot(new Pose2d(3, 3, Rotation2d.kZero));
    driveSubsystem.resetPose(new Pose2d(3.15, 3.1, Rotation2d.kZero));
  }

  public static void AutonomousConfigure() {
    SmartDashboard.putData(autoChooser);
     autoChooser.addOption("AutoMainOneRightBlue", new AutoMainOneRightBlue());
     autoChooser.addOption("AutoMainOneRightRed", new AutoMainOneRightRed());
     autoChooser.addOption("AutoWorldsHubSweepBlue", new AutoWorldsHubSweepBlue());
     autoChooser.addOption("AutoWorldsHubSweepRed", new AutoWorldsHubSweepRed());
    autoChooser.addOption("Alliance - Trench to outpost and shoot", new AutoBlueTrenchToOutpostAndShoot());

     autoChooser.addOption("Alliance - Middle to outpost and shoot", new AutoBlueMiddleToOutpostAndShoot());
    autoChooser.addOption("Auto Shoot Only", new AutoShootOnly());

     autoChooser.addOption("Alliance - Move from hub and shoot", new AutoBlueHubSimpleMoveAndShoot());
  }

  private void configureBindings() {
    RobotModeTriggers.disabled()
        .whileTrue(driveSubsystem.applyRequest(driveSubsystem::getIdle).ignoringDisable(true));
    competitionXBOXButtonBindings();
  }

  public static Controller getDriveController() {
    return xboxDriveController;
  }

 public static Joystick getTurretStick() {
    if (turretStick == null) {
      turretStick = new Joystick(OIContants.TEST_JOYSTICK_PORT);
    }
    return turretStick;
  }

  public static boolean isHubTrackingDisabledByButtonBox() {
    return DriverStation.isTeleopEnabled() && bb.getRawAxis(OIContants.BB_HUB_TRACKING_DISABLE_AXIS)
        < OIContants.BB_HUB_TRACKING_DISABLE_THRESHOLD;
  }

  private static boolean isPanicSwitchActive() {
    return bb.getRawAxis(OIContants.BB_PANIC_STOP_AXIS)
        > OIContants.BB_PANIC_STOP_THRESHOLD;
  }

  private static boolean isVisionSeedAxisActive() {
    return bb.getRawAxis(OIContants.BB_VISION_SEED_AXIS)
        < OIContants.BB_VISION_SEED_AXIS_VALUE;
  }

  public static boolean isPanicStopActive() {
    return panicStopLatched || isPanicSwitchActive();
  }

  private static void applyPanicStop() {
    driveSubsystem.stop();
    if (Constants.EnabledSubsystems.supervisor) {
      autoShootSupervisorSubsystem.setShootRequested(false);
    }
    if (Constants.EnabledSubsystems.shooter) {
      shooterSubsystem.stop();
    }
    if (Constants.EnabledSubsystems.transfer) {
      transferSubsystem.stop();
    }
    if (Constants.EnabledSubsystems.spindexer) {
      spindexerSubsystem.stop();
    }
    if (Constants.EnabledSubsystems.turret) {
      turretSubsystem.stop();
    }
    if (Constants.EnabledSubsystems.hood) {
      hoodSubsystem.stop();
    }
    if (Constants.EnabledSubsystems.intake) {
      intakeSubsystem.applyPanicStop();
    }
    if (Constants.EnabledSubsystems.climber) {
      climbSubsystem.stopMotors();
    }
  }

  private static void cancelAllCommandsForPanicStop() {
    CommandScheduler.getInstance().cancelAll();
    applyPanicStop();
  }

  private static Trigger operatorControl(java.util.function.BooleanSupplier pressed) {
    var gate = new frc.robot.lib.FreshPress();
    return new Trigger(() -> gate.update(isTeleopControlAllowed(), pressed.getAsBoolean()));
  }

  private static boolean isTeleopControlAllowed() {
    return DriverStation.isTeleopEnabled() && !isPanicStopActive();
  }

  /** A mode/panic transition is also a falling edge. It must not schedule a cleanup move. */
  private static Command onTeleopRelease(Command release) {
    // Check before scheduling: onlyIf on a command with requirements would still cancel auto.
    return Commands.runOnce(() -> { if (isTeleopControlAllowed()) CommandScheduler.getInstance().schedule(release); });
  }

  private void competitionXBOXButtonBindings() {
    panicStopLatched = isPanicSwitchActive();
    if (panicStopLatched) {
      cancelAllCommandsForPanicStop();
    }

    Trigger panicStopTrigger = new Trigger(RobotContainer::isPanicSwitchActive);
    Trigger teleopControls = new Trigger(RobotContainer::isTeleopControlAllowed);
    Trigger povUpTrigger = operatorControl(() -> xboxDriveController.getPOV() == 0);
    Trigger povDownTrigger = operatorControl(() -> xboxDriveController.getPOV() == 180);
    Trigger driverAButton = operatorControl(() -> xboxDriveController.getRawButton(OIContants.XBOX_BUTTON_A));
    Trigger driverYButton = operatorControl(() -> xboxDriveController.getRawButton(4));

    panicStopTrigger
        .onTrue(new InstantCommand(() -> {
          panicStopLatched = true;
          cancelAllCommandsForPanicStop();
        }).ignoringDisable(true))
        .onFalse(new InstantCommand(() -> panicStopLatched = false).ignoringDisable(true));

    new JoystickButton(bb, OIContants.BB_VISION_SEED_BUTTON)
        .and(new Trigger(RobotContainer::isVisionSeedAxisActive))
        .and(new Trigger(DriverStation::isDisabled))
        .and(() -> !isPanicStopActive())
        .onTrue(Commands.runOnce(RobotContainer::seedPoseFromPhotonVision, driveSubsystem)
            .ignoringDisable(true));

   operatorControl(() -> xboxDriveController.getRawAxis(OIContants.XBOX_LEFT_TRIGGER_AXIS)
        > OIContants.XBOX_TRIGGER_ACTIVE_THRESHOLD)
        .and(teleopControls)
        .whileTrue(Commands.defer(
            DeployAndRunIntakeWhileHeld::new,
            Set.of(intakeSubsystem)))
        .onFalse(onTeleopRelease(Commands.defer(
            () -> intakeSubsystem.shouldStayDeployedAfterTriggerRelease()
                ? new StopIntake()
                : new RetractIntakeSequence(),
            Set.of(intakeSubsystem))));

    driverAButton
        .and(teleopControls)
        .and(new Trigger(() -> xboxDriveController.getPOV() != 180))
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(true),
            intakeSubsystem).andThen(new DeployIntakeSequence()));

    povDownTrigger
        .and(driverAButton)
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(true),
            intakeSubsystem).andThen(new DeployIntakeSequence(true)));

    driverYButton // Y
        .and(teleopControls)
        .and(new Trigger(() -> xboxDriveController.getPOV() != 0))
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(false),
            intakeSubsystem).andThen(new RetractIntakeSequence()));

    povUpTrigger
        .and(driverYButton)
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(false),
            intakeSubsystem).andThen(new RetractIntakeSequence(true)));

    operatorControl(() -> xboxDriveController.getRawButton(2))
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MANUAL_PRESET_3M,
            false));

    operatorControl(() -> bb.getRawButton(OIContants.BB_MANUAL_RPM_UP))
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT +=
                Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_STEP_PERCENT));

    operatorControl(() -> bb.getRawButton(OIContants.BB_MANUAL_SHOT_3M))
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.Turret.AUTO_AIM_TRIM_DEG += 1.0));

    operatorControl(() -> xboxDriveController.getRawButton(3))
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MANUAL_PRESET_4M,
            false));

    operatorControl(() -> bb.getRawButton(OIContants.BB_MANUAL_SHOT_4M))
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.Turret.AUTO_AIM_TRIM_DEG -= 1.0));

    operatorControl(() -> bb.getRawButton(OIContants.BB_INTAKE_INIT_DEPLOY_6))
        .and(teleopControls)
        .whileTrue(Commands.defer(
            InitialAutoDeployWhileHeld::new,
            Set.of(intakeSubsystem)))
        .onFalse(onTeleopRelease(Commands.defer(
            RetractIntakeSequence::new,
            Set.of(intakeSubsystem))));

    operatorControl(() -> bb.getRawButton(OIContants.BB_MANUAL_RPM_DOWN))
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT -=
                Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_STEP_PERCENT));

    operatorControl(() -> xboxDriveController.getRawButton(5)) // LB
        .and(teleopControls)
        .whileTrue(new ReverseIntake());

    operatorControl(() -> xboxDriveController.getRawButton(6)) // RB
        .and(teleopControls)
        .whileTrue(new ReverseShooterTemporary());

    operatorControl(() -> xboxDriveController.getRawButton(8))
        .and(new Trigger(DriverStation::isTeleopEnabled))
        .and(teleopControls)
        .onTrue(Commands.runOnce(driveSubsystem::orientDriverForwardToCurrentHeading, driveSubsystem));

    operatorControl(() -> xboxDriveController.getRawAxis(3) > 0.3) // RT
        .and(teleopControls)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO,
            false));

    operatorControl(() -> xboxDriveController.getRawButton(2)) // B
        .and(new Trigger(() -> !RobotContainer.isHubTrackingDisabledByButtonBox()))
        .and(teleopControls)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.STATIC_TOWER_BASE,
            true));

    operatorControl(() -> xboxDriveController.getPOV() == 90)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .whileTrue(new TurretJogCommand(turretSubsystem, 0.18));

    operatorControl(() -> xboxDriveController.getPOV() == 270)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(teleopControls)
        .whileTrue(new TurretJogCommand(turretSubsystem, -0.18));

    operatorControl(() -> bb.getRawButton(OIContants.BB_INTAKE_REZERO))
      .and(teleopControls)
      .onTrue(new IntakeRezeroFromRetractedHardStop());

    operatorControl(() -> bb.getRawButton(OIContants.BB_TURRET_ZERO))
      .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
      .and(teleopControls)
      .onTrue(new TurretGoToZeroCommand());

  }

  private double getDriverXAxis() {
    return -xboxDriveController.getLeftStickY();
  }

  private double getDriverYAxis() {
    return -xboxDriveController.getLeftStickX();
  }

  private double getDriverOmegaAxis() {
    return -xboxDriveController.getRightStickX();
  }

  /**
   * Defers path loading/frame resolution until scheduling, validates the competition frame and start,
   * then follows with brake-only route qualification. resetToStart requires known physical placement;
   * normal competition callers keep it false. Errors produce a latched hold, never a skipped segment.
   */
  public static Command followCompetitionPath(String name,
      boolean resetToStart, frc.robot.commands.PrecisionPathCommands.FieldFrame frame) {
    return Commands.defer(() -> {
      try {
        if (!vision.hasCompetitionAimFrame() || DriverStation.getAlliance().isEmpty()) {
          return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
              "Named paths require the welded competition frame and a known alliance");
        }
        PathPlannerPath path = frc.robot.commands.PrecisionPathCommands.inFieldFrame(
            PathPlannerPath.fromPathFile(name), frame,
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red);
        boolean red = DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red;
        if (path.getAllPathPoints().stream().anyMatch(point -> !frc.robot.lib.FieldRules.onOwnAutoHalf(
            new Pose2d(point.position, Rotation2d.kZero), red))) {
          return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
              "Auto path crosses the conservative G403 center boundary: " + name);
        }
        if (!resetToStart && driveSubsystem.getPose().getTranslation().getDistance(
            path.getStartingHolonomicPose().orElseThrow().getTranslation()) > .35) {
          return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
              "Robot is too far from the start of path " + name);
        }
        ElasticHelpers.setAutoPathSingle(path);
        return frc.robot.commands.PrecisionPathCommands.followResolved(driveSubsystem, path,
            resetToStart, vision::isLocalizationReady,
            frc.robot.commands.PrecisionPathCommands.FinishPolicy.ROUTE_STOP);
      } catch (Exception ex) {
        return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
            "Cannot execute path " + name + ": " + ex.getMessage());
      }
    }, java.util.Set.of(driveSubsystem));
  }

  /** Approach the same resolved starting pose the first named path will use. */
  public static Command approachCompetitionPath(String name) {
    return Commands.defer(() -> {
      try {
        if (!vision.hasCompetitionAimFrame() || DriverStation.getAlliance().isEmpty()) {
          return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
              "Path approach requires the competition frame and known alliance");
        }
        var path = frc.robot.commands.PrecisionPathCommands.inFieldFrame(
            PathPlannerPath.fromPathFile(name), frc.robot.commands.PrecisionPathCommands.FieldFrame.ALLIANCE,
            DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red);
        Pose2d start = driveSubsystem.getPose();
        Pose2d end = path.getStartingHolonomicPose().orElseThrow();
        boolean red = DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red;
        // This opening approach is the straight trench corridor, not arbitrary pathfinding.
        if (Math.abs(start.getY() - end.getY()) > .25
            || start.getTranslation().getDistance(end.getTranslation()) > 3.0
            || frc.robot.lib.FieldRules.allianceX(start, red) > frc.robot.lib.FieldRules.ALLIANCE_ZONE_DEPTH_METERS) {
          return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
              "Opening auto requires placement in its starting trench corridor");
        }
        return runTrajectory2Poses(start, end,
            frc.robot.commands.PrecisionPathCommands.FinishPolicy.ROUTE_STOP);
      } catch (Exception ex) {
        return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
            "Cannot approach path " + name + ": " + ex.getMessage());
      }
    }, Set.of(driveSubsystem));
  }

  public static Command runTrajectory2Poses(Pose2d startPose, Pose2d endPose) {
    return runTrajectory2Poses(startPose, endPose,
        frc.robot.commands.PrecisionPathCommands.FinishPolicy.PRECISION_ALIGNMENT);
  }

  private static Command runTrajectory2Poses(Pose2d startPose, Pose2d endPose,
      frc.robot.commands.PrecisionPathCommands.FinishPolicy policy) {
    if (startPose.getTranslation().getDistance(endPose.getTranslation()) < 0.01) {
      Command finish = frc.robot.commands.PrecisionPathCommands.finishAt(driveSubsystem, endPose,
          vision::isLocalizationReady, policy);
      return Commands.either(finish, frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
          "Generated move requires a referenced pose"), vision::isLocalizationReady);
    }
    try {
      Rotation2d tangent = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(startPose.getTranslation(), tangent), new Pose2d(endPose.getTranslation(), tangent));
      PathPlannerPath path = new PathPlannerPath(waypoints, AutoConstants.pathConstraints,
          new IdealStartingState(0, startPose.getRotation()), new GoalEndState(0, endPose.getRotation()));
      path.preventFlipping = true; // Caller supplied absolute field coordinates.
      return frc.robot.commands.PrecisionPathCommands.followResolved(driveSubsystem, path,
          false, vision::isLocalizationReady, policy);
    } catch (Exception ex) {
      return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
          "Cannot generate two-pose path: " + ex.getMessage());
    }
  }

  /** Returns a reusable, once-composed deadline wrapper; cleanup runs on timeout and mode cancellation. */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    return selected == null ? Commands.run(driveSubsystem::stop, driveSubsystem)
        : boundedAutos.computeIfAbsent(selected, command -> command.withTimeout(AutoConstants.AUTO_PERIOD_SECONDS).finallyDo(interrupted -> {
          driveSubsystem.stop(); autoShootSupervisorSubsystem.setShootRequested(false);
          intakeSubsystem.stopIntake(); intakeSubsystem.stopPivotInBrake();
        }));
  }

}
