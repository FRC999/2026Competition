
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
import frc.robot.commands.ReverseSpindexer;
import frc.robot.commands.ReverseTransfer;
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
        Commands.runOnce(RobotContainer::seedPoseFromPhotonVision, driveSubsystem).ignoringDisable(true));
    SmartDashboard.putData("Vision/Capture camera jitter (disabled)",
        Commands.runOnce(vision::startCameraJitterCapture).ignoringDisable(true));
    SmartDashboard.putData("Vision/Stop camera jitter capture",
        Commands.runOnce(vision::stopCameraJitterCapture).ignoringDisable(true));
    SmartDashboard.putData("Calibration/Reseed turret ONLY when physically stowed (disabled)",
        Commands.runOnce(turretSubsystem::reseedIntegratedFromAbsoluteNow, turretSubsystem).ignoringDisable(true));
    SmartDashboard.putData("Calibration/Zero hood ONLY when fully down (disabled)",
        Commands.runOnce(hoodSubsystem::seedZeroFromDownHardStop, hoodSubsystem).ignoringDisable(true));
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
    return bb.getRawAxis(OIContants.BB_HUB_TRACKING_DISABLE_AXIS)
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

  private void competitionXBOXButtonBindings() {
    panicStopLatched = isPanicSwitchActive();
    if (panicStopLatched) {
      cancelAllCommandsForPanicStop();
    }

    Trigger panicStopTrigger = new Trigger(RobotContainer::isPanicSwitchActive);
    Trigger panicInactiveTrigger = new Trigger(() -> !RobotContainer.isPanicStopActive());
    Trigger povUpTrigger = new POVButton(xboxDriveController, 0);
    Trigger povDownTrigger = new POVButton(xboxDriveController, 180);
    JoystickButton driverAButton = new JoystickButton(xboxDriveController, OIContants.XBOX_BUTTON_A);
    JoystickButton driverYButton = new JoystickButton(xboxDriveController, 4);

    panicStopTrigger
        .onTrue(new InstantCommand(() -> {
          panicStopLatched = true;
          cancelAllCommandsForPanicStop();
        }).ignoringDisable(true))
        .onFalse(new InstantCommand(() -> panicStopLatched = false).ignoringDisable(true));

    new JoystickButton(bb, OIContants.BB_VISION_SEED_BUTTON)
        .and(new Trigger(RobotContainer::isVisionSeedAxisActive))
        .and(new Trigger(DriverStation::isDisabled))
        .and(panicInactiveTrigger)
        .onTrue(Commands.runOnce(RobotContainer::seedPoseFromPhotonVision, driveSubsystem)
            .ignoringDisable(true));

   new Trigger(() -> xboxDriveController.getRawAxis(OIContants.XBOX_LEFT_TRIGGER_AXIS)
        > OIContants.XBOX_TRIGGER_ACTIVE_THRESHOLD)
        .and(panicInactiveTrigger)
        .whileTrue(Commands.defer(
            DeployAndRunIntakeWhileHeld::new,
            Set.of(intakeSubsystem)))
        .onFalse(Commands.defer(
            () -> intakeSubsystem.shouldStayDeployedAfterTriggerRelease()
                ? new StopIntake()
                : new RetractIntakeSequence(),
            Set.of(intakeSubsystem)));

    driverAButton
        .and(panicInactiveTrigger)
        .and(new Trigger(() -> xboxDriveController.getPOV() != 180))
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(true),
            intakeSubsystem).andThen(new DeployIntakeSequence()));

    povDownTrigger
        .and(driverAButton)
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(true),
            intakeSubsystem).andThen(new DeployIntakeSequence(true)));

    driverYButton // Y
        .and(panicInactiveTrigger)
        .and(new Trigger(() -> xboxDriveController.getPOV() != 0))
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(false),
            intakeSubsystem).andThen(new RetractIntakeSequence()));

    povUpTrigger
        .and(driverYButton)
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> intakeSubsystem.setStayDeployedAfterTriggerRelease(false),
            intakeSubsystem).andThen(new RetractIntakeSequence(true)));

    new JoystickButton(xboxDriveController, 2)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MANUAL_PRESET_3M,
            false));

    new JoystickButton(bb, OIContants.BB_MANUAL_RPM_UP)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT +=
                Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_STEP_PERCENT));

    new JoystickButton(bb, OIContants.BB_MANUAL_SHOT_3M)
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.Turret.AUTO_AIM_TRIM_DEG += 1.0));

    new JoystickButton(xboxDriveController, 3)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MANUAL_PRESET_4M,
            false));

    new JoystickButton(bb, OIContants.BB_MANUAL_SHOT_4M)
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.Turret.AUTO_AIM_TRIM_DEG -= 1.0));

    new JoystickButton(bb, OIContants.BB_INTAKE_INIT_DEPLOY_6)
        .and(panicInactiveTrigger)
        .whileTrue(Commands.defer(
            InitialAutoDeployWhileHeld::new,
            Set.of(intakeSubsystem)))
        .onFalse(Commands.defer(
            RetractIntakeSequence::new,
            Set.of(intakeSubsystem)));

    new JoystickButton(bb, OIContants.BB_MANUAL_RPM_DOWN)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .onTrue(new InstantCommand(
            () -> Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_PERCENT -=
                Constants.OperatorConstants.AutoShoot.MANUAL_SHOT_RPM_TRIM_STEP_PERCENT));

    new JoystickButton(xboxDriveController, 5) // LB
        .and(panicInactiveTrigger)
        .whileTrue(new ReverseIntake());

    new JoystickButton(xboxDriveController, 6) // RB
        .and(panicInactiveTrigger)
        .whileTrue(new ReverseTransfer()
            .alongWith(new ReverseSpindexer())
            .alongWith(new ReverseShooterTemporary()))
        .onFalse(new StopIntake());

    new JoystickButton(xboxDriveController, 8)
        .and(new Trigger(DriverStation::isTeleopEnabled))
        .and(panicInactiveTrigger)
        .onTrue(Commands.runOnce(driveSubsystem::orientDriverForwardToCurrentHeading, driveSubsystem));

    new Trigger(() -> xboxDriveController.getRawAxis(3) > 0.3) // RT
        .and(panicInactiveTrigger)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.MOVING_AUTO,
            false));

    new JoystickButton(xboxDriveController, 2) // B
        .and(new Trigger(() -> !RobotContainer.isHubTrackingDisabledByButtonBox()))
        .and(panicInactiveTrigger)
        .whileTrue(new ShootWhileHeld(
            frc.robot.lib.ShotPlanner.Mode.STATIC_TOWER_BASE,
            true));

    new POVButton(xboxDriveController, 90)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .whileTrue(new TurretJogCommand(turretSubsystem, 0.18));

    new POVButton(xboxDriveController, 270)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .and(panicInactiveTrigger)
        .whileTrue(new TurretJogCommand(turretSubsystem, -0.18));

    new JoystickButton(bb, OIContants.BB_INTAKE_REZERO)
      .and(panicInactiveTrigger)
      .onTrue(new IntakeRezeroFromRetractedHardStop());

    new JoystickButton(bb, OIContants.BB_TURRET_ZERO)
      .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
      .and(panicInactiveTrigger)
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
        ElasticHelpers.setAutoPathSingle(path);
        return frc.robot.commands.PrecisionPathCommands.followResolved(driveSubsystem, path,
            resetToStart, vision::isLocalizationReady);
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
        return runTrajectory2Poses(driveSubsystem.getPose(), path.getStartingHolonomicPose().orElseThrow());
      } catch (Exception ex) {
        return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
            "Cannot approach path " + name + ": " + ex.getMessage());
      }
    }, Set.of(driveSubsystem));
  }

  public static Command runTrajectory2Poses(Pose2d startPose, Pose2d endPose) {
    if (startPose.getTranslation().getDistance(endPose.getTranslation()) < 0.01) {
      Command finish = frc.robot.commands.PrecisionPathCommands.finishAt(driveSubsystem, endPose,
          vision::isLocalizationReady);
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
          false, vision::isLocalizationReady);
    } catch (Exception ex) {
      return frc.robot.commands.PrecisionPathCommands.failedHold(driveSubsystem,
          "Cannot generate two-pose path: " + ex.getMessage());
    }
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    return selected == null ? Commands.run(driveSubsystem::stop, driveSubsystem)
        : boundedAutos.computeIfAbsent(selected, command -> command.withTimeout(AutoConstants.AUTO_PERIOD_SECONDS).finallyDo(interrupted -> {
          driveSubsystem.stop(); autoShootSupervisorSubsystem.setShootRequested(false);
          intakeSubsystem.stopIntake(); intakeSubsystem.stopPivotInBrake();
        }));
  }

}
