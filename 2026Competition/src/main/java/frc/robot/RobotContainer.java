// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import frc.robot.OdometryUpdates.OdometryUpdatesSubsystem;
import frc.robot.OdometryUpdates.QuestNavSubsystem;
import frc.robot.commands.AutoMainOneLeft;
import frc.robot.commands.AutoMainOneRight;
import frc.robot.commands.AutoMainTwoDepotHubSide;
import frc.robot.commands.AutoMainTwoDepotMiddle;
import frc.robot.commands.AutoShootUntilEmpty;
import frc.robot.commands.AutoSimpleMoveAndShootLastResort;
import frc.robot.commands.AutoStrategyEight;
import frc.robot.commands.AutoStrategyFive;
import frc.robot.commands.AutoStrategyFour;
import frc.robot.commands.AutoStrategyOne;
import frc.robot.commands.AutoStrategySeven;
import frc.robot.commands.AutoStrategySix;
import frc.robot.commands.AutoStrategyThree;
import frc.robot.commands.AutoStrategyTwo;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DeployIntakeSequence;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.IntakePowerIn;
import frc.robot.commands.IntakePowerOut;
import frc.robot.commands.IntakeRezeroFromRetractedHardStop;
import frc.robot.commands.IntakeToPositionAndHold;
import frc.robot.commands.RetractIntakeSequence;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShootCalibrationBurstWhileHeld;
import frc.robot.commands.ShootWhileHeld;
import frc.robot.commands.ShooterAdjustRpmCommand;
import frc.robot.commands.ShooterEnableCommand;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopClimb;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TestAuto;
import frc.robot.commands.TestTurretAngleCommand;
import frc.robot.commands.TurretCalibrationJogCommand;
import frc.robot.commands.TurretGoToZeroCommand;
import frc.robot.commands.TurretJogCommand;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KrakenMotorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.PrintTurretShotDiagnosticsCommand;

public class RobotContainer {

  // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Use open-loop control for drive motors
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private static Controller xboxDriveController = new Controller(OIContants.XBOX_CONTROLLER);
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;
  private static final Joystick turretStick = new Joystick(0);
  public static final Joystick bb = new Joystick(OIContants.BUTTON_BOX);

  public static KrakenMotorSubsystem m_kraken = new KrakenMotorSubsystem();

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  public static LLAprilTagSubsystem llAprilTagSubsystem = new LLAprilTagSubsystem();
  public static OdometryUpdatesSubsystem odometryUpdateSubsystem = new OdometryUpdatesSubsystem();
  public static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static TransferSubsystem transferSubsystem = new TransferSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public static AutoShootSupervisorSubsystem autoShootSupervisorSubsystem = new AutoShootSupervisorSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    // driveSubsystem.registerTelemetry(logger::telemeterize);

    setYaws();

        driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis(),
            () -> turretStick.getRawButton(2)));
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    AutonomousConfigure();
    if (RobotBase.isSimulation()) {
      // configureSimulation();
    }
    // testTurretShooter();
  }

  private static void configureSimulation() {
    // This method is for any simulation-specific configuration, such as setting up
    // simulated sensors or adjusting subsystem parameters for simulation.
    // For example, you might want to set up a simulated gyro or adjust the drive
    // subsystem's max speed for testing.
    driveSubsystem.resetCTREPose(new Pose2d(3.5, 5.7, new Rotation2d(0)));
  }

  public static void AutonomousConfigure() {
    // port autonomous routines as commands
    // sets the default option of the SendableChooser to the simplest autonomous
    // command. (from touching the hub, drive until outside the tarmac zone)
    SmartDashboard.putData(autoChooser);
    autoChooser.addOption("Auto Strategy One", new AutoStrategyOne());
    autoChooser.addOption("Auto Strategy Two", new AutoStrategyTwo());
    autoChooser.addOption("Auto Strategy Three", new AutoStrategyThree());
    autoChooser.addOption("Auto Strategy Four", new AutoStrategyFour());
    autoChooser.addOption("Auto Strategy Five", new AutoStrategyFive());
    autoChooser.addOption("Auto Strategy Six", new AutoStrategySix());
    autoChooser.addOption("Auto Strategy Seven", new AutoStrategySeven());
    autoChooser.addOption("Auto Strategy Eight", new AutoStrategyEight());
    autoChooser.addOption("Auto Main One Left", new AutoMainOneLeft());
    autoChooser.addOption("AutoMainOneRight", new AutoMainOneRight());
    autoChooser.addOption("AutoMainTwoDepotHubSide", new AutoMainTwoDepotHubSide());
    autoChooser.addOption("AutoMainTwoDepotMiddle", new AutoMainTwoDepotMiddle());
    autoChooser.addOption("Test Auto", new TestAuto());
    autoChooser.addOption("SimpleMoveAndShootLastResort", new AutoSimpleMoveAndShootLastResort());
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // driveSubsystem.setDefaultCommand(
    // // Drivetrain will execute this command periodically
    // driveSubsystem.applyRequest(() ->
    // driveSubsystem.getDrive().withVelocityX(-xboxDriveController.getLeftY() *
    // SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
    // .withVelocityY(-xboxDriveController.getLeftX() * SwerveConstants.MaxSpeed) //
    // Drive left with negative X (left)
    // .withRotationalRate(-xboxDriveController.getRightX() *
    // SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X
    // (left)
    // )
    // );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getIdle()).ignoringDisable(true));

    // xboxDriveController.a().whileTrue(driveSubsystem.applyRequest(() ->
    // driveSubsystem.getBrake()));
    // xboxDriveController.b().whileTrue(driveSubsystem.applyRequest(() ->
    // driveSubsystem.getPoint().withModuleDirection(new
    // Rotation2d(-xboxDriveController.getLeftY(), -xboxDriveController.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // xboxDriveController.back().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kForward));
    // xboxDriveController.back().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kReverse));
    // xboxDriveController.start().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kForward));
    // xboxDriveController.start().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    // xboxDriveController.leftBumper().onTrue(driveSubsystem.runOnce(() ->
    // driveSubsystem.seedFieldCentric()));

    // driveSubsystem.registerTelemetry(logger::telemeterize);

    // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
    // .onFalse(stopRobotCommand());
    // testTurretShooter();
    // testAuto();

    // --- Calibration bindings (easy on/off) ---
    // TODO: PLACEHOLDER: flip this boolean to enable calibration bindings
    // --- Calibration bindings (easy on/off) ---
    // TODO: PLACEHOLDER: flip this boolean to enable calibration bindings
    if (Constants.DebugTelemetrySubsystems.calibration) {
      //configureShooterCalibrationBindings(); 
      //configureHoodCalibrationBindings();
      //configureIntakeCalibrationBindings();
      //configureTurretCalibrationBindings();
      //configureTransferCalibrationBindings();  
      //configureSpindexerCalibrationBindings();
    }
    competitionXBOXButtonBindings();
    betaTesting();
    //setYaws();
  }

  public static Controller getDriveController() {
    return xboxDriveController;
  }

 public static Joystick getTurretStick() {
    return turretStick;
  }
  public static boolean isHubTrackingDisabledByButtonBox() {
    return bb.getRawAxis(OIContants.BB_HUB_TRACKING_DISABLE_AXIS)
        < OIContants.BB_HUB_TRACKING_DISABLE_THRESHOLD;
  }
  private void competitionXBOXButtonBindings() {
    new Trigger(() -> xboxDriveController.getRawAxis(2) > 0.3) // LT
        .whileTrue(new DeployIntakeSequence());

    new JoystickButton(xboxDriveController, 5) // LB
        .onTrue(new RetractIntakeSequence())
        .onFalse(new StopIntake());

    // new JoystickButton(xboxDriveController, 4)
    //     .onTrue(new ClimbUp())
    //     .onFalse(new StopClimb());

    // new JoystickButton(xboxDriveController, 1)
    //     .onTrue(new ClimbDown())
    //     .onFalse(new StopClimb());

    new JoystickButton(xboxDriveController, 8) // Left of X
        .onTrue(new InstantCommand(() -> driveSubsystem.zeroChassisYaw())
            .andThen(new InstantCommand(() -> questNavSubsystem.zeroYaw())));
    
    // new JoystickButton(xboxDriveController, 7)
    //     .onTrue(new InstantCommand(() -> questNavSubsystem.customQuestPose(new Pose2d(4.440, 0.613, Rotation2d.kZero)))
    //         .alongWith(new InstantCommand(() -> driveSubsystem.resetCTREPose(new Pose2d(4.440, 0.613, Rotation2d.kZero)))));

    // Trigger 3: MOVING shot while held (no drivetrain hold)
    new Trigger(() -> xboxDriveController.getRawAxis(3) > 0.3) // RT
        .whileTrue(new ShootWhileHeld(
            AutoShootSupervisorSubsystem.ShotMode.MANUAL_FIXED,
            false))
        .onFalse(new InstantCommand(() -> shooterSubsystem.stop())
            .alongWith(new InstantCommand(() -> transferSubsystem.stop()))
            .alongWith(new InstantCommand(() -> spindexerSubsystem.stop())));

    // Button 3: STATIC HUB BASE shot while held (drivetrain hold heading)
    new JoystickButton(xboxDriveController, 3) // X
        .whileTrue(new ShootWhileHeld(
            AutoShootSupervisorSubsystem.ShotMode.STATIC_HUB_BASE,
            true));

    // Button B: STATIC TOWER BASE shot while held (drivetrain hold heading)
    new JoystickButton(xboxDriveController, 2) // B
        .whileTrue(new ShootWhileHeld(
            AutoShootSupervisorSubsystem.ShotMode.STATIC_TOWER_BASE,
            true));

    new POVButton(xboxDriveController, 0) // AGR 2 OR Down Button
        .onTrue(new IntakePowerOut());        
        
    new POVButton(xboxDriveController, 180) // AGL 2 OR Up Button
        .onTrue(new IntakePowerIn());

    new POVButton(xboxDriveController, 90)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .whileTrue(new TurretJogCommand(turretSubsystem, 0.18));

    new POVButton(xboxDriveController, 270)
        .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
        .whileTrue(new TurretJogCommand(turretSubsystem, -0.18));

    new JoystickButton(bb, OIContants.BB_INTAKE_REZERO)
      .onTrue(new IntakeRezeroFromRetractedHardStop());

    new JoystickButton(bb, OIContants.BB_TURRET_ZERO)
      .and(new Trigger(RobotContainer::isHubTrackingDisabledByButtonBox))
      .onTrue(new TurretGoToZeroCommand());

  }
  private void configureSpindexerCalibrationBindings() {
  // Spindexer calibration buttons (turretStick has only 12 buttons).
  // IMPORTANT: Enable ONLY this calibration binding set when using it.

  // Live-tunable VELOCITY setpoints (RPS), not duty cycle
  final double[] baseRpsSet = new double[] { Constants.OperatorConstants.Spindexer.BASE_RPS };
  final double[] supplyRpsSet = new double[] { Constants.OperatorConstants.Spindexer.SUPPLY_RPS };

  final double BASE_STEP_RPS = 2.0;
  final double SUPPLY_STEP_RPS = 2.0;

  // Button 1: hold base mode
//   new JoystickButton(turretStick, 1)
//       .whileTrue(new RunCommand(() -> spindexerSubsystem.runBaseCal(baseRpsSet[0]), spindexerSubsystem))
//       .onFalse(new InstantCommand(() -> spindexerSubsystem.stopCal(), spindexerSubsystem));

  // Button 2: hold supply mode
  new JoystickButton(turretStick, 2)
      .whileTrue(new RunCommand(() -> spindexerSubsystem.runSupplyCal(supplyRpsSet[0]), spindexerSubsystem))
      .onFalse(new InstantCommand(() -> spindexerSubsystem.stopCal(), spindexerSubsystem));

  // Button 3: stop
  new JoystickButton(turretStick, 3)
      .onTrue(new InstantCommand(() -> spindexerSubsystem.stopCal(), spindexerSubsystem));

  // Button 4: base RPS up
  new JoystickButton(turretStick, 4)
      .onTrue(new InstantCommand(() -> baseRpsSet[0] += BASE_STEP_RPS));

  // Button 5: base RPS down
  new JoystickButton(turretStick, 5)
      .onTrue(new InstantCommand(() -> baseRpsSet[0] = Math.max(0.0, baseRpsSet[0] - BASE_STEP_RPS)));

  // Button 6: supply RPS up
  new JoystickButton(turretStick, 6)
      .onTrue(new InstantCommand(() -> supplyRpsSet[0] += SUPPLY_STEP_RPS));

  // Button 7: supply RPS down
  new JoystickButton(turretStick, 7)
      .onTrue(new InstantCommand(() -> supplyRpsSet[0] = Math.max(0.0, supplyRpsSet[0] - SUPPLY_STEP_RPS)));
}

  public static void resetQuestNav() {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new InstantCommand(() -> questNavSubsystem.resetQuestOdometry(new Pose3d())));
  }

  private void betaTesting() {
    new Trigger(() -> xboxDriveController.getRawAxis(2) > 0.3) // LT
        .onTrue(new DeployIntakeSequence())
        .onFalse(new StopIntake());

        // Button B: STATIC TOWER BASE shot while held (drivetrain hold heading)
    new JoystickButton(xboxDriveController, 2)
        .whileTrue(new ShootWhileHeld(
            AutoShootSupervisorSubsystem.ShotMode.STATIC_TOWER_BASE,
            true));
    
    new JoystickButton(xboxDriveController, 3)
        .whileTrue(new InstantCommand(() -> spindexerSubsystem.runBase()))
        .whileFalse(new InstantCommand(() -> spindexerSubsystem.stop()));

    new JoystickButton(turretStick, 1)
        .onTrue(new TestTurretAngleCommand());

    new JoystickButton(turretStick, 5)
    .onTrue(new PrintTurretShotDiagnosticsCommand());

        new JoystickButton(turretStick, 11)
        .whileTrue(new ShootWhileHeld(
            AutoShootSupervisorSubsystem.ShotMode.MANUAL_FIXED,
            false))
        .onFalse(new InstantCommand(() -> shooterSubsystem.stop())
            .alongWith(new InstantCommand(() -> transferSubsystem.stop()))
            .alongWith(new InstantCommand(() -> spindexerSubsystem.stop())));

    new JoystickButton(turretStick, 12)
        .onTrue(new InstantCommand(() -> transferSubsystem.runFeed()))
        .onFalse(new InstantCommand(() -> transferSubsystem.stop()));
  }

    private void configureTransferCalibrationBindings() {
    // Transfer calibration buttons (turretStick has only 12 buttons).
    // IMPORTANT: Enable ONLY this calibration binding set when using it,
    // or you will conflict with turret/shooter/hood calibration bindings.

    final int BTN_STAGE_HOLD = 1;
    final int BTN_FEED_HOLD  = 2;
    final int BTN_STOP_PRESS = 3;

    final int BTN_STAGE_UP   = 4;
    final int BTN_STAGE_DOWN = 5;
    final int BTN_FEED_UP    = 6;
    final int BTN_FEED_DOWN  = 7;

    // Live-tunable setpoints (no redeploy required)
    final double[] stageRpsSet = new double[] { Constants.OperatorConstants.Transfer.STAGE_RPS };
    final double[] feedRpsSet  = new double[] { Constants.OperatorConstants.Transfer.FEED_RPS };

    // If throat is blocked, stage should stop (or creep). Use your constant.
    final double blockedStageRps = Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS;

    // Steps (junior-friendly)
    final double STAGE_STEP_RPS = 2.0;
    final double FEED_STEP_RPS  = 5.0;
    final double RPM_A = 1500.0; // TODO: PLACEHOLDER - replace with your short-range shot RPM A

    // Stage (hold)
    // new JoystickButton(turretStick, 1)
    //     .whileTrue(new RunCommand(() -> transferSubsystem.runStageCal(stageRpsSet[0], blockedStageRps), transferSubsystem))
    //     .onFalse(new InstantCommand(() -> transferSubsystem.stopCal(), transferSubsystem));

    // // Feed (hold)
    // new JoystickButton(turretStick, 2)
    //     .whileTrue(new RunCommand(() -> transferSubsystem.runFeedCal(feedRpsSet[0]), transferSubsystem))
    //     .onFalse(new InstantCommand(() -> transferSubsystem.stopCal(), transferSubsystem));

    new JoystickButton(turretStick, 3)
        .whileTrue(new RunCommand(() -> transferSubsystem.runFeedMetered(), transferSubsystem))
        .onFalse(new InstantCommand(() -> transferSubsystem.stopCal(), transferSubsystem));

    new JoystickButton(turretStick, 1)
        .whileTrue(new InstantCommand(() -> transferSubsystem.runFeed()))
        .onFalse(new InstantCommand(() -> transferSubsystem.stop()));

    // Stop (press)
    // new JoystickButton(turretStick, 3)
    //     .onTrue(new InstantCommand(() -> transferSubsystem.stopCal(), transferSubsystem));

    // Adjust stage setpoint
    new JoystickButton(turretStick, 4)
        .onTrue(new InstantCommand(() -> stageRpsSet[0] += STAGE_STEP_RPS));
    new JoystickButton(turretStick, 5)
        .onTrue(new InstantCommand(() -> stageRpsSet[0] = Math.max(0.0, stageRpsSet[0] - STAGE_STEP_RPS)));

    // Adjust feed setpoint
    new JoystickButton(turretStick, 6)
        .onTrue(new InstantCommand(() -> feedRpsSet[0] += FEED_STEP_RPS));
    new JoystickButton(turretStick, 7)
        .onTrue(new InstantCommand(() -> feedRpsSet[0] = Math.max(0.0, feedRpsSet[0] - FEED_STEP_RPS)));
  }



  private void configureShooterCalibrationBindings() {
    // TODO: PLACEHOLDER - pick real button numbers (ok to reuse across subsystems
    // if you disable others)
    final int BTN_SHOOTER_SET_RPM_A = 1;
    final int BTN_SHOOTER_SET_RPM_B = 2;
    final int BTN_SHOOTER_STOP = 3;

    // Runs your existing volley state machine command (hold)
    final int BTN_SHOOTER_AUTOSHOOT_UNTIL_EMPTY = 4;

    // SysId routines (hold)
    final int BTN_SHOOTER_SYSID_QS_FWD = 9;
    final int BTN_SHOOTER_SYSID_QS_REV = 10;
    final int BTN_SHOOTER_SYSID_DYN_FWD = 11;
    final int BTN_SHOOTER_SYSID_DYN_REV = 12;

    // TODO: PLACEHOLDER - choose two practical calibration RPMs
    final double RPM_A = 2500.0; // TODO: PLACEHOLDER - replace with your short-range shot RPM A
    final double RPM_B = 4500.0; // TODO: PLACEHOLDER - replace with your short-range shot RPM B

    // Set RPM A (press)
    // new JoystickButton(turretStick, 1)
    //     .onTrue(new InstantCommand(() -> shooterSubsystem.setTargetRpm(RPM_A), shooterSubsystem))
    //     .onFalse(new InstantCommand(()-> shooterSubsystem.stop()));

      // Live-tunable VELOCITY setpoints (RPS), not duty cycle
  final double[] baseRpsSet = new double[] { Constants.OperatorConstants.Spindexer.BASE_RPS };
  final double[] supplyRpsSet = new double[] { Constants.OperatorConstants.Spindexer.SUPPLY_RPS };

  final double BASE_STEP_RPS = 2.0;
  final double SUPPLY_STEP_RPS = 2.0;

    // Button 2: hold supply mode

    // Set RPM B (press)
    // new JoystickButton(turretStick, 2)
    //     .onTrue(new InstantCommand(() -> shooterSubsystem.setTargetRpm(RPM_B), shooterSubsystem));

    // Stop shooter (press)
    // new JoystickButton(turretStick, 3)
    //     .onTrue(new InstantCommand(() -> shooterSubsystem.stop(), shooterSubsystem));

    new JoystickButton(turretStick, 1)
        .whileTrue(new InstantCommand(() -> shooterSubsystem.setTargetRpm(RPM_A), shooterSubsystem))
        .onFalse(new InstantCommand(() -> shooterSubsystem.stop()));

    new JoystickButton(turretStick, 2)
        .whileTrue(new InstantCommand(() -> transferSubsystem.runFeed()))
        .onFalse(new InstantCommand(() -> transferSubsystem.stop()));

    new JoystickButton(turretStick, 3)
        .whileTrue(new RunCommand(() -> spindexerSubsystem.runSupply()))
        .onFalse(new InstantCommand(() -> spindexerSubsystem.stopCal()));

    new JoystickButton(turretStick, 6)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(
                -44.667),
            RobotContainer.turretSubsystem))
        .onFalse(new InstantCommand(
            () -> RobotContainer.turretSubsystem.stop()));
      
    new JoystickButton(turretStick, 7)
        .onTrue(new InstantCommand(() -> turretSubsystem.zeroTurretAngle()));

    new JoystickButton(turretStick, 8)
        .onTrue(new InstantCommand(() -> {
          // Toggle by checking current target
          double currentDeg = Math.toDegrees(hoodSubsystem.getTargetAngleRad());
          double nextDeg = 21; //the degree you're going to
          hoodSubsystem.setTargetAngleRad(Math.toRadians(nextDeg));
        }));

    new JoystickButton(xboxDriveController, 1)
          .onTrue(questNavSubsystem.offsetAngleCharacterizationCommand())
          .onFalse(new StopRobot());

    new JoystickButton(xboxDriveController, 2)
          .onTrue(questNavSubsystem.offsetTranslationCharacterizationCommand())
          .onFalse(new StopRobot());

    // new JoystickButton(turretStick, 2)
    // .whileTrue(new ShootCalibrationBurstWhileHeld(RPM_A));

    // Auto shoot until empty (hold)
    // new JoystickButton(turretStick, 4)
    //     .whileTrue(new frc.robot.commands.AutoShootUntilEmpty());

    // // SysId routines (hold)
    // new JoystickButton(turretStick, 5)
    //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // new JoystickButton(turretStick, 6)
    //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // new JoystickButton(turretStick, 7)
    //     .whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // new JoystickButton(turretStick, 8)
    //     .whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

private void configureIntakeCalibrationBindings() {
  final int BTN_INTAKE_SEED_ZERO = 1;
  final int BTN_INTAKE_JOG_UP = 2;
  final int BTN_INTAKE_JOG_DOWN = 3;
  final int BTN_INTAKE_STEP_TOGGLE = 4;

  final int BTN_ROLLER_FWD = 5;
  final int BTN_ROLLER_REV = 6;
  final int BTN_ROLLER_STEP_TOGGLE = 7;
  final int BTN_ROLLER_STOP = 8;

  final int BTN_INTAKE_SYSID_QS_FWD = 9;
  final int BTN_INTAKE_SYSID_QS_REV = 10;
  final int BTN_INTAKE_SYSID_DYN_FWD = 11;
  final int BTN_INTAKE_SYSID_DYN_REV = 12;

  final double JOG_DUTY = Constants.OperatorConstants.IntakeConstants.CAL_PIVOT_JOG_DUTY;

  final double STEP_LOW_DEG = Constants.OperatorConstants.IntakeConstants.CAL_STEP_LOW_DEG;
  final double STEP_HIGH_DEG = Constants.OperatorConstants.IntakeConstants.CAL_STEP_HIGH_DEG;

  final double ROLLER_LOW_RPS = 20.0;
  final double ROLLER_HIGH_RPS = Constants.OperatorConstants.IntakeConstants.ROLLER_INTAKE_RPS;

//   new JoystickButton(turretStick, 1)
//       .onTrue(new InstantCommand(() -> intakeSubsystem.seedZeroFromRetractedHardStop()));

  new JoystickButton(turretStick, 2)
      .whileTrue(new RunCommand(() -> intakeSubsystem.setPivotDutyCycle(+JOG_DUTY), intakeSubsystem))
      .onFalse(new InstantCommand(() -> intakeSubsystem.exitOpenLoopHold()));

  new JoystickButton(turretStick, 3)
      .whileTrue(new RunCommand(() -> intakeSubsystem.setPivotDutyCycle(-JOG_DUTY), intakeSubsystem))
      .onFalse(new InstantCommand(() -> intakeSubsystem.exitOpenLoopHold()));

  new JoystickButton(turretStick, 4)
      .onTrue(new InstantCommand(() -> {
        double current = intakeSubsystem.getTargetPivotDeg();
        intakeSubsystem.setTargetPivotDeg(STEP_LOW_DEG);
      }));

  new JoystickButton(turretStick, 5)
      .whileTrue(new StartIntake())
      .onFalse(new StopIntake());

  new JoystickButton(turretStick, 6)
      .whileTrue(new RunCommand(
          () -> intakeSubsystem.runIntake(Constants.OperatorConstants.IntakeConstants.ROLLER_REVERSE_RPS),
          intakeSubsystem))
      .onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

  new JoystickButton(turretStick, 7)
      .onTrue(new InstantCommand(() -> {
        double current = intakeSubsystem.getRollerTargetRps();
        double mid = (ROLLER_LOW_RPS + ROLLER_HIGH_RPS) * 0.5;
        double next = (current < mid) ? ROLLER_HIGH_RPS : ROLLER_LOW_RPS;
        intakeSubsystem.runIntake(next);
      }));

  new JoystickButton(turretStick, 8)
      .onTrue(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

  new JoystickButton(turretStick, 9)
      .onTrue(new DeployIntakeSequence())
      .onFalse(new RetractIntakeSequence());

  new JoystickButton(turretStick, 10)
      .onTrue(new RetractIntakeSequence());

       new JoystickButton(turretStick, 1)
      .onTrue(new IntakeRezeroFromRetractedHardStop());



  // new JoystickButton(turretStick, BTN_INTAKE_SYSID_QS_FWD)
  //     .whileTrue(intakeSubsystem.sysIdPivotQuasistatic(SysIdRoutine.Direction.kForward));
  // new JoystickButton(turretStick, BTN_INTAKE_SYSID_QS_REV)
  //     .whileTrue(intakeSubsystem.sysIdPivotQuasistatic(SysIdRoutine.Direction.kReverse));
  // new JoystickButton(turretStick, BTN_INTAKE_SYSID_DYN_FWD)
  //     .whileTrue(intakeSubsystem.sysIdPivotDynamic(SysIdRoutine.Direction.kForward));
  // new JoystickButton(turretStick, BTN_INTAKE_SYSID_DYN_REV)
  //     .whileTrue(intakeSubsystem.sysIdPivotDynamic(SysIdRoutine.Direction.kReverse));
}

  private void configureHoodCalibrationBindings() {
    // Logitech Extreme 3D Pro suggested mapping (TODO: PLACEHOLDER change as
    // desired)
    final int BTN_JOG_DOWN = 5; // TODO: PLACEHOLDER
    final int BTN_JOG_UP = 6; // TODO: PLACEHOLDER
    final int BTN_SEED_ZERO = 7; // TODO: PLACEHOLDER
    final int BTN_STEP_TOGGLE = 8; // TODO: PLACEHOLDER

    // SysId buttons (hold)
    final int BTN_SYSID_QS_FWD = 9; // TODO: PLACEHOLDER
    final int BTN_SYSID_QS_REV = 10; // TODO: PLACEHOLDER
    final int BTN_SYSID_DYN_FWD = 11; // TODO: PLACEHOLDER
    final int BTN_SYSID_DYN_REV = 12; // TODO: PLACEHOLDER

    // Jog duty (slow + safe while you’re finding limits)
    final double JOG_DUTY = 0.10; // TODO: PLACEHOLDER start low and increase carefully if needed

    // Step test angles (for PID tuning)
    final double STEP_LOW_DEG = 5.0; 
    final double STEP_HIGH_DEG = 15.0; 
    final double STEP_VERY_LOW_DEG = 1.0;

    // Seed zero (press)
    new JoystickButton(turretStick, 7)
        .onTrue(new InstantCommand(() -> hoodSubsystem.seedZeroFromDownHardStop()));
// 51, 14, 13, 12, 11, 41, 42, 43, 44
    // Jog UP (hold)
    new JoystickButton(turretStick, 6)
        .whileTrue(new RunCommand(() -> hoodSubsystem.setCalibrationDutyCycle(+JOG_DUTY), hoodSubsystem))
        .onFalse(new InstantCommand(() -> hoodSubsystem.stop()));

    // Jog DOWN (hold)
    new JoystickButton(turretStick, 5)
        .whileTrue(new RunCommand(() -> hoodSubsystem.setCalibrationDutyCycle(-JOG_DUTY), hoodSubsystem))
        .onFalse(new InstantCommand(() -> hoodSubsystem.stop()));

    // Step test toggle (press): alternates between two angles
    new JoystickButton(turretStick, 8)
        .onTrue(new InstantCommand(() -> {
          // Toggle by checking current target
          double currentDeg = Math.toDegrees(hoodSubsystem.getTargetAngleRad());
          double nextDeg = STEP_VERY_LOW_DEG;
          hoodSubsystem.setTargetAngleRad(Math.toRadians(nextDeg));
        }));

    // SysId routines (hold)
    new JoystickButton(turretStick, 9)
        .whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(turretStick, 10)
        .whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(turretStick, 11)
        .whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(turretStick, 12)
        .whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command stopRobotCommand() {
    System.out.println("***Stopping Robot");
    return driveSubsystem.applyRequest(() -> driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y
                                                                                        // (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)

    );
  }

  public void setYaws() {
    new JoystickButton(xboxDriveController, 8)
        .onTrue(new InstantCommand(() -> driveSubsystem.zeroChassisYaw())
            .andThen(new InstantCommand(() -> questNavSubsystem.zeroYaw())));
    new JoystickButton(xboxDriveController, 7)
        .onTrue(new InstantCommand(() -> questNavSubsystem.resetToZeroPose()));
  }

  // Driver preferred controls
  private double getDriverXAxis() {
    // return -xboxController.getLeftStickY();
    // SmartDashboard.putNumber("X-Axis: ", -xboxDriveController.getRightStickY());
    return -xboxDriveController.getLeftStickY();
  }

  private double getDriverYAxis() {
    // return -xboxController.getLeftStickX();
    // SmartDashboard.putNumber("Y-Axis: ", -xboxDriveController.getRightStickX());
    return -xboxDriveController.getLeftStickX();
    // return 0;
  }

  private double getDriverOmegaAxis() {
    // return -xboxController.getLeftStickOmega();
    // SmartDashboard.putNumber("Z-Axis: ", -xboxDriveController.getLeftStickX() *
    // 0.6);
    return -xboxDriveController.getRightStickX() * 0.6;
  }

  public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {

    // alex test
    // System.out.println("Start drive routine");

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      ElasticHelpers.setAutoPathSingle(path);

      Pose2d startPose = path.getStartingHolonomicPose().get(); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (!shouldResetOdometryToStartingPose) {

        // alex test
        // System.out.println("Rigth before driving without reset");
        return AutoBuilder.followPath(path);

      } else { // reset odometry the right way

        // alex test
        // System.out.println("Rigth before driving with reset");

        return Commands.sequence(
            // new InstantCommand(
            // () -> questNavSubsystem.resetQuestOdometry(new
            // Pose3d(TrajectoryHelper.flipQuestPoseRed(startPose)))),
            AutoBuilder.resetOdom(startPose), new WaitCommand(0), AutoBuilder.followPath(path));

        // return Commands.sequence(AutoBuilder.resetOdom(startPose));

        // return Commands.sequence(new InstantCommand(() ->
        // questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))),
        // AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command runTrajectory2Poses(boolean shouldResetOdometryToStartingPose, Pose2d startPose,
      Pose2d endPose) {
    try {
      List<Waypoint> pathWaypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

      if (!shouldResetOdometryToStartingPose) {
        PathPlannerPath path = new PathPlannerPath(
            pathWaypoints,
            AutoConstants.pathConstraints,
            null,
            new GoalEndState(0, endPose.getRotation()));
        path.preventFlipping = true;
        // System.out.println("== Driving from " + startPose + " to " + endPose);
        return AutoBuilder.followPath(path);
      } else { // reset odometry, then follow the path
        PathPlannerPath path = new PathPlannerPath(
            pathWaypoints,
            AutoConstants.pathConstraints,
            new IdealStartingState(0, startPose.getRotation()),
            new GoalEndState(2, endPose.getRotation()));
        path.preventFlipping = true;
        // System.out.println("== Driving from " + startPose + " to " + endPose);

        // Keep the original CTRE pose reset behavior, but perform it at schedule-time.
        // AutoBuilder.resetOdom(startPose) is the PathPlanner-friendly reset; we run it
        // too.
        return Commands.sequence(
            Commands.runOnce(() -> driveSubsystem.resetCTREPose(startPose), driveSubsystem),
            AutoBuilder.resetOdom(startPose),
            AutoBuilder.followPath(path));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // alex test
  // public static Command testCommand2() {
  // return new PrintCommand("Test 2 Command");
  // }

  // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("Match/AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
      isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
      System.out.println("*** RED Alliance: " + isAllianceRed);
    }
  }

  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void testAuto() {
    new JoystickButton(xboxDriveController, 1)
        .onTrue(new InstantCommand(() -> odometryUpdateSubsystem.updateQuestAndState(
            new Pose2d(3.5, 4.0, new Rotation2d()))));
  }

  public static void testTurretShooter() {
    // new JoystickButton(turretStick, 1)
    // .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem,
    // Constants.OperatorConstants.Shooter.RPM_STEP));

    // new JoystickButton(turretStick, 2)
    // .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem,
    // -Constants.OperatorConstants.Shooter.RPM_STEP));

    // new JoystickButton(turretStick, 3).whileTrue(new
    // ShooterEnableCommand(shooterSubsystem));

    // new JoystickButton(turretStick, 5).whileTrue(new
    // TurretJogCommand(turretSubsystem, -0.25));
    // new JoystickButton(turretStick, 6).whileTrue(new
    // TurretJogCommand(turretSubsystem, 0.25));

    // =============================
    // Turret Manual Jog (Simulation)
    // =============================

    // new JoystickButton(turretStick, 3)
    // .whileTrue(Commands.startEnd(
    // () -> m_kraken.setDutyCycle(1.0),
    // () -> m_kraken.stop(),
    // m_kraken
    // ));
    // Hold A → rotate turret left
    // new JoystickButton(turretStick, 3).whileTrue(
    // Commands.runEnd(
    // () -> turretSubsystem.setVoltageVolts(-12),
    // () -> turretSubsystem.stop(),
    // turretSubsystem
    // )
    // );

    // // Hold B → rotate turret right
    // new JoystickButton(turretStick, 4).whileTrue(
    // Commands.runEnd(
    // () -> turretSubsystem.setVoltageVolts(12),
    // () -> turretSubsystem.stop(),
    // turretSubsystem
    // )
    // );

    // new JoystickButton(turretStick, 3).whileTrue(
    // Commands.runEnd(
    // () -> hopperSubsystem.setStageDuty(1.0),
    // () -> hopperSubsystem.stop(),
    // hopperSubsystem)
    // );

    // new JoystickButton(turretStick, 4).whileTrue(
    // Commands.runEnd(
    // () -> hopperSubsystem.setStageDuty(-1.0),
    // () -> hopperSubsystem.stop(),
    // hopperSubsystem)
    // );

    // new JoystickButton(turretStick, 5).whileTrue(
    // Commands.runEnd(
    // () -> hopperSubsystem.setStageDuty(0.5),
    // () -> hopperSubsystem.stop(),
    // hopperSubsystem)
    // );

    // new JoystickButton(xboxDriveController, 1).whileTrue(
    // Commands.runEnd(
    // () -> shooterSubsystem.setDutyCycle(.32),
    // () -> shooterSubsystem.stop(),
    // shooterSubsystem)
    // );

    // new JoystickButton(xboxDriveController, 2).whileTrue(
    // Commands.runEnd(
    // () -> shooterSubsystem.setDutyCycle(-.32),
    // () -> shooterSubsystem.stop(),
    // shooterSubsystem)
    // );

    // new JoystickButton(xboxDriveController, 3).whileTrue(
    // Commands.runEnd(
    // () -> hoodSubsystem.setDutyCycle(0.125),
    // () -> hoodSubsystem.stop(),
    // hoodSubsystem)
    // );

    // new JoystickButton(xboxDriveController, 4).whileTrue(
    // Commands.runEnd(
    // () -> hoodSubsystem.setDutyCycle(-0.125),
    // () -> hoodSubsystem.stop(),
    // hoodSubsystem)
    // );

  }

  private void 
  configureTurretCalibrationBindings() {
    // 1-2: hold-to-jog (open loop)
    new JoystickButton(turretStick, 1)
        .whileTrue(new TurretCalibrationJogCommand(-Constants.OperatorConstants.Turret.CAL_JOG_MAX_DUTY));

    new JoystickButton(turretStick, 2)
        .whileTrue(new TurretCalibrationJogCommand(Constants.OperatorConstants.Turret.CAL_JOG_MAX_DUTY));

    // 3: reseed integrated from absolute now
    new JoystickButton(turretStick, 3)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationReseedIntegratedFromAbsoluteNow(),
            RobotContainer.turretSubsystem));
    
    

    // 4: capture absolute ticks candidate (copy into ABS_FORWARD_TICKS manually)
    JoystickButton capture = new JoystickButton(turretStick, 4);
    capture.onTrue(new InstantCommand(
        () -> RobotContainer.turretSubsystem.calibrationCaptureAbsZeroTicksCandidate(),
        RobotContainer.turretSubsystem));

    // 5-9: Motion Magic step targets
    new JoystickButton(turretStick, 5)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(0.0),
            RobotContainer.turretSubsystem));

    new JoystickButton(turretStick, 6)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(
              50),
            RobotContainer.turretSubsystem))
        .onFalse(new InstantCommand(
            () -> RobotContainer.turretSubsystem.stop()));

    new JoystickButton(turretStick, 7)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(
                -Constants.OperatorConstants.Turret.CAL_STEP_SMALL_DEG),
            RobotContainer.turretSubsystem))
        .onFalse(new InstantCommand(
            () -> RobotContainer.turretSubsystem.stop()));

    new JoystickButton(turretStick, 8)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(
                Constants.OperatorConstants.Turret.CAL_STEP_LARGE_DEG),
            RobotContainer.turretSubsystem))
        .onFalse(new InstantCommand(
            () -> RobotContainer.turretSubsystem.stop()));;

    new JoystickButton(turretStick, 9)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationGoToAngleDeg(
                -Constants.OperatorConstants.Turret.CAL_STEP_LARGE_DEG),
            RobotContainer.turretSubsystem))
        .onFalse(new InstantCommand(
            () -> RobotContainer.turretSubsystem.stop()));;

    // 10: toggle sweep (sweep motion runs from TurretSubsystem.periodic while
    // enabled)
    new JoystickButton(turretStick, 10)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationToggleSweep(),
            RobotContainer.turretSubsystem));


    // 11/12: +kP / -kP
    new JoystickButton(turretStick, 11)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationAdjustKp(
                Constants.OperatorConstants.Turret.CAL_KP_STEP),
            RobotContainer.turretSubsystem));

    new JoystickButton(turretStick, 12)
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationAdjustKp(
                -Constants.OperatorConstants.Turret.CAL_KP_STEP),
            RobotContainer.turretSubsystem));

    // Modifier: hold button 4 while tapping 11/12 adjusts kD instead
    capture.and(new JoystickButton(turretStick, 11))
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationAdjustKd(
                Constants.OperatorConstants.Turret.CAL_KD_STEP),
            RobotContainer.turretSubsystem));

    capture.and(new JoystickButton(turretStick, 12))
        .onTrue(new InstantCommand(
            () -> RobotContainer.turretSubsystem.calibrationAdjustKd(
                -Constants.OperatorConstants.Turret.CAL_KD_STEP),
            RobotContainer.turretSubsystem));
  }

  public void publishPoseToAdvantageScope() {
    logger.telemeterize(driveSubsystem.getState());
  }

}
