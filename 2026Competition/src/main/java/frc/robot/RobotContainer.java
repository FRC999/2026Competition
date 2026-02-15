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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import frc.robot.OdometryUpdates.OdometryUpdatesSubsystem;
import frc.robot.OdometryUpdates.QuestNavSubsystem;
import frc.robot.commands.AutoStrategyFour;
import frc.robot.commands.AutoStrategyOne;
import frc.robot.commands.AutoStrategyThree;
import frc.robot.commands.AutoStrategyTwo;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ShooterAdjustRpmCommand;
import frc.robot.commands.ShooterEnableCommand;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TestAuto;
import frc.robot.commands.TurretJogCommand;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Use open-loop control for drive motors
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private final Controller xboxDriveController = new Controller(OIContants.XBOX_CONTROLLER);
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;
  private static final Joystick turretStick = new Joystick(0);

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  public static LLAprilTagSubsystem llAprilTagSubsystem = new LLAprilTagSubsystem();
  public static OdometryUpdatesSubsystem odometryUpdateSubsystem = new OdometryUpdatesSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static HopperSubsystem hopperSubsystem = new HopperSubsystem();
  public static TransferSubsystem transferSubsystem = new TransferSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static AutoShootSupervisorSubsystem autoShootSupervisorSubsystem = new AutoShootSupervisorSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // --- Elastic Dropdowns (real dropdown widgets) ---
  // These are published as SendableChoosers so Elastic/SmartDashboard shows
  // actual dropdowns instead of string inputs.
  private static SendableChooser<String> firstDestinationChooser = new SendableChooser<>();
  private static SendableChooser<String> nextPathChooser = new SendableChooser<>();
  private static final String CHOOSER_PLACEHOLDER = "<Select>";
  private static int sLastFirstOptionsHash = 0;
  private static int sLastNextOptionsHash = 0;

  // Cached NT4 pubs/subs for Elastic auto UI mirroring (avoid subscriber leaks)
  private static final edu.wpi.first.networktables.NetworkTable AUTO_NT_FOR_ELASTIC = NetworkTableInstance.getDefault()
      .getTable("Autos");
  private static final StringArraySubscriber SUB_FIRST_DEST_OPTIONS = AUTO_NT_FOR_ELASTIC
      .getStringArrayTopic("FirstDestinationOptions").subscribe(new String[0]);
  private static final StringArraySubscriber SUB_NEXT_OPTIONS = AUTO_NT_FOR_ELASTIC.getStringArrayTopic("NextOptions")
      .subscribe(new String[0]);
  private static final StringPublisher PUB_FIRST_DEST = AUTO_NT_FOR_ELASTIC.getStringTopic("FirstDestination")
      .publish();
  private static final StringPublisher PUB_CHAIN = AUTO_NT_FOR_ELASTIC.getStringTopic("Chain").publish();
  private static final edu.wpi.first.networktables.StringArrayPublisher PUB_SELECTED_SEGMENTS = AUTO_NT_FOR_ELASTIC
      .getStringArrayTopic("SelectedSegments").publish();
  private static String sLastFirstDest = "";
  private static String sLastPendingNext = "";
  private static boolean sLastClearChain = false;

  public RobotContainer() {
    configureBindings();
    driveSubsystem.registerTelemetry(logger::telemeterize);

    setYaws();

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis()));
    FollowPathCommand.warmupCommand().schedule();

    AutonomousConfigure();
    if (RobotBase.isSimulation()) {
      // configureSimulation();
    }
    testTurretShooter();
  }

  private static void configureSimulation() {
    // This method is for any simulation-specific configuration, such as setting up
    // simulated sensors or adjusting subsystem parameters for simulation.
    // For example, you might want to set up a simulated gyro or adjust the drive
    // subsystem's max speed for testing.
    driveSubsystem.resetCTREPose(new Pose2d(3.5, 5.7, new Rotation2d(0)));
  }

  public static void AutonomousConfigure() {
    // Auto chooser (existing)
    SmartDashboard.putData(autoChooser);
    autoChooser.addOption("Auto Strategy One", new AutoStrategyOne());
    autoChooser.addOption("Auto Strategy Two", new AutoStrategyTwo());
    autoChooser.addOption("Auto Strategy Three", new AutoStrategyThree());
    autoChooser.addOption("Auto Strategy Four", new AutoStrategyFour());
    autoChooser.addOption("Test Auto", new TestAuto());
    // Dynamic stitched auto (on-the-fly to first destination, then chained PP
    // paths)
    autoChooser.addOption(
        "Stitched (FirstDest + Chain)",
        edu.wpi.first.wpilibj2.command.Commands.deferredProxy(TrajectoryHelper::buildStitchedAutoCommand));

    // --- Elastic Auto Stitch UI ---
    // Drivers interact ONLY with the two dropdown widgets and an optional clear
    // button:
    // 1) Autos/First Destination (SendableChooser<String>)
    // 2) Autos/Next Path (SendableChooser<String>)
    // The selected sequence is stored as outputs (SelectedSegments + ChainDisplay)
    // and mirrored to NT table "Autos".

    // Publish dropdown widgets (will be populated dynamically in
    // updateElasticAutoDropdowns())
    SmartDashboard.putData("Autos/First Destination", firstDestinationChooser);
    SmartDashboard.putData("Autos/Next Path", nextPathChooser);

    // Optional: show option arrays for debugging (NOT for driver interaction)
    SmartDashboard.putStringArray("Autos/FirstDestinationOptions", new String[0]);
    SmartDashboard.putStringArray("Autos/NextOptions", new String[0]);

    // Read-only outputs
    SmartDashboard.putStringArray("Autos/SelectedSegments", new String[0]);
    SmartDashboard.putString("Autos/ChainDisplay", "");

    // Optional: quick reset button
    SmartDashboard.putBoolean("Autos/ClearChain", false);
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

    driveSubsystem.registerTelemetry(logger::telemeterize);

    // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
    // .onFalse(stopRobotCommand());
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
    return -xboxDriveController.getRightStickY();
  }

  private double getDriverYAxis() {
    // return -xboxController.getLeftStickX();
    // SmartDashboard.putNumber("Y-Axis: ", -xboxDriveController.getRightStickX());
    return -xboxDriveController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    // return -xboxController.getLeftStickOmega();
    // SmartDashboard.putNumber("Z-Axis: ", -xboxDriveController.getLeftStickX() *
    // 0.6);
    return -xboxDriveController.getLeftStickX() * 0.6;
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
            new InstantCommand(
                () -> questNavSubsystem.resetQuestOdometry(new Pose3d(TrajectoryHelper.flipQuestPoseRed(startPose)))),
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
        System.out.println("== Driving from " + startPose + " to " + endPose);
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        driveSubsystem.resetCTREPose(startPose);
        PathPlannerPath path = new PathPlannerPath(
            pathWaypoints,
            AutoConstants.pathConstraints,
            new IdealStartingState(0, startPose.getRotation()),
            new GoalEndState(0, endPose.getRotation()));
        path.preventFlipping = true;
        System.out.println("== Driving from " + startPose + " to " + endPose);
        // return Commands.sequence(AutoBuilder.resetOdom(startPose),
        // AutoBuilder.followPath(path));
        return AutoBuilder.resetOdom(startPose);
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
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
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

  public static void testTurretShooter() {
    // new JoystickButton(turretStick, 1)
    // .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem,
    // Constants.OperatorConstants.Shooter.RPM_STEP));

    // new JoystickButton(turretStick, 2)
    // .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem,
    // -Constants.OperatorConstants.Shooter.RPM_STEP));

    new JoystickButton(turretStick, 3).whileTrue(new ShooterEnableCommand(shooterSubsystem));

    new JoystickButton(turretStick, 5).whileTrue(new TurretJogCommand(turretSubsystem, -0.25));
    new JoystickButton(turretStick, 6).whileTrue(new TurretJogCommand(turretSubsystem, 0.25));
  }

  public void publishPoseToAdvantageScope() {
    logger.telemeterize(driveSubsystem.getState());
  }

  /**
   * Mirrors NetworkTables table "Autos" <-> SmartDashboard keys under
   * "Autos/...".
   *
   * Why: your Elastic workflow binds widgets to SmartDashboard keys, while the
   * stitching logic (TrajectoryHelper / ElasticHelpers) reads from the NT table
   * "Autos".
   *
   * - Robot publishes options to NT table "Autos": FirstDestinationOptions,
   * NextOptions
   * - Elastic writes selections to SmartDashboard: Autos/FirstDestination,
   * Autos/Chain
   * - This keeps both in sync.
   */

  private static int optionsHash(String[] options) {
    return java.util.Arrays.hashCode(options == null ? new String[0] : options);
  }

  private static boolean containsOption(String[] options, String value) {
    if (value == null || value.isBlank() || options == null)
      return false;
    for (String o : options) {
      if (o == null)
        continue;
      if (value.equals(o.trim()))
        return true;
    }
    return false;
  }

  private static SendableChooser<String> buildChooser(String preferredSelection, String[] options) {
    SendableChooser<String> ch = new SendableChooser<>();

    // If preferredSelection is valid, keep it selected by default; otherwise
    // default to placeholder.
    boolean preferredValid = containsOption(options, preferredSelection);
    if (preferredValid) {
      ch.setDefaultOption(preferredSelection, preferredSelection);
      ch.addOption(CHOOSER_PLACEHOLDER, "");
    } else {
      ch.setDefaultOption(CHOOSER_PLACEHOLDER, "");
    }

    java.util.HashSet<String> seen = new java.util.HashSet<>();
    if (preferredValid)
      seen.add(preferredSelection);

    if (options != null) {
      for (String o : options) {
        if (o == null)
          continue;
        String s = o.trim();
        if (s.isEmpty())
          continue;
        if (seen.add(s)) {
          ch.addOption(s, s);
        }
      }
    }
    return ch;
  }

  /**
   * Keeps Elastic/SmartDashboard dropdown widgets and the stitching backend in
   * sync.
   *
   * Drivers should only use these widgets:
   * - SmartDashboard/Autos/First Destination (dropdown)
   * - SmartDashboard/Autos/Next Path (dropdown)
   * - SmartDashboard/Autos/ClearChain (optional button)
   *
   * Everything else (SelectedSegments, ChainDisplay, and NT table "Autos") is
   * produced by the robot.
   */
  public static void updateElasticAutoDropdowns() {
    // 1) Pull latest option lists from NT table "Autos" (published by
    // TrajectoryHelper)
    String[] firstOptions = SUB_FIRST_DEST_OPTIONS.get();
    String[] nextOptions = SUB_NEXT_OPTIONS.get();

    // Also mirror the raw arrays to SmartDashboard for debugging (drivers should
    // NOT use these as inputs)
    SmartDashboard.putStringArray("Autos/FirstDestinationOptions", firstOptions);
    SmartDashboard.putStringArray("Autos/NextOptions", nextOptions);

    // 2) Rebuild dropdown widgets when their option lists change.
    int firstHash = optionsHash(firstOptions);
    if (firstHash != sLastFirstOptionsHash) {
      sLastFirstOptionsHash = firstHash;
      String keep = firstDestinationChooser.getSelected();
      firstDestinationChooser = buildChooser(keep, firstOptions);
      SmartDashboard.putData("Autos/First Destination", firstDestinationChooser);
    }

    int nextHash = optionsHash(nextOptions);
    if (nextHash != sLastNextOptionsHash) {
      sLastNextOptionsHash = nextHash;
      // For next-path chooser we generally want to reset to placeholder; preferred
      // selection is blank.
      nextPathChooser = buildChooser("", nextOptions);
      SmartDashboard.putData("Autos/Next Path", nextPathChooser);
    }

    // 3) Read driver selections from the dropdown widgets
    String firstDest = firstDestinationChooser.getSelected();
    if (firstDest == null)
      firstDest = "";
    firstDest = firstDest.trim();

    // If the first destination changed, reset chain state.
    if (!firstDest.equals(sLastFirstDest)) {
      sLastFirstDest = firstDest;
      sLastPendingNext = "";
      SmartDashboard.putStringArray("Autos/SelectedSegments", new String[0]);
      SmartDashboard.putString("Autos/ChainDisplay", "");
      SmartDashboard.putBoolean("Autos/ClearChain", false);
      sLastClearChain = false;

      // Reset next chooser to placeholder (even if options didn’t change yet)
      nextPathChooser = buildChooser("", nextOptions);
      SmartDashboard.putData("Autos/Next Path", nextPathChooser);
    }

    String[] selected = SmartDashboard.getStringArray("Autos/SelectedSegments", new String[0]);

    // Optional clear button (recommended so drivers can reset quickly)
    boolean clearPressedNow = SmartDashboard.getBoolean("Autos/ClearChain", false);
    boolean clearRising = clearPressedNow && !sLastClearChain;
    sLastClearChain = clearPressedNow;

    if (clearRising) {
      selected = new String[0];
      SmartDashboard.putStringArray("Autos/SelectedSegments", selected);
      sLastPendingNext = "";

      // Reset next chooser to placeholder
      nextPathChooser = buildChooser("", nextOptions);
      SmartDashboard.putData("Autos/Next Path", nextPathChooser);

      SmartDashboard.putBoolean("Autos/ClearChain", false);
      sLastClearChain = false;
    }

    // 4) Auto-append the chosen next path when the driver picks one from the
    // dropdown.
    String chosenNext = nextPathChooser.getSelected();
    if (chosenNext == null)
      chosenNext = "";
    chosenNext = chosenNext.trim();

    if (!chosenNext.isEmpty() && !chosenNext.equals(sLastPendingNext)) {
      sLastPendingNext = chosenNext;

      // Only append if it is one of the currently valid options
      boolean isValid = containsOption(nextOptions, chosenNext);
      if (isValid) {
        if (selected.length == 0 || !chosenNext.equals(selected[selected.length - 1])) {
          String[] nextSel = java.util.Arrays.copyOf(selected, selected.length + 1);
          nextSel[nextSel.length - 1] = chosenNext;
          selected = nextSel;
          SmartDashboard.putStringArray("Autos/SelectedSegments", selected);
        }

        // Reset the next-path dropdown back to placeholder immediately
        nextPathChooser = buildChooser("", nextOptions);
        SmartDashboard.putData("Autos/Next Path", nextPathChooser);
        sLastPendingNext = "";
      }
    }

    // 5) Produce driver-friendly display string and publish outputs
    StringBuilder sb = new StringBuilder();
    if (!firstDest.isEmpty()) {
      sb.append(firstDest);
      for (String seg : selected) {
        if (seg == null)
          continue;
        String s = seg.trim();
        if (s.isEmpty())
          continue;
        sb.append(" → ").append(s);
      }
    }
    String chainDisplay = sb.toString();
    SmartDashboard.putString("Autos/ChainDisplay", chainDisplay);

    // 6) Publish to NT table "Autos" that TrajectoryHelper / ElasticHelpers read.
    PUB_FIRST_DEST.set(firstDest);
    PUB_SELECTED_SEGMENTS.set(selected);

    // Keep legacy chain string updated for debugging only (NOT a driver input)
    String legacyChain = "";
    if (!firstDest.isEmpty()) {
      StringBuilder sc = new StringBuilder(firstDest);
      for (String seg : selected) {
        if (seg == null)
          continue;
        String s = seg.trim();
        if (s.isEmpty())
          continue;
        sc.append(";").append(s);
      }
      legacyChain = sc.toString();
    }
    PUB_CHAIN.set(legacyChain);
  }
}