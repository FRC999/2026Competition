---
layout: default
title: FRC Programming Guide
nav_order: 1
---

# FRC Robot Programming Guide
## MechaRAMS Team 999 — 2026 Rebuilt Season

**Audience:** New programmers with Java knowledge but no FRC experience.

This guide explains how FRC robots are programmed, using the actual MechaRAMS 2026 codebase as examples throughout. Every concept is tied to real files you can open and read. By the end you should be able to navigate the code, understand what any subsystem or command does, and start contributing.

---

## Table of Contents

1. [The Robot's Brain: Hardware Overview](#1-the-robots-brain-hardware-overview)
2. [Project Structure: How the Code Is Organized](#2-project-structure-how-the-code-is-organized)
3. [The Robot Lifecycle: When Code Runs](#3-the-robot-lifecycle-when-code-runs)
4. [Subsystems: Modeling the Robot's Mechanisms](#4-subsystems-modeling-the-robots-mechanisms)
5. [Commands: Making Things Happen](#5-commands-making-things-happen)
6. [PID Control: Making Motors Go Where You Want](#6-pid-control-making-motors-go-where-you-want)
7. [Swerve Drive: How the Robot Moves](#7-swerve-drive-how-the-robot-moves)
8. [Odometry: Knowing Where the Robot Is](#8-odometry-knowing-where-the-robot-is)
9. [Vision and AprilTags](#9-vision-and-apriltags)
10. [Autonomous: Running Without a Driver](#10-autonomous-running-without-a-driver)
11. [SmartDashboard and Telemetry](#11-smartdashboard-and-telemetry)
12. [Putting It All Together: The Shooting Pipeline](#12-putting-it-all-together-the-shooting-pipeline)
13. [Key Vocabulary Reference](#13-key-vocabulary-reference)
14. [Recommended Reading Order](#14-recommended-reading-order)

---

## 0. Introduction

### What Is FRC Programming?

FIRST Robotics Competition (FRC) robots are Java programs that run on a small embedded computer called the **roboRIO**. The program receives joystick inputs from human drivers, reads sensor values, runs control algorithms, and sends commands to motors — all in a loop that repeats every 20 milliseconds.

FRC programming is not just Java. It involves:
- **Hardware abstraction** — talking to motors, sensors, and cameras through vendor APIs
- **Control theory** — PID loops, feedforward, and motion profiling
- **State machines** — organizing complex behaviors into clean, predictable transitions
- **Autonomous programming** — making the robot operate completely on its own for 15 seconds

### The WPILib Command-Based Framework

All modern FRC teams use **WPILib** (the official FRC programming library). WPILib gives us a **command-based framework** — a structured way to organize robot behavior. The two core concepts are:

- **Subsystems** — represent physical mechanisms (the shooter, the intake, the drive base). Each subsystem owns its own hardware and manages its own state.
- **Commands** — represent actions the robot takes (shoot a ball, deploy the intake, follow a path). Commands are scheduled to run and they "require" subsystems to prevent conflicts.

This separation is powerful: the shooter subsystem always knows how to spin at a given RPM, and any command can request that. Commands don't need to know *how* the motors work — just which subsystem method to call.

---

## 1. The Robot's Brain: Hardware Overview

Before reading any code, you need to understand the physical hardware the code controls.

### The roboRIO

The **roboRIO** is a credit-card-sized embedded computer from National Instruments that runs the robot's Java program. It has:
- A dual-core ARM processor running Linux
- Digital and analog I/O pins for sensors
- USB, Ethernet, and RS-232 ports
- A **CAN bus** port — the main communication channel to all motor controllers and sensors

Think of the roboRIO as the robot's brain. Every sensor reading comes in through it, and every motor command goes out through it.

### The CAN Bus

The **CAN bus** (Controller Area Network) is a high-speed serial bus that connects the roboRIO to all motor controllers, encoders, and IMUs. Each device on the CAN bus has a unique **CAN ID** (a number from 0–62).

In our code, all CAN IDs are defined in `Constants.java`. Here is a map of every device:

| CAN ID(s) | Device Type | Component | Purpose |
|-----------|-------------|-----------|---------|
| 11, 13, 15, 17 | TalonFX | Swerve Drive Motors | Spin the wheels forward/backward |
| 12, 14, 16, 18 | TalonFX | Swerve Steer Motors | Rotate each wheel module's direction |
| 21, 23, 25, 27 | CANcoder | Swerve Module Encoders | Measure each wheel's absolute angle |
| 40 | Pigeon2 | Gyroscope (IMU) | Measure robot rotation (yaw/pitch/roll) |
| 41 | TalonFX | Turret Motor | Rotate the turret left/right |
| 42 | TalonFX | Hood Motor | Tilt the shooter's launch angle |
| 43, 44 | TalonFX (Kraken X60) | Shooter Flywheels | Launch the ball at high speed |
| 45 | CANcoder | Turret Encoder | Absolute turret angle |
| 50 | TalonFX | Spindexer | Circulate balls in the hopper |
| 51, 52 | TalonFX | Transfer Motors | Move balls into the shooter |
| 54, 55 | TalonFX | Intake Roller Motors | Spin intake rollers to collect balls |
| 56, 57 | TalonFX | Intake Pivot Motors | Deploy/retract the intake arm |
| 60, 61 | TalonFX | Climb Motors | Extend/retract the climbing arms |

We also use separate **DIO** (Digital Input/Output) pins on the roboRIO:
- DIO 1, 2: Infrared (IR) beam-break sensors in the ball transfer path — when a ball crosses a beam, the circuit breaks and the roboRIO detects it.

### Motor Controllers: TalonFX / Kraken X60

You cannot just "set a motor speed" directly. Every motor has a **motor controller** (the TalonFX) that:
1. Accepts a command over CAN (e.g., "spin at 50 RPS")
2. Reads built-in encoder feedback at 1000 Hz
3. Runs a PID loop *inside the controller* to achieve the target
4. Converts the calculated output to a PWM voltage signal to the motor

The **Kraken X60** is the motor itself — a brushless DC motor made by CTR Electronics. The TalonFX is the motor controller integrated into the Kraken. When you see `TalonFX` in code, that *is* the Kraken.

We use **Phoenix 6** — CTRE's Java API for TalonFX — for all motor control. The key import is `com.ctre.phoenix6`.

### Absolute Encoders: CANcoder

A **CANcoder** is an absolute rotary encoder. Unlike a motor's built-in incremental encoder (which loses its position at power-cycle), a CANcoder always knows the exact angle — even after the robot is turned off and back on. This is critical for the swerve modules and turret, which must know their angles at startup.

### The Pigeon2 IMU

The **Pigeon2** is an Inertial Measurement Unit (IMU) — it measures the robot's **yaw** (rotation around the vertical axis), **pitch** (forward/back tilt), and **roll** (left/right tilt). The yaw angle is critical for field-centric driving and odometry. CAN ID: 40.

### IR Beam-Break Sensors

These are simple digital sensors: when an object (like a ball) blocks the infrared beam, the roboRIO reads `true`; when clear, it reads `false`. They are wired to DIO ports 1 and 2 and tell the transfer subsystem whether a ball is present at the entry or throat of the conveyor.

### The Driver Station

The **Driver Station** is a laptop running FRC's official software. It:
- Enables/disables the robot (the robot does nothing while disabled)
- Sends joystick/controller data to the roboRIO over the Ethernet/Wi-Fi link
- Selects the robot's operating mode (disabled, autonomous, teleop, test)
- Provides a console for printing robot messages

Our robot uses:
- **Xbox controller** (port 5): primary driver control
- **Button box** (port 4): operator auxiliary controls (12 buttons/toggles)

### Power Flow

```
12V LiPo Battery
    → Power Distribution Hub (PDH)
        → TalonFX Motor Controllers (one per motor)
            → Motors (mechanical output)
        → roboRIO (logic power)
        → Limelight cameras (vision)
        → Other electronics
```

Everything runs on the same 12V battery. Under heavy load (multiple motors accelerating simultaneously), battery voltage can drop, which is why we simulate this in code (see `Robot.java` `simulationPeriodic()`).

---

## 2. Project Structure: How the Code Is Organized

```
2026Competition/
├── src/main/java/frc/robot/
│   ├── Robot.java                     ← Main robot class; entry point
│   ├── RobotContainer.java            ← Creates all subsystems; wires button bindings
│   ├── Constants.java                 ← All numeric constants (PID gains, CAN IDs, etc.)
│   ├── Controller.java                ← Xbox controller wrapper with deadbanding
│   ├── subsystems/                    ← One file per physical mechanism
│   │   ├── DriveSubsystem.java
│   │   ├── TurretSubsystem.java
│   │   ├── ShooterSubsystem.java
│   │   ├── HoodSubsystem.java
│   │   ├── IntakeSubsystem.java
│   │   ├── TransferSubsystem.java
│   │   ├── SpindexerSubsystem.java
│   │   ├── ClimbSubsystem.java
│   │   └── AutoShootSupervisorSubsystem.java
│   ├── commands/                      ← ~70 command files
│   │   ├── DriveManuallyCommand.java  ← Default teleop driving
│   │   ├── ShootWhileHeld.java        ← Shoot while button is held
│   │   ├── DeployIntakeSequence.java  ← Deploy intake to field position
│   │   ├── AutoWorldsHubSweepBlue.java  ← Full autonomous routine
│   │   └── ...
│   ├── OdometryUpdates/               ← Vision and pose estimation
│   │   ├── OdometryUpdatesSubsystem.java  ← Master odometry state machine
│   │   ├── LLAprilTagSubsystem.java       ← Limelight AprilTag processing
│   │   ├── QuestNavSubsystem.java         ← QuestNav local odometry
│   │   └── LLAprilTagConstants.java       ← Vision tuning constants
│   └── lib/                           ← Utility helpers
│       ├── LimelightHelpers.java      ← Limelight API wrapper
│       └── VisionHelpers.java         ← Vision pose utilities
├── src/main/deploy/
│   ├── pathplanner/paths/             ← PathPlanner trajectory JSON files (40+ paths)
│   ├── pathplanner/autos/             ← PathPlanner auto sequence files
│   └── artillery/                     ← Ballistics tables (CSV): distance → RPM/angle
└── vendordeps/                        ← External library descriptors
    ├── Phoenix6.json                  ← CTRE Phoenix 6 (motor controllers)
    ├── PathplannerLib.json            ← PathPlanner (autonomous paths)
    └── ...
```

### The Four Most Important Files

**`Robot.java`** — The entry point. It manages the match lifecycle (autonomous, teleop, disabled, test). Almost no robot logic lives here; it mostly delegates to `RobotContainer` and the command scheduler.

**`RobotContainer.java`** — The "wiring harness" of the robot. All subsystem objects are created here as `public static final` fields. All button-to-command bindings are set up here in `configureBindings()`. This is the first place to look when you want to understand "what does pressing button X do?"

**`Constants.java`** — All magic numbers. PID gains, CAN IDs, motor limits, field geometry, ballistics — nothing is hard-coded in logic files. When you want to tune the turret's kP, you change it in `Constants.java`.

**`subsystems/`** — Each file owns one mechanism. These files are where hardware is configured and where state machines live.

---

## 3. The Robot Lifecycle: When Code Runs

### Match Phases

An FRC match follows this sequence:

```
[DISABLED] → [AUTONOMOUS] (15 sec) → [TELEOP] (2 min 15 sec) → [DISABLED]
                                ↑
                   (optional TEST mode for debugging)
```

- **Disabled**: Robot is powered but not running. Motors are neutralized. The code still runs — odometry and vision can initialize during this phase.
- **Autonomous**: No human input. The robot runs pre-programmed commands for 15 seconds.
- **Teleop**: Drivers control the robot via joysticks.
- **Test**: A special mode used for mechanism tuning (SysId characterization, calibration).

### The 20 ms Control Loop

The WPILib scheduler runs every **20 milliseconds** (50 Hz). On each cycle:

1. `robotPeriodic()` is called — this runs `CommandScheduler.getInstance().run()`, which:
   - Polls all active `Trigger` conditions (button presses, sensor thresholds)
   - Runs `periodic()` on every registered subsystem
   - Runs `execute()` on every scheduled command
   - Checks `isFinished()` on every running command

This is the heartbeat of the robot. Everything happens in response to this 20 ms clock.

### The Lifecycle Methods in Robot.java

```java
public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        // Called ONCE at robot power-on.
        // We call setIfAllianceRed() to read the Driver Station alliance color.
    }

    @Override
    public void robotPeriodic() {
        // Called every 20ms REGARDLESS of mode (even disabled).
        // This is where CommandScheduler.run() lives — the engine of everything.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        // Robot is disabled but code still runs.
        // We seed field-relative driving and trigger Limelight re-anchoring here.
        m_robotContainer.driveSubsystem.seedFieldRelativeOnce();
        if (m_robotContainer.driveSubsystem.hasFinishedSeeding()) {
            m_robotContainer.odometryUpdateSubsystem.handlePostYawSeed();
        }
    }

    @Override
    public void autonomousInit() {
        // Called ONCE when autonomous mode starts.
        // We retrieve the selected auto command from the chooser and schedule it.
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        // Called ONCE when teleop starts.
        // Cancel the autonomous command so it doesn't fight with the driver.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
}
```

> **Key insight:** Notice that `autonomousPeriodic()` and `teleopPeriodic()` are empty. All the work happens via commands and subsystems through `robotPeriodic()` → `CommandScheduler.run()`. You rarely put logic directly in the `*Periodic()` methods.

### Why LoggedRobot Instead of TimedRobot?

We extend `LoggedRobot` from the **AdvantageKit** library instead of WPILib's `TimedRobot`. AdvantageKit logs every sensor input and robot state to a file at every loop cycle. This enables **replay** — you can feed a match log back into the code and re-simulate exactly what happened. This is invaluable for debugging: "why did the turret miss that shot at 2:14 in the semifinals?" — just replay the log.

### The Command Scheduler: The Engine

`CommandScheduler.getInstance()` is a singleton that:
- Maintains a list of all active commands and their required subsystems
- Calls `execute()` on each active command every loop
- Cancels old commands when new ones require the same subsystem
- Calls `periodic()` on every registered subsystem

You never call `CommandScheduler` directly in most code — it happens automatically through `robotPeriodic()`.

---

## 4. Subsystems: Modeling the Robot's Mechanisms

### What Is a Subsystem?

A subsystem represents one physical mechanism on the robot. The rule is: one physical mechanism = one subsystem class.

Our robot has these subsystems:

| Subsystem | File | What It Controls |
|-----------|------|-----------------|
| DriveSubsystem | `DriveSubsystem.java` | 4-wheel swerve drivetrain |
| TurretSubsystem | `TurretSubsystem.java` | Rotating turret platform |
| ShooterSubsystem | `ShooterSubsystem.java` | Dual flywheel ball launcher |
| HoodSubsystem | `HoodSubsystem.java` | Shooter launch angle |
| IntakeSubsystem | `IntakeSubsystem.java` | Ball collection arm |
| TransferSubsystem | `TransferSubsystem.java` | Ball conveyor to shooter |
| SpindexerSubsystem | `SpindexerSubsystem.java` | Ball circulation in hopper |
| ClimbSubsystem | `ClimbSubsystem.java` | End-game climbing arms |
| AutoShootSupervisorSubsystem | `AutoShootSupervisorSubsystem.java` | Shot sequencing logic |
| LLAprilTagSubsystem | `LLAprilTagSubsystem.java` | Limelight vision |
| QuestNavSubsystem | `QuestNavSubsystem.java` | Quest visual odometry |
| OdometryUpdatesSubsystem | `OdometryUpdatesSubsystem.java` | Vision fusion state machine |

### Anatomy of a Subsystem

Every subsystem follows this pattern:

```java
public class SpindexerSubsystem extends SubsystemBase {

    // 1. STATE MACHINE ENUMS — all possible operating modes
    private enum DesiredMode {
        OFF,
        BASE_FORWARD,
        SUPPLY_FORWARD,
        MANUAL_VELOCITY,
        OPEN_LOOP_DUTY
    }

    private enum AntiJamState {
        NORMAL,
        UNJAM_REVERSE,
        UNJAM_SETTLE_FORWARD,
        COOLDOWN
    }

    // 2. HARDWARE OBJECTS
    private TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

    // 3. CURRENT STATE
    private DesiredMode desiredMode = DesiredMode.OFF;
    private AntiJamState antiJamState = AntiJamState.NORMAL;
    private double targetRps = 0.0;

    // 4. CONSTRUCTOR — hardware setup called once
    public SpindexerSubsystem() {
        motor = new TalonFX(Constants.OperatorConstants.Spindexer.MOTOR_ID);
        configureHardware();   // sets PID gains, current limits, etc.
    }

    // 5. PERIODIC — runs every 20ms
    @Override
    public void periodic() {
        // Read sensors, update state machine, apply motor outputs
        updateAntiJamStateMachine();
        applyMotorOutput();
        updateTelemetry();
    }

    // 6. PUBLIC API — methods commands call
    public void runBaseForward() { desiredMode = DesiredMode.BASE_FORWARD; }
    public void stop()           { desiredMode = DesiredMode.OFF; }
    public boolean isRunning()   { return desiredMode != DesiredMode.OFF; }
}
```

**Key principles:**

1. **Extending `SubsystemBase`** automatically registers the subsystem with the command scheduler so its `periodic()` is called every loop.

2. **`configureHardware()`** is called once in the constructor. It applies all Phoenix 6 motor configurations (PID gains, current limits, inversion, brake mode) to the hardware. If you ever change a PID gain in `Constants.java`, it only takes effect because `configureHardware()` reads the constant and applies it.

3. **State machines live in subsystems, not commands.** The `DesiredMode` and `AntiJamState` enums above show this clearly. The state persists between command calls — if a command sets the spindexer to `BASE_FORWARD` and then ends, the subsystem remembers that state until something explicitly changes it. Commands just poke the state; the subsystem runs the machinery.

4. **Public API, not raw motors.** External code (commands, other subsystems) should never write to `motor` directly. They call named methods like `runBaseForward()`. This encapsulation means you can change the hardware implementation without breaking any command.

5. **`addRequirements()`** is called in the command's constructor (not in the subsystem) and tells the scheduler which subsystem a command "owns." Only one command can own a subsystem at a time.

### The Enabled Subsystems Gate

Notice this pattern at the top of every subsystem constructor:

```java
public SpindexerSubsystem() {
    if (!EnabledSubsystems.spindexer) {
        return;  // bail out early — don't create hardware objects
    }
    // ... hardware setup
}
```

In `Constants.java`, `EnabledSubsystems` is a class of booleans:

```java
public static final class EnabledSubsystems {
    public static final boolean spindexer = true;
    public static final boolean climber   = false;  // disabled this season
    // ...
}
```

Setting a subsystem to `false` safely disables it without removing any code — great for testing with hardware missing.

---

## 5. Commands: Making Things Happen

### What Is a Command?

A command represents one action the robot takes. Commands are the "verbs" of the system; subsystems are the "nouns." Examples:
- "Deploy the intake" — `DeployIntakeSequence`
- "Shoot while the trigger is held" — `ShootWhileHeld`
- "Drive to a waypoint" — a PathPlanner follow command
- "Stop everything" — `StopRobot`

### The Command Lifecycle

Every command has four methods that the scheduler calls at specific times:

```java
@Override
public void initialize() {
    // Called ONCE when the command is first scheduled.
    // Set up initial state, start timers, send the first motor command.
}

@Override
public void execute() {
    // Called every 20ms while the command is running.
    // Read sensors, update setpoints, run control logic.
}

@Override
public void end(boolean interrupted) {
    // Called ONCE when the command ends — either naturally or when interrupted.
    // 'interrupted' is true if another command cancelled this one.
    // Clean up: stop motors, reset state.
}

@Override
public boolean isFinished() {
    // Called every 20ms. Return true to end the command naturally.
    // Return false to keep running.
}
```

Timeline:

```
schedule() → initialize() → [execute() → isFinished()?] → ... → end(interrupted)
                                    ↑______________________________|
                                    (repeats every 20ms until done)
```

### A Simple Command: DeployIntakeSequence

Here is the complete source of `DeployIntakeSequence.java`:

```java
public class DeployIntakeSequence extends Command {
    private final Timer timeoutTimer = new Timer();
    private final boolean useTeleopPowerBoost;

    public DeployIntakeSequence(boolean useTeleopPowerBoost) {
        this.useTeleopPowerBoost = useTeleopPowerBoost;
        addRequirements(RobotContainer.intakeSubsystem);  // claim the intake
    }

    @Override
    public void initialize() {
        timeoutTimer.restart();                            // start a safety timer
        RobotContainer.intakeSubsystem.stopIntake();
        RobotContainer.intakeSubsystem.setIntakePositionWithAngle(IntakePositions.IntakeDeployedDeg);
    }

    @Override
    public void execute() {
        // Nothing to do here — the subsystem's periodic() drives the motor
        // toward the position target on its own.
    }

    @Override
    public void end(boolean interrupted) {
        timeoutTimer.stop();
        if (!interrupted) {
            RobotContainer.intakeSubsystem.releaseDeployHoldToCoast();
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when the pivot is at the target position, or if we've waited too long
        return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeDeployedDeg)
            || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_POSITION_COMMAND_TIMEOUT_SEC);
    }
}
```

Notice:
- `addRequirements(intakeSubsystem)` means no other command can use the intake while this one runs.
- `initialize()` just sends the target position — the motor's internal PID loop does the actual work.
- `execute()` is empty because the subsystem's `periodic()` handles ongoing motor control.
- `isFinished()` has two exit conditions: success (at position) and safety timeout.

### A Complex Command: ShootWhileHeld

`ShootWhileHeld.java` is a more complex example. It:
- Requires five subsystems simultaneously (shooter, hood, transfer, spindexer, turret)
- Optionally requires the drive subsystem to hold a heading while shooting
- Uses a WPILib `PIDController` for heading correction
- Runs indefinitely until the button is released (`isFinished()` always returns false)

```java
public class ShootWhileHeld extends Command {
    private final PIDController headingPid;

    public ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode mode, boolean holdDriveHeading) {
        addRequirements(
            RobotContainer.shooterSubsystem,
            RobotContainer.hoodSubsystem,
            RobotContainer.transferSubsystem,
            RobotContainer.spindexerSubsystem,
            RobotContainer.turretSubsystem);

        if (holdDriveHeading) {
            addRequirements(RobotContainer.driveSubsystem);
        }

        headingPid = new PIDController(
            Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kP,  // 0.08
            Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kI,  // 0.0
            Constants.OperatorConstants.AutoShoot.STATIC_HOLD_HEADING_kD); // 0.0
        headingPid.enableContinuousInput(-180.0, 180.0);  // handles 359→0 wrapping
    }

    @Override
    public void initialize() {
        RobotContainer.autoShootSupervisorSubsystem.setShotMode(mode);
        RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);

        if (holdDriveHeading) {
            headingSetpointDeg = RobotContainer.driveSubsystem.getYaw();
            headingPid.reset();
            headingPid.setSetpoint(headingSetpointDeg);
        }
    }

    @Override
    public void execute() {
        if (!holdDriveHeading) return;

        double currentDeg = RobotContainer.driveSubsystem.getYaw();
        double omegaDegPerSec = headingPid.calculate(currentDeg);
        omegaDegPerSec = MathUtil.clamp(omegaDegPerSec, -maxOmega, +maxOmega);
        RobotContainer.driveSubsystem.drive(0.0, 0.0, Math.toRadians(omegaDegPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
        RobotContainer.autoShootSupervisorSubsystem.setShotMode(ShotMode.MOVING_AUTO);
        if (holdDriveHeading) {
            RobotContainer.driveSubsystem.drive(0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // runs until button is released, which cancels the command
    }
}
```

### Command Types Reference

| Type | Use Case | Example |
|------|----------|---------|
| `InstantCommand` | Single action, no looping | Zero the gyro, print diagnostics |
| `Command` (base class) | Custom multi-step logic | `ShootWhileHeld`, `DeployIntakeSequence` |
| `SequentialCommandGroup` | Chain commands one after another | All autonomous routines |
| `Commands.parallel()` | Run commands simultaneously | Drive + intake at same time |
| `Commands.race()` | Run until the first command finishes | `pathFollow.raceWith(new WaitCommand(5))` |
| `Commands.defer()` | Create command lazily at schedule-time | When the command constructor needs live data |
| `WaitCommand` | Pause for N seconds | Delays between auto steps |

### Button Bindings with Triggers

In `RobotContainer.java` `configureBindings()`, every button is wired to a command via the `Trigger` system:

```java
// Right trigger (RT, axis > 0.3) held → shoot
new Trigger(() -> xboxDriveController.getRawAxis(3) > 0.3)
    .whileTrue(new ShootWhileHeld(ShotMode.MOVING_AUTO, false));

// Y button pressed once → retract intake
new JoystickButton(xboxDriveController, XboxController.Button.kY.value)
    .onTrue(new RetractIntakeSequence());

// Left bumper held → reverse intake
new JoystickButton(xboxDriveController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new ReverseIntake());
```

The three binding styles:
- `.onTrue()` — fires once when the condition becomes true (button press edge)
- `.whileTrue()` — fires while the condition is true; cancels when condition goes false (button held)
- `.onFalse()` — fires once when the condition becomes false (button release edge)

### Default Commands

Every subsystem can have a **default command** — a command that runs whenever no other command requires that subsystem. The drivetrain's default command is manual joystick control:

```java
driveSubsystem.setDefaultCommand(
    new DriveManuallyCommand(
        () -> getDriverXAxis(),    // forward/backward
        () -> getDriverYAxis(),    // left/right strafe
        () -> getDriverOmegaAxis() // rotation
    )
);
```

When an auto command takes over the drivetrain, `DriveManuallyCommand` stops. When the auto command finishes, `DriveManuallyCommand` automatically resumes.

---

## 6. PID Control: Making Motors Go Where You Want

This is the most important control theory concept in FRC. Without PID, motors overshoot, oscillate, or can't hold a position.

### The Problem PID Solves

Imagine you want to rotate the turret to 45 degrees. If you just apply full power until you get close, then cut power, the motor's momentum carries it past 45. So you apply power in reverse — now it overshoots the other way. Without feedback control, the turret oscillates forever.

**PID** (Proportional-Integral-Derivative) is a feedback loop that continuously calculates how much motor output to apply based on the *error* between where you are and where you want to be.

```
error = setpoint − measured_value
output = kP×error + kI×∫error dt + kD×(d(error)/dt)
```

### The P Term: Proportional

```
P_output = kP × error
```

- If the turret is 40° away from target: apply a lot of power.
- If the turret is 2° away: apply a little power.
- The larger `kP`, the stronger the response.

**Too small kP:** The turret moves slowly and may never reach the target (doesn't overcome friction).  
**Too large kP:** The turret overshoots, corrects, overshoots again — it oscillates.  
**Just right kP:** The turret moves quickly and settles close to the target.

### The I Term: Integral

```
I_output = kI × (sum of all past errors × time)
```

The I term accumulates error over time. If the turret never quite reaches 45° (it stops at 43° because friction balances the P output), the integral builds up and adds extra push to close the gap.

**In FRC, you almost always set kI = 0.** Integral windup is a real problem — if a mechanism is blocked from moving, the integral term spikes to a huge value and causes violent motion when the blockage clears. Instead, we use **feedforward** (explained below) to handle steady-state error.

### The D Term: Derivative

```
D_output = kD × (change in error / change in time)
```

The D term reacts to how *fast* the error is changing. When the turret is approaching its target quickly (error decreasing fast), the D term applies a braking force to prevent overshoot.

**Think of D as a shock absorber.** Too little D: bouncy/oscillating. Too much D: amplifies sensor noise and causes jitter.

### Understanding PID Response Visually

```
Position
   ↑
   |  .....setpoint.....................................
   |         ___
   |        /   \___
   |       /        \_____————————————  (critically damped — ideal)
   |      /
   |     /  ↗ (overdamped: P too low — slow, sluggish)
   |    /
   |   /___/\/\/\/\/\/\/\______________ (underdamped: P too high — oscillates)
   |
   +————————————————————————————————→ Time
```

### Feedforward: Knowing What to Expect Before You Measure

PID is *reactive* — it corrects after seeing an error. **Feedforward** is *predictive* — it applies output based on what you expect the mechanism needs, before measuring error.

For motor control, Phoenix 6 uses:
- **kS** (static friction) — minimum voltage to overcome stiction and start the motor moving
- **kV** (velocity feedforward) — voltage needed to maintain a given speed
- **kA** (acceleration feedforward) — voltage needed to accelerate

The combined output is:

```
total_output = kS×sign(v) + kV×v + kA×a + kP×error + kI×∫error + kD×(d_error/dt)
```

**Example:** The shooter flywheels need to spin at 3000 RPM. Without feedforward, the P term has to do all the work to fight air resistance. With `kV=0.12925` (measured), the motor already knows "apply 0.12925 volts per RPS to hold speed" — the P term only needs to correct small deviations.

### Phoenix 6 Motor PID: Hardware-Embedded

**TalonFX runs its PID loop inside the motor controller at 1000 Hz** — 20× faster than the robot's 50 Hz loop. Motor PID is configured using `Slot0Configs`:

```java
// From TurretSubsystem.java configureHardware():
Slot0Configs slot0 = new Slot0Configs()
    .withKP(Constants.OperatorConstants.Turret.kP)   // 40.0
    .withKI(Constants.OperatorConstants.Turret.kI)   // 0.0
    .withKD(Constants.OperatorConstants.Turret.kD)   // 2.0
    .withKS(Constants.OperatorConstants.Turret.kS)   // 0.0
    .withKV(Constants.OperatorConstants.Turret.kV)   // 0.0
    .withKA(Constants.OperatorConstants.Turret.kA);  // 0.0

turret.getConfigurator().apply(slot0);
```

Control requests tell the motor what to do:

```java
// Position control — move turret to 45 degrees (in motor rotations):
MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
turret.setControl(mmRequest.withPosition(targetRotations));

// Velocity control — spin shooter at 50 RPS:
VelocityVoltage velRequest = new VelocityVoltage(0).withSlot(0);
shooter.setControl(velRequest.withVelocity(50.0));
```

### PID Gains Table for Our Robot

Every tuned PID gain in the robot — all from `Constants.java`:

| Mechanism | Control Type | kP | kD | kS | kV | Notes |
|-----------|-------------|----|----|----|----|-------|
| Turret | MotionMagic position | 40.0 | 2.0 | 0 | 0 | ±110° range |
| Hood | MotionMagic position | 55.0 | 2.0 | 0 | 0 | 0°–62° range |
| Shooter (velocity) | VelocityVoltage | 0.51 | 0 | 0 | 0.129 | Dual Kraken flywheels |
| Intake pivot (deployed) | MotionMagic position | 18.0 | 1.2 | – | – | Slot 1 |
| Intake pivot (retracted) | MotionMagic position | 36.0 | 1.2 | – | – | Slot 2 (stiffer hold) |
| Intake roller | VelocityVoltage | 0.1 | 0.01 | – | 0.12 | |
| Spindexer | VelocityVoltage | 0.51 | 0 | 0 | 0.129 | Same as shooter |
| Transfer | VelocityVoltage | 0.04 | 0 | 0.05 | 0.012 | |
| Swerve steer | Position | 100.0 | 0.5 | 0.1 | 2.49 | Per swerve module |
| Swerve drive | VelocityVoltage | 0.1 | 0 | 0 | 0.124 | Per swerve module |
| Heading hold (ShootWhileHeld) | WPILib PIDController | 0.08 | 0 | – | – | Degrees input/output |
| PathPlanner translation | WPILib PID | 5.0 | 0 | – | – | AutoBuilder config |
| PathPlanner rotation | WPILib PID | 7.0 | 0 | – | – | AutoBuilder config |

### MotionMagic: Smooth Profiled Motion

Plain position PID applies maximum power instantly on large moves → mechanical stress and overshoot. **MotionMagic** generates a **trapezoidal velocity profile** instead:

```
Velocity
   ↑
   |      cruise velocity
   |    /——————————————\
   |   /                \
   |  /  accelerate      \ decelerate
   | /                    \
   +————————————————————————→ Time
```

Configuration:

```java
MotionMagicConfigs mm = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(cruiseRps)   // Turret: 240 RPS cruise
    .withMotionMagicAcceleration(accelRps2);     // Turret: 240 RPS/s² acceleration
```

All position-controlled mechanisms on our robot use MotionMagic: turret, hood, intake pivot, climb.

### SysId: How to Find kS, kV, kA

**SysId** (System Identification) characterizes mechanisms to find feedforward constants. It runs voltage ramps and step tests, then fits the data to the motor model equations to extract kS, kV, kA. The output is constants you paste into `Constants.java`. SysId routines are built into the code — look in `RobotContainer.java` for the test joystick bindings (port 0) that trigger them.

---

## 7. Swerve Drive: How the Robot Moves

### What Makes Swerve Special

A **swerve drive** has four independently steerable wheel modules — each wheel can point in any direction while spinning at any speed. This lets the robot:
- **Translate** in any direction (strafe sideways without turning)
- **Rotate** while translating (spin while driving forward)
- Change direction instantly without stopping

### Swerve Module Anatomy

Each of the four swerve modules contains:
- **Drive Motor (TalonFX):** spins the wheel (forward/backward speed)
- **Steer Motor (TalonFX):** rotates the module direction (0°–360° steering angle)
- **CANcoder (absolute encoder):** knows exact steering angle even after power cycle

Our module CAN IDs:

| Module | Location | Drive ID | Steer ID | CANcoder ID |
|--------|----------|----------|----------|-------------|
| MOD0 | Front Left | 11 | 12 | 21 |
| MOD1 | Front Right | 13 | 14 | 23 |
| MOD2 | Back Left | 15 | 16 | 25 |
| MOD3 | Back Right | 17 | 18 | 27 |

### Module Offsets: The Critical Calibration Value

Each CANcoder has a **magnet offset** — the value that means "wheel pointing forward." These are physically measured by aligning all wheels straight forward, then reading the CANcoder values in Phoenix Tuner X.

```java
// From Constants.java:
public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord(
    11,           // drive motor ID
    12,           // steer motor ID
    21,           // CANcoder ID
    -0.498047,    // ← THE OFFSET: what angle reading means "forward"
    true,         // drive motor inverted
    false,        // steer motor inverted
    false         // CANcoder inverted
);
```

> **If you ever remove and reinstall a wheel module, the magnet offset must be re-measured.** This is one of the most common sources of swerve problems.

### Field-Centric vs Robot-Centric

**Robot-centric:** "Forward" means toward the robot's current front. Confusing for drivers when the robot has rotated.

**Field-centric:** "Forward" on the joystick always means the same direction on the field, regardless of how the robot is oriented. This is how our robot drives.

```java
// Blue alliance: operator forward = field X+ direction
// Red alliance:  operator forward = field X- direction (auto-flipped)
driveSubsystem.setOperatorPerspectiveForward(isRed ? Rotation2d.k180deg : Rotation2d.kZero);
```

### CTRE Phoenix 6 SwerveDrivetrain

We use CTRE's pre-built `SwerveDrivetrain` class. Our `DriveSubsystem` extends it:

```java
public void drive(double xVel, double yVel, double omega) {
    setControl(
        fieldCentricRequest
            .withVelocityX(xVel)       // meters per second
            .withVelocityY(yVel)       // meters per second
            .withRotationalRate(omega) // radians per second
    );
}
```

**Key performance parameters:**
- Wheel radius: 1.9577 inches
- Drive gear ratio: 6.03:1
- Maximum speed: 5.85 m/s (≈ 19 ft/sec)
- Maximum rotation rate: 4.71 rad/s (≈ 270°/sec)

---

## 8. Odometry: Knowing Where the Robot Is

### Why Odometry Matters

The robot needs to know its position on the field to aim at the hub, follow autonomous paths, and make strategic decisions.

### The Field Coordinate System

WPILib uses a standard coordinate system:
- **Origin (0, 0):** Bottom-left corner from the Blue alliance perspective
- **X axis:** Points toward the Red alliance wall
- **Y axis:** Points toward the top of the field
- **Angles:** 0° = facing Red wall; counter-clockwise is positive

```
                    Y
                    ↑
  Blue DS    ——————————————    Red DS
            |                |
            ————————————————
  (0,0) ——————————————————→ X (16.54 m)

  Blue Hub: (4.626, 4.035)   Red Hub: (11.915, 4.035)
```

A `Pose2d` combines position and orientation:

```java
Pose2d robotPose = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(90));
// Robot is at (4m, 3m), facing 90° (toward top of field)
```

### Wheel Odometry: Dead Reckoning

Every 20ms: measure how far each wheel moved → apply kinematics → update position estimate. Accurate over short distances but accumulates error from wheel slip and gyro drift over time.

### Sensor Fusion: Fixing Accumulated Drift

```java
driveSubsystem.addVisionMeasurement(pose, timestamp, standardDeviations);
```

This injects an absolute position fix from the cameras into an **Extended Kalman Filter** (EKF). The EKF blends wheel odometry and vision measurements weighted by their uncertainty.

**Standard deviations** express trust — smaller = more trust:

```java
Matrix<N3, N1> highTrust = VecBuilder.fill(0.1, 0.1, 0.05);  // trust this strongly
Matrix<N3, N1> lowTrust  = VecBuilder.fill(0.5, 0.5, 0.3);   // gentle nudge only

driveSubsystem.addVisionMeasurement(visionPose, captureTimestamp, highTrust);
```

### Our Hybrid Odometry: The 5-State Machine

`OdometryUpdatesSubsystem` selects which sensor is the "primary truth" at any given time:

| State | Meaning |
|-------|---------|
| `INITIALIZE` | Robot just booted; routes to appropriate seeking state |
| `SEEKING_TAGS_Q` | Quest healthy; waiting for Limelight to give first field anchor |
| `SEEKING_TAGS_NO_Q` | No Quest; waiting for Limelight to establish first anchor |
| `CALIBRATED_Q` | Quest is primary; Limelight injects occasional corrections |
| `CALIBRATED_NO_Q` | Limelight-only mode; used when Quest is unavailable |

Key timing values:
- Quest hold timer: 5 seconds before falling back from `CALIBRATED_Q` to `CALIBRATED_NO_Q`
- Re-anchor delay: 2 seconds before requiring fresh anchor after tag loss
- Pose history buffer: 0.6 seconds for latency-compensated vision injection

### Pose Latency Compensation

Camera images take time to process — the pose estimate arrives milliseconds after the image was captured. We stamp each measurement with the *capture* timestamp and inject it at that time, not now:

```java
driveSubsystem.addVisionMeasurement(
    poseEstimate.pose,
    poseEstimate.timestampSeconds,  // ← when the camera saw this (in the past)
    stdDevMatrix
);
```

---

## 9. Vision and AprilTags

### What Are AprilTags?

**AprilTags** are square fiducial markers placed at known fixed positions around the FRC field. Each tag has a unique ID. When a camera detects a tag, it can compute the robot's absolute position on the field.

The 2026 Rebuilt field has tags at:
- Red Reef faces: IDs 6–11
- Blue Reef faces: IDs 17–22
- Coral stations and processors: other IDs

### The Limelight Camera

A **Limelight** is a self-contained smart camera designed for FRC. It runs its own vision pipeline and communicates via **NetworkTables**. Our robot has two: `"limelight-middle"` and `"limelight-right"`. Both are configured with the camera-to-robot-center transform so pose estimates are in robot-center field coordinates.

### MegaTag 1 vs MegaTag 2: The Most Important Vision Concept

#### MegaTag 1 (MT1) — Fully Independent

MT1 uses **only the camera image** to solve for the robot's full 6-DOF pose (x, y, z, roll, pitch, yaw).

**Strengths:**
- Fully independent — doesn't need any other sensors
- Can estimate yaw heading
- With 2+ tags: very accurate

**Weaknesses:**
- With only 1 tag: **pose ambiguity** — two possible robot positions exist for one-tag observations
- Slower and noisier than MT2

**When we use MT1:** During the initial startup **anchor** phase to establish the robot's first absolute field position. We require ≥2 tags and ambiguity < 0.20.

#### MegaTag 2 (MT2) — IMU-Assisted

MT2 uses the camera image **plus the robot's known gyro heading** (from the Pigeon2). Since yaw is known, it only solves for x and y.

**Strengths:**
- Much more stable and accurate for x/y translation
- Works reliably with a single tag
- Less noise

**Weaknesses:**
- **Cannot correct yaw errors** — trusts the gyro completely for heading

**When we use MT2:** During **normal operation** once the initial yaw is well-established.

#### The Workflow in Our Code

```
STARTUP (SEEKING_TAGS state):
    1. Feed gyro yaw to Limelight (IMU Mode SEED)
    2. Wait for MT1 multi-tag observation (≥2 tags, ambiguity < 0.20)
    3. Reset odometry to MT1 pose → CALIBRATED

NORMAL OPERATION (CALIBRATED state):
    1. Continuously feed gyro yaw to Limelight (enables MT2)
    2. Use MT2 for ongoing pose corrections
```

Limelight IMU modes set in code via `ensureIMUMode()`:
- **Mode 1 (SEED):** Feed robot yaw to help Limelight's MT1 solve
- **Mode 2 (TRACKING_INTERNAL):** Limelight's own built-in IMU (fallback)
- **Mode 3 (TRACKING_MT1_ASSIST):** MT1 gently corrects Limelight's IMU (alpha=0.001)

### Standard Deviations: How Much to Trust Vision

Our dynamic trust formula in `LLAprilTagSubsystem.java`:

```
stdDev = 0.08
       + 0.03 × distanceToTagMeters       (farther = less reliable)
       - 0.02 × clamp(tagCount-1, 0, 3)   (more tags = more reliable)
       + 0.25 × ambiguityScore             (more ambiguous = less reliable)
```

### Tag Filtering Rules

| Filter | Threshold | Reason |
|--------|-----------|--------|
| MT1 single-tag ambiguity | must be < 0.20 | Above this, pose is ambiguous |
| Tag distance (operational) | must be < 3.0 m | Far tags are geometrically unstable |
| Tag distance (initial seed) | must be < 4.0 m | Slightly more lenient for first anchor |
| MT1 initial seed | must have ≥ 2 tags | Single tag too ambiguous for seeding |
| MT2 settle time | 5 seconds after MT1 seed | Allow MT1 to anchor before switching |

### QuestNav: The Second Odometry Source

The **QuestNav** is a repurposed Meta Quest VR headset used as a local visual odometry sensor. It uses inside-out SLAM tracking — very smooth and stable, doesn't need field landmarks.

- Transform from robot center: -0.246m X, +0.25m Y, 90° rotation
- Needs field calibration at match start (operator presses button box recal button)
- Acts as primary pose source in `CALIBRATED_Q` state

---

## 10. Autonomous: Running Without a Driver

### PathPlanner

**PathPlanner** is a third-party library for autonomous path planning. The workflow:

1. **Design paths** in the PathPlanner GUI. Draw waypoints on a field image, set velocity constraints. Export as JSON to `src/main/deploy/pathplanner/paths/`.
2. **Load paths** in code: `PathPlannerPath.fromPathFile("MyPath")`
3. **Follow paths** using `AutoBuilder.followPath(path)` — returns a `Command`

```java
// From DriveSubsystem.java configureAutoBuilder():
AutoBuilder.configure(
    () -> getState().Pose,          // where the robot is now
    this::resetPose,                // how to reset odometry
    () -> getState().Speeds,        // current robot speeds
    (speeds, feedforwards) -> setControl(...),  // how to drive
    new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0),  // translation PID
        new PIDConstants(7, 0, 0)), // rotation PID
    config,
    () -> isRed()                   // flip path for Red alliance
);
```

When `isRed()` returns `true`, PathPlanner automatically mirrors all path coordinates — the same path file works for both alliances.

### Autonomous Routine Structure

All our autos are `SequentialCommandGroup` subclasses:

```java
addCommands(
    // Step 1: Reset pose to known starting position
    AutoBuilder.resetOdom(startingPose),

    // Step 2: Drive to hub while running intake
    AutoBuilder.followPath(hubApproachPath)
        .raceWith(new StartIntake()),

    // Step 3: Shoot for up to 4 seconds
    new ShootWhileHeld(ShotMode.HUB_BASE, false)
        .raceWith(new WaitCommand(4.0)),

    // Step 4: Done
    new StopRobot()
);
```

Key patterns:
- `.raceWith()` runs two commands simultaneously and ends when the *first* finishes
- `new WaitCommand(N)` adds a time delay
- Path following + intake running in parallel is very common

### Our Autonomous Strategies

| Auto Name | Description |
|-----------|-------------|
| AutoMainOneRight (B/R) | Primary scoring auto from right starting position |
| AutoWorldsHubSweep (B/R) | Multi-ball collection sweep through hub zone |
| AutoBlueTrenchToOutpostAndShoot | Collect from trench, drive to outpost, shoot |
| AutoBlueHubSimpleMoveAndShoot | Simple: move to hub, shoot preloaded balls |
| AutoBlueMiddleToOutpostAndShoot | Middle start → outpost shot |
| AutoShootOnly | Stay put, shoot preloaded balls only |
| No Auto | Do nothing (always available as fallback) |

---

## 11. SmartDashboard and Telemetry

### SmartDashboard

**SmartDashboard** shows live robot data in a browser or app.

```java
SmartDashboard.putNumber("Shooter/RPM",    shooterRpm);
SmartDashboard.putBoolean("Shooter/Ready", isReady);

// Reading values (for live tuning):
double newKP = SmartDashboard.getNumber("Turret/kP", defaultKP);
```

Data is sent over **NetworkTables** — a key-value store the Driver Station laptop can read in real time.

### Telemetry Gating

Publishing data every loop is expensive. We gate it with booleans in `Constants.DebugTelemetrySubsystems`:

```java
if (Constants.DebugTelemetrySubsystems.turret) {
    SmartDashboard.putNumber("Turret/AngleDeg",        continuousDeg);
    SmartDashboard.putNumber("Turret/TargetDeg",       targetDeg);
    SmartDashboard.putNumber("Turret/VelocityDeg/sec", estVelDegPerSec);
}
```

In competition, most flags are `false`. During calibration, flip the relevant flag to `true` and rebuild.

### AdvantageKit: Replay Logging

We extend `LoggedRobot` which logs every sensor input at every loop cycle to a `.wpilog` file. After a match, download the log file and open it in **AdvantageScope** to see exact values of every sensor at any timestamp — and replay the match to see what the code "thought" was happening. Invaluable for debugging.

---

## 12. Putting It All Together: The Shooting Pipeline

A complete trace of what happens when the driver holds the Right Trigger to shoot:

### Step 1: Button Binding (RobotContainer.java)

```java
new Trigger(() -> xboxDriveController.getRawAxis(3) > 0.3)
    .whileTrue(new ShootWhileHeld(ShotMode.MOVING_AUTO, false));
```

### Step 2: Command Starts (ShootWhileHeld.initialize())

```java
RobotContainer.autoShootSupervisorSubsystem.setShotMode(ShotMode.MOVING_AUTO);
RobotContainer.autoShootSupervisorSubsystem.setShootRequested(true);
```

### Step 3: Supervisor Computes the Shot (AutoShootSupervisorSubsystem.periodic())

Every 20ms, the supervisor:
1. Reads robot pose → computes distance from turret pivot to hub
2. Looks up RPM and hood angle from the ballistics table (`deploy/artillery/` CSV)
3. Sends targets to `turretSubsystem`, `shooterSubsystem`, `hoodSubsystem`
4. Checks readiness: shooter RPM within 3%, hood within 1.5°, turret within 2°
5. When all ready → `VolleyState.FIRING` → commands `transferSubsystem.feedBall()`

### Step 4: Motor Subsystems Execute (periodic() every 20ms)

- **ShooterSubsystem:** TalonFX runs `VelocityVoltage(targetRps)` at 1 kHz. Reports ready at 97% of target.
- **HoodSubsystem:** TalonFX runs `MotionMagicVoltage(targetRotations)`. Reports at-position within tolerance.
- **TurretSubsystem:** TalonFX runs `MotionMagicVoltage(targetRotations)`. Handles ±110° soft limits.
- **TransferSubsystem:** Runs conveyor at full speed. IR beam-break sensors confirm ball transit.
- **SpindexerSubsystem:** Runs `SUPPLY_FORWARD` to push ball up into transfer entry.

### Step 5: Button Released (ShootWhileHeld.end())

```java
RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
RobotContainer.autoShootSupervisorSubsystem.setShotMode(ShotMode.MOVING_AUTO);
```

Supervisor sees `shootRequested = false` → transitions to `IDLE` → all mechanisms return to idle.

### The Full Chain

```
[Driver holds RT]
        ↓
Trigger → ShootWhileHeld.initialize()
        ↓
autoShootSupervisorSubsystem.setShootRequested(true)
        ↓
AutoShootSupervisorSubsystem.periodic():
    ├─ read pose → compute distance to hub
    ├─ lookup RPM + hood angle from ballistics table
    ├─ turretSubsystem.setFieldRelativeAngle()
    ├─ shooterSubsystem.setTargetRpm()
    ├─ hoodSubsystem.setPositionDeg()
    └─ when all ready → transferSubsystem.feedBall()
        ↓
Motors execute via Phoenix 6 PID (1 kHz):
    ├─ TurretSubsystem: MotionMagicVoltage → turret rotates
    ├─ ShooterSubsystem: VelocityVoltage → flywheels spin up
    ├─ HoodSubsystem: MotionMagicVoltage → hood tilts
    └─ TransferSubsystem: DutyCycleOut → ball feeds into shooter
        ↓
IR beam-break confirms ball launched
        ↓
[Driver releases RT] → ShootWhileHeld.end() → all idle
```

---

## 13. Key Vocabulary Reference

| Term | Meaning |
|------|---------|
| **roboRIO** | The robot's main computer (runs your Java code) |
| **CAN bus** | Serial bus connecting all motor controllers and sensors |
| **CAN ID** | Unique number (0–62) identifying each CAN device |
| **TalonFX** | CTRE motor controller integrated in the Kraken X60 motor |
| **Kraken X60** | CTRE brushless motor with integrated TalonFX controller |
| **CANcoder** | Absolute rotary encoder that communicates over CAN |
| **Pigeon2** | CTRE IMU (gyroscope + accelerometer) — measures robot rotation |
| **Phoenix 6** | CTRE's Java API for TalonFX, CANcoder, Pigeon2 |
| **WPILib** | The official FRC programming library |
| **Subsystem** | A class representing one physical mechanism; extends `SubsystemBase` |
| **Command** | A class representing one robot action; extends `Command` |
| **CommandScheduler** | The singleton that runs all commands and subsystem periodics |
| **Trigger** | A boolean supplier that activates commands when true |
| **Default Command** | The command that runs when nothing else requires a subsystem |
| **PID** | Proportional-Integral-Derivative feedback control loop |
| **kP, kI, kD** | PID gain constants (tune these to control response) |
| **kS, kV, kA** | Feedforward constants (static friction, velocity, acceleration) |
| **MotionMagic** | CTRE's trapezoidal motion profiling for smooth position control |
| **SysId** | WPILib tool to characterize mechanisms and find kS/kV/kA |
| **VelocityVoltage** | Phoenix 6 control request for velocity PID |
| **MotionMagicVoltage** | Phoenix 6 control request for profiled position PID |
| **Slot0Configs** | Phoenix 6 configuration block for one set of PID gains |
| **Swerve module** | One steerable wheel unit (drive motor + steer motor + encoder) |
| **Module offset** | CANcoder calibration value: what reading means "wheel forward" |
| **Field-centric** | Driving where joystick "forward" = same field direction always |
| **Robot-centric** | Driving where joystick "forward" = toward robot's current front |
| **Pose2d** | Robot position (x, y) + orientation (Rotation2d) |
| **Odometry** | Estimating robot position by integrating sensor data over time |
| **Dead reckoning** | Estimating position using only wheel encoders + gyro |
| **Sensor fusion** | Combining multiple sensors via Kalman filter for better accuracy |
| **EKF** | Extended Kalman Filter — the math behind sensor fusion in WPILib |
| **AprilTag** | Fiducial marker on the FRC field; cameras use them for localization |
| **MegaTag 1 (MT1)** | Limelight pose mode using camera only; gives full 6DOF including yaw |
| **MegaTag 2 (MT2)** | Limelight pose mode using camera + gyro; more stable, can't fix yaw |
| **Ambiguity** | MT1 metric: how uncertain the pose solution is (0=certain, 1=ambiguous) |
| **Standard deviation** | How uncertain a measurement is; smaller = trust more |
| **QuestNav** | Meta Quest headset repurposed as a local visual odometry sensor |
| **PathPlanner** | Library for drawing and following autonomous paths |
| **SequentialCommandGroup** | A command that runs sub-commands one after another |
| **SmartDashboard** | WPILib dashboard for displaying and reading live robot data |
| **NetworkTables** | FRC's key-value networking protocol between robot and laptop |
| **AdvantageKit** | Logging library that enables match replay for debugging |
| **LoggedRobot** | AdvantageKit's version of `TimedRobot` with logging support |
| **Driver Station** | The laptop app that enables/disables the robot and streams joystick data |
| **Alliance** | Red or Blue — determines which side of the field the team plays on |
| **Field layout** | JSON file defining every AprilTag ID and its field position |

---

## 14. Recommended Reading Order

Read these files in order. Each one builds on the last.

| # | File | What You Learn |
|---|------|---------------|
| 1 | `Robot.java` | The robot lifecycle; how the loop works |
| 2 | `RobotContainer.java` | How subsystems are created; how buttons are wired |
| 3 | `Constants.java` | All tunable values; how the robot is configured |
| 4 | `SpindexerSubsystem.java` | Clean subsystem example with dual state machines |
| 5 | `DeployIntakeSequence.java` | Clean simple command example |
| 6 | `ShootWhileHeld.java` | Complex command with WPILib PID + multi-subsystem |
| 7 | `TurretSubsystem.java` | MotionMagic, CANcoder seeding, position wrapping |
| 8 | `ShooterSubsystem.java` | Velocity control, readiness detection |
| 9 | `DriveSubsystem.java` | Swerve drive, PathPlanner AutoBuilder config |
| 10 | `LLAprilTagSubsystem.java` | MT1 vs MT2, tag filtering, standard deviations |
| 11 | `OdometryUpdatesSubsystem.java` | Full vision fusion state machine |
| 12 | `AutoWorldsHubSweepBlue.java` | Complex autonomous routine combining everything |

---

*MechaRAMS Team 999 · 2026 FRC Rebuilt Season*
