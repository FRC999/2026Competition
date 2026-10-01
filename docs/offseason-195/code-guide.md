# Code guide — OffSeason-195

This describes the active 2026 robot code. Start with [README.md](README.md) for setup,
[testing.md](testing.md) for acceptance, and [SESSION_STATE.md](SESSION_STATE.md) for delivery/open work.
The [full audit](full-refactor-audit.md) and [second audit](second-pass-audit.md) explain why it changed.
This guide describes current contracts; those audits preserve historical problems and evidence.

## Responsibilities and flow

| Area | Entry points | Contract |
|---|---|---|
| Lifecycle and bindings | `Robot`, `RobotContainer` | Assemble hardware, schedule/cancel commands, resolve operator actions and cache the enclosing AUTO deadline. |
| Localization | `VisionFactory`, `VisionIOPhotonVision`, `Vision`, `VisionPolicy`, `LocalizationBootstrap` | Read calibrated observations, validate capture time/geometry, establish disabled field reference and supply weighted corrections. |
| Drive | `DriveSubsystem`, `PrecisionDrive` | Own CTRE requests, reference trust, pose history, operator perspective and the single FPGA-to-CTRE timestamp conversion. |
| Paths and endpoints | `PrecisionPathCommands`, `StopAtRouteEnd`, `DriveToPosePrecisionCommand` | Resolve alliance once; follow a route; distinguish brake-only acceptance from corrective alignment; hold on failure. |
| Shot intent and decisions | `ShootWhileHeld`, `ShotIntent`, `AutoShootSupervisorSubsystem`, `ShotReadiness` | A command requests a shot; the supervisor plans and reevaluates every feed gate each loop. |
| Shot calculations | `AimGeometry`, `MovingAimModel`, `ShotPlanner`, `ShotTable`, `ShotFlightTimeTable` | Produce candidate setpoints from explicit observations and calibration. No actuator or feed authorization. |
| Mechanisms | Intake, turret, hood, shooter, transfer, spindexer and climb subsystems | Own device configuration, trust and guarded output. Temporary modes must unwind on interruption. |
| Desktop simulation | `RotaryMotorSim`, `VisionIOPhotonVisionSim`, drive notifier | Supply synthetic sensors through normal control/vision paths without affecting real IO selection. |

The normal data flow is camera observations → vision policy → drive estimator → shot planner →
continuous readiness → mechanisms. Driver/auto commands supply intent, not a second parallel shot
calculation. `calculateDiagnosticSolution()` shares the planner and must remain read-only.

The scheduler runs subsystem `periodic()` and command lifecycle methods on one robot thread.
Do not assume a particular relative ordering of different subsystem periodic callbacks. Keep state
changes and command scheduling on that thread. The drive simulation notifier is a separate 5 ms
thread; placement and truth updates share synchronization, and truth reads are volatile. Close the
notifier before the drivetrain and close vision cameras with their owner.

## Frames, units and time

| Value | Convention |
|---|---|
| Field pose, path coordinates and targets | Blue-origin field frame for both alliances; meters and WPILib rotations. |
| Robot vectors and chassis input | +X forward, +Y left; positive omega counterclockwise. Robot-relative m/s and rad/s unless named field-relative. |
| Turret mount | Retained measured robot-frame offset: negative X behind, negative Y right. Rotate this vector by robot heading before adding field position. |
| Turret commands | Degrees relative to mechanical zero. Aim limits differ from the commanded/measured perimeter window. |
| Hood commands | Radians internally; measured shot CSV and camera JSON rotations use degrees at their parsing boundaries. |
| Shooter | Motor RPM; intake roller interfaces explicitly identify duty or mechanism RPS. |
| Camera timestamps and pose history | FPGA capture seconds. Connection/reception time is not capture time. |
| Estimator fusion time | `DriveSubsystem.addVisionMeasurement` converts FPGA time to CTRE time exactly once. |
| Simulation reduction | Motor rotations per mechanism rotation. `RotaryMotorSim` outputs raw rotor turns/RPS ready for CTRE SimState. |

Resolve alliance in the path/target selection boundary, not again in the estimator. `FieldFrame.ALLIANCE`
respects a source path's `preventFlipping`; `FORCE_RED` explicitly transforms; `ABSOLUTE` preserves
coordinates. A resolved copy disables downstream flipping. Never mutate PathPlanner's cached source.

## Startup, reference and configuration

Real cameras load `2026Competition/src/main/deploy/vision/cameras.json`; desktop simulation loads the
separate `2026Competition/simulation/vision.json`. Configuration is startup-only. Camera transforms
must be measured, and the field-layout acknowledgment must correspond to the same Pi and robot field
selection. A hash is identity evidence, not remote verification. Configuration errors leave manual
driving available with no accepted vision IO; they do not substitute synthetic calibration.

`hasCompetitionAimFrame()` identifies the selected profile. `hasRecentMeasurement()` identifies a
recent accepted capture after the last reset. `isLocalizationReady()` additionally requires a trusted
absolute reference and fresh gyro. These predicates are not interchangeable.

Disabled stationary initialization accepts only stable caller-vetted MultiTag samples. A single healthy
camera can establish reference; fresh camera disagreement blocks it. Enabled heading remains gyro-owned.
A plain `resetPose` revokes reference and clears pose history. Qualified reset wrappers deliberately
restore trust. A gyro reset revokes trust; driver-forward reset only changes input perspective. Estimator
resets never relocate the simulated physical robot.

## Command lifetime and cancellation

- A command declares every mechanism it controls. A group owns the union of its children's requirements
  throughout its entire lifetime, including waiting phases. Supervisor arbitration uses that scheduler owner.
- Every actuator command's `end(interrupted)` must stop outputs or deliberately transfer a documented hold.
  Timeout is an end path too. Interrupted PathPlanner followers need an explicit stop because the underlying
  follower can retain its last request. Never assume command removal alone zeros a controller.
- Operator actions are enabled-teleop only. `FreshPress` requires release/repress across mode/panic gates.
  Falling-edge cleanup checks permission **before scheduling** a required command; `onlyIf` does not prevent
  a denied command from taking requirements and canceling another owner.
- `ShootWhileHeld.end()` clears feed intent and restores the default planning mode. Idle spin remains an
  intentional supervisor behavior. Jam clear owns shooter, transfer and spindexer together and never restores
  an old shoot request. Turret/hood/calibration owners suppress competing supervisor output.
- Intake deployment/retraction is bounded. Success may deliberately coast the deployed pivot or hold the
  retracted position. Interruption/failure stops it. Current/gain boosts restore on every end path; the applied
  CTRE request must leave the boosted slot, not merely clear a Java flag. Homing timeout never proves zero.
- SysId checks permission at schedule time, continuously during execution, and at its output callback. Its stop
  callback is safe even if denied before initialization. Test mode/configuration/panic guards remain required.

## Path finish and shot permission

Competition paths use `ROUTE_STOP`: hold and check actual endpoint/motion/localization without corrective
motion. `PRECISION_ALIGNMENT` is for explicit alignment and may move while settling. Both distinguish success
from timeout/interruption. `failedHold` latches an autonomous failure, inhibits feed and prevents advancing the
sequence until the deadline or mode exit cancels it. Positive-speed joins must agree in position, velocity
direction and holonomic heading. The source path audit generates nominal timing and checks both alliances.

The original full Main/Worlds plans are over budget. The mentor's first-collection-versus-conditional-pickup
choice is still pending; the new documentation/commit requests did not select a strategy. Default AUTO remains
Do nothing. See the timing table in [second-pass-audit.md](second-pass-audit.md).

`ShotPlanner.Solution.valid` means usable mathematics/table coverage, not readiness. `ShotReadiness` returns
the first failing gate; only READY feeds. Gates remain active during FIRING. A clamped turret command does not
make an unreachable original aim valid. No feed timer overrides failed RPM/hood/turret/pose/zone/path checks.

Automatic hub feeding requires confirmed own-alliance-zone position, including estimated release movement.
The mentor retained a driver-confirmed manual fallback when localization is unavailable; its dashboard/log flag
must remain visible. Trench approach inhibition requests neutral hood and latches until a fresh request outside
the guard. Nominal field geometry, padding and lookahead do not certify robot swept clearance or lowering time.

Hub and passing use separate measured shot tables; the pass table is intentionally empty. Malformed settings
reject the complete table. Flight-time interpolation never extrapolates; uncovered distances use the explicitly
labeled retained empirical lead. Neither a simulated hit nor a valid calculation calibrates real flight time.

## Logs and verification

| Symptom | Inspect first |
|---|---|
| Slow or absent localization | `Vision/Initialization/State`, `Vision/Initialization/StableSamples`, per-camera `Startup/FirstConnectedSeconds`, `FirstFrameSeconds`, `FirstPoseSeconds`, `FirstFusionSeconds`, configuration hashes and rejection reasons. |
| Heading/reference trouble | `Drive/FieldReferenceEstablished`, `Drive/FieldReferenceSource`, actual surveyed heading, gyro freshness and reset history. |
| Auto stops advancing | `DriveToPose/Failure`, `Auto/RouteStop/Result`, endpoint/motion errors and the selected finish policy. |
| Shooting inhibited | `AutoShoot/FeedReason`, `ShootRequested`, `TrenchLocked`, `FieldZoneAllowed`, `ManualZoneConfirmationRequired` and individual readiness outputs. |
| Intake state trouble | `Intake/PivotTrusted`, `HardwareConfigured`, `PivotCurrentBoost`, `PivotClosedLoopBoost`, target/measured angle and roller mode. |
| Simulation behavior | `Drive/SimulationTruth`, `Simulation/TotalCurrentAmps`, `Simulation/BatteryVoltage`; separate model assumptions from measured hardware evidence. |

From `2026Competition/`, `gradlew.bat javadoc` renders API documentation to `build/docs/javadoc/index.html`.
Edit Java comments/package documentation, then regenerate; do not edit generated HTML. Use the WPILib Java 17
runtime described in README. Generated docs stay outside the deployed source/data tree.

For behavior changes run the applicable pure/scheduler tests and `gradlew.bat test robotSmoke build` for wiring
changes. The smoke test uses a separate JVM because of static hardware/logger ownership. Physics does not
model validated collision, gravity/hard-stop contact, fuel transport or shot flight. Follow the physical acceptance
steps in testing.md. For comment-only edits, compile/Javadoc and a check that non-comment Java tokens are unchanged
are appropriate; previous behavioral test results are not presented as a new hardware validation.

`tools/change_audit.py` counts physical source lines, including comments. Documentation additions therefore
increase reported Java churn without changing runtime behavior. Its explicit shared-integration bucket avoids
claiming every changed line has a uniquely identifiable vision-versus-strategy cause.
