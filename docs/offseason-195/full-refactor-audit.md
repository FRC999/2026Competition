# Full robot audit and refactor — OffSeason-195

Reviewed against Houston robot code `6c4ecb4c196541236e7f3a702e2ad1099a094e1c`, the first retrofit
`5a3e43165acde503481590a9e65a65aa9ad7cb71`, and the 2027 prototype at
`d20594af6fde49686fcbd9ed63250cf94463aaf5`. Completed October 1, 2026.

The review covers the active `2026Competition` robot project: initialization, localization, drive
ownership, path frames and selected autos, targeting and shooting, turret, hood, shooter, intake,
transfer, spindexer, climb, controls, configuration, telemetry and their supporting code. The
historical `QuestVibeGPT` project and the prototype remain unchanged. Hardware dimensions, CAN IDs,
gearing, motor inversions, measured shot rows and PID gains were not replaced with prototype values.

**The result is a software test candidate. No robot was deployed, moved or fired during this work.**
The source establishes several defects; it cannot establish which one caused a particular match
miss without that match's configuration, measurements and logs. Desktop results below establish
code behavior, not physical accuracy or a complete replay of CTRE hardware.

## What changed at the architecture level

- `VisionIOPhotonVision` owns camera decoding; `VisionPolicy` owns frame acceptance;
  `LocalizationBootstrap` establishes a disabled absolute reference; `DriveSubsystem` owns the
  estimator, gyro relationship and driver perspective. Alliance changes never establish field yaw.
- `PrecisionPathCommands.FieldFrame` explicitly resolves path coordinates once. Opening moves use
  the next named path's resolved starting pose. The full path runs before measured settling at a
  stopping endpoint. Failure holds and inhibits autonomous feeding.
- `AimGeometry`, `FieldTargeting`, `MovingAimModel`, `ShotTable` and `ShotPlanner` calculate the shot.
  `ShotIntent` owns the request/trench/external-control state. `ShotReadiness` returns a feed reason
  every loop. The supervisor orchestrates these pieces and publishes results; diagnostics do not
  mutate control state.
- Mechanism commands own their requirements and stop on cancellation. Direct feed/reverse requests
  update the subsystem's actual control mode and telemetry. Position mechanisms reject untrusted
  references after resets; a timeout cannot establish a physical zero.

Primary replacement code: [localization bootstrap](../../2026Competition/src/main/java/frc/robot/subsystems/vision/LocalizationBootstrap.java),
[path frame policy](../../2026Competition/src/main/java/frc/robot/commands/PrecisionPathCommands.java),
[shot planner](../../2026Competition/src/main/java/frc/robot/lib/ShotPlanner.java),
[supervisor](../../2026Competition/src/main/java/frc/robot/subsystems/AutoShootSupervisorSubsystem.java),
[aim geometry](../../2026Competition/src/main/java/frc/robot/lib/AimGeometry.java),
[driver input](../../2026Competition/src/main/java/frc/robot/lib/DriverInput.java), and
[homing evidence policy](../../2026Competition/src/main/java/frc/robot/lib/HardStopHoming.java).

## Why Houston could take much longer to localize

The old path is visible in [Houston's odometry state machine][old-odometry], especially
`selectInitialSeedPoseFromMt1ImuThenMt2` (line 320), `resetRobotPoseFromVision` (483), and
`handleDelayedMegaTag1Recalibration` (756):

1. Enter `INITIALIZE`, then `SEEKING_TAGS_Q` or `SEEKING_TAGS_NO_Q` according to Quest availability.
2. Wait for a qualifying **two-tag MegaTag1** pose, within the configured range. Save that pose and
   rewrite Pigeon/estimator heading from it.
3. Wait a mandatory **2 seconds**, then look for MegaTag2 agreeing with the saved MegaTag1 pose
   within about **0.5 m and 10 degrees**.
4. If agreement fails, fall back to the **saved** MegaTag1 pose. That fallback does not require the
   original capture to still be fresh.
5. After a MegaTag1 fallback, schedule another reanchor **5 seconds** later. Persistent disagreement
   can repeat the seek/wait/fallback cycle. Normal tracking also used small innovation gates, which
   can reject the large correction needed after a wrong anchor.

Separately, old `Robot.disabledPeriodic` seeded yaw from alliance, and driver button 8 had three
bindings capable of resetting yaw. A late alliance update or an operator reset could invalidate a
correct field heading and make an otherwise reasonable turret calculation aim incorrectly.

The [prototype][prototype] has no MegaTag1/MegaTag2/Quest handshake. It drains current PhotonVision
results, checks each observation and feeds the estimator directly, with an explicit disabled
MultiTag seed available. It provides chassis aiming with teaching/empirical flight assumptions;
it does not provide a calibrated competition turret/hood/shooter controller to copy wholesale.

The new competition bootstrap requires a calibrated camera and acknowledged matching field,
disabled stationary robot, fresh trustworthy MultiTag geometry, **at least four distinct samples
spanning 0.10 s**, and agreement within **0.10 m / 3 degrees**. Samples must be no more than 0.25 s
old. These are initial validation thresholds, not a promise of 100 ms power-on localization.
One healthy camera is sufficient; a second camera need not be online. Fresh disagreeing cameras
block automatic seeding instead of picking an arbitrary winner. Far-away starting poses are not
rejected merely because the estimator started at the origin. While enabled, there is no automatic
hard pose reset; the gyro owns heading and ordinary accepted vision corrects position.

The first retrofit's arbitrary early-auto/reset waiting intervals were also removed. Actual frames
captured before a pose reset remain rejected, as do stale, duplicate, future and malformed data.
Simulation truth is independent of estimator resets, so it no longer needs a delay to hide a
reset moving the simulated camera world.

There is still no evidence here about historical camera boot, exposure, Ethernet, CPU, NT connection
or tag visibility delays. Logs now separate these stages: per camera, inspect
`Vision/CameraN/Startup/FirstConnectedSeconds`, `FirstFrameSeconds`, `FirstPoseSeconds`,
`FirstFusionSeconds`, `AcquisitionPhase`, accepted/frame ages and rejection reasons, together with
`Vision/Initialization/State` and `Drive/FieldReferenceSource`. A camera connected with no frames,
a camera with frames but no pose, and a valid pose blocked by configuration are different faults.

## Issue and fix ledger

Types: **F** = functionality/architecture replacement; **L** = logic defect; **T** = implementation,
maintenance or observability. Old paths below are relative to `2026Competition/src/main/java/frc/robot`
at the Houston commit unless another revision is named. A code-confirmed defect is not proof of its
frequency on the physical robot. Test classes live under `2026Competition/src/test/java/frc/robot`.

### Localization, configuration and heading

| ID/type | Old location and concrete problem | Replacement | Evidence / remaining limit |
|---|---|---|---|
| V01 F/L | `OdometryUpdates/OdometryUpdatesSubsystem`, initial seek: MT1 → 2 s → MT2 agreement → old MT1 fallback → 5 s re-seek | Photon IO/policy plus disabled `LocalizationBootstrap`; no LL/Quest runtime | Bootstrap and fusion integration tests cover far initial pose, one healthy camera, staggered cameras, duplicates, disagreement and enabled reset prohibition. Historical delay magnitude still needs logs. |
| V02 L | `Robot.disabledPeriodic`, `DriveSubsystem.seedFieldRelativeOnce`: blind alliance yaw can overwrite field reference | Reference comes from trusted disabled MultiTag or an explicit known field pose; gyro reset revokes it | Full robot smoke changes alliance after seeding without changing field yaw. |
| V03 L | `RobotContainer.configureBindings`, competition bindings and `setYaws`: duplicate button-8 yaw-reset handlers | One teleop-only handler calls `setOperatorPerspectiveForward(currentHeading)` | Smoke verifies operator-forward changes while pose stays fixed. CTRE `seedFieldCentric` was deliberately not substituted: in pinned 26.3.0 it also resets pose rotation. |
| V04 F/T | No separate distinction between “recent camera position” and “trusted absolute heading”; first retrofit only tested freshness | `isLocalizationReady = hasFieldReference && hasRecentMeasurement`; aim, assist, path starts/finishes consume it | Integration tests reject fresh XY without a reference; Pigeon reset requires disabled recovery. |
| V05 L | First retrofit/prototype reset quarantine and early-auto delay reject otherwise fresh post-reset captures | Reject by actual capture time relative to the reset; no arbitrary waiting interval | Fusion tests accept genuinely post-reset frames and reject pre-reset/duplicate/future data. |
| V06 F/T | Camera assumptions embedded in old configuration; changing field JSON alone could retain incompatible aim/path geometry | Startup six-DOF camera config, calibrated/layout acknowledgment, profile/hash checks; welded frame required for current targets/routes | Config tests, survey fit/apply tests and local NT capture tests. Remote Pi agreement still requires operator verification. |
| V07 T | Logging startup disabled; camera failure stages difficult to distinguish | AK WPILOG/NT4, build/config/layout hashes, raw IO/rejections and startup stage timing | Full startup test produces logs; not a roboRIO performance benchmark or full deterministic hardware replay. |

### Driving, field frames and autonomous behavior

| ID/type | Old location and concrete problem | Replacement | Evidence / remaining limit |
|---|---|---|---|
| D01 L | `RobotContainer.runTrajectory2Poses`: robot yaw used as Bezier tangent, reset branch ended at 2 m/s | Geometric tangent separated from holonomic orientation; generated move stops at the goal | Controller/precision tests and source review. |
| D02 L | Same method: direct pose reset followed by `AutoBuilder.resetOdom`; RED reset could flip an already absolute pose | One explicit reset path only when requested; current-pose generated moves never reset themselves | Field-frame matrix verifies ALLIANCE/FORCE_RED/ABSOLUTE and source flags; no second AutoBuilder reset. |
| D03 L | Cached `PathPlannerPath` could retain altered `preventFlipping`; red-authored route could be flipped again | Copy cached path, resolve once, mark only the resolved copy non-flippable | Twelve frame/alliance/prevent combinations; repeat resolution leaves source unchanged. Old unused red-authored outpost auto retired. |
| D04 L | `AutoMainOneRightBlue/Red`, `AutoWorldsHubSweepBlue/Red`: absolute first waypoint plus alliance-flipped later paths could mix sides | Explicit expected-alliance guard; opening approach derives from the exact first resolved path | Path start no longer comes from duplicate rounded `TrajectoryHelper` poses. Wrong/unknown alliance holds. |
| D05 L | Main route's first stored approach point differed from first path by ~27 cm; `BlueNearBump_BlueTrenchRight` → `BlueTrenchRight_BlueTrenchRight2` jumped ~22.4 cm at nonzero speed; next join differed 1.3 cm | First approach uses path start. Two successor anchors/control points translated to match their preceding endpoints | Shipped multi-path routes join within 2 mm on BLUE and RED. Terrain clearance and velocity continuity still require physical route tests. |
| D06 F/L | Timed coarse-path end did not establish settled arrival; default drive could overwrite stopping requests | Complete route/events, then pose/module/gyro/fresh-vision qualification for stopping goals; retain nonzero pass-through ends and measured-angle hold | Precision controller tests cover convergence, motion escape, loss of permission, interruption and timeout. A timeout latches feed inhibition and holds. |
| D07 L | `AutoShootOnly` loaded a path at a fixed spot; selecting it elsewhere could move the chassis | Shoot from current referenced pose with an explicit stopped-drive command | Source/chooser review and stationary default-auto smoke; physical shot validation pending. |
| D08 F/L | Outpost routines scheduled 5+4+10 s of shooting after travel; middle route waited for all intake cycles; no enclosing duration in the command itself | Path is the intake deadline; shoot continuously after arrival for remaining time; selected command has a cached 20 s wrapper and mode-exit cancellation | REBUILT is 20 s AUTO, confirmed from official sources. The draft's 15 s assumption was corrected. Repeated enable smoke catches command-composition reuse. Actual routes still need timing. |
| D09 L | `Controller` cubed input before deadband, including a tiny additive offset; near-center signs could reverse; right stick used wrong shaping choice | Pure `DriverInput`: finite/clamped raw value → raw deadband → optional cubic shaping; no second deadband later | Monotonic, symmetric, bounded/sign/deadband tests. Robot-centric drive now preserves Y input instead of forcing it to zero. |
| D10 L/T | `DriveManuallyCommand` mixed target/bearing calculations and reported a raw turret angle as chassis correction | Shared geometry, known alliance/referenced vision, neutral-stick/low-speed gates, correct turn-to-window telemetry | Geometry tests; fixed 1.5 rad/s assist is retained and needs braking/driver testing. It is teleop-only. |

### Targeting, turret and shooting

| ID/type | Old location and concrete problem | Replacement | Evidence / remaining limit |
|---|---|---|---|
| A01 L | `AutoShootSupervisorSubsystem.getHubCommandRelativeAngleDeg` (343) called state-changing `chooseSoftLimitedEquivalent` (1399); dashboard publication invoked it | Pure `AimGeometry.safeCommand`; one shot planner shared by control and diagnostics | Planner repeatability and full robot diagnostic/ownership assertions. Enabling telemetry cannot set a hidden suppression timer. |
| A02 F/L | Supervisor repeated aim/table/state calculations; old artillery/inverse solver remained even when unused | Pure planner, target selector, intent state and feed reason; small hardware orchestrator | Planner/readiness/intent tests plus complete robot startup. Dead `TurretHelpers` removed after extracting the live measured interpolation. |
| A03 L | Hardcoded blue/red zone thresholds were not the same rotational geometry; static presets could select a neutral target by region | One canonical BLUE-frame zone selector with provisional 0.15 m hysteresis; static/manual presets select HUB | Rotational BLUE/RED and hysteresis tests. Existing target coordinates retained, not represented as newly surveyed. |
| A04 L | Passing used hub-table RPM while replacing its hood setting with 13°, despite no matching measured pass table | Separate `pass_shots.csv`; no pass solution until measured rows exist | Empty/unmeasured passing is invalid in planner tests. This intentionally changes availability: moving-auto passing is inhibited in the shipped branch. |
| A05 L | One-second readiness override and FIRING-state shortcuts allowed feeding after conditions changed | Eleven inputs evaluated each loop: request, solution, pose, path failure, trench, turret trust/aim, RPM, hood, motion and cooldown | Every individual gate tested, including loss during firing. No timed force-ready bypass. |
| A06 L | `ReverseShooterTemporary` saved an old request, then restored it even if the driver released shoot while clearing; supervisor could oppose reverse | Explicit external ownership, consistent requirements, no restored snapshot; fresh shoot request required | Intent tests and full robot schedule/cancel test. No positive supervisor shooter target during reverse ownership. |
| A07 L | Trench lock was entry-edge based, alliance-half dependent and could rearm while inside; RED rectangles reflected only X | Check all four physical regions with rotational geometry; hood neutral/feed inhibited while locked; exit plus fresh request required | Target-region and intent tests cover both alliances, enter/exit and request while inside. Rectangles are provisional point regions, not a measured swept robot envelope. |
| A08 L | Turret small-error early return could leave stale output after stop; clamped feedback concealed overshoot; failed seed fell back to an assumed angle | Send every valid setpoint; continuous unclamped feedback, configured/fresh/trusted position, no untrusted fallback | `TurretMotionPolicyTest`, boot smoke and source review. Physical stow still cannot be inferred from the pinion encoder. |
| A09 L | Turret travel only limited in selected Java commands; resets invalidated integrated angle | Retained ±110° motor envelope enforced in rotor soft limits; auto aim ±105°, feed comfort ±95°; reset revokes trust | Pure sign/limit tests; real motor sign, clearance and zero remain operator checks. |
| A10 L | Hood near-target neutral allowed drift; `stop()` could restore position output next loop; angle maximum exceeded motor soft limit | Hold ordinary targets, explicit IDLE stop, consistent maximum, reset/config/position freshness and disabled/panic checks | Source integration and smoke. Retained 55.8° maximum follows existing motor conversion/limit; it is not a new clearance measurement. |
| A11 L | Shooter readiness could describe an earlier target; tiny target changes reset statistics continuously; stopped follower did not resume | Current-target/fresh-signal check, rolling history across small changes, explicit follower restoration and reset/disable clearing | `ShooterReadinessPolicyTest` plus reverse ownership smoke. Retained ±10% speed tolerance is broad and still needs measurement. |
| A12 F/L | Moving aim omitted the full relationship between rotating offset pivot, inherited release velocity and relative heading at release | `MovingAimModel`: release heading, translated/rotated pivot and omega-cross-offset velocity; measured timing when available | Rotation invariance, stationary backward shot, lateral intercept closure and rotation tests. Constant-field-velocity model does not model acceleration/drag. |
| A13 L | Shot-table distance clamped to end rows; malformed/partial files could silently leave usable calibration; temporary RPM compensation changed hood without measurement | Reject distance extrapolation and whole corrupt table; hash/status/row count; apply measured hood without invented dip compensation | `ShotTableTest`, planner tests. Near-distance lowest-hood / farther closest-preferred-RPM selection retained. Existing all-angle-zero data is still a distance-only assumption. |
| A14 T | Duplicate console diagnostic math diverged from actual trim/solver behavior | Diagnostic command prints shared planner result and actual field bearing | Read-only dashboard command retained; no actuator requirements or control mutation. |

### Intake, feed mechanisms, climb and controls

| ID/type | Old location and concrete problem | Replacement | Evidence / remaining limit |
|---|---|---|---|
| M01 L | `IntakeSubsystem.setPivotNeutralMode` applied a new whole MotorOutput config to change only neutral mode | Use CTRE `setNeutralMode`; preserve inversion/other output fields | Source/API review. This was a configuration-clobber risk; no claim that the old default necessarily differed from the current inversion. |
| M02 L | Intake position seed/config success not propagated; reset could leave cached target trusted; no mechanism-position soft limits | Checked configuration/seed, fresh position trust, reset/cache invalidation, soft limits in mechanism rotations (0 to 53/360) | Integrated startup and code review. Retained ratio means these limits must not be multiplied by gearing again. Physical boot requires retracted placement. |
| M03 L | `IntakeRezeroFromRetractedHardStop` could declare zero after timeout | Bounded homing with minimum motion time and continuous fresh current evidence from both motors; timeout/interruption never seeds | `HardStopHomingTest`. A sustained stall can also be an obstruction: operator must verify unobstructed retract travel. Only this routine bypasses the reverse software limit at retained limited duty. |
| M04 L | Intake position commands could continue closed-loop motion after interruption or timeout; duplicate timeout command obscured behavior | Brake/zero output on interruption or failed timeout; coast only after confirmed deploy success; one retract command | Lifecycle source review and robot wiring smoke; physical inertia still needs testing. Method renamed `stopPivotInBrake` to describe actual behavior. |
| M05 L/T | Intake/transfer/spindexer direct duty calls bypassed desired-mode bookkeeping and telemetry; old velocity/anti-jam state could contradict direct commands | Common guarded duty setters and explicit mode/target telemetry; finite/config/enable/panic checks; resets stop | Integration smoke exercises feed reverse ownership. Hardware jam detection, sensors, duty signs and current limits require operator tests. |
| M06 L/T | Climb configured the supply current limit twice, did not actively establish the intended follower, and used hood enable in sim-current logic | Correct lower-current field, follower command, guards/null-safe diagnostics and correct simulation enable | Compile/startup inspection only. Climb remains disabled; no validated homing, travel limits or follower direction are claimed. |
| M07 L | Panic latch relied on ordinary enabled commands, so a switch transition while disabled could be consumed without updating the latch | Disabled-capable latch updates plus direct switch state; immediate drive/mechanism stop; consistent panic guards | Full robot smoke changes panic while disabled, then enables and verifies stopped shooter. |
| M08 T/L | Unselected old autos, duplicate bindings, nonexistent “until empty” completion, commented sweep code and unused subsystem examples obscured ownership | Remove unused autos/wrappers, duplicate calibration paths, dead artillery code, alternate intake-driver state, obsolete telemetry/helpers and stale constants/comments | Full compilation, route tests, robot wiring test and Git diff. History preserves retired sources. Active controls and selected route strategy remain documented below. |
| M09 T | Optional SysId callbacks did not consistently require test mode/panic clearance | Keep diagnostic APIs behind configuration, test-mode and panic checks; motor callbacks retain trust/output guards | Source/build verification; no SysId run or gain retuning performed. |

## BLUE/RED and path flags

All estimator poses, camera poses and targets retain the BLUE field origin. For the official welded
layout used here, `L = 16.541 m`, `W = 8.069 m`. PathPlanner is explicitly configured to rotational
symmetry: `(x, y, heading) → (L-x, W-y, heading+180°)`.

| Requested frame | BLUE | RED | Source `preventFlipping=true` |
|---|---|---|---|
| `ALLIANCE` | Copy unchanged | Rotate once | Copy unchanged on either alliance |
| `FORCE_RED` | Rotate once | Rotate once | Explicit force still rotates once |
| `ABSOLUTE` | Copy unchanged | Copy unchanged | Copy unchanged |

The resolved copy always has `preventFlipping=true`, so AutoBuilder cannot flip it again. The
source cache is never changed. Reversed travel is a separate PathPlanner property and is preserved;
it does not mean RED. Existing event markers, rotation/point-toward/constraint zones, starting
orientation and end velocity are retained when copying. Explicit known-start resets use that
resolved pose once. Generated current-pose approaches are already absolute and do not reset pose.

Alliance-specific Main/Worlds chooser selections require their named alliance. The outpost and
simple hub routes are BLUE-authored but labeled **Alliance** in the chooser and flip normally on
RED. Unknown alliance or an incompatible field profile holds instead of guessing. The default
remains **Do nothing**.

The enclosing budget is 20 s and the command is cancelled on autonomous mode exit. This matches
[REBUILT match timing][game-timing]; practice DS defaults may still need adjustment. Endpoint settling
adds up to four seconds per stopping goal. Therefore the budget does not establish that every
historical multi-cycle routine completes: log duration per segment, validate transitions/clearance,
and shorten route strategy deliberately if the final scoring step is unreachable in time.

## Aiming math: the pivot signs are correct

The mentor confirmed robot axes: **+X forward, +Y left**, so retained pivot `r=(-0.18,-0.06) m` means
18 cm behind and 6 cm right of chassis origin. With robot field position `p` and field yaw `theta`:

```text
pivot_field = p + R(theta) * r
bearing_field = atan2(target_y - pivot_y, target_x - pivot_x)
turret_angle = wrap(bearing_field - theta - 180 degrees) + aim_trim
```

The `180°` is the retained shooter-facing direction when turret angle is zero. It is subtracted
from the **bearing**, not added to the robot heading when rotating the pivot. Negating the pivot
because zero points backward would be wrong.

| Robot yaw | Pivot displacement in field X,Y |
|---|---|
| 0° | (-0.18, -0.06) m |
| +90° | (+0.06, -0.18) m |
| 180° | (+0.18, +0.06) m |
| -90° | (-0.06, +0.18) m |

Cardinal-heading tests and simultaneous field-rotation tests verify this. No pivot sign reversal
was justified by the audit. Aiming errors can instead arise from incorrect field heading, an
incorrect physical turret zero, biased camera extrinsics, unmeasured hood/RPM combinations or
release prediction. At 3 m, 1° direction error gives about 5.2 cm cross-range error at the target
plane; 3° gives about 15.7 cm. These are geometric illustrations, not measured robot errors.

For moving shots, the model first converts robot-relative velocity to field velocity. It predicts
the chassis heading at release (`theta + omega * releaseDelay`), rotates the pivot by that heading,
and adds `omega × pivotOffsetField` to the ball's inherited translational velocity. It subtracts
radial/lateral inherited displacement from the target vector and calculates turret angle relative
to **release heading**, not impact heading. With a common measured flight time, the planar test
closes the intercept after adding inherited velocity back in.

The shipped flight-time CSV is header-only. The retained 0.12 s release delay and 0.30/0.45 s
radial/lateral lead are labeled empirical assumptions. The hub CSV's measured settings span about
1.45–5.75 m; all turret-angle entries are zero. No new measurements, ballistic fit or moving-hit-rate
claim was manufactured. The pass CSV is separately header-only. Fill it from measured **pass-target**
trials; do not copy hub settings and substitute a different hood angle.

The largest unresolved zero ambiguity is mechanical: an absolute sensor on an 11:1 pinion repeats
each **32.73° of turret travel**. Its nearest-stow branch is unique only within **±16.36°**. Fresh
stable sensor readings cannot identify another repeated branch. Preserve physical stow at boot and
disabled reseed. An output-shaft absolute sensor or independently sensed home would remove that
ambiguity; software changes here do not.

## Current controls and visible behavior changes

| Control | Behavior |
|---|---|
| Driver sticks | Shaped translation/rotation; neutral preserves module-angle stop |
| Driver button 8 | Teleop driver-forward perspective reset; does not rotate field pose |
| RT | Moving hub shot or measured pass selection; stationary chassis assist only with neutral sticks and qualified pose |
| B, tracking enabled | Static tower-base preset with drive heading hold; target remains HUB |
| B / X, tracking disabled | Manual-aim measured 3 m / 4 m presets; static-motion/readiness gates still apply |
| LT | Deploy and run intake; release retracts unless stay-deployed mode is selected |
| A / Y | Select deployed / retracted intake behavior; existing POV power-boost combinations retained |
| LB / RB | Reverse intake / clear transfer-spindexer-shooter; release ends reverse; shoot requires a new press |
| Manual POV left/right | Turret jog while tracking disabled, with position/perimeter/panic guards |
| Button box | Existing trim, initial deploy, rezero and turret-zero bindings retained; panic and disabled vision-seed gating corrected |
| Dashboard | Disabled turret/hood reseeds, manual vision seed, camera jitter capture and read-only shot-plan diagnostics |

Unused alternate calibration-controller mappings were removed; enabling a telemetry flag no longer
implicitly selects another controls map. SysId APIs remain for an explicit future test binding;
they do not authorize an unsupervised characterization run. Climb remains disabled.

## Verification and next physical evidence

The final check receipt and commands are in [testing.md](testing.md). The Java suite includes pure
geometry/state/policy tests, simulated PhotonVision IO and delayed-drive controller tests; the
separate full robot test also exercises automatic reference, late RED alliance, driver perspective,
enabled-seed rejection, shooting/reverse ownership, read-only diagnostics, disabled panic and
repeated auto enable. Python tests cover surveyed layout, fitting, validation/apply and local NT
capture. Startup desktop CAN/joystick/loop-overrun warnings remain; no real CAN-fault coverage or
roboRIO timing claim follows from those tests.

Before robot use: independently verify stow/zero and motor direction, complete camera intrinsics
and six-DOF extrinsics, confirm identical field JSON on every device, survey pose/aim on both
alliances, test all request/release/panic/reset transitions, then measure endpoints and stationary
shots. Next measure moving flight/release times and separate passing settings. Re-time each selected
auto in a 20 s practice window. Climb needs a separate homing/limits/follower-direction review before
enabling. Follow [the staged test plan](testing.md); passing desktop tests is not that acceptance.

[old-odometry]: https://github.com/FRC999/2026Competition/blob/6c4ecb4c196541236e7f3a702e2ad1099a094e1c/2026Competition/src/main/java/frc/robot/OdometryUpdates/OdometryUpdatesSubsystem.java
[prototype]: https://github.com/FRC999/2027Prototyping/tree/d20594af6fde49686fcbd9ed63250cf94463aaf5/VisionTestingAndCalibration
[game-timing]: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
