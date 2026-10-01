# Retrofit decisions and code audit

The [full refactor ledger](full-refactor-audit.md) extends this initial retrofit audit with startup
state machines, all active mechanisms, controls, RED path flags, old locations and verification.
Its current behavior supersedes earlier descriptions where called out.

## Provenance and scope

Target base: `FRC999/2026Competition`, `Houston---afternoon-Friday`,
`6c4ecb4c196541236e7f3a702e2ad1099a094e1c` (May 1, 2026, 14:01:15 -04:00).
`Worlds-Championship` ends at `a19f756` on April 25. Both Houston branches share `3050acb`;
`Houston---we-have-a-problem` adds a later documentation-only commit `8747ad7`. No deployment log
was available to prove the exact Worlds binary.

Vision/precision source: `FRC999/2027Prototyping`,
`d20594af6fde49686fcbd9ed63250cf94463aaf5`, `VisionTestingAndCalibration/`.
The source uses a different chassis. Its module geometry, camera offsets, noise calibration and
focus-dependent intrinsics were not transplanted. Its notes include unresolved physical-accuracy
and H4 settling work; those results are not asserted as 2026 validation. Historical external-team
attributions retained in copied comments describe prototype provenance, not fresh independent
verification of each team's current implementation.

The historical `QuestVibeGPT` project is preserved. Active 2026 robot classes and vendordeps no
longer depend on Limelight or Quest. Other mechanisms retain their existing hardware constants.

## Changes and their rationale

| Area | Finding | Branch behavior |
|---|---|---|
| Camera transforms | Prototype calibration belongs to another robot | Startup JSON has six editable values per camera; unmeasured cameras cannot fuse |
| Field identity | Two-tag and competition layouts can silently disagree across devices | File hash acknowledgment, official profile/layout consistency check, explicit startup-only switching |
| Vision freshness | Queued/duplicate/future frames can corrupt timing | Drain all unread results, use newest solvable pose per camera, gate age/order/future time, reject captures preceding a reset |
| Time bases | Existing CTRE consumer already converted FPGA timestamps | Exactly one conversion remains in DriveSubsystem |
| Heading | A single AprilTag is a weak yaw source | Single-tag rotation is never fused; enabled heading remains gyro-owned; disabled stationary stable MultiTag initializes automatically; manual seed remains available |
| Endpoint arrival | Timed PathPlanner completion did not prove settled arrival | Complete stopping paths end with profiled pose/motion qualification; timeout differs from success |
| Path reuse | Cached paths can inherit mutated flip flags | Copy path, resolve alliance once, set preventFlipping only on the resolved copy |
| Two-pose generation | Robot yaw was used as path tangent; some reset paths ended at 2 m/s | Geometric tangent is separate from desired yaw; generated stopping move has zero final velocity |
| Stopping | A later default motion request can undo a precision hold | Capture measured module angles; neutral/auto default behavior preserves hold; new motion clears hold |
| Failure handling | A failed endpoint must not advance to a subsequent shot | Hold sequence and latch autonomous feed inhibition, including parallel shooting commands |
| Pose angles | `new Rotation2d(±90)` was radians, not degrees | First retrofit corrected radians; full refactor then removed the unused pose constants |
| Turret setpoint | Small-error early return could retain a stale controller request after stop | Every valid setpoint is sent; continuous angle is not wrapped across prohibited travel |
| Turret position | Clamped feedback could conceal an overshoot; failed seed used fallback angle | Report actual angle, require configured/fresh/trusted position; invalid seed does not authorize output |
| Turret limits | Java clamping alone did not cover all motor-control modes | CTRE rotor soft limits preserve existing ±110° perimeter range; auto aim retains ±105° |
| Turret reset | Motor reboot can invalidate integrated position | Reboot revokes trust; operator must reseed disabled at known physical stow |
| Hood hold/stop | Neutral near target allowed drift; stop restored closed-loop mode on the next loop | Keep position control at normal target; actual stop enters IDLE; reset/config trust checked |
| Hood target range | Angle clamp allowed a target above the existing motor soft limit | Maximum matches the retained soft limit (55.8° with current conversion) |
| Feeding | A one-second override and FIRING state could bypass current readiness | Re-evaluate all permissions each loop; no timed force-ready bypass |
| Shooter target | Supervisor could read ready before updating RPM | Re-read after commanding; validate measured RPM against current target and fresh status |
| Moving RPM | >1 RPM changes cleared the readiness window every loop | Small changes retain history; steps outside the retained RPM tolerance rearm it |
| Shooter follower | stopMotor cancels follower mode | Restore follower mode for subsequent velocity/duty/voltage requests |
| Moving lead | Rotation moves an offset pivot, and turret angle is needed at release | Include omega-cross-offset velocity and release heading; optional measured flight time |
| Shot tables | Out-of-range distance silently used edge values; NaN parsing | Reject unmeasured distance range/nonfinite inputs; retain optional unknown battery metadata |
| Logging | AdvantageKit setup was commented out | WPILOG + NT4, build/source/config hashes, camera inputs, drive ownership and shot gates |

## Turret behavior and unresolved observability

The mentor clarified that these are **perimeter protection limits**, not physical hard stops: the
motor can protrude if the turret turns farther. This branch does not enlarge them. Keep the existing
turret pivot offset `(-0.18,-0.06) m`, robot-forward zero offset and motor/encoder geometry unless an
independent measurement demonstrates a correction is needed.

The three retained ranges have different purposes: motor commands ±110°, automatic aiming targets
±105°, and feeding/RT-assist comfort window ±95°. Turret zero points 180° from robot-forward in the
existing coordinate convention; zero turret angle does not mean the shooter faces robot-forward.

The existing software gearing is 11 motor/pinion revolutions per turret revolution. A single-turn
absolute encoder on that pinion repeats every approximately **32.73° of turret movement**. Its
nearest-zero boot interpretation is unique only within **±16.36° of the known stow branch**. Stable,
fresh sensor data cannot reveal which repeated branch the turret occupies. Booting or reseeding
after manually moving it outside that branch can yield a plausible wrong position. The shipped
trust flag verifies communication/configuration/seed acceptance; it cannot verify physical stow.

Near-term procedure: physically stow at the known reference before boot and disabled reseeding.
Long-term robust solutions are an absolute sensor on the turret output or an independently sensed
home reference with a validated homing routine. Remembering a number across power cycles alone
cannot prove that the mechanism was not moved while off. These hardware changes were not made.

When the requested direction lies outside the legal window, inhibit feed, hold/clamp to a safe
angle and signal the driver. The existing intentional RT chassis assist is retained with additional
stationary-input, low-translation-speed and fresh-vision checks. Direct rotation input overrides it.
Its fixed angular speed and braking behavior still need field tuning. An autonomous route should
deliberately provide a legal shooting heading; do not silently twist the chassis off its route.

## Models and limits still requiring measurements

- Precision controller gains, covariance coefficients and sim lens/FPS/noise values are initial
  settings. Simulator convergence does not validate them on this robot. Isotropic covariance and
  PnP single-tag mode remain the defaults; alternative trig/anisotropic strategies are available for
  controlled experiments but have not been field-tuned here.
- The moving model assumes approximately constant field velocity over release/flight prediction.
  The existing 0.12 s release delay and fallback 0.30/0.45 s leads are retained empirical assumptions.
  `flight_times.csv` is intentionally empty. No measured flight time or full-speed hit rate is claimed.
- Existing broad shooter RPM tolerance, very tight hood tolerance, short shot cooldown, ball sensors,
  supply behavior, hood down-at-boot assumption, motor inversions/gearing and mechanism clearances
  require the physical checks in testing.md. Do not treat old comments labeled TODO as measurements.
- The retained welded hub coordinates and trench rectangles are existing season values, not newly
  surveyed geometry. AndyMark localization is supported; automatic aiming/named paths are blocked
  for AndyMark/custom profiles pending a full target/route migration.
- Automatic feed requires recent accepted vision. Temporary occlusion may reduce shot availability;
  this is intentional for the initial retrofit. Measure dropout behavior before designing a longer
  odometry-only confidence window.
- Default auto is Do nothing. Selected autos have a 20 s deadline and may take longer because of explicit endpoint
  settling and must be re-timed, checked for path events and validated on the correct field/alliance.
- Complete deterministic hardware replay is not provided. Vision has logged IO; most mechanism and
  drivetrain access still uses the existing direct CTRE interfaces.

## Runtime versus evidence

Deploy only runtime camera configuration, field JSON, path assets and shot tables under
`2026Competition/src/main/deploy/`. Surveys, captures, photographs, logs, test reports, AI prompts and
skills belong outside `src/`. Runtime camera config hashes and fit-report hashes connect the robot
configuration to archived evidence. Hash acknowledgment is not an authenticated query of Pi settings.

Official references checked for this retrofit:

- [PhotonVision v2026.3.4 release and Plus image](https://github.com/PhotonVision/photonvision/releases/tag/v2026.3.4)
- [WPILib v2026.2.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.1)
- [WPILib 2026 field resources](https://github.com/wpilibsuite/allwpilib/tree/v2026.2.1/apriltag/src/main/native/resources/edu/wpi/first/apriltag)
- [CTRE 2026 dependency manifest](https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json)
- [AdvantageKit v26.0.2](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2)
- [PathPlanner release manifest](https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json)
