---
name: frc999-drive-aim-audit
description: Review or tune FRC999 2026 precision path completion, turret tracking, moving-shot lead and feeding gates using logs and independent physical measurements.
---

Read the [second-pass audit](../../../docs/offseason-195/second-pass-audit.md),
[full audit](../../../docs/offseason-195/full-refactor-audit.md) and [test plan](../../../docs/offseason-195/testing.md).
Use session state for the exact code/calibration under test. Separate verified code behavior,
simulation evidence and physical robot evidence in the result.
Use the [code guide](../../../docs/offseason-195/code-guide.md) for units, frames, ownership and API
contracts. Keep Java/package comments current; `gradlew javadoc` validates rendered API documentation.
The [team overview](../../../docs/offseason-195/team-overview.md) and
[programming diagrams](../../../docs/offseason-195/programming-diagrams.md) visualize these decisions.
When a diagrammed contract changes, update its Mermaid block and regenerate the adjacent SVG with
`node tools/render_diagrams.mjs`; keep source and rendering together.

- Preserve the measured/retained 2026 drivetrain and turret pivot geometry. Do not transplant
  prototype chassis or camera constants while porting algorithms.
- Resolve path alliance once without mutating cached paths. Preserve route events and intentional
  nonzero pass-through velocity only when position, velocity direction and holonomic heading agree.
  Competition route stops brake and qualify without corrective jitter (ROUTE_STOP, <=0.50 s);
  strict PRECISION_ALIGNMENT is an explicit precision-test/caller choice. Both require position,
  gyro/module motion and fresh vision; timeout is not arrival. Preserve failure hold and feed inhibition.
  Check ALLIANCE/FORCE_RED/ABSOLUTE and source preventFlipping independently; never reset/flip twice.
  Derive opening approaches from the actual resolved path start and verify path-join continuity.
  Check enclosing race/deadline/timeout groups against REBUILT's 20 s budget, not a generic 15 s
  assumption. Preserve cancellation on mode exit and repeated scheduling.
- Neutral/default commands must preserve the precision module-angle hold. Explicit new motion
  takes ownership. The teleop default must not drive during autonomous.
- Turret +/-110° command and +/-105° aim limits protect the robot perimeter, not hard stops.
  Keep continuous angles and correct motor sign. Never hide an overshoot by clamping measured angle.
  A pinion absolute encoder at 11:1 repeats every 32.73° of turret travel; software cannot establish
  boot turn identity outside the known physical stow branch. Reboots invalidate integrated trust.
- Re-evaluate feed gates on every loop, including FIRING. Require current RPM/hood/turret readiness;
  do not reinstate a timed force-ready bypass. Stop must not be undone by stale subsystem control
  modes, and shooter follower mode must resume after stop. Share a pure planner with diagnostics;
  telemetry must never mutate decisions. Jam clearing has explicit ownership and never restores an
  old shoot request. Trench exit requires a fresh request; no rearm while inside.
  Include nominal full trench structures and approach lookahead; guard padding/time are provisional,
  not measured robot clearance. G407 hub feeding requires a confirmed alliance zone; manual fallback
  without localization remains driver-confirmed per mentor, with its dashboard indication visible.
- Moving lead uses release heading and omega-cross-pivot velocity. Flight-time rows must be measured;
  distinguish retained empirical lead from calibrated timing. A distance-only table may be inadequate
  when RPM/hood choices vary. Do not claim dynamic accuracy during aggressive acceleration.
  Passing requires its own measured table; do not combine hub RPM with an invented hood setting.
  Corrupt calibration rejects the whole table. Preserve behind/right pivot signs in robot axes.
- An unreachable aim inhibits feeding and signals the driver. Intentional stationary chassis assist
  must respect operator input, pose validity and the perimeter limit, and must be physically tuned.

Intake homing timeout is failure, never zero evidence. Position resets revoke trust; command
cancellation/timeout stops motion. Check panic transitions while disabled as well as enabled.
Operator controls must not schedule/cancel AUTO. Check release cleanup before scheduling required
commands, and require release/repress across mode/panic gates. Jam clear is one owner. Reset boost
hardware slots/current limits in cleanup; honor bounded boost time. SysId permission is evaluated at
schedule time and throughout execution, never only during construction.
Climb stays disabled until its physical home, limits and follower direction are validated.

Use pure-model and controller integration tests for behavioral changes, then the complete desktop
smoke test for wiring changes. Real acceptance requires independent final-pose measurement and shot
trials with build/config hashes, not only estimator error or camera jitter. Update docs, session and
prompt records, and synchronize this skill with `.claude/skills/` using
`python tools/verify_skill_mirrors.py`.
