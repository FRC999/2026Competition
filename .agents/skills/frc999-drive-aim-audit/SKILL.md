---
name: frc999-drive-aim-audit
description: Review or tune FRC999 2026 precision path completion, turret tracking, moving-shot lead and feeding gates using logs and independent physical measurements.
---

Read the [audit](../../../docs/offseason-195/audit.md) and [test plan](../../../docs/offseason-195/testing.md).
Use session state for the exact code/calibration under test. Separate verified code behavior,
simulation evidence and physical robot evidence in the result.

- Preserve the measured/retained 2026 drivetrain and turret pivot geometry. Do not transplant
  prototype chassis or camera constants while porting algorithms.
- Resolve path alliance once without mutating cached paths. Preserve route events and intentional
  nonzero pass-through velocity. For stopping goals require position, gyro/module motion and fresh
  vision qualification; timeout is not arrival. Preserve failure hold and autonomous feed inhibition.
  Check enclosing race/deadline/timeout groups and re-time every changed auto against its match budget.
- Neutral/default commands must preserve the precision module-angle hold. Explicit new motion
  takes ownership. The teleop default must not drive during autonomous.
- Turret +/-110° command and +/-105° aim limits protect the robot perimeter, not hard stops.
  Keep continuous angles and correct motor sign. Never hide an overshoot by clamping measured angle.
  A pinion absolute encoder at 11:1 repeats every 32.73° of turret travel; software cannot establish
  boot turn identity outside the known physical stow branch. Reboots invalidate integrated trust.
- Re-evaluate feed gates on every loop, including FIRING. Require current RPM/hood/turret readiness;
  do not reinstate a timed force-ready bypass. Stop must not be undone by stale subsystem control
  modes, and shooter follower mode must resume after stop.
- Moving lead uses release heading and omega-cross-pivot velocity. Flight-time rows must be measured;
  distinguish retained empirical lead from calibrated timing. A distance-only table may be inadequate
  when RPM/hood choices vary. Do not claim dynamic accuracy during aggressive acceleration.
- An unreachable aim inhibits feeding and signals the driver. Intentional stationary chassis assist
  must respect operator input, pose validity and the perimeter limit, and must be physically tuned.

Use pure-model and controller integration tests for behavioral changes, then the complete desktop
smoke test for wiring changes. Real acceptance requires independent final-pose measurement and shot
trials with build/config hashes, not only estimator error or camera jitter. Update docs, session and
prompt records, and synchronize this skill with `.claude/skills/` using
`python tools/verify_skill_mirrors.py`.
