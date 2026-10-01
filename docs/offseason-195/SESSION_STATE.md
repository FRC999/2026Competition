# OffSeason-195 session state

## Status — September 30 / October 1, 2026

Software implementation, guides, shared skills and final integrated build/package checks are complete.
Delivery branch is OffSeason-195; use Git history and its origin tracking status for delivery receipts.
No physical robot operation or deployment has occurred. User authorized commits/push to this branch;
no PR requested. Remaining validation requires the team's measured calibration and physical testing.

## Locations and pinned sources

- Target clone: `S:/Projects/MechaRAMS/2026Competition`; project: `2026Competition/`.
- Target branch: `OffSeason-195`, created from `origin/Houston---afternoon-Friday` at
  `6c4ecb4c196541236e7f3a702e2ad1099a094e1c`.
- Source clone: `S:/Projects/MechaRAMS/2027Prototyping`, main at
  `d20594af6fde49686fcbd9ed63250cf94463aaf5`.
- Existing T: checkout remains untouched.
- Git HTTPS needed per-command `-c http.sslBackend=schannel` to use Windows certificate trust.

## Base evidence

`Worlds-Championship` ends April 25, 2026 01:22:20 -04:00 (`a19f756`).
Both Houston branches share robot-code commit `3050acb` (May 1, 10:12:56 -04:00).
`Houston---afternoon-Friday` adds `6c4ecb4`, "relax inital seeding process", May 1 14:01:15 -04:00.
`Houston---we-have-a-problem` instead adds documentation commit `8747ad7`, May 1 23:02:46 -05:00.
Thus afternoon-Friday has the latest robot code among requested candidates. Actual deployed binary
cannot be proved from branch names/dates; no deployment log yet supplied.

## Important findings

- Prototype is WPILib 2026 / PhotonLib v2026.3.4, CTRE and AdvantageKit IO, not a 2027 WPILib migration.
- Prototype chassis is 2025. Do not transplant hardware constants.
- Prototype latest notes distinguish estimated position from surveyed physical accuracy.
- Sept 28 stopping/default-command ownership fix has robot evidence, but H4 settling remains open.
- Latest left-camera focus change had no subsequent intrinsic recalibration; do not reuse its calibration.
- Historical prototype calibration guide has suspect pitch wording and transform/UI advice; verify against official docs before carrying instructions forward.
- Existing 2026 DriveSubsystem already converts FPGA timestamps to CTRE time in addVisionMeasurement;
  avoid double conversion when adapting the prototype consumer.
- The base Robot had logging commented out; the retrofit enables it and records build metadata.

## Confirmed follow-ups

OV9782 cameras; two Orange Pi 5 Plus available. Drivetrain/turret/shooter mechanically unchanged.
Preserve turret location and offsets. Mentor wants speed/direction-dependent moving lead; most shots stationary.
Mentor authorized compatible latest stable 2026 libraries, and requests audit of existing 2026 logic.
Use old LL mount locations as provisional if their values can be recovered; six values per new camera must be editable.

## Latest resolved answers

No old LL mount values needed. User now wants rear-perimeter camera mounts, initially 12 inches high,
and proposed pitch/yaw with zero roll. Proposed rear-left +165 yaw / rear-right -165 yaw, pitch -15
(15 degrees UP in WPILib), roll 0. Actual xyz remains null/unmeasured; no invented calibrated transform.
Rear pair enabled for capture; optional front pair disabled. Fusion requires calibrated and matching-layout acknowledgment.
Turret can physically rotate farther: Â±110 code limits are extension-envelope limits, not hard stops.
Retain Â±110 commands / Â±105 automatic aim, now enforce rotor-position soft limits in CTRE too.
User explicitly approves AdvantageKit for testing. No pending user answers currently block desktop work.

## Implemented and checked

- Ported PhotonVision IO, policies, simulation, jitter and optional single-tag trig/anisotropic modes.
  PnP/isotropic defaults retained. Raw capture works before extrinsics are fitted; fusion does not.
- Added strict startup camera/profile/layout configuration, official competition layout consistency,
  stale/duplicate/future/reset guards, covariance floors, single FPGA-to-CTRE conversion and explicit seed.
- Added two-to-four camera configuration with proposed rear orientations and no fabricated XYZ.
- Ported precision controller, measured module-angle hold and default-command ownership behavior.
  Resolve alliance once on a path copy; complete route/events then precise stopping endpoint. Failed
  endpoints hold the sequence and inhibit autonomous feeding (including parallel shooting).
- Preserved 2026 hardware geometry. Fixed degree/radian trajectory constants and aligned PathPlanner
  module centers/wheel radius with existing CTRE values. Custom/AndyMark localization does not authorize
  welded competition targets or named paths.
- Audited turret commands/seed/trust/software limits; preserved +/-110 perimeter, +/-105 aim and +/-95
  feed comfort window. Pinion turn ambiguity remains a physical boot-stow requirement (+/-16.36 deg).
- Hood keeps normal position hold, has a real IDLE stop and boot/config/reset trust checks; its target
  max matches the existing motor soft limit. Operator disabled reseed commands added to dashboard.
- Feeding rechecks all readiness each loop, with no 1-second bypass. Shooter current-target readiness
  and follower resumption fixed; gradual RPM corrections retain history. Motor reset revokes position trust.
- Moving lead includes release heading, gyro angular velocity and omega-cross-pivot velocity. Optional
  measured flight-time CSV is header-only; retained .12 release/.30 radial/.45 lateral assumptions are labeled.
- AdvantageKit WPILOG/NT4, source/build/config/layout hashes and drive/vision/shot telemetry enabled.
  Full deterministic replay of direct CTRE mechanisms is not implemented.
- Python survey-layout/capture/6DOF-fit/held-out-validation/apply tool implemented. Reports and backups
  stay outside src. Six offline tests plus a real NT localhost capture/rejection test passed.
- Wrote installation.md, calibration.md, testing.md, audit.md and a linked guide README. Three repo
  skills have identical .agents/.claude copies; all six SKILL.md files passed quick_validate.py.
- Compatible pinned versions: WPILib2026.2.1, Photon2026.3.4, CTRE26.3.0, AK26.0.2, PP2026.1.2.
  Verified official release includes orangepi5plus.img.xz (quick-install table omits Plus).
- Headless full robot startup test passed: synthetic vision corrected pose and default auto remained
  stopped. Desktop CAN/joystick/startup overrun warnings occurred; no physical timing/performance claim.
- Final Java suite: 89 passing normal tests plus one passing robotSmoke test; zero failures/skips.
  `gradlew test robotSmoke build` passed. Seven Python tests passed. All six skills validated and
  mirror checks passed. JAR inspection found no docs, prompt/session records, skills, calibration
  tools or synthetic field fixture. `git diff --check` passed.

## Team follow-up

1. Follow installation.md and calibration.md: mount/focus cameras, calibrate intrinsics, survey the
   custom two-tag field and independent robot stations, fit all six camera extrinsics, validate
   held-out stations, then restore and acknowledge the identical competition field on every device.
2. Follow testing.md from disabled mechanism checks through independently measured endpoint accuracy,
   stationary shots and measured moving lead. Re-time every selected 15-second autonomous routine.
3. Preserve the pinion boot-stow requirement and perimeter limits. No desktop result establishes
   physical accuracy or correct motor direction/zero. Default autonomous remains Do nothing.

## Reproduction and evidence

From robot project, use JAVA_HOME=C:/Users/Public/wpilib/2026/jdk and, on this workstation,
JAVA_TOOL_OPTIONS=-Djavax.net.ssl.trustStoreType=Windows-ROOT -Djavax.net.ssl.trustStore=NONE.
Run gradlew.bat test robotSmoke build --console=plain. Native robotSmoke uses a separate JVM.
From repo root: .venv/Scripts/python.exe -m unittest discover -s tools -v (7 tests), and
python tools/verify_skill_mirrors.py. Local .venv uses system NumPy/SciPy plus pyntcore2026.2.1.
Log/test artifacts and simulation persistence are ignored, not shipped. No physical measurements,
robot motion, robot deployment, calibrated shot data or competition-ready accuracy have been claimed.
