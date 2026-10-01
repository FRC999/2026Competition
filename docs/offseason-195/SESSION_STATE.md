# OffSeason-195 session state

## Status — October 1, 2026

Initial implementation delivered at 5a3e431 and the first full refactor at 80acda9.
The second audit's independent fixes are verified; autonomous strategy selection remains pending.
Read second-pass-audit.md for current interruption, rules, route-stop and simulation behavior;
full-refactor-audit.md retains the preceding 40 findings and their evidence.
The user explicitly confirmed robot-frame turret offsets: +X forward, +Y left; negative X is behind
and negative Y is right. The cardinal-heading math is correct; no pivot sign reversal was justified.
Delivery branch is OffSeason-195; use Git history and its origin tracking status for delivery receipts.
No physical robot operation or deployment has occurred. User authorized commits/push to this branch;
no PR requested. Remaining work is the mentor's auto strategy choice, its implementation/final count
refresh, and the team's measured calibration and physical testing.

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
Turret can physically rotate farther: ±110 code limits are extension-envelope limits, not hard stops.
Retain ±110 commands / ±105 automatic aim, now enforce rotor-position soft limits in CTRE too.
User explicitly approves AdvantageKit for testing. No pending user answers currently block desktop work.

## Initial delivery at 5a3e431 (historical; current follow-up below supersedes behavior)

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
   stationary shots and measured moving lead. Re-time every selected 20-second REBUILT autonomous routine.
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


## Completed full refactor follow-up

The mentor explicitly expanded this review to all active 2026Competition code, not only vision or
prototype-derived algorithms. Preserve measured hardware, but replace/refactor faulty algorithms.
The report must give old locations, high-level effects, replacement code and verification evidence.

- Houston's two-tag MT1 → mandatory 2 s MT2 wait → saved MT1 fallback → 5 s reanchor loop was a
  confirmed software delay path. No match log establishes its exact historical contribution.
- Disabled stationary stable MultiTag automatically establishes field reference: >=4 unique frames,
  >=0.10 s span, <=0.25 s age, <=0.10 m / 3° spread. One healthy camera suffices; fresh disagreement
  blocks reset. Enabled heading stays gyro-owned. Gyro reset revokes reference. No arbitrary
  early-auto/reset quarantine remains; pre-reset capture timestamps are still rejected.
- Alliance and driver button 8 now alter operator perspective only. No blind field-yaw seed or
  duplicate button binding remains. Aim/path starts/finish require referenced fresh localization.
- Path frames are ALLIANCE/FORCE_RED/ABSOLUTE with cache-safe copies and one flip/reset maximum.
  Opening approaches use real path starts; two Main route joins were aligned (22.4 cm / 1.3 cm).
  Alliance-specific autos reject wrong/unknown alliance; BLUE-authored generic autos flip on RED.
  Shoot-only holds chassis stopped. Outpost shooting uses remaining time, with a 20 s enclosing
  deadline and mode-exit cancellation. Official 2026 timing corrected an intermediate 15 s assumption.
- Pure AimGeometry/FieldTargeting/MovingAimModel/ShotPlanner/ShotTable/ShotIntent/ShotReadiness replace
  duplicated supervisor calculations and dead TurretHelpers ballistics. Diagnostic reads cannot
  mutate control. All readiness gates remain active during FIRING; reverse never restores stale intent.
- Trench checks cover all four physical rectangles on either alliance; no rearm inside, neutral hood,
  and fresh request required after exit. Rectangles remain provisional point regions, not swept volume.
- Passing is intentionally inhibited until the separate pass_shots.csv has measured data. Hub rows
  remain unchanged; no forced 13° hood or invented RPM-dip hood compensation. Corrupt table rejects
  as a unit and logs hash/status/row count. Flight-time CSV remains empty; empirical lead labeled.
- Driver shaping applies raw deadband before cubic, has zero/sign/finite guards and correct axis
  choices; robot-centric Y is preserved. Intake configuration/seed/reset/soft-limit bookkeeping fixed;
  timeout/interruption stops pivot, homing timeout never seeds, and neutral changes preserve inversion.
  Transfer/spindexer duty goes through guarded mode setters. Disabled panic changes are honored.
  Climb config/follower/null/simulation defects corrected, but it remains disabled pending physical checks.
- Retired unused autos, duplicate timeout/calibration wrappers, dead intake-driver state, unused
  artillery constants/arrays, obsolete telemetry/helpers and example subsystems. History retains them.
  Kept useful SysId APIs behind config/test-mode/panic guards; no gains were retuned or hardware run.
- Updated full-refactor-audit.md, audit.md, README, calibration/testing guides, prompt record and
  matching Codex/Claude skills. Current test receipt: 105 normal Java tests + 1 expanded robotSmoke,
  7 Python tests, six skill validations and three mirror checks; full test/robotSmoke/build passes.
  Desktop CAN/joystick/loop-overrun warnings persist; no physical/performance claim follows.

Remaining work is operator-led calibration and physical acceptance, not unfinished desktop refactoring.
Use Git history and origin tracking for the delivered follow-up commit. No deployment occurred.

## Second audit — verified independent changes, strategy decision pending

The preceding refactor was delivered as 80acda9. A new mentor request authorizes a detailed second
review of interruption/concurrency, 2026 rules and auto stop/timing behavior, simulation isolation,
and a reproducible change-count comparison against Worlds (with vision/simulation/other attribution).
Previous completion text refers only to the preceding delivery. All independent corrections are now
implemented and tested. Do not mark the new task complete while the strategy decision remains open.

- Mentor confirmed **Houston 6c4ecb4** as the Worlds comparison baseline and **manual fallback with
  driver-confirmed position** when localization is unavailable. Both answers are in PROMPTS.md.
- Teleop-only bindings, pre-scheduling release guards and FreshPress prevent AUTO cancellation and
  held-input revival across mode/panic changes. One command owns all three jam-clear mechanisms.
  Mode exits cancel commands; interrupted PathPlanner followers stop. SysId gates evaluate at schedule
  time and revoke during execution. Intake gain/current boosts roll back; initial boost is <=0.75 s;
  held LT pivot deployment is bounded. Gyro-reference loss inhibits new auto motion requests.
- Automatic hub feed checks a conservative own-alliance-zone center boundary now and at release.
  Manual localization fallback remains mentor-authorized and indicated. Trench guards now cover full
  nominal structures and a swept approach segment with provisional 0.45 m padding/0.35 s lookahead.
  Physical envelopes, hood lowering time and starting bumper overlap remain operator validation.
- Competition path stops use brake-only ROUTE_STOP (0.06 s calm qualification, <=0.50 s timeout),
  never strict corrective precision jitter. Failed stops hold the sequence and inhibit feed. Explicit
  precision tests retain stricter alignment. Five physically discontinuous moving path joins now stop;
  the valid 0.5 m/s Main handoff retains continuous velocity and heading.
- Corrected seven rotary simulations' inertia/gearing order, rotor conversion, motor count and follower
  signals; guarded sim construction/writes; included drivetrain battery load; fixed synthetic camera
  world lifecycle and 5 ms truth/placement synchronization; closed notifier/camera resources.
- **Open question already sent to mentor:** first collection then shoot for the remainder, or later pickup
  only with sufficient time reserved for return/shooting. Main's named paths alone total 20.829 s.
  Worlds Blue/Red have 13.282/11.652 s of named paths plus waits/shots that already exceed 20 s before
  opening/deploy/stop checks. Do not silently choose a new strategy. Original full sequences remain
  deadline-bounded and documented as over budget; default remains Do nothing.
- `tools/change_audit.py` produces a Git-numstat-checked production Java accounting and per-line CSV
  against 6c4ecb4, with separate vision/simulation/other/shared attribution. Shared integration is not
  a precise causal split. Refresh after the auto strategy implementation; generated counts are snapshots.
- Latest verification: **117 normal Java tests + 1 expanded robotSmoke**, all passing; full
  `gradlew test robotSmoke build` passes. Seven Python tests, six skill validations and three mirror
  pairs pass. The smoke test confirms the actual CTRE request leaves boosted slot 2 and held LT does
  not restart across modes. Desktop CAN/joystick/loop-overrun warnings persist; no hardware claim.
- No deployment or physical operation. Independent audit changes can be reviewed on OffSeason-195;
  check Git/origin for receipt. Final strategy work and final summary/counts remain outstanding.

### Branch delivery receipt

The mentor reiterated that the new code must be committed to the new branch with proper comments.
Verified that the implementation is committed and pushed to **OffSeason-195**:

- `5a3e431`: initial PhotonVision retrofit, precision controls and calibration workflow.
- `80acda9`: localization, autonomous frames, shot control and mechanism ownership refactor.
- `a37f951`: interruption, route-stop, rule/position guards and simulation isolation fixes.

Non-obvious behavior has inline rationale; full-refactor-audit.md and second-pass-audit.md document
the issues, replacements, evidence and physical limitations. Latest code validation remains 117 Java
tests plus the full robot smoke test, seven Python tests, six skill validations and three skill mirrors.
The request to commit does not select a new autonomous strategy; that choice remains pending.
