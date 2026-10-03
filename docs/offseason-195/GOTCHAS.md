# Gotchas to preserve when continuing

**Updated October 3, 2026.** These are current constraints and known limitations, not a second backlog.
Use [TASKS.md](TASKS.md) for open actions, [code guide](code-guide.md) for API contracts and
[programming diagrams](programming-diagrams.md) for exact decisions.

## Repository and evidence

- Work on `OffSeason-195`, in the nested `2026Competition/` robot project. Preserve unrelated local
  changes. The other historical robot project and the old T: checkout are not retrofit targets.
- The confirmed Worlds comparison is **Houston `6c4ecb4`**, not the older branch named
  `Worlds-Championship`. Branch history cannot prove which binary was deployed at the event.
- The 2027 prototype reference is `d20594a` and uses another chassis. Preserve this robot's measured
  dimensions, module geometry and turret offset. Algorithms may change; measurements cannot be guessed.
- Gradle archives robot `src/` into its JAR. Keep Markdown, skills, prompts, reports, capture logs,
  surveys and tooling outside that tree. Runtime deploy JSON/CSV/path assets stay in deploy as intended.
- Tests and simulated truth establish desktop behavior. They do not establish physical accuracy,
  clearance, legality or shot percentage. No physical calibration/deployment was performed in this session.
- A fresh clone contains code and documented receipts, not ignored historical WPILOGs/build outputs.
  Future valuable evidence under ignored `artifacts/` needs deliberate archival to transfer it.

## Vision, coordinates and startup

- Camera JSON is startup-only. Rear XYZ remains null; `calibrated=false` and layout acknowledgment is
  empty. Raw calibration capture can work while fusion is intentionally blocked. Synthetic desktop
  geometry must never become real camera calibration.
- Proposed mounts are 12-inch lens height, roll 0, WPILib pitch **-15 degrees** (up), and rear yaws
  +165/-165 degrees. Confirm actual transforms by measurement/fit after final focus and intrinsics.
- Field coordinates keep the blue origin on RED too. Transform paths/targets at their boundary;
  never flip the estimator or a cached path twice. `ALLIANCE`, `FORCE_RED` and `ABSOLUTE` differ.
- Robot +X is forward and +Y is left. Negative turret X means behind, negative Y means right.
  Rotate the measured offset by robot/release heading; include omega-cross-offset velocity for lead.
- Camera capture timestamps are FPGA seconds. Convert once to CTRE time in Drive, not in both layers.
  Drain Photon unread results and use the newest solvable pose; do not recreate an old-frame FIFO.
- Connection, first frame, first solvable pose and first accepted fusion are separate milestones.
  Single-tag yaw is untrusted; enabled heading stays gyro-owned. Stable disabled MultiTag can seed
  absolute reference. One camera suffices; fresh disagreement blocks initialization.
- No arbitrary early-AUTO waiting interval exists. Reject pre-reset captures and stale/future/duplicate
  frames instead. Plain pose/gyro resets revoke trust; qualified reset wrappers restore it deliberately.
  Driver-forward reset changes input perspective only.
- Profile identity, recent vision and established reference are separate conditions. Survey/simulation
  profiles do not grant welded-field automatic targets/paths. Hash acknowledgment is not remote proof
  that the Pi actually loaded the correct layout.

## Commands, drive and autonomous

- Groups own all child requirements for their whole lifetime. Compare scheduler owners correctly;
  do not let supervisor periodic output fight a calibration/jog/SysId owner.
- A command can end in any phase. Stop or explicitly transfer its output/hold in `end()`, restore
  boosts and clear stale intent. PathPlanner can retain its last request on interruption; keep the
  wrapper's explicit stop. A Java mode flag alone does not neutralize a controller request.
- Teleop action bindings cannot steal AUTO requirements. `FreshPress` needs release/repress after
  mode/panic gates reopen. Falling-edge cleanup checks permission before scheduling required commands;
  `onlyIf` alone would still permit a denied command to cancel an existing owner.
- Neutral/default drive preserves the measured module-angle hold. Explicit allowed motion releases it.
  The teleop default cannot drive during AUTO. Loss of field reference during an AUTO motion request
  latches failure. Recent-camera loss alone is a start/finish gate, not a continuous coarse-path abort.
- Competition `ROUTE_STOP` is brake-only: provisional 0.20 m / 5-degree acceptance, calm measured motion
  for 0.06 s, at most 0.50 s. Strict `PRECISION_ALIGNMENT` can correct/settle and is used only explicitly.
  Timeout is not arrival; failure holds the sequence and inhibits AUTO feeding.
- Preserve nonzero joins only with compatible position, heading and velocity direction. The opening
  approach is a constrained straight trench corridor, not obstacle-aware pathfinding.
- AUTO is bounded to 20 s. Full original Main/Worlds sequences remain over budget and await the mentor's
  strategy choice. A deadline does not make their mission complete. Default is Do nothing.

## Shooting and mechanisms

- `Solution.valid` means a usable candidate, not feed permission. All feed gates run every loop,
  including FIRING. No timed force-ready bypass; RPM loss cannot invent a hood correction.
- Hub shots need known zone permission at current and predicted release position. Manual non-moving
  shot mode with hub tracking disabled retains the mentor-authorized fallback when localization is
  unavailable; the driver confirms position and the dashboard flag stays visible. Other gates remain.
- Trench approach latches feed inhibition and requests neutral hood. Leaving does not rearm a held
  request: clear/reissue outside the guard. Entry itself does not set `requested=false`. Without a field
  reference, software cannot recognize a new trench encounter; an existing lock remains effective.
- Trench pad 0.45 m and lookahead 0.35 s are provisional software settings, not measured swept clearance
  or hood response. Field-center and path-point checks cannot certify bumper/extension rules.
- Jam clear is one owner for shooter/transfer/spindexer and never restores the interrupted volley.
  Releasing shoot stops feed; permitted 2,200 RPM idle spin is intentional, not a stuck shoot command.
- Turret ±110-degree command limits protect the perimeter, not mechanical hard stops. Automatic aim
  uses ±105 and feed comfort ±95. Do not clamp measured continuous angle to hide overshoot.
- The 11:1 pinion encoder repeats every 32.73 turret degrees. Boot/reseed needs the known physical stow
  branch within ±16.36 degrees; wrapped samples cannot prove turn identity. Motor reset revokes trust.
- Intake homing timeout never establishes zero. Stall evidence can also mean obstruction; use the
  physical procedure. Current/gain boosts must unwind in actual hardware requests on all end paths.
- SysId permission is evaluated when scheduled, during execution and in output callbacks. Keep TEST,
  configuration and panic checks. Climb remains disabled pending physical validation.
- Passing needs its own measured table, currently empty. Flight-time table is also empty. The fallback
  0.12 s release delay and 0.30/0.45 s radial/lateral lead are retained empirical assumptions. Fixed/hub
  settings are retained season data, not newly validated by the refactor.

## Simulation and tool portability

- Construct/write synthetic IO only in simulation. Inertia precedes gearing in rotary model setup;
  CTRE sensor signals use rotor units including reduction. Account for paired motors and positive load.
- Simulated physical truth is independent of estimator resets. Placement and the 5 ms notifier share
  synchronization; command/shot decisions remain on the scheduler thread. Close notifiers/cameras.
- No shared static camera world. No claim of validated gravity, hard-stop contact, collisions, fuel
  transport, projectile flight or complete deterministic robot replay.
- Use the Gradle wrapper/committed vendor versions and run native `robotSmoke` in its separate JVM.
  Recreate Python and optional Node dependencies from [HANDOFF.md](HANDOFF.md); do not copy local venvs,
  dependency caches or personal absolute paths. Diagram images are already committed.
- Source counts include comments and deletions; churn is not unique modified lines. Shared integration
  must stay explicit rather than being counted entirely as “just vision.” The current snapshot targets
  `5cedfe0`; refresh after the final strategy implementation.
