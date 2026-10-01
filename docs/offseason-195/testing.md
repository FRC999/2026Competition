# Desktop checks and robot acceptance

## Evidence from this branch

The final desktop suite has 89 passing Java tests, plus one passing whole-robot startup smoke test.
All seven Python tests pass, including fit/layout/apply and live NetworkTables **localhost** capture.
`gradlew test robotSmoke build` completed successfully. All six Codex/Claude skill files validated,
and their three pairs match byte for byte. The built JAR contains none of the new documentation,
prompt/session records, skills, calibration tools or synthetic field fixture.

These tests cover controller convergence with a simple delayed plant, blocked-motion timeout,
finish permission, alliance resolution without mutating cached paths, camera configuration,
timestamp/reset/duplicate fusion gates, single-tag math, simulated MultiTag decoding, moving lead,
turret command limits and shooting readiness. They do not model carpet slip, frame flex, ball flight
variability, exact lenses, real CAN faults or robot contact with the field.

The complete robot smoke test starts the normal robot loop without a GUI, receives synthetic vision,
checks the estimated pose against separately maintained simulated truth, and checks that the default
auto stays stopped. It produced startup CAN/joystick warnings and desktop loop overruns. This test
checks integration, not roboRIO timing performance or validated mechanism simulation.

## Reproduce on the development computer

From repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools -v
```

Use `tools/requirements.txt` as described in calibration.md. Without `pyntcore`, the localhost test
is explicitly skipped; the offline math tests still run. The localhost test uses port 15810 and
does not contact a robot.

From `2026Competition/`, using the WPILib Java 17 JDK:

```powershell
.\gradlew.bat test robotSmoke build --console=plain
```

Reports are under `build/reports/tests/test`, `build/reports/tests/robotSmoke`, and
`build/reports/jacoco/test/html`. Coverage is reported, not represented as a universal 90% guarantee.
Smoke logs are under `logs/sim`. Desktop fixtures under `simulation/` are synthetic and are not
deployable calibration. A GUI run may use `simulateJava`; the startup placement deliberately gives
the estimator a small offset from true pose so a camera correction can be seen.

## Record each physical test

Robot operations require the team's normal operator-led test setup. None were performed while
creating this branch. Keep one evidence directory per test date/configuration outside `src/`.
Record build Git SHA, dirty flag, SourceSHA256, runtime ConfigSHA256/LayoutSHA256, Pi versions and
settings exports, camera fit reports, battery condition, tire state, field profile, target pose,
start pose, alliance, path name, measured final pose and pass/fail reason.

AdvantageKit writes WPILOGs on the robot (the default WPILOGWriter uses the usual USB destination)
and publishes NT4. Confirm a log file is actually being created before testing. Use AdvantageScope
to overlay `Drive/Pose`, `DriveToPose/TargetPose`, accepted/rejected camera poses, module states,
gyro rate and requested motion. Compare source/config hashes when comparing runs. Camera IO is
logged; a complete deterministic replay of every hardware subsystem is not implemented here.

## Stage 1 — disabled and mechanism checks

1. Physically place the turret at its known boot stow and the hood fully down **before power-up**.
   Verify angle sign and displayed zero. The pinion absolute encoder cannot identify full turret
   turns; software cannot prove the stow assumption. See audit.md.
2. Verify the config/profile/hash and both camera names. Complete calibration and independently
   survey at least two additional poses. Check translations, yaw and field handedness on both
   alliances. With the robot disabled, the explicit PhotonVision seed needs a fresh trusted MultiTag
   result; it does not accept a lone tag or an uncalibrated camera.
3. With the robot secured and mechanism space clear, verify turret movement in small steps first:
   zero, ±10°, ±30°, then approach the existing perimeter limits under supervision. Confirm actual
   angle and motor extension remain safe. **Do not widen ±110° command / ±105° aim limits** merely
   because the mechanism can physically turn farther. Motor direction, gearing, zero and latency
   must be checked on the real mechanism.
4. Stop and resume the turret and hood. A small new turret setpoint should be sent after stopping;
   the hood should hold an ordinary position command, while `stop()` should actually stop. A motor
   reboot invalidates turret/hood position trust; recover disabled from the physical reference.
5. Verify shooter leader/follower both resume after a stop, including repeated spin-up/stop cycles.
   With no feeding, check logged RPM, readiness and the configured tolerance. The retained 10% RPM
   band is broad; measure shot consistency before tightening it. Readiness history no longer resets
   on every small moving-shot RPM update, but measured speed must satisfy the current target.
6. Test panic stop, trigger release, lost shooter/hood/turret readiness and camera loss. Transfer and
   spindexer must stop feeding when any automatic-shot gate fails. A manually forced feed is an
   existing explicit operator override, not evidence that the supervisor approved a shot.

## Stage 2 — endpoint accuracy on carpet

1. Start with the explicit `OffSeason: Precision 1m forward (clear area)` auto. Establish the starting
   pose and correct heading; provide a clear corridor and visible tags. The default remains Do nothing.
2. Run 10 repetitions at low speed, then from different headings. Measure final chassis origin and
   heading using floor marks/jig or an independent surveying method. Record physical error alongside
   the estimator's error. A tiny estimated error can coexist with a biased real pose.
3. Initial acceptance target: at least 9/10 inside 4 cm and 1.5° **physically**, no false SUCCEEDED,
   no renewed motion after arrival and no uncontrolled output after interruption. Record worst case
   and the full distribution, not just the best attempt. These limits are goals to verify, not a
   guarantee derived from the prototype.
4. Test zero-distance rotation, short translation plus yaw, diagonal motion, a curved coarse path,
   an intentionally unreachable goal, and camera occlusion near the endpoint. A timeout must be
   labeled `TIMED_OUT`, not arrival. A stopping path failure holds the sequence and latches an
   autonomous feed inhibition, including when a shooting command runs in parallel with that path.
5. Let the default drive command resume with neutral sticks: measured module angles should stay
   held instead of snapping to unrelated targets. Then move a stick and verify immediate manual
   ownership. Test autonomous with nonzero simulated/test-controller stick values: it must not let
   the teleop default drive command take over.
6. Progress through operator-selected speed steps (for example 0.5, 1.0, 1.5 m/s) after each passes.
   Test low battery and worn tires separately. Tune velocity-loop tracking before increasing final
   pose gains. The prototype's 1.6 m/s endpoint limit and settling parameters are initial 2026 trials.

Every named stopping path runs its complete route/events, then the precision finish. This can add
up to the 4-second precision timeout per stop; it can materially change a 15-second auto. Re-time
each selected auto end to end. Check stops versus intentionally nonzero pass-through end velocities,
intake races, shooting windows and field clearance. Do not shorten a route by adding an early
spatial handoff without verifying that the remaining corridor and event markers are safe to skip.
Blue/red chooser names must agree with the actual alliance and any explicitly authored red poses.

## Stage 3 — stationary aim before moving aim

At measured legal shooting spots, test fixed distances across the measured shot-table range, both
turret directions and both alliances. Log raw desired aim, filtered setpoint, measured turret angle,
RPM/hood targets and actuals, feed permission, field target, and shots made/attempted. Verify physical
turret zero and pivot offset before changing aiming trims. Existing geometry was preserved, not
re-measured. A shot outside the table's distance range must be invalid rather than borrowing an edge row.

For an unreachable turret direction, expect no feed and a left rumble indication. The existing RT
assist is allowed only with neutral translation/rotation input, low measured translation speed and
recent competition-frame vision. Rotation input overrides it. Verify its fixed turn speed before
using it near other robots. It rotates the chassis to the retained comfort window; it does not
relax turret perimeter limits. A proportional/decelerating assist can be tuned later from logs.

## Stage 4 — measured moving-shot lead

Keep the stationary RPM/hood table as the foundation. The lead model uses measured field velocity,
the existing turret pivot offset, gyro angular rate and the retained 0.12-second release delay.
It adds pivot tangential velocity (`omega × offset`) and computes turret orientation at release.

The shipped `artillery/flight_times.csv` has a header only. It deliberately contains no invented
measurements. Without a covered measured flight-time range the solver uses the labeled legacy
0.30-second radial / 0.45-second lateral lead. Those coefficients are empirical and need validation.

1. Use timestamped/high-frame-rate video or another measured timing method. Measure from actual
   ball exit to target-plane crossing, along with turret-to-target distance, RPM, hood, battery and
   shot outcome. Measure command-to-release delay separately; a feeder request is not ball exit.
2. Collect several trials at each distance in the normal RPM/hood operating schedule. Use a
   representative flight time and record spread. If different RPM/hood choices at the same distance
   have materially different flight times, the current distance-only table is insufficient: extend
   it before claiming that model is calibrated.
3. Add at least two measured rows `distance_m,flight_time_s`, with unique increasing positive
   distances and times. Interpolation is only inside that measured range. Save raw measurements
   outside deploy and refer to them in the test record.
4. Begin slow lateral passes in both directions, then radial approach/recede, then slow combined
   translation/rotation. Compare hit rates with stationary shots and legacy lead using equal trial
   counts, not impressions. Measure 0.5 m/s first and increase only after acceptable results.
5. Verify no feed during lost vision, excessive static-mode motion, a turret limit or hardware
   fault. Constant-velocity prediction does not model aggressive acceleration, wheel slip, ball
   spin/drag changes or collision impulses. Do not claim full-speed moving-shot accuracy from these
   desktop tests.
