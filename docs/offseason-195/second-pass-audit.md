# Interruption, autonomous, rules and simulation audit

October 1, 2026. This follows the full refactor at `80acda9`. The mentor confirmed Houston
`6c4ecb4` as the Worlds comparison baseline and retained driver-confirmed manual preset shooting
when localization is unavailable. The earlier audit remains historical evidence, not the current
description where this document supersedes it.

## Interruption and ownership findings

| Finding in the preceding revision | Correction | Evidence |
|---|---|---|
| Controller bindings could schedule mechanism/drive commands during AUTO and cancel a whole autonomous group. | All operator actions require enabled teleop; manual tracking override is ignored in AUTO. | Full robot simulation presses/releases LT, RT and A during AUTO and checks ownership. |
| A mode change is a trigger falling edge: LT release cleanup could retract and cancel the newly scheduled auto. A required `onlyIf` command would still take requirements before rejecting execution. | Check mode/panic before **scheduling** the required release command. | Full robot simulation changes modes with LT held. |
| A button held through enable/panic recovery could become a new rising edge without a fresh operator request. | `FreshPress` requires release/repress after a closed mode/panic gate. | Pure gate test and full robot mode transition. |
| Independent RB reverse commands shared a supervisor requirement and could cancel each other. | One jam-clear command owns shooter, transfer, spindexer and supervisor and stops all three on every end path. RB release no longer stops an unrelated intake. | Full robot verifies transfer reverse duty, ownership, cancellation and no stale shot revival. |
| PathPlanner's interrupted follower deliberately leaves its last drive request active. | Our follower wrapper stops on interruption; autonomous mode exit and the enclosing deadline clean up drive, feed and intake. | Pinned PathPlanner 2026.1.2 source inspection; existing command finish tests. |
| Intake boost cleanup changed the Java flag but left a successful hold using slot 2. Initial deploy repeatedly enabled boost and ignored its 0.75 s maximum. Held LT could keep a blocked deployment powered indefinitely. | Reissue the held target with the normal slot, honor the retained boost maximum, and bound LT pivot deployment to the retained 5 s timeout while keeping rollers usable. | CTRE applied-request integration assertion; bounded command paths and existing homing tests. |
| SysId permission was evaluated when the command was constructed, usually while disabled, so a later allowed test could remain a no-op. | `GuardedSysId` evaluates on schedule and ends/stops on revocation, interruption or completion; motor callbacks check the gate too. | Scheduler test covers allowed/denied/revoked operation. |
| An active auto follower could keep driving after gyro reset revoked its field reference. | New auto motion requests latch a failure and stop if field reference is lost. | Shared guarded output path; localization reset tests. |

WPILib command groups own the union of their children’s requirements for their entire lifetime.
Supervisor arbitration compares the scheduler owner, so a group containing a shoot child remains a
single owner. Direct SysId/jog commands retain exclusive control. Shoot interruption immediately
clears feed intent; intentional idle shooter RPM is retained. No interrupt restores an old shot request.
Intake timeouts/cancellation stop the pivot; timeout is never homing evidence. Autonomous commands
are canceled on exit, and repeated selection uses a cached enclosing deadline instead of recomposing
the same command. Teleop/test exits also cancel active commands.

## Auto stop and trajectory findings

At `80acda9`, Main Blue/Red used **5** strict precision finishes each (including the opening approach),
Worlds Blue **4**, and Worlds Red **3**. Each could spend up to 4 s trying to settle. That was inappropriate
for ordinary route transitions and turret-aimed shooting stations.

Competition paths now request `ROUTE_STOP`: brake/hold the module angles, make **no corrective
translation or rotation**, and check position within 0.20 m, heading within 5 degrees, module speed
at most 0.15 m/s, gyro rate at most 8 deg/s, and fresh referenced vision for 0.06 s. The check has a
0.50 s maximum. Failure holds the auto and inhibits feeding; it is not accepted as arrival. These are
initial route acceptance settings, not measured robot accuracy. Strict precision alignment remains
available to the explicit precision test/API. Consequently competition autos have **zero jittering
precision finishes**. This does not establish that physical wheel steering will never move under hold.

The route audit found five nonzero-speed joins with incompatible velocity directions: approximately
84, 33 and 83 degrees in Main, and 67 and 60 degrees in Worlds. Adjacent path end/start speeds now
agree at zero at those joins. Initial path speeds also match the stopped opening approach. The
continuous Main shooting-to-trench handoff retains 0.5 m/s and matching holonomic heading. Route
geometry was not redesigned. After these repairs the full original sequences contain Main **8**,
Worlds Blue **6**, Worlds Red **5** ordinary brake checks including the approach.

`AutoRouteAuditTest` generates `2026Competition/build/reports/auto-route-audit.md` from the checked-in
PathPlanner configuration, verifies field-half containment on both alliances, and checks join speed,
heading and nonzero-speed tangent continuity. Planned Main maximum blue-side X is 7.612 m and Worlds
is 7.474 m. These are centerline checks, not collision simulation or measured bumper clearance.

| Full original routine | Named-path time after feasible joins | Additional known time, before opening/deploy/stop checks |
|---|---:|---|
| Main Blue/Red | 20.829 s | 5 s final shot; first moving shot is already in path time |
| Worlds Blue | 13.282 s | 1 s start wait + 3.75 s shot + 1.5 s deploy window + 2 s shot |
| Worlds Red | 11.652 s | 3 s start wait + 3.75 s shot + 1.5 s deploy window + 2 s shot |

The opening approach and initial deploy (up to 3 s) add more. The 20 s deadline stops overruns, but
does not make these complete strategies feasible. **Mentor decision pending:** keep the first collection
and shoot for the remainder, or consider a later pickup only with sufficient remaining time. Do not
claim these full original sequences complete in AUTO. A later-pickup option needs an explicit return
and shooting reserve, including failure handling; checking only time to start the outbound leg is wrong.

Named paths reject starts over 0.35 m from their planned start. The generated opening approach is
limited to the starting trench corridor (within 0.25 m lateral, no more than 3 m away and on the alliance
side of the starting boundary). It is not obstacle-aware pathfinding. Physical starting placement and
bumper overlap still need operator verification.

## Rules and position gates

Checked the official [2026 REBUILT manual, TU22](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf),
sections 5.3, 5.6, 6.4, G303/G402/G403/G407/G413 and R104–R107:

- AUTO is 20 s; a 3 s disabled scoring interval precedes 140 s TELEOP. Teleop has a 10 s transition,
  four 25 s alliance shifts, and 30 s endgame. Inactive-hub fuel earns no points; this is not a general shot ban.
- G402 disallows driver interaction in AUTO. G403 prohibits completely crossing the center line.
  G407 requires bumper overlap with the alliance zone for hub shots; zone depth is 158.6 in.
- G303 includes starting-line bumper overlap, no BUMP contact, starting configuration and at most eight fuel.
- R104–R107 cover 110 in starting perimeter, 30 in height, 12 in horizontal extension and one-side extension.
  G413’s momentary exception also requires no strategic benefit.
- Trench structures are 65.65 in wide and 47 in deep; the opening is 50.34 in wide and 22.25 in high.

Software now checks the robot center conservatively inside its own hub-scoring zone, with a provisional
0.10 m allowance and velocity prediction to estimated release. The gate is reevaluated during firing.
Unknown/stale position blocks automatic shots. Per mentor instruction, manual preset fallback remains
available with driver responsibility; `AutoShoot/ManualZoneConfirmationRequired` identifies that case.
Known manual position is still checked. These checks cannot establish legality when the pose is wrong.

The old trench regions were only about 0.3 m wide. They now cover the nominal full structures at all
four locations. A line-segment approach check adds provisional 0.45 m padding and 0.35 s lookahead;
entering it clears shooting and requests neutral hood, with a fresh request required after exit.
These guards are software settings, not an invented robot footprint or measured hood response.
Verify the actual swept envelope, neutral height, lowering time and localization error before relying
on clearance. Turret perimeter limits remain unchanged. Disabled climb remains disabled. No code
review can certify construction, bumper geometry, startup placement or measured mechanism dimensions.

## Simulation defects and corrections

| Defect | Correction and scope |
|---|---|
| Seven rotary models supplied gearing and inertia in the wrong argument order, producing incorrect dynamics. | Shared `RotaryMotorSim` uses `DCMotorSim`/`LinearSystemId` with inertia then gearing and distinguishes mechanism rotations from rotor rotations. Tests cover all retained reductions, free speed and inertia. |
| Turret mechanism rotations were supplied to a raw rotor sensor; encoder sign/offset and physical boot stow were inconsistent. | Convert through the retained 11:1 reduction; synthesize the pinion absolute reading using the same sign/zero convention as production. Simulation starts at physical stow. |
| Paired mechanisms had single-motor loads or stale follower sensors. | Shooter/intake pivot/climb use two-motor models; follower rotor and supply signals are updated with configured alignment. |
| Sim models were constructed on the real path, and synthetic camera construction had no explicit real-robot guard. | Models are created only in simulation; sim callbacks and synthetic-camera construction are guarded. Real IO/calibration selection stays separate. |
| Static camera worlds could retain old layouts/cameras and be advanced once per camera each loop. | Each camera owns its synthetic world and closes camera resources. Frames still use production PhotonVision ingestion. |
| Battery simulation omitted the drivetrain and could treat reversed current as negative load. | Include drive and steer supply currents; mechanism load draw is nonnegative. Log total current and battery voltage. |
| Simulation placement and the 5 ms drivetrain update could race; notifier/camera resources outlived shutdown. | Synchronize placement/update, retain volatile truth, stop/close notifier before drivetrain closure, and close camera IO with Robot. |

Physics remains deliberately limited: rotary inertias and synthetic lens/noise values are assumptions;
there is no validated gravity/friction/hard-stop, bumper collision, fuel transport, impact or shot-flight
model. The tests establish control/IO behavior, not real shooting accuracy, route timing, motor direction,
mechanical clearance or brownout fidelity. Full robot desktop startup can emit CAN/joystick and loop-overrun
warnings, particularly during synthetic vision initialization; do not interpret desktop timing as roboRIO timing.

## Comparison and verification

[Change counts](change-counts.md) and the line-by-line CSV are produced by `tools/change_audit.py`.
The baseline is mentor-confirmed Houston `6c4ecb4`. Production Java, tests and data are reported separately.
Each production change is counted once; shared integration stays explicit instead of pretending an
exact causal split between camera replacement and the broader overhaul. Deleted vendored helpers and
dead legacy code are included in churn. Do not read added+deleted as the count of unique edited lines.

Run `gradlew test robotSmoke build`, Python unittest discovery, skill validation and mirror verification.
The smoke test also inspects the applied CTRE intake slot, ignores AUTO controller input/releases,
checks fresh presses across modes and verifies repeated auto enable. No robot was deployed or operated.
Final receipt and remaining strategy decision are recorded in SESSION_STATE.md.
