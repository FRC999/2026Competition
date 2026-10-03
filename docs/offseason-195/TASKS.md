# Current task list

**Updated October 3, 2026.** This is the authoritative open-work list. Earlier “complete” statements
in the audits/session record refer to their respective delivered phases. New human instructions and
new measurements can change this list; record those decisions in [PROMPTS.md](PROMPTS.md).

## Next software decision

- [ ] **A1 — Mentor selects the Main/Worlds AUTO strategy.** The question is still unanswered:
  **first collection, then shoot for the remaining time** (previous recommendation), or **allow a later
  pickup only when the time budget reserves return and shooting**. No answer is implied by subsequent
  requests to document, commit or transfer the project. Ask for this decision before dependent edits.
  Independent setup, review and calibration preparation can continue.
- [ ] **A2 — Implement the selected strategy for both alliances and affected routines.** Preserve
  alliance guards, path events, intake/shoot cleanup and the outer 20 s deadline. If conditional pickup
  is selected, budget the whole excursion/return/stop/shot, not just departure. Preserve trench
  release/re-request semantics when deciding where shooting can resume.
- [ ] **A3 — Verify timing and interruption.** Cover both alliances, repeated enable, timeout,
  cancellation during each phase, failed localization/route stop, and prevention of feeding after a
  path failure. Regenerate the route audit and run `test robotSmoke build`; field timing is a later
  physical acceptance step. Update audits and both visual guides to match the final strategy.
- [ ] **A4 — Refresh the final comparison and delivery summary.** Use Houston `6c4ecb4` and the final
  source revision with `tools/change_audit.py`. Keep vision, simulation, other and shared integration
  distinct; show additions/deletions and explain churn. Commit the refreshed report and CSV, then push.

Why A1 matters: current named paths alone take Main **20.829 s**, Worlds Blue **13.282 s**, Worlds Red
**11.652 s**, before opening/deployment/stop checks and additional waits/shots. The complete original
sequences do not fit. The timeout stops overrun but does not complete the strategy. Default AUTO is
**Do nothing**. See [second-pass audit](second-pass-audit.md#auto-stop-and-trajectory-findings).

## Physical calibration and acceptance

These need the team's hardware, measured data and a separate operator request before physical action.
Desktop changes cannot mark them complete.

- [ ] **P1 — Mount/focus the OV9782 cameras and calibrate intrinsics** for each camera/resolution.
  Two Orange Pi 5 Plus boards are available; rear pair first, optional front pair later.
- [ ] **P2 — Survey the calibration field and independent robot stations.** Fit all six extrinsics per
  camera, evaluate held-out stations, and save captures/reports/Pi exports deliberately. Real XYZ is
  currently null/unmeasured; proposed orientations/heights are not calibration.
- [ ] **P3 — Restore and verify the identical competition layout on robot and every Pi.** Confirm
  hashes and independent known poses, then enable the calibrated configuration through the procedure.
- [ ] **P4 — Check mechanism references and physical envelope.** Turret boot stow/11:1 ambiguity,
  motor direction and perimeter limits; hood zero/travel; intake zero/homing/boost cleanup; measured
  trench clearance and hood lowering time; starting bumper placement. Climb stays disabled pending
  its separate home, limit and follower-direction validation.
- [ ] **P5 — Measure stationary pose/aim and route-stop accuracy on the robot.** Validate ordinary
  brake stops separately from strict precision alignment. Use independent truth and logged revisions.
- [ ] **P6 — Calibrate moving lead and passing.** Stationary shots first; measure flight times and
  evaluate release/rotation lead, then populate a separate passing table. Current flight/pass tables
  have no measured rows; do not invent them to enable a behavior.
- [ ] **P7 — Validate the selected autonomous strategy and event rules physically.** Time realistic
  opening placement, intake and shots; verify all field/extension guards and the applicable event's
  rules. Software point/center checks and simulation are not a bumper or collision inspection.

Use [installation](installation.md), [calibration](calibration.md) and [testing](testing.md) for detail.
Full deterministic robot replay and predictive collision/fuel/ballistic simulation are not implemented;
they are optional future projects, not hidden prerequisites or promised completed work.

## Delivered work — do not repeat from scratch

- [x] PhotonVision/Pigeon/wheel localization replaces active Limelight/Quest dependency.
- [x] Disabled stable MultiTag initialization, fresh/reset capture rules and startup diagnostics.
- [x] Broad drive/aim/shot/command/mechanism audit, cleanup and source documentation.
- [x] Interruption/mode/ownership fixes, guarded SysId and restored temporary intake settings.
- [x] Brake-only competition route stops and repaired incompatible moving joins.
- [x] Continuous feed/field/trench gates and explicit driver-confirmed manual fallback.
- [x] Simulation IO/units/load/lifecycle corrections and expanded desktop checks.
- [x] Houston source-count snapshot, audits, setup/calibration/test procedures and mirrored skills.
- [x] Two Markdown visual guides with 24 editable Mermaid diagrams and rendered SVGs.
- [x] Portable handoff, continuation prompt, gotchas, task list and a dependency-free handoff verifier.

Current prior verification: 117 normal Java tests + 1 robotSmoke, 7 Python tests, Javadoc generation,
24 diagram renders, six skill validations and three matching mirror pairs. Details and limitations are
in [HANDOFF.md](HANDOFF.md#revisions-and-previous-verification). Handoff documentation did not change
robot behavior; rerun relevant checks on the new machine before changing or relying on its environment.
