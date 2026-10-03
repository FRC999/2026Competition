# Continue on another computer

Updated October 3, 2026. Clone or update **FRC999/2026Competition**, switch to **OffSeason-195**, and
open that repository in the coding assistant. Use the repository root, which contains `AGENTS.md`;
the actual robot project is its `2026Competition/` subdirectory.

Copy the entire block below into the new chat. It is self-contained once the repository is available.
The old chat, old computer's drive letters and personal/global skills are not needed.

```text
Continue the FRC999 2026 competition robot overhaul on the OffSeason-195 branch of
https://github.com/FRC999/2026Competition.git. Work in this local checkout; do not assume the previous
computer's S: or T: paths exist. The robot project is 2026Competition/ inside the repository root.
QuestVibeGPT-Imported2026beta is historical and is not the target.

First inspect git status, branch and current revision. Preserve any local work. Read AGENTS.md,
docs/offseason-195/HANDOFF.md, SESSION_STATE.md, TASKS.md and GOTCHAS.md. Those latter four files are
all under docs/offseason-195/. Read the relevant repo skills under .agents/skills/; their equivalent
Claude copies are under .claude/skills/. Read PROMPTS.md for the original instructions and decisions,
then the second-pass audit and code guide as needed. Do not restart work already recorded as complete.
Run python tools/verify_handoff.py to verify the portable repository materials.

The PhotonVision migration, broad algorithm refactor, concurrency/interruption audit, simulation
corrections, source documentation and 24 diagrams have been delivered. This is not yet a physically
calibrated competition release. Main/Worlds autonomous strategy is the remaining software decision:
their full original sequences exceed the 20-second AUTO budget. Ask me to choose first collection
then shooting for the remaining time, versus a later pickup only when enough return/shooting time
is reserved, unless TASKS.md or my new message already records that choice. Do not silently choose
it or claim the old sequences fit. Once resolved, implement it, test interruption and timing,
update documentation/diagrams and refresh the final change counts.

Preserve measured 2026 hardware geometry. The 2027 prototype uses a different chassis; it is an
algorithm reference, not a source of robot dimensions or camera calibration. Keep PhotonVision plus
Pigeon/wheel odometry, blue-origin field coordinates, gyro-owned enabled heading, one FPGA-to-CTRE
timestamp conversion and one alliance path transformation. Preserve continuous feed gates,
interruption cleanup and brake-only ROUTE_STOP for competition paths. Camera XYZ, field surveys,
flight times and passing shots must be measured, never invented. Manual shooting fallback without
localization remains authorized with driver-confirmed position and a visible dashboard indication.
Use Houston 6c4ecb4c196541236e7f3a702e2ad1099a094e1c as the confirmed Worlds comparison baseline.

Continue the authorized code/docs/tests work autonomously; ask only for genuinely missing behavior
decisions or measurements. Commits and push to OffSeason-195 are authorized. Do not push to the
Houston/Worlds branches, deploy code, operate physical hardware or enable climb without a separate
operator request. Keep documents, skills, prompts and evidence outside the robot src/ tree.

Update SESSION_STATE.md, TASKS.md, PROMPTS.md and any affected docs/skills as work progresses. Keep
.agents and .claude skill copies identical. Use the pinned build configuration and the verification
commands in HANDOFF.md, distinguishing desktop results from physical evidence. Finish completed
work with a descriptive commit, push, and verify the remote branch. Start by briefly stating the
current state and the next actionable task, including the pending strategy question if still open.
```

For clone, dependencies, troubleshooting and exactly what is stored in Git, use [HANDOFF.md](HANDOFF.md).
For the remaining work, use [TASKS.md](TASKS.md). This prompt grants no scheduled background work and
does not depend on transferring the original chat.
