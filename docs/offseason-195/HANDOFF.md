# Portable project handoff

**Updated October 3, 2026 · repository FRC999/2026Competition · branch OffSeason-195**

Everything needed to continue the source, documentation and desktop work is in this branch. Start with
the [copy-and-paste continuation prompt](CONTINUE_PROMPT.md), [current task list](TASKS.md) and
[gotchas](GOTCHAS.md). [SESSION_STATE.md](SESSION_STATE.md) is the fuller chronological record;
[PROMPTS.md](PROMPTS.md) preserves human instructions. Current status and tasks supersede historical
completion statements about earlier phases.

The package at `e1a91c9` was verified in a fresh full-history checkout without the old virtual
environment: 60 required tracked files, 229 local links/anchors, 24 diagram pairs and three matching
skill pairs passed. The Houston baseline was present and the checkout was clean. This verifies the
handoff contents; full robot build/hardware evidence remains distinct below.

## Get the correct checkout

From a parent folder of your choice on the new computer:

```powershell
git clone --branch OffSeason-195 https://github.com/FRC999/2026Competition.git
Set-Location 2026Competition
git status --short --branch
git log -5 --oneline
```

The clone needs your normal GitHub repository access. Keep full Git history: the change-count tool
compares against the Houston commit. If you already have a checkout, inspect its status first; once
local work is accounted for, fetch, switch to `OffSeason-195`, and update with `git pull --ff-only`.
Do not reset or overwrite unrelated changes to reproduce this machine's state.

The repository root contains `AGENTS.md`, `CLAUDE.md`, `docs/` and `tools/`. The robot project is the
**nested `2026Competition/` directory**, containing `build.gradle` and `gradlew.bat`. Do not build or
modify `QuestVibeGPT-Imported2026beta` for this task. The previous machine used an S: clone and left
a T: checkout untouched; those are historical locations, not required paths on the new computer.

## Read in this order

1. [AGENTS.md](../../AGENTS.md) and [current tasks](TASKS.md).
2. [Session state](SESSION_STATE.md), [gotchas](GOTCHAS.md), then [human decisions](PROMPTS.md) when needed.
3. The relevant [retrofit](../../.agents/skills/frc999-photon-retrofit/SKILL.md),
   [drive/aim audit](../../.agents/skills/frc999-drive-aim-audit/SKILL.md) or
   [camera calibration](../../.agents/skills/frc999-camera-calibration/SKILL.md) skill.
4. [Second audit](second-pass-audit.md) and [code guide](code-guide.md) for current behavior;
   [full refactor audit](full-refactor-audit.md) for issue/fix history.
5. [Team overview](team-overview.md) and [programming diagrams](programming-diagrams.md) for visual flow.
6. [Installation](installation.md), [calibration](calibration.md) and [testing](testing.md) for the relevant work.

The three repo skills have matching `.agents/skills/` and `.claude/skills/` copies. `CLAUDE.md` points
to the same entry instructions. They are ordinary tracked files: read them explicitly if the new
assistant does not discover repository skills automatically. No personal skill installation, Figma
plugin, Codex bundled runtime or old chat state is required to read or continue this project.

## Revisions and previous verification

| Revision | Meaning |
|---|---|
| `6c4ecb4c196541236e7f3a702e2ad1099a094e1c` | Mentor-confirmed Houston/Worlds comparison baseline. |
| `5a3e431` | Initial PhotonVision retrofit and calibration tools. |
| `80acda9` | Broad localization, drive, shot and mechanism refactor. |
| `a37f951` | Second-audit interruption, route-stop, field guards and simulation corrections. Latest runtime behavior change at this handoff. |
| `5cedfe09beb8969dcadaf15bcfeb01459f0dcacc` | Java/API documentation; current source-count snapshot target. |
| `d22e7a2` | Refreshed source-count report and receipt. |
| `cc094fc959fc2a2f9fa4fc0e8fe598be587ca978` | Two visual guides and 24 rendered diagrams. |

The handoff itself is a later documentation/tooling commit. `git log -1` and the remote tracking
revision give its receipt without a self-referential hash in this file. Use current Git status and
history rather than assuming these historical revisions are always HEAD.

Last full behavior verification on the previous Windows computer: **117 normal Java tests + one
complete robot smoke test**, `test robotSmoke build` passed, and **7 Python tests** passed. Javadoc
generation passed after the source-comment work, with remaining missing-member/tag warnings. The
visual guides passed 24 Mermaid render/check runs and 114 local links/anchors; six skills validated
and three mirror pairs matched. These are prior results, not evidence from a new computer or a robot.
CAN/joystick/startup loop-overrun warnings occurred during desktop smoke testing. No physical robot
operation, deployment, measured shot validation or verified field-clearance acceptance occurred.

## Recreate the development tools

Robot code uses **Java 17 / WPILib 2026**. GradleRIO is pinned to `2026.2.1`; the Gradle 8.11 wrapper
and vendor JSONs are committed. Vendor versions are PhotonLib `2026.3.4`, Phoenix `26.3.0`,
AdvantageKit `26.0.2` and PathPlanner `2026.1.2`. Use the checked-in versions before considering upgrades.
The first build downloads dependencies; the old computer's caches are not part of the handoff.

On Windows, point `JAVA_HOME` to the **new computer's** WPILib Java 17 JDK. If installed at the usual
location, the command is:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
& "$env:JAVA_HOME\bin\java.exe" -version
Push-Location 2026Competition
.\gradlew.bat test robotSmoke build --console=plain
.\gradlew.bat javadoc --console=plain
Pop-Location
```

Use Python 3.13 to match the verified Windows setup. From repository root:

```powershell
python tools/verify_handoff.py
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
.\.venv\Scripts\python.exe -m unittest discover -s tools -v
.\.venv\Scripts\python.exe tools/verify_skill_mirrors.py
```

The previous interpreter was Python 3.13.6 with NumPy 2.5.1, SciPy 1.18.0 and pyntcore 2026.2.1.
The maintained requirements permit compatible NumPy/SciPy versions and pin the 2026 NT API. Install
into a fresh environment; do not copy `.venv` between computers. Without pyntcore, the localhost NT
test is skipped, so that result is not the same as seven tests passing. Its port is 15810; it does not
contact a physical robot. The handoff and skill-mirror checks use Python's standard library only.

On another OS, use its WPILib 2026 Java 17 runtime, `./gradlew` inside the robot directory, and
`.venv/bin/python` for the Python commands. Native vendor support and a full run on that OS must be
verified there; this handoff's recorded full suite ran on Windows.

## Diagram tools are optional

The committed SVGs display without any diagram tool. To edit and regenerate them, use Node.js and
local packages; the previous setup used Node 22.18.0, Mermaid 11.12.2 and Playwright 1.62.1:

```powershell
npm install --prefix artifacts/diagram-renderer --no-audit --no-fund mermaid@11.12.2 playwright@1.62.1
node artifacts/diagram-renderer/node_modules/playwright/cli.js install chromium
node tools/render_diagrams.mjs
node tools/render_diagrams.mjs --check
```

The renderer can also use existing package/browser paths via its documented command-line options.
It serves local Mermaid assets only on loopback. Keep Mermaid source blocks and SVGs in the same
commit. Exact render comparisons can reflect font/browser differences; inspect intentional layout
changes. No diagram editor plugin or remote diagram account is required.

## What Git carries, and what must be recreated or measured

| Included in the repository | Location |
|---|---|
| Entry instructions and shared skills | `AGENTS.md`, `CLAUDE.md`, `.agents/skills/`, `.claude/skills/` |
| State, prompts, tasks, gotchas and guides | `docs/offseason-195/` |
| Robot source/tests, build wrapper and vendors | `2026Competition/src/`, `gradle/`, `gradlew*`, `build.gradle`, `vendordeps/` inside the robot project |
| Real camera configuration and field JSONs | `2026Competition/src/main/deploy/vision/` |
| Shot tables and PathPlanner assets | `2026Competition/src/main/deploy/artillery/`, `pathplanner/` |
| Separate synthetic desktop profile/layout | `2026Competition/simulation/` |
| Calibration, comparison, rendering and verification tools | `tools/` |
| Rendered diagrams and source-count evidence | `docs/offseason-195/diagrams/`, `change-counts.md`, `change-counts.csv` |

Ignored/recreated material includes `.venv`, Gradle caches and `build/`, `artifacts/`, desktop `logs/sim`,
generated Javadoc/test reports and local diagram packages/previews. Historical raw desktop reports and
WPILOGs are not included; the committed receipts describe them and the commands regenerate evidence.
No completed physical camera survey, fitted real transforms, Pi intrinsic export, measured passing
table or measured flight-time data exists from this session. Their absence is an open task, not a
missing upload. Future physical evidence must be deliberately archived outside robot `src/`; placing
it under ignored `artifacts/` alone does not transfer it through Git.

The 2027 prototype checkout is optional for historical comparison, not required to build this branch.
If needed, clone `https://github.com/FRC999/2027Prototyping.git` separately and inspect
`d20594af6fde49686fcbd9ed63250cf94463aaf5`. Do not substitute its current main branch or chassis data.

## Workstation gotchas and finishing a task

- Windows Git certificate errors on the previous machine were resolved per command with
  `git -c http.sslBackend=schannel ...`. Use this only when needed on Windows.
- Java dependency certificate errors there used the Windows root store:
  `$env:JAVA_TOOL_OPTIONS = '-Djavax.net.ssl.trustStoreType=Windows-ROOT -Djavax.net.ssl.trustStore=NONE'`.
  This is a Windows workaround, not a cross-platform requirement or disabled certificate verification.
- Node 22.18 needed `--use-system-ca` for npm on that machine. Resolve the new machine's trust setup
  if needed; do not carry over personal absolute npm/Codex paths or disable TLS verification.
- Start robot builds/tests from the nested project directory; start Python/diagram examples from
  repository root. Run smoke separately through its Gradle task, which uses a dedicated JVM.
- No running service, agent, background job or uncommitted runtime patch is needed to resume.

When behavior changes, run the relevant tests and complete integration checks; update state, tasks,
human prompts, affected guides and mirrored skills. After final strategy code is committed, refresh
the source-count report from repository root with:

```powershell
python tools/change_audit.py --base 6c4ecb4 --target HEAD
```

This writes `change-counts.md` and `.csv`; review and commit them. Added + deleted counts are source
churn including comments, not unique lines edited. Keep shared integration separate from direct vision
and simulation attribution. Commit/push to **OffSeason-195** is already authorized. Physical operation
and deployment need a separate operator request. Verify a clean/understood worktree and matching remote
revision before telling the mentor the next handoff is complete.
