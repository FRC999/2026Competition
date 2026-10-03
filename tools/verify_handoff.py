"""Read-only, standard-library check of the portable OffSeason-195 handoff.

Run from any directory: python <repo>/tools/verify_handoff.py
Checks tracked files, local Markdown links/anchors, skill mirrors, diagram pairs and history.
Does not contact a network, install dependencies, deploy, or run robot hardware/simulation.
"""
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs/offseason-195"
BASE = "6c4ecb4c196541236e7f3a702e2ad1099a094e1c"


def git(*args):
    result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, encoding="utf-8")
    if result.returncode:
        raise SystemExit(result.stderr.strip() or f"Git command failed: {args}")
    return result.stdout.strip()


def require(condition, message):
    if not condition:
        raise SystemExit(message)


def main():
    tracked = set(git("ls-files", "-z").split("\0"))
    required = ["AGENTS.md", "CLAUDE.md", "README.md", "tools/verify_handoff.py",
                "tools/verify_skill_mirrors.py", "tools/requirements.txt", "tools/vision_calibration.py",
                "tools/test_vision_calibration.py", "tools/test_capture_loopback.py",
                "tools/change_audit.py", "tools/render_diagrams.mjs"]
    required += ["docs/offseason-195/" + name for name in (
        "HANDOFF.md", "CONTINUE_PROMPT.md", "TASKS.md", "GOTCHAS.md", "SESSION_STATE.md", "PROMPTS.md",
        "README.md", "code-guide.md", "team-overview.md", "programming-diagrams.md", "installation.md",
        "calibration.md", "testing.md", "audit.md", "full-refactor-audit.md", "second-pass-audit.md",
        "change-counts.md", "change-counts.csv")]
    required += ["2026Competition/" + name for name in (
        "build.gradle", "settings.gradle", "gradlew", "gradlew.bat", ".wpilib/wpilib_preferences.json",
        "gradle/wrapper/gradle-wrapper.jar", "gradle/wrapper/gradle-wrapper.properties",
        "vendordeps/AdvantageKit.json", "vendordeps/PathplannerLib-2026.1.2.json",
        "vendordeps/Phoenix6-26.3.0.json", "vendordeps/WPILibNewCommands.json", "vendordeps/photonlib.json",
        "simulation/vision.json", "simulation/two-tag-field.json", "src/main/deploy/vision/cameras.json",
        "src/main/deploy/vision/fields/2026-rebuilt-welded.json",
        "src/main/deploy/vision/fields/2026-rebuilt-andymark.json",
        "src/main/deploy/artillery/flight_times.csv", "src/main/deploy/artillery/pass_shots.csv",
        "src/main/deploy/artillery/moving_auto_shots.csv", "src/main/deploy/pathplanner/settings.json",
        "src/main/java/frc/robot/Robot.java", "src/main/java/frc/robot/RobotContainer.java",
        "src/test/java/frc/robot/RobotStartupSmokeTest.java", "src/test/java/frc/robot/commands/AutoRouteAuditTest.java")]
    skill_names = ("frc999-photon-retrofit", "frc999-drive-aim-audit", "frc999-camera-calibration")
    required += [f"{tree}/skills/{name}/SKILL.md" for tree in (".agents", ".claude") for name in skill_names]
    for relative in required:
        require(relative in tracked and (ROOT / relative).is_file(), f"Missing tracked handoff file: {relative}")
    for name in skill_names:
        require((ROOT / f".agents/skills/{name}/SKILL.md").read_bytes()
                == (ROOT / f".claude/skills/{name}/SKILL.md").read_bytes(), f"Skill mirror mismatch: {name}")
    require(git("cat-file", "-t", BASE) == "commit", "Houston baseline is missing; fetch full history")

    link_count = 0
    markdown_files = [ROOT / "AGENTS.md", ROOT / "CLAUDE.md", ROOT / "README.md", *DOCS.glob("*.md")]
    markdown_files += [ROOT / f"{tree}/skills/{name}/SKILL.md"
                       for tree in (".agents", ".claude") for name in skill_names]
    for source in markdown_files:
        body = re.sub(r"```[^\n]*\n[\s\S]*?```", "", source.read_text(encoding="utf-8"))
        for target in re.findall(r"!?\[[^\]]*\]\(([^)]+)\)", body):
            if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target):
                continue
            local, _, fragment = unquote(target).partition("#")
            destination = (source.parent / local).resolve() if local else source
            require(destination.exists(), f"{source.relative_to(ROOT)}: missing link {target}")
            if destination.is_file():
                try:
                    relative = destination.relative_to(ROOT).as_posix()
                except ValueError:
                    raise SystemExit(f"Nonportable link outside repo: {source}: {target}")
                require(relative in tracked, f"Link target is not tracked: {relative}")
            if fragment and destination.suffix == ".md":
                headings = re.findall(r"^#{1,6} (.*)$", destination.read_text(encoding="utf-8"), re.M)
                slugs = {re.sub(r"[^\w\- ]", "", heading.lower()).replace(" ", "-") for heading in headings}
                require(fragment in slugs, f"{source.relative_to(ROOT)}: missing anchor {target}")
            link_count += 1

    diagram_count = 0
    for guide in ("team-overview.md", "programming-diagrams.md"):
        body = (DOCS / guide).read_text(encoding="utf-8")
        images = re.findall(r"!\[[^\]]+\]\((diagrams/[^)]+\.svg)\)", body)
        require(len(images) == len(re.findall(r"^```mermaid$", body, re.M)), f"Unpaired diagrams: {guide}")
        for relative in images:
            svg = ET.fromstring((DOCS / relative).read_text(encoding="utf-8"))
            require(svg.get("role") == "img" and svg.get("aria-label"), f"Missing diagram description: {relative}")
            diagram_count += 1
    print(f"Verified {len(required)} required tracked files, {link_count} local links/anchors,")
    print(f"{diagram_count} diagram pairs, 3 skill mirror pairs and the Houston comparison commit.")
    print(f"Branch: {git('branch', '--show-current') or '(detached)'}; HEAD: {git('rev-parse', '--short', 'HEAD')}")
    if git("status", "--porcelain"):
        print("Working tree/index has local changes; review them before pulling or transferring.")
    else:
        print("Working tree is clean.")
    print("This checks handoff contents, not a Java build, physics, calibration or robot readiness.")


if __name__ == "__main__":
    main()
