"""Reproducible source churn with conservative, non-overlapping attribution.

Run from any directory: python tools/change_audit.py --base 6c4ecb4 --target HEAD.
Omit --target to compare the worktree (stage new files or use git add -N first).
Attribution is a documented accounting convention, not a counterfactual cost estimate.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
JAVA = "2026Competition/src/main/java/"
CATEGORIES = ("Vision migration", "Simulation", "Other fixes and strategy", "Shared integration")
SHARED = {"Robot.java", "RobotContainer.java", "Constants.java", "DriveSubsystem.java",
          "ElasticHelpers.java", "Telemetry.java"}
SIM_METHOD = re.compile(r"\b(?:simulationPeriodic|getSimCurrentDrawAmps|startSimThread|placeSimulationRobot|getSimulationTruthPose)\s*\(")
SIM_FIELD = re.compile(r"^\s*(?:private|public|protected)\s+.*\b(?:FlywheelSim|DCMotorSim|RotaryMotorSim|simNotifier|simulationTruth|kSimLoopPeriod|lastSimTime)\b")

def git(*args: str) -> str:
    return subprocess.run(["git", "-c", "core.quotepath=false", *args], cwd=ROOT,
                          check=True, capture_output=True, encoding="utf-8").stdout

def simulation_lines(source: str) -> set[int]:
    """Tag complete simulation methods/fields; leave ambiguous constructor wiring shared."""
    lines = source.splitlines()
    tagged = set()
    for i, line in enumerate(lines):
        method = bool(SIM_METHOD.search(line) and re.search(r"\b(public|private|protected)\b", line))
        field = bool(SIM_FIELD.search(line) and not method)
        if method or field:
            depth, opened = 0, False
            for j in range(i, len(lines)):
                tagged.add(j + 1)
                # Source here contains no brace-bearing strings in these simulation blocks.
                code = lines[j].split("//", 1)[0]
                depth += code.count("{") - code.count("}")
                opened |= "{" in code
                if (method and opened and depth <= 0) or (field and ";" in code):
                    break
        if line.lstrip().startswith("import ") and any(s in line for s in
                (".simulation.", "SimState", "FlywheelSim", "DCMotorSim", "RotaryMotorSim")):
            tagged.add(i + 1)
    return tagged

def category(path: str, line_number: int, sim: set[int]) -> str:
    name = Path(path).name
    if "/simulation/" in path or name == "VisionIOPhotonVisionSim.java" or line_number in sim:
        return "Simulation"
    if ("/subsystems/vision/" in path or "/OdometryUpdates/" in path or name in {
            "OffseasonVisionConfig.java", "VisionConstants.java", "LimelightHelpers.java",
            "QuestHelpers.java", "VisionHelpers.java"}):
        return "Vision migration"
    return "Shared integration" if name in SHARED else "Other fixes and strategy"

def changed_lines(diff: str):
    old = new = 0
    for line in diff.splitlines():
        if line.startswith("@@"):
            m = re.match(r"@@ -(\d+)(?:,\d+)? \+(\d+)(?:,\d+)? @@", line)
            old, new = map(int, m.groups())
        elif line.startswith(("---", "+++", "\\")):
            continue
        elif line.startswith("-"):
            yield "deleted", old, line[1:]
            old += 1
        elif line.startswith("+"):
            yield "added", new, line[1:]
            new += 1
        elif line.startswith(" "):
            old += 1
            new += 1

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="6c4ecb4")
    parser.add_argument("--target")
    parser.add_argument("--output", default="docs/offseason-195/change-counts")
    args = parser.parse_args()
    revisions = [args.base] + ([args.target] if args.target else [])
    paths = git("diff", "--name-only", "--no-renames", *revisions, "--", JAVA).splitlines()
    counts = defaultdict(Counter)
    byfile = defaultdict(Counter)
    rows = []
    for path in paths:
        def content(revision):
            if revision is None:
                f = ROOT / path
                return f.read_text(encoding="utf-8") if f.exists() else ""
            # A missing file on one side is an ordinary addition/deletion.
            result = subprocess.run(["git", "show", f"{revision}:{path}"], cwd=ROOT,
                                    capture_output=True, encoding="utf-8")
            return result.stdout if result.returncode == 0 else ""
        masks = {"deleted": simulation_lines(content(args.base)), "added": simulation_lines(content(args.target))}
        diff = git("diff", "--no-renames", "--unified=0", "--no-ext-diff", *revisions, "--", path)
        for kind, number, text in changed_lines(diff):
            group = category(path, number, masks[kind])
            counts[group][kind] += 1
            byfile[(path, group)][kind] += 1
            rows.append([path, kind, number, group, text])
    expected = Counter()
    for row in git("diff", "--numstat", "--no-renames", *revisions, "--", JAVA).splitlines():
        add, delete, _ = row.split("\t", 2)
        expected.update(added=int(add), deleted=int(delete))
    actual = sum(counts.values(), Counter())
    assert actual == expected, (actual, expected)
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.with_suffix(".csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(["file", "change", "source_line", "category", "source_text"])
        writer.writerows(rows)
    base = git("rev-parse", args.base).strip()
    target = git("rev-parse", args.target).strip() if args.target else "working tree (including indexed new files)"
    report = ["# Source change counts", "", f"Baseline: `{base}` (Houston, confirmed by mentor).",
              f"Target: `{target}`.", "", "Production Java only; physical source lines including comments/blanks. "
              "Added + deleted is churn, not unique edited lines. No rename detection; deleted legacy code is included.", "",
              "| Attribution | Added | Deleted | Churn |", "|---|---:|---:|---:|"]
    for label in (*CATEGORIES, "TOTAL"):
        c = actual if label == "TOTAL" else counts[label]
        report.append(f"| {label} | {c['added']:,} | {c['deleted']:,} | {sum(c.values()):,} |")
    report += ["", "## Attribution convention", "",
        "Every changed line appears once in the CSV. Simulation files, named simulation methods/fields and "
        "simulation imports take precedence. Vision includes the replacement localization stack, configuration, "
        "and retirement of Limelight/Quest helpers. Other includes precision driving, aim/shot algorithms, "
        "commands, mechanism fixes and retired dead code. Shared integration keeps Robot, RobotContainer, "
        "Constants, DriveSubsystem, ElasticHelpers and Telemetry changes unallocated except their explicit "
        "simulation scopes. Those files combine vision wiring with behavioral changes; calling all of them "
        "'just PhotonVision' would be misleading. The vision and simulation buckets are conservative direct "
        "attributions; a unique causal allocation of every shared line is not possible from a final diff.", "",
        "Tests, path/configuration data, build dependencies, calibration tools and documentation are excluded "
        "from production Java totals. Separate tracked-file counts follow. The generated report/CSV are excluded "
        "from those supplemental totals to avoid self-counting.", "",
        "## Supplemental files", "", "| Scope | Added | Deleted | Churn |", "|---|---:|---:|---:|"]
    extras = defaultdict(Counter)
    for row in git("diff", "--numstat", "--no-renames", *revisions).splitlines():
        add, delete, path = row.split("\t", 2)
        if path.startswith(JAVA) or "change-counts." in path or add == "-":
            continue
        scope = ("Java tests" if "/src/test/" in path else "Path/configuration data" if "/src/main/deploy/" in path
                 else "Tools and Python tests" if path.startswith("tools/") else "Documentation/skills/build/other")
        extras[scope].update(added=int(add), deleted=int(delete))
    for scope, c in sorted(extras.items()):
        report.append(f"| {scope} | {c['added']:,} | {c['deleted']:,} | {sum(c.values()):,} |")
    report += ["", "## Production detail", "", "| File | Attribution | Added | Deleted |", "|---|---|---:|---:|"]
    for (path, group), c in sorted(byfile.items()):
        report.append(f"| `{path.removeprefix(JAVA)}` | {group} | {c['added']} | {c['deleted']} |")
    output.with_suffix(".md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print("\n".join(report[:13]))

if __name__ == "__main__":
    main()
