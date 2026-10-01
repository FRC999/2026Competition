"""Check that the repo's Codex and Claude skills remain byte-identical."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
codex = {p.parent.name: p for p in (root / ".agents/skills").glob("*/SKILL.md")}
claude = {p.parent.name: p for p in (root / ".claude/skills").glob("*/SKILL.md")}
if not codex or codex.keys() != claude.keys():
    raise SystemExit("Missing skill or mismatched Codex/Claude skill names")
for name, path in codex.items():
    if path.read_bytes() != claude[name].read_bytes():
        raise SystemExit(f"Skill copies differ: {name}")
print(f"Verified {len(codex)} identical skill pairs")
