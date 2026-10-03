# OffSeason-195 development

Read `docs/offseason-195/SESSION_STATE.md` before work. Robot project: `2026Competition/`.
The other historical robot project is not the retrofit target.
For a new computer/chat, read `docs/offseason-195/HANDOFF.md`, `TASKS.md` and `GOTCHAS.md`
(all three under `docs/offseason-195/`). The copyable prompt is `CONTINUE_PROMPT.md` there.
Treat TASKS.md as the current open-work list; older phase-completion text is historical.

- Preserve 2026 hardware constants; the 2027 prototype uses a different chassis.
- Treat camera transforms, focus, intrinsics, field surveys and turret limits as measured data. Never invent them.
- Keep decisions and physical validation separate. Passing simulation does not establish robot accuracy.
- Use PhotonVision and Pigeon/wheel odometry; no active Limelight or Quest dependency in the retrofit.
- Read the relevant repo skill under `.agents/skills/`; Claude equivalents live under `.claude/skills/`.
- Update session state and TASKS.md before/after substantial work, append human prompts to `PROMPTS.md`, and update affected skills/docs when behavior or procedures change. Keep both skill copies synchronized.
- Documentation and AI records belong outside `2026Competition/src/` so Gradle does not package them in the robot JAR or deploy tree.
- Run relevant desktop checks. Do not deploy or operate physical hardware without a separate operator request.
- Branch requested by mentor: `OffSeason-195`. Commits and push to that branch are authorized; do not push to the Houston/Worlds branches.
