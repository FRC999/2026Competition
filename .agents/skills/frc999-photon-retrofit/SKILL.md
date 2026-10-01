---
name: frc999-photon-retrofit
description: Maintain the FRC999 OffSeason-195 PhotonVision retrofit, including field configuration, source provenance and runtime logging. Use for this 2026 robot, not unrelated robots or a 2027 hardware migration.
---

Read [session state](../../../docs/offseason-195/SESSION_STATE.md) and the relevant section of the
[full audit](../../../docs/offseason-195/full-refactor-audit.md) before changing the retrofit. The robot subproject is
`2026Competition/`; the historical QuestVibeGPT project is not the target.
The [code guide](../../../docs/offseason-195/code-guide.md) maps the current runtime contracts and
logging. Update it and source/package comments when those contracts change; verify `gradlew javadoc`.

- Preserve 2026 hardware constants. Prototype source `d20594a` uses another chassis; its measured
  camera offsets/noise and focus-dependent intrinsics are not transferable.
- Keep raw PhotonVision field-to-camera, robot-to-camera and field-to-robot transforms distinct.
  Coordinates always retain the blue field origin. FPGA-to-CTRE time conversion belongs in
  DriveSubsystem exactly once. Do not reintroduce a stale frame FIFO or Limelight/Quest fallback.
- Camera JSON is startup-only: require measured extrinsics plus an operator acknowledgment of the
  identical Pi/robot field JSON before fusion. Hash acknowledgment cannot prove remote Pi settings.
  Custom calibration and simulation profiles must not silently enable competition targets/paths.
- Keep enabled heading gyro-owned and single-tag yaw untrusted unless a separately evaluated change
  establishes another policy. Disabled stationary initialization requires stable fresh unique MultiTag
  samples; one healthy camera can seed, fresh camera disagreement cannot. Preserve capture-time
  rejection of frames preceding a reset, without an arbitrary early-auto/reset waiting interval.
  Alliance/driver-forward changes must not alter field pose; aim/motion require reference plus freshness.
  Log connection, first frame, first pose and first fusion separately before attributing a startup delay.
- Keep configuration/source hashes, raw camera IO and rejection reasons available in AdvantageKit.
  Distinguish complete robot replay from vision IO logging; the former is not currently implemented.
- Put documentation, prompts, skills and evidence outside robot `src/`, because Gradle archives that
  tree. Runtime field JSON and camera/shot configuration belong in `src/main/deploy`.
- Simulation constructors and sensor writes are guarded; model inertia precedes gearing and raw
  rotor units include the reduction. Keep simulated truth independent of estimator resets, include
  follower/load effects, avoid static camera worlds, and close notifiers/cameras. See the
  [second-pass audit](../../../docs/offseason-195/second-pass-audit.md) for model limitations.
- Reproduce Worlds comparisons against mentor-confirmed Houston `6c4ecb4` with
  `tools/change_audit.py`; keep shared integration explicit and added/deleted churn separate from
  unique edited lines. Do not inflate 'just vision' by assigning all shared behavioral edits to it.

Use [installation](../../../docs/offseason-195/installation.md) for board/image/network changes and
[calibration](../../../docs/offseason-195/calibration.md) for measured transforms. Run the relevant
Java/Python checks and, for integration changes, `gradlew test robotSmoke build` from the robot project
with the WPILib 2026 Java 17 JDK. Do not claim physical accuracy from desktop tests.

Update session state and human prompt records, affected procedures and this skill when behavior
changes. Synchronize its byte-identical Claude copy under `.claude/skills/`; run
`python tools/verify_skill_mirrors.py`. Preserve the session's branch/commit authorization; code work
does not itself authorize operating or deploying to a physical robot.
