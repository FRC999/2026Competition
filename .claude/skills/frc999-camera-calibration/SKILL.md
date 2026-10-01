---
name: frc999-camera-calibration
description: Calibrate and validate FRC999 OV9782 camera intrinsics and six robot mount offsets using the surveyed two-tag workflow, or restore its competition field layout.
---

Use [calibration.md](../../../docs/offseason-195/calibration.md) as the maintained operator procedure
and `tools/vision_calibration.py` as the executable workflow. Read
[installation.md](../../../docs/offseason-195/installation.md) when focus, resolution, brackets or
camera identity changed.

- Proposed rear mounts are 12-inch lens height, roll 0°, WPILib pitch -15° (up), yaw +165°/-165°.
  They are not measurements. X/Y and final six offsets must come from a survey/fit.
- Intrinsics belong to each physical camera, focus state and resolution. Refocus first, then
  calibrate; a saved calibration from another lens or camera is not a substitute.
- Independently survey tag centers/orientations and robot origin/heading. On nonlevel ground also
  survey robot roll/pitch/height. Never use the camera/estimator being calibrated as its own truth.
  Unsurveyed planar driving with two unsurveyed tags does not establish full absolute 6DOF accuracy.
- Use >=4 fitting stations and >=2 predeclared held-out stations with pose diversity. Keep the robot
  disabled/stationary; capture only fresh unique raw MultiTag frames. Preserve failed reports too.
- Inspect fitted signs, dimensions and held-out residuals. Do not loosen thresholds merely to obtain
  PASS or claim a 30-minute guarantee. The time target excludes initial board/station preparation.
- `apply` changes a local camera config and backs it up beside the report; it does not deploy.
  Keep survey/capture/report/backup files outside `src/`. Save Pi exports with the evidence.
- Field changes require the identical JSON on every Pi and the robot, acknowledged by hash and
  verified at known poses after an authorized disabled restart. Preserve calibrated camera offsets.
  AndyMark localization is available, but welded targets/trench/paths need migration before its
  automatic competition behaviors can be enabled.

Run Python calibration tests after tool edits; use the localhost NT test to check capture API changes
without a robot. Update [session state](../../../docs/offseason-195/SESSION_STATE.md), affected docs
and both skill copies. Verify mirrors with `python tools/verify_skill_mirrors.py`.
