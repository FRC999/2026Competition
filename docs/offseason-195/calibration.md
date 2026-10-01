# Fast camera calibration and field switching

There are two calibrations: **intrinsics** describe the lens and image, while **extrinsics** locate
the camera on the robot. The proposed workflow targets **25–30 minutes per camera after preparation**.
Preparing a flat board, surveying tags and marking robot stations is a one-time job that can take
longer. Failed checks require more time; do not lower thresholds to meet the clock.

| Recurring stage | Planning allowance |
|---|---:|
| Check locked focus; capture varied intrinsic images | 8–10 min |
| Six prepared stationary robot positions, raw capture | 8–10 min |
| Fit, held-out checks, inspect result and save | 3–5 min |
| Reload and independently check pose | 3–5 min |

## Prepare the common survey area once

1. Use two flat, rigidly fixed AprilTags with distinct known IDs. Use the tag family and printed
   black-square size configured in PhotonVision. Measure the printed size; page scaling changes
   distance estimates. Keep white borders visible. The two-tag AprilTag board is a different target
   from the ChArUco board used for intrinsic calibration.
2. Choose a fixed origin on a level floor, with +X/+Y marked perpendicular and +Z upward. Survey
   both tag **centers** in meters and their complete orientations in this frame. A vertical tag whose
   face normal points along +X has roll/pitch/yaw `[0,0,0]`; a face normal along -X has yaw 180°.
   Check this using a known camera position and the displayed axes before collecting a dataset.
3. Prefer approximately 0.6–1 m separation when the room and camera FOV permit. The tool requires
   at least 0.3 m. Measure each center; do not infer its position from an unmeasured sheet edge.
   Set heights that permit both tags to appear from every chosen station with the 12-inch cameras.
4. Mark at least six independently surveyed robot placements. Four are fitting stations and two
   are reserved validation stations. Use several distances and left/right views with at least 0.5 m
   translation span and 20° heading span among the fitting stations. The software enforces those
   minima. Keep both tags visible at each station.
5. Mark the projection of the drivetrain origin and two chassis reference points. Use a square,
   plumb line, measured jig or surveyed reference to place the origin and heading reproducibly.
   Do not use PhotonVision, fused odometry or the camera being calibrated as the survey truth.
   A Pigeon heading without an independent alignment is not survey truth either.
6. On a verified level floor, the robot-frame origin is at `z=0`, with roll/pitch zero. Otherwise
   measure these too and enter them. Height, floor slope, tag lean and chassis alignment errors
   directly bias the recovered offsets. Aim for substantially better than the desired 4 cm / 2°
   validation tolerance (for example, a few millimeters and a few tenths of a degree).

The fit uses `robotToCamera = inverse(fieldToRobotSurvey) * fieldToCameraMeasured`. It can recover
all six offsets because the robot and tags are independently surveyed. Driving around on a plane
with two **unsurveyed** tags cannot establish the same absolute accuracy. Shared survey bias can
survive all held-out checks; inspect the final physical dimensions too.

## A. Intrinsics after final focus

In PhotonVision's Cameras / Camera Calibration UI, choose the final resolution and a ChArUco target.
Print at 100%, mount flat, measure square and marker dimensions, and enter those measured values.
Keep the camera fixed and collect at least the required 12 varied views across the image, with
different distances and tilts up to 45°. Cover edges as well as the center. Solve and save; inspect
per-image errors and retake blurred or poorly detected views. Record the reported error, resolution,
camera identity and focus state. As an initial team review target, investigate an error above
0.5 pixels rather than assuming that a saved calibration is good. Low reprojection error alone
does not establish physical accuracy. Repeat after a lens/focus/resolution change.

The UI procedure and board conventions come from the
[PhotonVision calibration guide](https://docs.photonvision.org/en/v2026.3.4/docs/calibration/calibration.html).

## B. Generate and load your custom field

From repository root, install the tools in a local Python environment:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
New-Item -ItemType Directory -Force artifacts/calibration/session-01
```

Create `artifacts/calibration/session-01/survey.json` using **your measured values**. This is an
illustrative format, not a measurement of your board:

```json
{
  "length": 6.0,
  "width": 5.0,
  "tags": [
    {"id": 1, "translationMeters": [1.0, 2.1, 0.8], "rotationDegrees": [0, 0, 0]},
    {"id": 2, "translationMeters": [1.0, 2.9, 0.8], "rotationDegrees": [0, 0, 0]}
  ]
}
```

```powershell
.\.venv\Scripts\python.exe tools/vision_calibration.py layout --survey artifacts/calibration/session-01/survey.json --output artifacts/calibration/session-01/two-tag.json
```

The command writes a WPILib-format layout and prints its SHA-256. It refuses to overwrite an existing
output. Copy that JSON into `2026Competition/src/main/deploy/vision/fields/` with a distinct name.
Back up `cameras.json` outside `src/`, then edit these fields:

```json
"profile": "calibration",
"fieldLayout": "fields/two-tag.json",
"confirmedCoprocessorLayoutSha256": ""
```

Keep each camera uncalibrated until its fit passes. Upload **the same file** on both Pis: Settings →
Device Control → Import Settings → AprilTag Layout. Inspect the loaded IDs and poses on each Pi.
After checking both, place the file's lowercase SHA-256 in `confirmedCoprocessorLayoutSha256`.
PowerShell can calculate it with:

```powershell
(Get-FileHash 2026Competition/src/main/deploy/vision/fields/two-tag.json -Algorithm SHA256).Hash.ToLowerInvariant()
```

This is an **operator acknowledgment**, not automatic verification of the Pis. PhotonLib cannot
switch the Pi's layout. A wrong layout can produce a plausible but biased pose. Follow the
[MultiTag field-layout procedure](https://docs.photonvision.org/en/v2026.3.4/docs/apriltag-pipelines/multitag.html).

An authorized robot operator must deploy the configuration and restart robot code while disabled.
The configuration is read only at startup. Verify `Vision/Profile=calibration` and the robot's
`Vision/LayoutSHA256` against the file. Named competition paths and automatic competition aiming
are blocked in this profile. Manual mechanism calibration remains a separate operator activity.

## C. Capture six stationary placements per camera

Stow mechanisms, disable the robot, and place it on the surveyed station marks. Keep all wheels
stationary; wait for `Calibration/CapturePermitted=true`. Capture reads NetworkTables only and
does not reset odometry or control motors. Raw MultiTag data is available even before extrinsics
are calibrated. Each accepted capture needs at least 30 fresh, unique MultiTag frames.

Example for a surveyed station at `(3.0,2.5,0)` with heading zero (replace with measured values):

```powershell
.\.venv\Scripts\python.exe tools/vision_calibration.py capture --server 10.9.99.2 --camera back-left --layout artifacts/calibration/session-01/two-tag.json --station fit-1 --robot-xyz 3.0 2.5 0 --robot-rpy 0 0 0 --seconds 5 --output artifacts/calibration/session-01/left-fit-1.json
```

Repeat for `fit-2`, `fit-3`, `fit-4`, changing pose and filename each time. Capture the last two with
`--holdout` and station labels `check-1`, `check-2`; these must be reserved before fitting, not chosen
after seeing which samples look good. Repeat for `back-right`. Both cameras can use the same surveyed
placements if both see the board, but each needs its own recordings and fit.

If capture fails: verify disabled state, stationary wheels, the exact camera name, both tags visible,
3D/MultiTag enabled, matching field hash and live robot/Pi connection. A stale stream is not a sample.
Do not duplicate a recording under a new station name. Output files contain survey truth, identity,
timestamps, raw camera pose and holdout designation; keep them intact.

## D. Fit, validate and apply locally

PowerShell expands the capture list explicitly:

```powershell
$captures = (Get-ChildItem artifacts/calibration/session-01/left-*.json).FullName
.\.venv\Scripts\python.exe tools/vision_calibration.py fit --camera back-left --layout artifacts/calibration/session-01/two-tag.json --captures $captures --output artifacts/calibration/session-01/back-left-fit.json
```

The report includes all six offsets in meters/degrees, station residuals, per-frame 95th-percentile
errors, outlier counts, input file hashes and explicit pass/fail. It gives equal weight to each
fitting station, averages rotations as quaternions, and does not use held-out stations to optimize
the result. Gross outliers are limited to less than 20% per station. A pass requires every station
mean within **4 cm / 2°** and per-frame 95th-percentile residual within **8 cm / 4°**. These are initial
acceptance limits, not claimed robot performance or statistical confidence bounds.

Inspect X/Y/Z signs and plausible height; compare against rough physical measurements. A rear camera
normally has negative X; left has positive Y. With these mounts, pitch should be near -15° and yaw
near ±165°. A large disagreement deserves investigation, not blind application of the report.

Only after a passing report and physical sanity check:

```powershell
.\.venv\Scripts\python.exe tools/vision_calibration.py apply --config 2026Competition/src/main/deploy/vision/cameras.json --report artifacts/calibration/session-01/back-left-fit.json
```

`apply` edits that local camera entry, sets `calibrated=true`, records the report hash and creates a
timestamped backup beside the report, outside the deploy tree. It requires the calibration field
still selected. It does not deploy. Keep reports and backups in the evidence folder.
Repeat for the right camera. Archive successful evidence and the matching Pi exports in the team's
chosen storage; `artifacts/` is ignored by Git so raw data is not accidentally committed or deployed.

Reload through an authorized deployment/restart, then check additional measured locations. Rotate
the robot in place: a large position circle indicates an offset/sign problem. Compare each camera
independently; agreement between two cameras alone cannot expose a shared survey error.

## Return to the competition field

1. Ask the event organizer which construction is present: **welded or AndyMark**. Do not infer it
   from the event name. Both official WPILib 2026.2.1 layouts are included under `vision/fields/`.
2. This branch's existing target coordinates, trench regions and named competition paths are based
   on the **welded** field. Use `profile=competition-welded` and
   `fieldLayout=fields/2026-rebuilt-welded.json` for that field. Camera extrinsics stay the same.
3. For AndyMark, `competition-andymark` and its JSON support localization, but automatic aiming and
   named competition paths remain blocked until the field targets/trench/paths are reviewed and
   adapted. Merely swapping tag JSON is not a complete field-geometry migration.
4. Clear the acknowledgment, import the chosen identical JSON on **every Pi**, inspect its contents,
   and then acknowledge its exact SHA-256 in the robot config. Keep blue-origin coordinates on both
   alliances; never rotate the PhotonVision field origin for red.
5. Deploy/restart disabled. Check the profile, hash, camera identities and calibrated flags in the
   log. Physically check at least two surveyed poses, including a heading change. While disabled
   and stationary, stable fresh MultiTag should establish `Vision/LocalizationReady` automatically.
   Manual seed is an optional fresh-MultiTag override, also disabled/stationary only. Field seeding
   is independent of alliance; verify alliance separately before selecting competition paths/targets.
6. Verify the autonomous start on the field, the intended path/alliance transformation and actual
   camera visibility. Run the staged acceptance tests before using a revised auto in a match.

Do not copy `simulation/vision.json` into deploy: it contains synthetic transforms. The robot rejects
the simulation profile on real hardware. Camera pose remains startup-only; there is no enabled-time
field or calibration hot swap.
