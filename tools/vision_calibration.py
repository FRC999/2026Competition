#!/usr/bin/env python3
"""Surveyed-pose camera extrinsics. SI units, WPILib NWU, quaternion order w,x,y,z.

No robot writes. Capture reads NT; apply edits a local config only. See calibration.md.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def pose(xyz, rpy):
    values = np.asarray([*xyz, *rpy], dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Pose requires six finite xyz/roll-pitch-yaw values")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    result[:3, 3] = xyz
    return result


def frame_pose(frame):
    a = np.asarray(frame, dtype=float)
    if a.shape != (9,) or not np.isfinite(a).all() or a[8] < 2:
        raise ValueError("Expected finite MultiTag [time,x,y,z,qw,qx,qy,qz,count]")
    if abs(np.linalg.norm(a[4:8]) - 1) > 0.01:
        raise ValueError("Invalid camera quaternion")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(a[[5, 6, 7, 4]]).as_matrix()
    result[:3, 3] = a[1:4]
    return result


def average(transforms):
    result = np.eye(4)
    result[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    result[:3, :3] = Rotation.from_matrix(np.array([t[:3, :3] for t in transforms])).mean().as_matrix()
    return result


def residual(reference, transforms):
    return np.array([[np.linalg.norm(t[:3, 3] - reference[:3, 3]),
                      np.degrees(Rotation.from_matrix(reference[:3, :3].T @ t[:3, :3]).magnitude())]
                     for t in transforms])


def make_layout(survey):
    length, width = float(survey["length"]), float(survey["width"])
    if not np.isfinite([length, width]).all() or min(length, width) <= 0:
        raise ValueError("Survey needs positive field length and width")
    if len(survey["tags"]) != 2:
        raise ValueError("This workflow requires exactly two surveyed tags")
    tags, ids = [], set()
    for tag in survey["tags"]:
        tag_id = tag["id"]
        if not isinstance(tag_id, int) or tag_id < 0 or tag_id in ids:
            raise ValueError("Tag IDs must be distinct nonnegative integers")
        ids.add(tag_id)
        t = pose(tag["translationMeters"], tag["rotationDegrees"])
        if not (0 <= t[0, 3] <= length and 0 <= t[1, 3] <= width and t[2, 3] > 0):
            raise ValueError("Tag center outside survey or below floor")
        q = Rotation.from_matrix(t[:3, :3]).as_quat()
        tags.append({"ID": tag_id, "pose": {"translation": dict(zip("xyz", t[:3, 3])),
                    "rotation": {"quaternion": dict(zip("WXYZ", q[[3, 0, 1, 2]]))}}})
    if np.linalg.norm(np.array(survey["tags"][0]["translationMeters"]) -
                      np.array(survey["tags"][1]["translationMeters"])) < 0.3:
        raise ValueError("Separate tag centers by at least 0.3 m")
    return {"tags": tags, "field": {"length": length, "width": width}}


def fit(records, camera, layout_hash):
    stations, train, hold = [], [], []
    station_names = set()
    for record in records:
        if record["camera"] != camera or record["layoutSha256"] != layout_hash:
            raise ValueError("Mixed camera or field-layout identity")
        station = record["station"]
        if station in station_names:
            raise ValueError("One capture per distinct surveyed station; do not reuse station labels")
        station_names.add(station)
        robot = pose(record["robotTranslationMeters"], record["robotRotationDegrees"])
        frames = record["frames"]
        if len(frames) < 30 or len({f[0] for f in frames}) != len(frames):
            raise ValueError("Each station needs at least 30 unique frames")
        transforms = [np.linalg.inv(robot) @ frame_pose(f) for f in frames]
        # Fixed broad outlier cut: never tighten the cut until a biased capture appears precise.
        center = average(transforms)
        center[:3, 3] = np.median([t[:3, 3] for t in transforms], axis=0)
        errors = residual(center, transforms)
        kept = [t for t, e in zip(transforms, errors) if e[0] <= 0.15 and e[1] <= 10]
        if len(kept) < 30 or len(kept) < 0.8 * len(transforms):
            raise ValueError("Too many outliers; fix focus, survey or tag visibility and recapture")
        entry = {"name": station, "holdout": bool(record["holdout"]), "robot": robot,
                 "center": average(kept), "transforms": kept, "rejected": len(frames) - len(kept)}
        stations.append(entry)
        (hold if entry["holdout"] else train).append(entry)
    if len(train) < 4 or len(hold) < 2:
        raise ValueError("Need >=4 fitting stations and >=2 predeclared held-out stations")
    span = max(np.linalg.norm(a["robot"][:2, 3] - b["robot"][:2, 3]) for a in train for b in train)
    angle_span = max(residual(a["robot"], [b["robot"]])[0, 1] for a in train for b in train)
    if span < 0.5 or angle_span < 20:
        raise ValueError("Fitting stations need >=0.5 m position span and >=20 degrees heading span")
    # Equal weight per station, so longer recordings cannot dominate the survey.
    estimate = average([s["center"] for s in train])
    checks = []
    for s in stations:
        e = residual(estimate, [s["center"]])[0]
        p95 = np.percentile(residual(estimate, s["transforms"]), 95, axis=0)
        checks.append({"station": s["name"], "holdout": s["holdout"], "translationErrorMeters": float(e[0]),
                       "rotationErrorDegrees": float(e[1]), "frameP95Meters": float(p95[0]),
                       "frameP95Degrees": float(p95[1]), "rejectedFrames": s["rejected"],
                       "pass": bool(e[0] <= 0.04 and e[1] <= 2 and p95[0] <= 0.08 and p95[1] <= 4)})
    return {"schemaVersion": 1, "camera": camera, "layoutSha256": layout_hash,
            "createdUtc": datetime.now(timezone.utc).isoformat(),
            "translationMeters": estimate[:3, 3].tolist(),
            "rotationDegrees": Rotation.from_matrix(estimate[:3, :3]).as_euler("xyz", degrees=True).tolist(),
            "passed": all(c["pass"] for c in checks), "stations": checks,
            "limits": {"stationMeters": 0.04, "stationDegrees": 2, "frameP95Meters": 0.08, "frameP95Degrees": 4},
            "caveat": "Conditional on independently surveyed robot and tag poses; shared survey bias is not observable."}


def capture(args):
    import ntcore  # Optional for offline fitting; pip install pyntcore==2026.2.1
    if not np.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        raise ValueError("Capture duration must be 1..30 seconds")
    pose(args.robot_xyz, args.robot_rpy)
    nt = ntcore.NetworkTableInstance.create()
    nt.startClient4("FRC999-extrinsic-capture")
    nt.setServer(args.server, args.port)
    table = nt.getTable("SmartDashboard")
    permitted = table.getBooleanTopic("Calibration/CapturePermitted").subscribe(False)
    robot_time = table.getDoubleTopic("Calibration/RobotTimestamp").subscribe(0)
    raw = table.getDoubleArrayTopic(f"Calibration/{args.camera}/FieldToCamera").subscribe([])
    layout = table.getStringTopic("Vision/LayoutSHA256").subscribe("")
    expected = digest(args.layout)
    frames, timestamps = [], set()
    try:
        deadline = time.monotonic() + 5
        while not nt.isConnected() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not nt.isConnected():
            raise ValueError("No robot NT connection")
        # Allow initial subscriptions to arrive. All subsequent state must remain live.
        time.sleep(0.3)
        end = time.monotonic() + args.seconds
        last_robot_time, last_change = None, time.monotonic()
        while time.monotonic() < end:
            if not nt.isConnected() or not permitted.get():
                raise ValueError("Capture requires connected, disabled, stationary robot")
            if layout.get() != expected:
                raise ValueError("Robot layout hash differs from capture layout")
            if robot_time.get() != last_robot_time:
                last_robot_time, last_change = robot_time.get(), time.monotonic()
            if time.monotonic() - last_change > 0.5:
                raise ValueError("Robot telemetry is stale")
            data = list(raw.get())
            if data:
                frame_pose(data)
                age = robot_time.get() - data[0]
                if 0 <= age <= 0.5 and data[0] not in timestamps:
                    timestamps.add(data[0])
                    frames.append(data)
            time.sleep(0.01)
        if len(frames) < 30:
            raise ValueError("Fewer than 30 fresh MultiTag frames; no capture saved")
        write_new(args.output, {"schemaVersion": 1, "camera": args.camera, "station": args.station,
                               "holdout": args.holdout, "layoutSha256": expected,
                               "robotTranslationMeters": args.robot_xyz, "robotRotationDegrees": args.robot_rpy,
                               "createdUtc": datetime.now(timezone.utc).isoformat(), "frames": frames})
    finally:
        nt.stopClient()
        ntcore.NetworkTableInstance.destroy(nt)


def apply_report(config_path, report_path):
    config, report = read(config_path), read(report_path)
    if report.get("passed") is not True or not report.get("stations"):
        raise ValueError("Only a passing fit can be applied")
    pose(report["translationMeters"], report["rotationDegrees"])
    base = Path(config_path).resolve().parent
    layout = (base / config["fieldLayout"]).resolve()
    if not layout.is_relative_to(base) or digest(layout) != report["layoutSha256"]:
        raise ValueError("Apply to the calibration layout used for the fit, then switch field profile")
    camera = next((c for c in config["cameras"] if c["name"] == report["camera"]), None)
    if camera is None:
        raise ValueError("Camera not found in config")
    camera.update({k: report[k] for k in ("translationMeters", "rotationDegrees")})
    camera.update(calibrated=True, calibrationReportSha256=digest(report_path), mountStatus="SURVEY FIT; see calibration report")
    backup = Path(report_path).resolve().parent / ("cameras-before-" + report["camera"] + "-"
             + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    backup.write_bytes(Path(config_path).read_bytes())
    Path(config_path).write_text(json.dumps(config, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("layout")
    p.add_argument("--survey", required=True)
    p.add_argument("--output", required=True)
    p = sub.add_parser("capture")
    for key in ("server", "camera", "layout", "station", "output"):
        p.add_argument("--" + key, required=True)
    p.add_argument("--robot-xyz", type=float, nargs=3, required=True)
    p.add_argument("--robot-rpy", type=float, nargs=3, required=True)
    p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--port", type=int, default=5810)
    p.add_argument("--holdout", action="store_true")
    p = sub.add_parser("fit")
    p.add_argument("--captures", nargs="+", required=True)
    for key in ("camera", "layout", "output"):
        p.add_argument("--" + key, required=True)
    p = sub.add_parser("apply")
    p.add_argument("--config", required=True)
    p.add_argument("--report", required=True)
    args = parser.parse_args()
    try:
        if args.command == "layout":
            write_new(args.output, make_layout(read(args.survey)))
            print("Layout SHA256:", digest(args.output))
        elif args.command == "capture":
            capture(args)
        elif args.command == "fit":
            report = fit([read(p) for p in args.captures], args.camera, digest(args.layout))
            report["captureFiles"] = [{"path": str(p), "sha256": digest(p)} for p in args.captures]
            write_new(args.output, report)
            print("PASS" if report["passed"] else "FAIL: inspect station residuals; do not apply")
            if not report["passed"]:
                return 2
        else:
            apply_report(args.config, args.report)
        return 0
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, f"Calibration rejected: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
