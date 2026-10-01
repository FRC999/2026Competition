import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from vision_calibration import pose, fit, make_layout, apply_report, write_new, read, digest


class CalibrationTest(unittest.TestCase):
    def records(self):
        rng = np.random.default_rng(999)
        camera = pose([-0.30, 0.29, 0.3048], [1.2, -15.5, 164.2])
        records = []
        for i in range(6):
            xyz, rpy = [2 + i * .3, 2 + (i % 2) * .4, 0], [0, 0, i * 8]
            robot = pose(xyz, rpy)
            frames = []
            for j in range(60):
                observed = robot @ camera
                observed[:3, 3] += rng.normal(0, .002, 3)
                q = Rotation.from_matrix(observed[:3, :3]).as_quat()
                frames.append([i * 100 + j * .02, *observed[:3, 3], *q[[3, 0, 1, 2]], 2])
            records.append(dict(camera="back-left", layoutSha256="hash", station=str(i), holdout=i >= 4,
                                robotTranslationMeters=xyz, robotRotationDegrees=rpy, frames=frames))
        return records

    def test_recovers_six_offsets_with_heldout_stations(self):
        report = fit(self.records(), "back-left", "hash")
        self.assertTrue(report["passed"])
        np.testing.assert_allclose(report["translationMeters"], [-.30, .29, .3048], atol=.001)
        np.testing.assert_allclose(report["rotationDegrees"], [1.2, -15.5, 164.2], atol=.001)

    def test_rejects_holdout_bias_even_when_training_is_consistent(self):
        records = self.records()
        for f in records[-1]["frames"]:
            f[1] += .1
        self.assertFalse(fit(records, "back-left", "hash")["passed"])

    def test_isolated_outliers_do_not_dominate(self):
        records = self.records()
        records[0]["frames"][0][1] += 1
        result = fit(records, "back-left", "hash")
        self.assertTrue(result["passed"])
        self.assertEqual(result["stations"][0]["rejectedFrames"], 1)

    def test_malformed_or_duplicate_frames_and_insufficient_geometry_rejected(self):
        for mutation in (lambda r: r[0]["frames"].append(r[0]["frames"][0]),
                         lambda r: r[0].update(layoutSha256="wrong"),
                         lambda r: r[0]["frames"][0].__setitem__(1, float("nan")),
                         lambda r: [s.update(robotRotationDegrees=[0, 0, 0]) for s in r],
                         lambda r: [s.update(holdout=False) for s in r]):
            with self.subTest(mutation=mutation):
                records = self.records()
                mutation(records)
                with self.assertRaises(ValueError):
                    fit(records, "back-left", "hash")

    def test_layout_uses_tag_centers_and_quaternion_yaw(self):
        survey = dict(length=6, width=5, tags=[dict(id=i, translationMeters=[1, i, .8],
                      rotationDegrees=[0, 0, 180]) for i in (1, 2)])
        result = make_layout(survey)
        self.assertAlmostEqual(result["tags"][0]["pose"]["rotation"]["quaternion"]["Z"], 1)
        survey["tags"][1]["id"] = 1
        with self.assertRaises(ValueError):
            make_layout(survey)

    def test_apply_requires_pass_and_matching_layout_and_preserves_other_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_new(root / "field.json", {"test": True})
            records = self.records()
            for r in records:
                r["layoutSha256"] = digest(root / "field.json")
            result = fit(records, "back-left", digest(root / "field.json"))
            write_new(root / "report.json", result)
            config = {"fieldLayout": "field.json", "cameras": [{"name": "back-left", "calibrated": False},
                       {"name": "back-right", "calibrated": False}]}
            write_new(root / "config.json", config)
            apply_report(root / "config.json", root / "report.json")
            updated = read(root / "config.json")
            self.assertTrue(updated["cameras"][0]["calibrated"])
            self.assertEqual(updated["cameras"][1], config["cameras"][1])
            result["passed"] = False
            write_new(root / "failed.json", result)
            with self.assertRaises(ValueError):
                apply_report(root / "config.json", root / "failed.json")


if __name__ == "__main__":
    unittest.main()
