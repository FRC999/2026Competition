"""Exercise the real Python NT API against localhost only; no robot connection."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import threading
import time
import unittest
from vision_calibration import capture, digest, read, write_new


@unittest.skipUnless(importlib.util.find_spec("ntcore"), "Optional pyntcore not installed")
class CaptureLoopbackTest(unittest.TestCase):
    def test_captures_fresh_frames_and_rejects_enabled_robot(self):
        import ntcore
        server = ntcore.NetworkTableInstance.create()
        server.startServer("", "127.0.0.1", 0, 15810)
        table = server.getTable("SmartDashboard")
        permit = table.getBooleanTopic("Calibration/CapturePermitted").publish()
        clock = table.getDoubleTopic("Calibration/RobotTimestamp").publish()
        raw = table.getDoubleArrayTopic("Calibration/back-left/FieldToCamera").publish()
        layout = table.getStringTopic("Vision/LayoutSHA256").publish()
        stop = threading.Event()
        permitted = [True]
        def publish():
            while not stop.is_set():
                now = time.monotonic()
                permit.set(permitted[0]); clock.set(now)
                raw.set([now - .03, 1, 2, .3, 1, 0, 0, 0, 2])
                server.flush()
                stop.wait(.02)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write_new(root / "field.json", {"syntheticLoopback": True})
            layout.set(digest(root / "field.json"))
            thread = threading.Thread(target=publish)
            thread.start()
            try:
                args = argparse.Namespace(server="127.0.0.1", port=15810, camera="back-left", station="loopback",
                    robot_xyz=[0, 0, 0], robot_rpy=[0, 0, 0], holdout=False, seconds=4,
                    layout=root / "field.json", output=root / "capture.json")
                capture(args)
                data = read(args.output)
                self.assertGreaterEqual(len(data["frames"]), 30)
                self.assertEqual(len(data["frames"]), len({f[0] for f in data["frames"]}))
                permitted[0] = False
                args.output = root / "must-not-exist.json"
                with self.assertRaisesRegex(ValueError, "disabled, stationary"):
                    capture(args)
                self.assertFalse(args.output.exists())
            finally:
                stop.set(); thread.join(2)
                for pub in (permit, clock, raw, layout):
                    pub.close()
                server.stopServer()
                ntcore.NetworkTableInstance.destroy(server)


if __name__ == "__main__":
    unittest.main()
