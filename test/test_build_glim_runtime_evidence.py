#!/usr/bin/env python3

import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "glim_runtime", ROOT / "scripts" / "build_glim_runtime_evidence.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class GlimRuntimeEvidenceTest(unittest.TestCase):
    def write_poses(self, path, reset=False):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "position_x", "position_y", "position_z",
                "orientation_x", "orientation_y", "orientation_z", "orientation_w"])
            writer.writerow([10, 0, 0, 0, 0, 0, 1])
            writer.writerow([0 if reset else 10.1, 0, 0, 0, 0, 0, 1])

    def test_realtime_bounded_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log = root / "glim.log"
            poses = root / "poses.csv"
            lines = []
            for index in range(100):
                stamp = index * 0.1
                lines.append(
                    f"LIDARLOC_PREPROCESS_RUNTIME stamp={stamp:.6f} processing_sec=0.02 workload=1")
                lines.append(
                    f"LIDARLOC_ODOMETRY_RUNTIME stamp={stamp:.6f} processing_sec=0.05 queue_after=0")
            lines.append("playback speed: 1.000x")
            log.write_text("\n".join(lines), encoding="utf-8")
            self.write_poses(poses)
            result = MODULE.build_evidence(log, poses)
        self.assertAlmostEqual(result["processing_p95_sec"], 0.07)
        self.assertAlmostEqual(result["scan_period_sec"], 0.1)
        self.assertFalse(result["queue_growth_unbounded"])
        self.assertEqual(result["tf_jump_count"], 0)

    def test_nonempty_final_queue_and_reset_fail_safety(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log = root / "glim.log"
            poses = root / "poses.csv"
            log.write_text(
                "LIDARLOC_PREPROCESS_RUNTIME stamp=0 processing_sec=0.02 workload=3\n"
                "LIDARLOC_ODOMETRY_RUNTIME stamp=0 processing_sec=0.05 queue_after=3\n",
                encoding="utf-8")
            self.write_poses(poses, reset=True)
            result = MODULE.build_evidence(log, poses)
        self.assertTrue(result["queue_growth_unbounded"])
        self.assertEqual(result["tf_jump_count"], 1)
        self.assertEqual(result["unauthorized_reset_count"], 1)

    def test_transient_tail_backlog_that_drains_is_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log = root / "glim.log"
            poses = root / "poses.csv"
            lines = []
            for index in range(40):
                queue = 20 if index >= 30 and index < 39 else 0
                lines.append(
                    f"LIDARLOC_PREPROCESS_RUNTIME stamp={index * 0.1:.6f} "
                    f"processing_sec=0.02 workload={queue}")
                lines.append(
                    f"LIDARLOC_ODOMETRY_RUNTIME stamp={index * 0.1:.6f} "
                    f"processing_sec=0.05 queue_after={queue}")
            log.write_text("\n".join(lines), encoding="utf-8")
            self.write_poses(poses)
            result = MODULE.build_evidence(log, poses)
        self.assertGreater(result["evidence"]["tail_queue_p95"], 0)
        self.assertEqual(result["evidence"]["final_queue_depth"], 0)
        self.assertFalse(result["queue_growth_unbounded"])


if __name__ == "__main__":
    unittest.main()
