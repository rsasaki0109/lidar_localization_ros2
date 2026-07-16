#!/usr/bin/env python3

import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "convert_glim_dump_to_pose_csv.py"
SPEC = importlib.util.spec_from_file_location("convert_glim", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ConvertGlimDumpTest(unittest.TestCase):
    def test_merges_submaps_and_deduplicates_boundaries(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "000000").mkdir()
            (root / "000001").mkdir()
            (root / "000000" / "imu_rate.txt").write_text(
                "1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n", encoding="utf-8")
            (root / "000001" / "imu_rate.txt").write_text(
                "2.0 1 0 0 0 0 0 1\n3.0 2 0 0 0 0 0 1\n", encoding="utf-8")
            output = root / "poses.csv"
            summary = MODULE.convert(root, output, "map")
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(summary["pose_count"], 3)
        self.assertEqual(summary["duplicate_source_row_count"], 1)
        self.assertEqual([row["stamp_sec"] for row in rows], [
            "1.000000000", "2.000000000", "3.000000000"])
        self.assertEqual(rows[0]["frame_id"], "map")

    def test_rejects_conflicting_duplicate_boundary(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "000000").mkdir()
            (root / "000001").mkdir()
            (root / "000000" / "imu_rate.txt").write_text(
                "1.0 0 0 0 0 0 0 1\n", encoding="utf-8")
            (root / "000001" / "imu_rate.txt").write_text(
                "1.0 1 0 0 0 0 0 1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "conflicting poses"):
                MODULE.convert(root, root / "poses.csv", "map")

    def test_applies_global_anchor_and_planar_constraint(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "000000").mkdir()
            (root / "000000" / "imu_rate.txt").write_text(
                "1.0 1 2 3 0 0 0 1\n2.0 2 3 4 0 0 0 1\n", encoding="utf-8")
            (root / "odom_lidar.txt").write_text(
                "1.0 1 2 3 0 0 0 1\n", encoding="utf-8")
            (root / "traj_lidar.txt").write_text(
                "1.0 11 22 33 0 0 0 1\n", encoding="utf-8")
            output = root / "poses.csv"
            summary = MODULE.convert(root, output, "map", True, True)
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertTrue(summary["global_correction_applied"])
        self.assertTrue(summary["planar_z_constraint_applied"])
        self.assertEqual(float(rows[0]["position_x"]), 11.0)
        self.assertEqual(float(rows[1]["position_x"]), 12.0)
        self.assertEqual(float(rows[0]["position_z"]), 33.0)
        self.assertEqual(float(rows[1]["position_z"]), 33.0)


if __name__ == "__main__":
    unittest.main()
