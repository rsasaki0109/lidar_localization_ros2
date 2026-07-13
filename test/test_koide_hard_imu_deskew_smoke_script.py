#!/usr/bin/env python3

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/run_koide_hard_imu_deskew_smoke.sh"


class TestKoideHardImuDeskewSmokeScript(unittest.TestCase):
    def test_print_only_generates_deskew_plan_without_dataset(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            output_dir = root / "out"
            data_dir = root / "data"

            result = subprocess.run(
                [
                    str(SCRIPT),
                    "--data-dir",
                    str(data_dir),
                    "--output-dir",
                    str(output_dir),
                    "--mode",
                    "deskew",
                    "--print-only",
                ],
                cwd=REPO_ROOT,
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            plan = json.loads((output_dir / "run_plan.json").read_text(encoding="utf-8"))
            self.assertEqual([run["mode"] for run in plan["runs"]], ["deskew"])
            self.assertEqual(
                plan["bag_path"],
                str(data_dir / "sequences/outdoor_hard_01a"),
            )
            self.assertEqual(
                plan["map_path"],
                str(data_dir / "map_outdoor_hard.ply"),
            )
            self.assertIn("--require-deskew-applied", plan["runs"][0]["imu_validation_command"])

            config = (output_dir / "deskew/localization.yaml").read_text(encoding="utf-8")
            self.assertIn("use_imu_preintegration: true", config)
            self.assertIn("use_continuous_time_deskew: true", config)
            self.assertIn("initial_pose_x: -86.03759", config)

    def test_windowed_print_only_uses_gt_pose_at_start_offset(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            output_dir = root / "out"
            data_dir = root / "data"
            gt_dir = data_dir / "gt"
            gt_dir.mkdir(parents=True)
            (gt_dir / "traj_lidar_outdoor_hard_01.txt").write_text(
                "100 1 2 3 0 0 0 1\n"
                "185 10 20 30 0 0 0.7071068 0.7071068\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    str(SCRIPT),
                    "--data-dir", str(data_dir),
                    "--output-dir", str(output_dir),
                    "--start-offset", "85",
                    "--duration", "27",
                    "--mode", "deskew",
                    "--print-only",
                ],
                cwd=REPO_ROOT,
                check=False,
                text=True,
                capture_output=True,
            )

            config = (output_dir / "deskew/localization.yaml").read_text(encoding="utf-8")
            plan = json.loads((output_dir / "run_plan.json").read_text(encoding="utf-8"))

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("initial_pose_x: 10.0", config)
        self.assertIn("initial_pose_y: 20.0", config)
        self.assertIn("--bag-start-offset 85.0", plan["runs"][0]["benchmark_command"])



if __name__ == "__main__":
    sys.exit(unittest.main())
