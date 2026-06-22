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


if __name__ == "__main__":
    sys.exit(unittest.main())
