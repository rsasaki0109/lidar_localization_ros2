#!/usr/bin/env python3

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/prepare_koide_hard_relocalization_assets.sh"


class TestKoideHardRelocalizationAssetsScript(unittest.TestCase):
    def test_print_only_generates_deskew_recovery_commands_without_dataset(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            data_dir = root / "data"
            output_dir = root / "generated"

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
            stdout = result.stdout

            self.assertIn("create_lidar_localization_config.py", stdout)
            self.assertIn("--map-path", stdout)
            self.assertIn(str(data_dir / "map_outdoor_hard.ply"), stdout)
            self.assertIn("--imu-mode preintegration", stdout)
            self.assertIn("--enable-continuous-time-deskew", stdout)
            self.assertIn("--reinitialization-trigger-threshold 0.40", stdout)
            self.assertIn("--reinitialization-trigger-gap-scale-sec 10.0", stdout)
            self.assertIn("generate_occupancy_map_from_pcd.py", stdout)
            self.assertIn("--reference-csv", stdout)
            self.assertIn("global_localization_recovery.launch.py", stdout)
            self.assertIn("cloud_topic:=/livox/points", stdout)
            self.assertIn("imu_topic:=/livox/imu", stdout)
            self.assertIn("base_frame_id:=livox_frame", stdout)
            self.assertIn("publish_lidar_tf:=false", stdout)
            self.assertIn("use_imu_preintegration:=true", stdout)
            self.assertIn("use_continuous_time_deskew:=true", stdout)
            self.assertIn("g2_use_cpp_backend:=true", stdout)
            self.assertIn("g2_angular_resolution_deg:=10.0", stdout)
            self.assertIn("g2_max_scan_points:=256", stdout)
            self.assertIn("g2_nms_radius_m:=0.5", stdout)
            self.assertIn("supervisor_query_timeout_sec:=20.0", stdout)
            self.assertIn("supervisor_settle_timeout_sec:=8.0", stdout)
            self.assertIn("supervisor_recovery_confirmation_samples:=3", stdout)
            self.assertIn("supervisor_max_walk_candidates:=1", stdout)
            self.assertIn("supervisor_enable_seed_motion_compensation:=true", stdout)
            self.assertIn("supervisor_max_seed_speed_mps:=3.0", stdout)
            self.assertIn("supervisor_max_seed_latency_sec:=30.0", stdout)
            self.assertIn("ros2 bag play", stdout)

    def test_lidar_only_mode_disables_imu_in_recovery_launch(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            result = subprocess.run(
                [
                    str(SCRIPT),
                    "--data-dir",
                    str(Path(tmp_dir) / "data"),
                    "--output-dir",
                    str(Path(tmp_dir) / "generated"),
                    "--mode",
                    "lidar_only",
                    "--no-initial-pose",
                    "--print-only",
                ],
                cwd=REPO_ROOT,
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("--imu-mode off", result.stdout)
            self.assertIn("use_imu_preintegration:=false", result.stdout)
            config_line = next(
                line
                for line in result.stdout.splitlines()
                if "create_lidar_localization_config.py" in line
            )
            self.assertNotIn("--initial-pose ", config_line)

    def test_seed_motion_compensation_can_be_disabled_in_launch_command(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            result = subprocess.run(
                [
                    str(SCRIPT),
                    "--data-dir",
                    str(Path(tmp_dir) / "data"),
                    "--output-dir",
                    str(Path(tmp_dir) / "generated"),
                    "--disable-seed-motion-compensation",
                    "--supervisor-max-walk-candidates",
                    "3",
                    "--supervisor-settle-timeout-sec",
                    "12.5",
                    "--supervisor-recovery-confirmation-samples",
                    "5",
                    "--g2-nms-radius-m",
                    "0.0",
                    "--supervisor-max-seed-speed-mps",
                    "2.5",
                    "--supervisor-max-seed-latency-sec",
                    "8.0",
                    "--print-only",
                ],
                cwd=REPO_ROOT,
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(
                "supervisor_enable_seed_motion_compensation:=false",
                result.stdout,
            )
            self.assertIn("supervisor_max_walk_candidates:=3", result.stdout)
            self.assertIn("supervisor_settle_timeout_sec:=12.5", result.stdout)
            self.assertIn("supervisor_recovery_confirmation_samples:=5", result.stdout)
            self.assertIn("g2_nms_radius_m:=0.0", result.stdout)
            self.assertIn("supervisor_max_seed_speed_mps:=2.5", result.stdout)
            self.assertIn("supervisor_max_seed_latency_sec:=8.0", result.stdout)


if __name__ == "__main__":
    sys.exit(unittest.main())
