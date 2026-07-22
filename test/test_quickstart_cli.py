#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import shlex
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "quickstart", ROOT / "scripts" / "quickstart.py")
QUICKSTART = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = QUICKSTART
SPEC.loader.exec_module(QUICKSTART)


class TestQuickstartCli(unittest.TestCase):
    def test_typed_topic_parser_and_safe_discovery(self):
        parsed = QUICKSTART.parse_typed_topics(
            "/cloud [sensor_msgs/msg/PointCloud2]\n/imu [sensor_msgs/msg/Imu]\n")
        self.assertEqual(parsed, [
            ("/cloud", "sensor_msgs/msg/PointCloud2"),
            ("/imu", "sensor_msgs/msg/Imu"),
        ])

    def test_dry_run_generates_one_command_global_workflow(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            occupancy = root / "site.yaml"
            output = root / "generated.yaml"
            state = root / "pose.json"
            map_path.write_bytes(b"pcd")
            occupancy.write_text("image: site.pgm\n", encoding="utf-8")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main([
                    "--map", str(map_path),
                    "--occupancy-map", str(occupancy),
                    "--output", str(output),
                    "--state-file", str(state),
                    "--no-discover-topics",
                    "--dry-run",
                ])
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertTrue(output.exists())
            self.assertIn("quickstart.launch.py", text)
            self.assertIn("restore_saved_pose:=true", text)
            self.assertIn("enable_global_initialization:=true", text)
            self.assertIn("g2_use_cpp_backend:=true", text)
            self.assertIn("g2_enable_registration_scoring:=true", text)
            self.assertIn("require_global_registration_scoring:=true", text)
            self.assertIn("guarded global search", text)

    def test_explicit_pose_disables_automatic_publishers(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.ply"
            occupancy = root / "site.yaml"
            map_path.write_bytes(b"ply")
            occupancy.write_text("image: site.pgm\n", encoding="utf-8")
            args = QUICKSTART.build_arg_parser().parse_args([
                "--map", str(map_path),
                "--occupancy-map", str(occupancy),
                "--initial-pose", "1", "2", "0", "0", "0", "0", "1",
                "--no-discover-topics",
            ])
            config_args = QUICKSTART._config_args(args, "/cloud", "/imu")
            parts = QUICKSTART.launch_parts(
                args, config_args, root / "config.yaml", root / "pose.json")
            command = shlex.join(parts)
            self.assertIn("restore_saved_pose:=false", command)
            self.assertIn("enable_global_initialization:=false", command)
            self.assertIn("initial_pose_preconfigured:=true", command)

    def test_mid360_records_the_remapped_pose_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            args = QUICKSTART.build_arg_parser().parse_args([
                "--map", str(map_path), "--profile", "mid360",
                "--no-discover-topics",
            ])
            config_args = QUICKSTART._config_args(args, "/livox/points", "/livox/imu")
            command = shlex.join(QUICKSTART.launch_parts(
                args, config_args, root / "config.yaml", root / "pose.json"))
            self.assertIn("pose_topic:=/localization/pose_with_covariance", command)
            self.assertNotIn("occupancy_yaml:=", command)

    def test_missing_map_is_actionable_error(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            result = QUICKSTART.main([
                "--map", "/definitely/missing/map.pcd",
                "--no-discover-topics",
                "--dry-run",
            ])
        self.assertEqual(result, 2)
        self.assertIn("Map file does not exist", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
