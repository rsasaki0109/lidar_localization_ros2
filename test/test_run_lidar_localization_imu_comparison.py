#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
SCRIPT_PATH = SCRIPTS_DIR / "run_lidar_localization_imu_comparison.py"
spec = importlib.util.spec_from_file_location("run_lidar_localization_imu_comparison", SCRIPT_PATH)
comparison_tool = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(comparison_tool)


class TestRunLidarLocalizationImuComparison(unittest.TestCase):
    def test_default_modes_are_lidar_imu_and_deskew(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
        ])

        self.assertEqual(
            [mode.name for mode in comparison_tool.selected_modes(args)],
            ["lidar_only", "imu_preintegration", "deskew"],
        )

    def test_print_only_writes_configs_and_plan(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = comparison_tool.main([
                    "--bag-path",
                    "/bags/site",
                    "--map-path",
                    "/maps/site.pcd",
                    "--output-dir",
                    tmp_dir,
                    "--profile",
                    "mid360",
                    "--mode",
                    "lidar_only",
                    "--mode",
                    "deskew",
                    "--cloud-topic",
                    "/livox/points",
                    "--imu-topic",
                    "/livox/imu",
                    "--print-only",
                ])

            self.assertEqual(code, 0)
            plan = json.loads((Path(tmp_dir) / "run_plan.json").read_text(encoding="utf-8"))
            self.assertEqual([run["mode"] for run in plan["runs"]], ["lidar_only", "deskew"])
            lidar_config = Path(plan["runs"][0]["config_path"]).read_text(encoding="utf-8")
            deskew_config = Path(plan["runs"][1]["config_path"]).read_text(encoding="utf-8")
            self.assertIn("use_imu_preintegration: false", lidar_config)
            self.assertIn("use_continuous_time_deskew: false", lidar_config)
            self.assertIn("use_imu_preintegration: true", deskew_config)
            self.assertIn("use_continuous_time_deskew: true", deskew_config)
            self.assertIn("use_sim_time:=true", plan["runs"][0]["system_command"])
            self.assertIn("map_path:=/maps/site.pcd", plan["runs"][0]["system_command"])
            self.assertIn("use_imu_preintegration:=false", plan["runs"][0]["system_command"])
            self.assertIn("--require-imu-seed-source", plan["runs"][1]["imu_validation_command"])
            self.assertIn("--require-deskew-applied", plan["runs"][1]["imu_validation_command"])
            self.assertIn("benchmark_compare_runs", plan["compare_command"])
            self.assertEqual(plan["report_path"], str(Path(tmp_dir) / "comparison.md"))
            self.assertEqual(plan["command_script_path"], str(Path(tmp_dir) / "run_commands.sh"))
            command_script = (Path(tmp_dir) / "run_commands.sh").read_text(encoding="utf-8")
            self.assertIn("set -euo pipefail", command_script)
            self.assertIn("== lidar_only: benchmark ==", command_script)
            self.assertIn("--report-only", command_script)

    def test_trajectory_eval_command_is_optional(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
        ])

        self.assertIsNone(comparison_tool.trajectory_eval_command(args, Path("/tmp/run")))

        args.reference_csv = "/ref.csv"
        command = comparison_tool.trajectory_eval_command(args, Path("/tmp/run"))

        self.assertIn("benchmark_eval_trajectory", command)
        self.assertIn("--reference-csv /ref.csv", command)

    def test_input_check_reports_missing_files_before_running(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/missing/bag",
            "--map-path",
            "/missing/map.pcd",
            "--reference-csv",
            "/missing/ref.csv",
            "--output-dir",
            "/tmp/out",
        ])

        errors = comparison_tool.input_check_errors(args)

        self.assertTrue(any("Bag path does not exist" in error for error in errors))
        self.assertTrue(any("Map path does not exist" in error for error in errors))
        self.assertTrue(any("Reference CSV does not exist" in error for error in errors))

    def test_input_check_skips_bag_path_when_bag_command_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            map_path = Path(tmp_dir) / "map.pcd"
            map_path.write_text("VERSION .7\n", encoding="utf-8")
            args = comparison_tool.build_arg_parser().parse_args([
                "--bag-path",
                "/missing/bag",
                "--bag-command",
                "ros2 bag play /real/bag --clock",
                "--map-path",
                str(map_path),
                "--output-dir",
                "/tmp/out",
            ])

            errors = comparison_tool.input_check_errors(args)

        self.assertEqual(errors, [])

    def test_input_check_reads_bag_metadata_topics(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bag_dir = root / "bag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /livox/points
        type: sensor_msgs/msg/PointCloud2
    - topic_metadata:
        name: /livox/imu
        type: sensor_msgs/msg/Imu
""",
                encoding="utf-8",
            )
            map_path = root / "map.pcd"
            map_path.write_text("VERSION .7\n", encoding="utf-8")
            args = comparison_tool.build_arg_parser().parse_args([
                "--bag-path",
                str(bag_dir),
                "--map-path",
                str(map_path),
                "--output-dir",
                str(root / "out"),
                "--profile",
                "mid360",
            ])

            errors = comparison_tool.input_check_errors(args)

        self.assertEqual(errors, [])

    def test_input_check_reports_bag_topic_mismatch_with_available_topics(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bag_dir = root / "bag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /ouster/points
        type: sensor_msgs/msg/PointCloud2
    - topic_metadata:
        name: /imu/data
        type: sensor_msgs/msg/Imu
""",
                encoding="utf-8",
            )
            map_path = root / "map.pcd"
            map_path.write_text("VERSION .7\n", encoding="utf-8")
            args = comparison_tool.build_arg_parser().parse_args([
                "--bag-path",
                str(bag_dir),
                "--map-path",
                str(map_path),
                "--output-dir",
                str(root / "out"),
                "--profile",
                "mid360",
            ])

            errors = comparison_tool.input_check_errors(args)

        self.assertTrue(any("/ouster/points" in error for error in errors))
        self.assertTrue(any("--cloud-topic" in error for error in errors))
        self.assertTrue(any("/imu/data" in error for error in errors))
        self.assertTrue(any("--imu-topic" in error for error in errors))

    def test_input_check_reports_wrong_topic_type(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bag_dir = root / "bag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /livox/points
        type: sensor_msgs/msg/Image
    - topic_metadata:
        name: /livox/imu
        type: geometry_msgs/msg/TwistStamped
    - topic_metadata:
        name: /points_raw
        type: sensor_msgs/msg/PointCloud2
    - topic_metadata:
        name: /imu/data
        type: sensor_msgs/msg/Imu
""",
                encoding="utf-8",
            )
            map_path = root / "map.pcd"
            map_path.write_text("VERSION .7\n", encoding="utf-8")
            args = comparison_tool.build_arg_parser().parse_args([
                "--bag-path",
                str(bag_dir),
                "--map-path",
                str(map_path),
                "--output-dir",
                str(root / "out"),
                "--profile",
                "mid360",
            ])

            errors = comparison_tool.input_check_errors(args)

        self.assertTrue(any("has type sensor_msgs/msg/Image" in error for error in errors))
        self.assertTrue(any("expected sensor_msgs/msg/PointCloud2" in error for error in errors))
        self.assertTrue(any("/points_raw" in error for error in errors))
        self.assertTrue(any("has type geometry_msgs/msg/TwistStamped" in error for error in errors))
        self.assertTrue(any("expected sensor_msgs/msg/Imu" in error for error in errors))
        self.assertTrue(any("/imu/data" in error for error in errors))

    def test_lidar_only_mode_does_not_require_imu_topic_in_bag(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bag_dir = root / "bag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /livox/points
        type: sensor_msgs/msg/PointCloud2
""",
                encoding="utf-8",
            )
            map_path = root / "map.pcd"
            map_path.write_text("VERSION .7\n", encoding="utf-8")
            args = comparison_tool.build_arg_parser().parse_args([
                "--bag-path",
                str(bag_dir),
                "--map-path",
                str(map_path),
                "--output-dir",
                str(root / "out"),
                "--profile",
                "mid360",
                "--mode",
                "lidar_only",
            ])

            errors = comparison_tool.input_check_errors(args)

        self.assertEqual(errors, [])

    def test_main_returns_input_error_without_print_only(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = comparison_tool.main([
                    "--bag-path",
                    "/missing/bag",
                    "--map-path",
                    "/missing/map.pcd",
                    "--output-dir",
                    tmp_dir,
                    "--mode",
                    "lidar_only",
                ])

        self.assertEqual(code, 2)
        self.assertIn("input error:", stderr.getvalue())
        self.assertIn("Use --print-only", stderr.getvalue())

    def test_report_only_regenerates_markdown_without_bag_or_map_args(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir)
            run_dir = output_dir / "lidar_only"
            run_dir.mkdir()
            (output_dir / "run_plan.json").write_text(
                json.dumps(
                    {
                        "bag_path": "/bags/site",
                        "map_path": "/maps/site.pcd",
                        "profile": "mid360",
                        "output_dir": str(output_dir),
                        "runs": [{"mode": "lidar_only", "run_dir": str(run_dir)}],
                    }
                ),
                encoding="utf-8",
            )
            (output_dir / "comparison.json").write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "run_dir": str(run_dir),
                                "backend_hint": "lidar_only",
                                "trajectory_eval": {},
                                "alignment": {},
                                "resource_monitor": {},
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = comparison_tool.main([
                    "--output-dir",
                    str(output_dir),
                    "--report-only",
                ])

            report = (output_dir / "comparison.md").read_text(encoding="utf-8")

        self.assertEqual(code, 0)
        self.assertIn("Wrote", stdout.getvalue())
        self.assertIn("runtime health only", report)

    def test_imu_validation_is_skipped_for_lidar_only(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
        ])

        self.assertIsNone(
            comparison_tool.imu_validation_command(
                args,
                comparison_tool.MODES["lidar_only"],
                Path("/tmp/run"),
            )
        )

    def test_markdown_report_combines_comparison_and_imu_validation(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir)
            lidar_run_dir = output_dir / "lidar_only"
            run_dir = output_dir / "imu_preintegration"
            lidar_run_dir.mkdir()
            run_dir.mkdir()
            plan = {
                "bag_path": "/bags/site",
                "map_path": "/maps/site.pcd",
                "profile": "mid360",
                "output_dir": str(output_dir),
                "runs": [
                    {
                        "mode": "lidar_only",
                        "run_dir": str(lidar_run_dir),
                    },
                    {
                        "mode": "imu_preintegration",
                        "run_dir": str(run_dir),
                    }
                ],
            }
            (output_dir / "comparison.json").write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "run_dir": str(lidar_run_dir),
                                "backend_hint": "lidar_only",
                                "trajectory_eval": {
                                    "translation_rmse_m": 0.2,
                                    "rotation_rmse_deg": 2.0,
                                    "matched_sample_count": 40,
                                },
                                "pose_trace": {
                                    "sample_count": 41,
                                    "last_elapsed_sec": 10.0,
                                    "requested_duration_ratio": 1.0,
                                },
                                "alignment": {
                                    "row_count": 45,
                                    "non_ok_row_count": 4,
                                    "alignment_time_median_sec": 0.014,
                                    "fitness_median": 0.9,
                                },
                                "timing_sec": {
                                    "requested_bag_duration_seconds": 10.0,
                                },
                                "resource_monitor": {
                                    "cpu_percent_max": 30.0,
                                    "rss_mb_max": 200.0,
                                },
                            },
                            {
                                "run_dir": str(run_dir),
                                "backend_hint": "imu_preintegration",
                                "trajectory_eval": {
                                    "translation_rmse_m": 0.1234,
                                    "rotation_rmse_deg": 1.5,
                                    "matched_sample_count": 42,
                                },
                                "pose_trace": {
                                    "sample_count": 43,
                                    "last_elapsed_sec": 9.5,
                                    "requested_duration_ratio": 0.95,
                                },
                                "alignment": {
                                    "row_count": 50,
                                    "non_ok_row_count": 2,
                                    "alignment_time_median_sec": 0.012,
                                    "fitness_median": 0.8,
                                },
                                "timing_sec": {
                                    "requested_bag_duration_seconds": 10.0,
                                },
                                "resource_monitor": {
                                    "cpu_percent_max": 35.5,
                                    "rss_mb_max": 210.0,
                                },
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            (run_dir / "imu_validation.json").write_text(
                json.dumps(
                    {
                        "summary": {
                            "imu_active_ratio": 0.75,
                            "imu_fallback_count": 0,
                            "continuous_time_deskew_applied_ratio": 0.0,
                        },
                        "checks": [{"level": "OK", "message": "ok"}],
                    }
                ),
                encoding="utf-8",
            )

            report_path = comparison_tool.write_markdown_report(output_dir, plan)
            report = report_path.read_text(encoding="utf-8")

        self.assertIn("# LiDAR Localization IMU Comparison", report)
        self.assertIn("imu_preintegration", report)
        self.assertIn("0.123", report)
        self.assertIn("-0.077", report)
        self.assertIn("75.0%", report)
        self.assertIn("| imu_preintegration | 0.123 | -0.077 |", report)
        self.assertIn("| 42 | 43 | 9.5 | OK | 75.0%", report)
        self.assertIn("12.000", report)
        self.assertIn("Negative delta RMSE", report)
        self.assertIn("IMU preintegration runtime check: `OK`.", report)

    def test_markdown_report_warns_when_pose_coverage_is_short(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir)
            run_dir = output_dir / "lidar_only"
            run_dir.mkdir()
            plan = {
                "bag_path": "/bags/site",
                "map_path": "/maps/site.pcd",
                "profile": "standalone",
                "output_dir": str(output_dir),
                "runs": [{"mode": "lidar_only", "run_dir": str(run_dir)}],
            }
            (output_dir / "comparison.json").write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "run_dir": str(run_dir),
                                "backend_hint": "lidar_only",
                                "trajectory_eval": {
                                    "translation_rmse_m": 0.1,
                                    "rotation_rmse_deg": 1.0,
                                    "matched_sample_count": 20,
                                },
                                "pose_trace": {
                                    "sample_count": 21,
                                    "last_elapsed_sec": 22.0,
                                    "requested_duration_ratio": 0.22,
                                },
                                "timing_sec": {
                                    "requested_bag_duration_seconds": 100.0,
                                },
                                "alignment": {},
                                "resource_monitor": {},
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            report_path = comparison_tool.write_markdown_report(output_dir, plan)
            report = report_path.read_text(encoding="utf-8")

        self.assertIn("Coverage warnings", report)
        self.assertIn("published poses through 22.0s of the requested 100.0s", report)

    def test_markdown_report_warns_without_reference_eval(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir)
            run_dir = output_dir / "lidar_only"
            run_dir.mkdir()
            plan = {
                "bag_path": "/bags/site",
                "map_path": "/maps/site.pcd",
                "profile": "mid360",
                "output_dir": str(output_dir),
                "runs": [{"mode": "lidar_only", "run_dir": str(run_dir)}],
            }
            (output_dir / "comparison.json").write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "run_dir": str(run_dir),
                                "backend_hint": "lidar_only",
                                "trajectory_eval": {},
                                "alignment": {},
                                "resource_monitor": {},
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            report_path = comparison_tool.write_markdown_report(output_dir, plan)
            report = report_path.read_text(encoding="utf-8")

        self.assertIn("runtime health only", report)
        self.assertIn("cannot prove accuracy improvement", report)


if __name__ == "__main__":
    sys.exit(unittest.main())
