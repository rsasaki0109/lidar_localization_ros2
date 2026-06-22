#!/usr/bin/env python3

import argparse
import importlib.util
import shlex
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))


def load_script_module(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS_DIR / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


config_tool = load_script_module(
    "create_lidar_localization_config",
    "create_lidar_localization_config.py",
)
comparison_tool = load_script_module(
    "run_lidar_localization_imu_comparison",
    "run_lidar_localization_imu_comparison.py",
)
bringup_cli = load_script_module(
    "check_lidar_localization_bringup",
    "check_lidar_localization_bringup.py",
)
imu_validator = load_script_module(
    "validate_lidar_localization_imu",
    "validate_lidar_localization_imu.py",
)


def config_args(**overrides):
    defaults = {
        "map_path": "/maps/site.pcd",
        "output": "generated.yaml",
        "profile": "standalone",
        "cloud_topic": None,
        "imu_topic": None,
        "lidar_frame": None,
        "imu_frame": None,
        "require_cloud_time_field": False,
        "base_frame": "base_link",
        "odom_frame": "odom",
        "global_frame": "map",
        "lidar_tf": None,
        "no_publish_lidar_tf": False,
        "imu_tf": None,
        "no_publish_imu_tf": False,
        "initial_pose": None,
        "imu_mode": "profile",
        "enable_imu": False,
        "enable_continuous_time_deskew": False,
        "deskew_reference_time_sec": 0.0,
        "score_threshold": None,
        "enable_open_loop_strict_score_threshold": False,
        "open_loop_strict_min_accepted_gap_sec": 15.0,
        "open_loop_strict_min_seed_translation_m": 100.0,
        "open_loop_strict_score_threshold": 5.25,
        "reinitialization_trigger_threshold": 0.95,
        "reinitialization_trigger_gap_scale_sec": 30.0,
        "reinitialization_trigger_seed_translation_scale_m": 100.0,
        "reinitialization_trigger_reject_streak_scale": 200.0,
        "reinitialization_trigger_fitness_explosion_threshold": 1000.0,
        "voxel_leaf_size": None,
        "scan_max_range": None,
        "scan_min_range": None,
        "scan_period": 0.1,
        "ndt_resolution": 1.0,
        "ndt_threads": 4,
        "ndt_max_iterations": 35,
        "overwrite": False,
    }
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def command_args_after_executable(command: str, executable: str):
    tokens = shlex.split(command)
    index = tokens.index(executable)
    return tokens[index + 1:]


class TestCliCommandContract(unittest.TestCase):
    def test_generated_doctor_commands_parse_for_all_profiles(self):
        for profile in ("standalone", "nav2", "mid360"):
            with self.subTest(profile=profile):
                tool_args = config_args(
                    profile=profile,
                    imu_mode="preintegration",
                    enable_continuous_time_deskew=(profile == "mid360"),
                    require_cloud_time_field=(profile != "standalone"),
                )
                command = config_tool.doctor_command(tool_args)
                parser = bringup_cli.build_arg_parser()

                parsed = parser.parse_args(
                    command_args_after_executable(
                        command,
                        "check_lidar_localization_bringup.py",
                    )
                )

                self.assertEqual(parsed.profile, profile)
                self.assertTrue(parsed.require_imu)
                if profile != "standalone":
                    self.assertTrue(parsed.require_cloud_time_field)

    def test_generated_imu_validation_commands_parse_for_imu_modes(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
        ])
        for mode_name in ("imu_preintegration", "deskew"):
            mode = comparison_tool.MODES[mode_name]
            with self.subTest(mode=mode_name):
                command = comparison_tool.imu_validation_command(args, mode, Path("/tmp/run"))
                self.assertIsNotNone(command)
                parser = imu_validator.build_arg_parser()

                parsed = parser.parse_args(
                    command_args_after_executable(
                        str(command),
                        "validate_lidar_localization_imu.py",
                    )
                )

                self.assertEqual(parsed.alignment_csv, "/tmp/run/alignment_status.csv")
                self.assertEqual(parsed.output_json, "/tmp/run/imu_validation.json")
                self.assertEqual(parsed.output_md, "/tmp/run/imu_validation.md")
                self.assertEqual(parsed.require_deskew_applied, mode_name == "deskew")

    def test_comparison_runner_passes_open_loop_strict_gate_to_config(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
            "--enable-open-loop-strict-score-threshold",
            "--open-loop-strict-min-accepted-gap-sec",
            "1.0",
            "--open-loop-strict-min-seed-translation-m",
            "0.5",
            "--open-loop-strict-score-threshold",
            "2.0",
        ])

        tool_args = comparison_tool.config_args_for_mode(
            args,
            comparison_tool.MODES["deskew"],
            Path("/tmp/out/deskew/localization.yaml"),
        )
        params = config_tool.make_params(tool_args)

        self.assertTrue(params["enable_open_loop_strict_score_threshold"])
        self.assertEqual(params["open_loop_strict_min_accepted_gap_sec"], 1.0)
        self.assertEqual(params["open_loop_strict_min_seed_translation_m"], 0.5)
        self.assertEqual(params["open_loop_strict_score_threshold"], 2.0)

    def test_doctor_parser_accepts_boolean_optional_flags_used_by_docs(self):
        parser = bringup_cli.build_arg_parser()

        parsed = parser.parse_args([
            "--profile",
            "mid360",
            "--no-require-cloud-time-field",
            "--require-imu-base-tf",
            "--no-require-map-odom-tf",
        ])

        self.assertFalse(parsed.require_cloud_time_field)
        self.assertTrue(parsed.require_imu_base_tf)
        self.assertFalse(parsed.require_map_odom_tf)


if __name__ == "__main__":
    sys.exit(unittest.main())
