#!/usr/bin/env python3

import argparse
import importlib.util
import re
import shlex
import sys
import unittest
from pathlib import Path
from typing import Dict
from typing import Set


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

CONFIG_SCRIPT_PATH = SCRIPTS_DIR / "create_lidar_localization_config.py"
config_spec = importlib.util.spec_from_file_location(
    "create_lidar_localization_config",
    CONFIG_SCRIPT_PATH,
)
config_tool = importlib.util.module_from_spec(config_spec)
assert config_spec.loader is not None
config_spec.loader.exec_module(config_tool)

COMPARISON_SCRIPT_PATH = SCRIPTS_DIR / "run_lidar_localization_imu_comparison.py"
comparison_spec = importlib.util.spec_from_file_location(
    "run_lidar_localization_imu_comparison",
    COMPARISON_SCRIPT_PATH,
)
comparison_tool = importlib.util.module_from_spec(comparison_spec)
assert comparison_spec.loader is not None
comparison_spec.loader.exec_module(comparison_tool)


LAUNCH_ARG_RE = re.compile(r"DeclareLaunchArgument\(\s*['\"]([^'\"]+)['\"]")


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
        "use_sim_time": False,
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


def declared_launch_arguments(launch_file: str) -> Set[str]:
    text = (REPO_ROOT / "launch" / launch_file).read_text(encoding="utf-8")
    return set(LAUNCH_ARG_RE.findall(text))


def launch_file_from_command(command: str) -> str:
    for token in shlex.split(command):
        if token.endswith(".launch.py"):
            return token
    raise AssertionError(f"no launch file token found in command: {command}")


def launch_argument_keys_from_command(command: str) -> Set[str]:
    launch_file_seen = False
    keys: Set[str] = set()
    for token in shlex.split(command):
        if token.endswith(".launch.py"):
            launch_file_seen = True
            continue
        if launch_file_seen and ":=" in token:
            keys.add(token.split(":=", 1)[0])
    return keys


def assert_command_args_declared(testcase: unittest.TestCase, command: str) -> None:
    launch_file = launch_file_from_command(command)
    declared = declared_launch_arguments(launch_file)
    generated = launch_argument_keys_from_command(command)
    testcase.assertFalse(
        generated - declared,
        f"{launch_file} is missing DeclareLaunchArgument for {sorted(generated - declared)}",
    )


class TestLaunchArgumentContract(unittest.TestCase):
    def test_config_generator_commands_use_declared_launch_arguments(self):
        for profile in ("standalone", "nav2", "mid360"):
            with self.subTest(profile=profile):
                tool_args = config_args(
                    profile=profile,
                    output=f"/tmp/{profile}.yaml",
                    lidar_tf=[0.0, 0.1, 0.2, 0.0, 0.0, 0.3],
                    imu_tf=[0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
                    use_sim_time=True,
                    enable_continuous_time_deskew=(profile == "mid360"),
                )
                command = config_tool.launch_command(tool_args, Path(tool_args.output))

                assert_command_args_declared(self, command)

    def test_comparison_runner_system_commands_use_declared_launch_arguments(self):
        args = comparison_tool.build_arg_parser().parse_args([
            "--bag-path",
            "/bags/site",
            "--map-path",
            "/maps/site.pcd",
            "--output-dir",
            "/tmp/out",
            "--profile",
            "mid360",
            "--mode",
            "lidar_only",
            "--mode",
            "deskew",
        ])
        for mode in comparison_tool.selected_modes(args):
            with self.subTest(mode=mode.name):
                tool_args = comparison_tool.config_args_for_mode(
                    args,
                    mode,
                    Path(f"/tmp/{mode.name}/localization.yaml"),
                )
                command = comparison_tool.system_command(
                    tool_args,
                    Path(f"/tmp/{mode.name}/localization.yaml"),
                    extra_launch_args=[],
                )

                assert_command_args_declared(self, command)

    def test_nav2_launch_declares_generated_imu_overrides(self):
        declared = declared_launch_arguments("nav2_lidar_localization.launch.py")

        self.assertIn("use_imu_preintegration", declared)
        self.assertIn("imu_preintegration_use_base_frame_transform", declared)
        self.assertIn("use_continuous_time_deskew", declared)
        self.assertIn("continuous_time_deskew_reference_time_sec", declared)

    def test_global_recovery_launch_passes_core_localization_overrides(self):
        declared = declared_launch_arguments("global_localization_recovery.launch.py")

        for key in (
            "imu_topic",
            "global_frame_id",
            "odom_frame_id",
            "base_frame_id",
            "publish_lidar_tf",
            "lidar_frame_id",
            "publish_imu_tf",
            "imu_frame_id",
            "use_imu_preintegration",
            "imu_preintegration_use_base_frame_transform",
            "use_continuous_time_deskew",
            "continuous_time_deskew_reference_time_sec",
            "supervisor_recovery_confirmation_samples",
            "supervisor_max_seed_speed_mps",
            "supervisor_max_seed_latency_sec",
            "g2_nms_radius_m",
        ):
            with self.subTest(key=key):
                self.assertIn(key, declared)


if __name__ == "__main__":
    sys.exit(unittest.main())
