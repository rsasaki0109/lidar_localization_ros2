#!/usr/bin/env python3

import argparse
import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "create_lidar_localization_config.py"
spec = importlib.util.spec_from_file_location("create_lidar_localization_config", SCRIPT_PATH)
config_tool = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(config_tool)


def args(**overrides):
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


class TestCreateLidarLocalizationConfig(unittest.TestCase):
    def test_standalone_defaults_do_not_require_odom_tf_or_imu(self):
        tool_args = args()
        params = config_tool.make_params(tool_args)
        launch_command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertEqual(params["map_path"], "/maps/site.pcd")
        self.assertFalse(params["enable_map_odom_tf"])
        self.assertFalse(params["use_imu"])
        self.assertFalse(params["use_imu_preintegration"])
        self.assertFalse(params["use_continuous_time_deskew"])
        self.assertEqual(params["continuous_time_deskew_reference_time_sec"], 0.0)
        self.assertFalse(params["set_initial_pose"])
        self.assertEqual(params["initial_pose_qw"], 1.0)
        self.assertIn("use_imu_preintegration:=false", launch_command)
        self.assertIn("imu_preintegration_use_base_frame_transform:=false", launch_command)

    def test_relative_map_path_is_written_as_absolute_path(self):
        relative_path = "maps/site.pcd"
        params = config_tool.make_params(args(map_path=relative_path))

        self.assertEqual(params["map_path"], str(Path(relative_path).resolve()))

    def test_map_path_warnings_report_missing_or_unexpected_extension(self):
        warnings = config_tool.map_path_warnings("/tmp/not_a_pointcloud.txt")

        self.assertTrue(any("does not exist" in warning for warning in warnings))
        self.assertTrue(any("expected .pcd or .ply" in warning for warning in warnings))

    def test_nav2_profile_enables_map_odom_tf_and_launch_wrapper(self):
        tool_args = args(profile="nav2", output="/tmp/nav2.yaml")
        params = config_tool.make_params(tool_args)
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertTrue(params["enable_map_odom_tf"])
        self.assertTrue(params["use_twist_prediction"])
        self.assertIn("nav2_lidar_localization.launch.py", command)
        self.assertIn("cloud_topic:=/velodyne_points", command)
        self.assertIn("global_frame_id:=map", command)
        self.assertIn("odom_frame_id:=odom", command)
        self.assertIn("base_frame_id:=base_link", command)
        self.assertIn("use_imu_preintegration:=false", command)
        self.assertIn("imu_preintegration_use_base_frame_transform:=false", command)

    def test_mid360_profile_defaults_to_preintegration_only(self):
        tool_args = args(profile="mid360", output="/tmp/mid360.yaml")
        params = config_tool.make_params(tool_args)
        launch_command = config_tool.launch_command(tool_args, Path(tool_args.output))
        doctor_command = config_tool.doctor_command(tool_args)

        self.assertFalse(params["use_imu"])
        self.assertTrue(params["use_imu_preintegration"])
        self.assertTrue(params["imu_preintegration_use_base_frame_transform"])
        self.assertIn("map_path:=/maps/site.pcd", launch_command)
        self.assertIn("use_imu_preintegration:=true", launch_command)
        self.assertIn("imu_preintegration_use_base_frame_transform:=true", launch_command)
        self.assertIn("--require-imu", doctor_command)
        self.assertIn("--imu-frame livox_imu_frame", doctor_command)
        self.assertIn("--require-imu-base-tf", doctor_command)

    def test_mid360_preintegration_can_be_disabled_explicitly(self):
        tool_args = args(profile="mid360", imu_mode="off", output="/tmp/mid360.yaml")
        params = config_tool.make_params(tool_args)
        launch_command = config_tool.launch_command(tool_args, Path(tool_args.output))
        doctor_command = config_tool.doctor_command(tool_args)

        self.assertFalse(params["use_imu"])
        self.assertFalse(params["use_imu_preintegration"])
        self.assertFalse(params["imu_preintegration_use_base_frame_transform"])
        self.assertIn("use_imu_preintegration:=false", launch_command)
        self.assertIn("imu_preintegration_use_base_frame_transform:=false", launch_command)
        self.assertNotIn("--require-imu", doctor_command)
        self.assertNotIn("--require-imu-base-tf", doctor_command)

    def test_initial_pose_is_embedded_when_requested(self):
        params = config_tool.make_params(
            args(initial_pose=[1.0, 2.0, 0.0, 0.0, 0.0, 0.707, 0.707])
        )

        self.assertTrue(params["set_initial_pose"])
        self.assertEqual(params["initial_pose_x"], 1.0)
        self.assertEqual(params["initial_pose_qz"], 0.707)

    def test_open_loop_strict_gate_can_be_enabled(self):
        params = config_tool.make_params(
            args(
                enable_open_loop_strict_score_threshold=True,
                open_loop_strict_min_accepted_gap_sec=1.0,
                open_loop_strict_min_seed_translation_m=0.5,
                open_loop_strict_score_threshold=2.0,
            )
        )

        self.assertTrue(params["enable_open_loop_strict_score_threshold"])
        self.assertEqual(params["open_loop_strict_min_accepted_gap_sec"], 1.0)
        self.assertEqual(params["open_loop_strict_min_seed_translation_m"], 0.5)
        self.assertEqual(params["open_loop_strict_score_threshold"], 2.0)

    def test_reinitialization_request_output_is_explicit_in_generated_params(self):
        params = config_tool.make_params(args())

        self.assertTrue(params["enable_reinitialization_request_output"])
        self.assertTrue(params["enable_reinitialization_request_latch"])
        self.assertEqual(params["reinitialization_trigger_threshold"], 0.95)
        self.assertEqual(params["reinitialization_trigger_gap_scale_sec"], 30.0)

    def test_reinitialization_trigger_can_be_tuned_for_recovery_experiments(self):
        params = config_tool.make_params(
            args(
                reinitialization_trigger_threshold=0.4,
                reinitialization_trigger_gap_scale_sec=10.0,
                reinitialization_trigger_seed_translation_scale_m=50.0,
                reinitialization_trigger_reject_streak_scale=80.0,
                reinitialization_trigger_fitness_explosion_threshold=500.0,
            )
        )

        self.assertEqual(params["reinitialization_trigger_threshold"], 0.4)
        self.assertEqual(params["reinitialization_trigger_gap_scale_sec"], 10.0)
        self.assertEqual(params["reinitialization_trigger_seed_translation_scale_m"], 50.0)
        self.assertEqual(params["reinitialization_trigger_reject_streak_scale"], 80.0)
        self.assertEqual(params["reinitialization_trigger_fitness_explosion_threshold"], 500.0)

    def test_mid360_initial_pose_is_added_to_launch_command(self):
        tool_args = args(
            profile="mid360",
            initial_pose=[1.0, 2.0, 0.0, 0.0, 0.0, 0.707, 0.707],
        )
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("set_initial_pose:=true", command)
        self.assertIn("initial_pose_x:=1.0", command)
        self.assertIn("initial_pose_qz:=0.707", command)

    def test_custom_frames_are_in_params_and_launch_command(self):
        tool_args = args(
            global_frame="earth",
            odom_frame="wheel_odom",
            base_frame="robot_base",
            output="/tmp/custom_frames.yaml",
        )
        params = config_tool.make_params(tool_args)
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertEqual(params["global_frame_id"], "earth")
        self.assertEqual(params["odom_frame_id"], "wheel_odom")
        self.assertEqual(params["base_frame_id"], "robot_base")
        self.assertIn("global_frame_id:=earth", command)
        self.assertIn("odom_frame_id:=wheel_odom", command)
        self.assertIn("base_frame_id:=robot_base", command)

    def test_launch_command_can_enable_sim_time_for_bag_replay(self):
        tool_args = args(use_sim_time=True)
        params = config_tool.make_params(tool_args)
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertNotIn("use_sim_time", params)
        self.assertIn("use_sim_time:=true", command)

    def test_lidar_tf_is_added_to_launch_command_when_requested(self):
        tool_args = args(lidar_tf=[1.0, 0.2, 1.8, 0.0, 0.0, 1.57])
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("lidar_tf_x:=1.0", command)
        self.assertIn("lidar_tf_y:=0.2", command)
        self.assertIn("lidar_tf_z:=1.8", command)
        self.assertIn("lidar_tf_yaw:=1.57", command)

    def test_lidar_tf_can_be_disabled_when_robot_already_publishes_it(self):
        tool_args = args(no_publish_lidar_tf=True)
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("publish_lidar_tf:=false", command)

    def test_lidar_tf_is_disabled_when_base_and_lidar_frames_match(self):
        tool_args = args(base_frame="velodyne", lidar_frame="velodyne")
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("publish_lidar_tf:=false", command)

    def test_imu_tf_is_added_to_launch_command_when_requested(self):
        tool_args = args(
            profile="mid360",
            imu_frame="livox_imu_frame",
            imu_tf=[0.1, 0.0, 0.2, 0.0, 0.0, 0.3],
        )
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("publish_imu_tf:=true", command)
        self.assertIn("imu_frame_id:=livox_imu_frame", command)
        self.assertIn("imu_tf_x:=0.1", command)
        self.assertIn("imu_tf_z:=0.2", command)
        self.assertIn("imu_tf_yaw:=0.3", command)

    def test_imu_tf_can_be_disabled_when_robot_already_publishes_it(self):
        tool_args = args(no_publish_imu_tf=True)
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("publish_imu_tf:=false", command)

    def test_imu_tf_is_disabled_when_base_and_imu_frames_match(self):
        tool_args = args(base_frame="imu_link", imu_frame="imu_link")
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("publish_imu_tf:=false", command)

    def test_launch_command_quotes_output_path_for_shell_copy_paste(self):
        tool_args = args(output="/tmp/site config.yaml")
        command = config_tool.launch_command(tool_args, Path(tool_args.output))

        self.assertIn("'localization_param_dir:=/tmp/site config.yaml'", command)

    def test_command_line_quotes_option_values_for_shell_copy_paste(self):
        command = config_tool.command_line(["ros2", "run", "pkg", "tool", "--flag", "a b"])

        self.assertEqual(command, "ros2 run pkg tool --flag 'a b'")

    def test_doctor_command_matches_custom_topics_and_frames(self):
        tool_args = args(
            cloud_topic="/ouster/points",
            imu_topic="/ouster/imu",
            lidar_frame="os_sensor",
            global_frame="earth",
            odom_frame="wheel_odom",
            base_frame="robot_base",
        )
        command = config_tool.doctor_command(tool_args)

        self.assertIn("check_lidar_localization_bringup.py", command)
        self.assertIn("--profile standalone", command)
        self.assertIn("--cloud-topic /ouster/points", command)
        self.assertIn("--imu-topic /ouster/imu", command)
        self.assertIn("--lidar-frame os_sensor", command)
        self.assertIn("--imu-frame imu_link", command)
        self.assertIn("--global-frame earth", command)
        self.assertIn("--odom-frame wheel_odom", command)
        self.assertIn("--base-frame robot_base", command)

    def test_doctor_command_makes_nav2_tf_requirements_explicit(self):
        command = config_tool.doctor_command(args(profile="nav2"))

        self.assertIn("--require-odom-base-tf", command)
        self.assertIn("--require-map-odom-tf", command)

    def test_doctor_command_requires_imu_when_generated_config_uses_imu(self):
        command = config_tool.doctor_command(args(imu_mode="preintegration"))

        self.assertIn("--require-imu", command)

    def test_doctor_command_can_require_cloud_time_field_for_deskew_readiness(self):
        command = config_tool.doctor_command(args(require_cloud_time_field=True))

        self.assertIn("--require-cloud-time-field", command)

    def test_continuous_time_deskew_sets_params_launch_and_doctor_checks(self):
        tool_args = args(
            profile="mid360",
            enable_continuous_time_deskew=True,
            deskew_reference_time_sec=0.05,
            output="/tmp/mid360.yaml",
        )
        params = config_tool.make_params(tool_args)
        launch_command = config_tool.launch_command(tool_args, Path(tool_args.output))
        doctor_command = config_tool.doctor_command(tool_args)

        self.assertTrue(params["use_continuous_time_deskew"])
        self.assertEqual(params["continuous_time_deskew_reference_time_sec"], 0.05)
        self.assertIn("use_continuous_time_deskew:=true", launch_command)
        self.assertIn("continuous_time_deskew_reference_time_sec:=0.05", launch_command)
        self.assertIn("--require-imu", doctor_command)
        self.assertIn("--require-cloud-time-field", doctor_command)
        self.assertIn("--require-imu-base-tf", doctor_command)

    def test_continuous_time_deskew_requires_imu_preintegration(self):
        tool_args = args(enable_continuous_time_deskew=True)

        self.assertIn("requires IMU preintegration", config_tool.validate_args(tool_args))

    def test_enable_imu_compatibility_alias_enables_both_paths(self):
        params = config_tool.make_params(args(enable_imu=True))

        self.assertTrue(params["use_imu"])
        self.assertTrue(params["use_imu_preintegration"])

    def test_render_quotes_paths_and_uses_yaml_booleans(self):
        rendered = config_tool.render_ros_params(
            {"map_path": "/tmp/a map.pcd", "use_imu": False, "score_threshold": 6.0}
        )

        self.assertIn('map_path: "/tmp/a map.pcd"', rendered)
        self.assertIn("use_imu: false", rendered)
        self.assertIn("score_threshold: 6.0", rendered)

    def test_main_refuses_to_overwrite_without_flag(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output = Path(tmp_dir) / "config.yaml"
            output.write_text("existing", encoding="utf-8")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = config_tool.main(["--map-path", "/map.pcd", "--output", str(output)])

        self.assertEqual(code, 2)

    def test_main_prints_map_path_warnings(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output = Path(tmp_dir) / "config.yaml"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = config_tool.main([
                    "--map-path",
                    "/tmp/missing_map.txt",
                    "--output",
                    str(output),
                ])

        self.assertEqual(code, 0)
        self.assertIn("Warnings:", stdout.getvalue())
        self.assertIn("expected .pcd or .ply", stdout.getvalue())

    def test_main_rejects_continuous_time_deskew_without_imu(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            output = Path(tmp_dir) / "config.yaml"
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = config_tool.main([
                    "--map-path",
                    "/map.pcd",
                    "--output",
                    str(output),
                    "--enable-continuous-time-deskew",
                ])

        self.assertEqual(code, 2)
        self.assertIn("requires IMU preintegration", stderr.getvalue())


if __name__ == "__main__":
    sys.exit(unittest.main())
