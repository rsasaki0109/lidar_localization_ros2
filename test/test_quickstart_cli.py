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
    "quickstart", ROOT / "scripts" / "quickstart.py"
)
QUICKSTART = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = QUICKSTART
SPEC.loader.exec_module(QUICKSTART)


class TestQuickstartCli(unittest.TestCase):
    def test_topic_discovery_stops_once_cloud_and_imu_appear(self):
        graph = [
            [("/rosout", ["rcl_interfaces/msg/Log"])],
            [("/cloud", ["sensor_msgs/msg/PointCloud2"])],
            [
                ("/cloud", ["sensor_msgs/msg/PointCloud2"]),
                ("/imu", ["sensor_msgs/msg/Imu"]),
            ],
        ]
        now = [0.0]

        def sleep(seconds):
            now[0] += seconds

        typed = QUICKSTART.wait_for_typed_topics(
            lambda: graph[min(round(now[0] * 10), 2)], 3.0, lambda: now[0], sleep
        )
        self.assertEqual(
            typed,
            [
                ("/cloud", "sensor_msgs/msg/PointCloud2"),
                ("/imu", "sensor_msgs/msg/Imu"),
            ],
        )
        self.assertAlmostEqual(now[0], 0.2)

        # Without an IMU it returns what it saw at the timeout.
        now[0] = 0.0
        typed = QUICKSTART.wait_for_typed_topics(
            lambda: graph[1], 3.0, lambda: now[0], sleep
        )
        self.assertEqual(typed, [("/cloud", "sensor_msgs/msg/PointCloud2")])
        self.assertGreaterEqual(now[0], 3.0)

    def test_help_lists_tuning_options_only_with_help_all(self):
        core = QUICKSTART.build_arg_parser().format_help()
        full = io.StringIO()
        with contextlib.redirect_stdout(full):
            self.assertEqual(QUICKSTART.main(["--help-all"]), 0)
        for flag in ("--map", "--occupancy-map", "--odom-tf-prediction", "--rviz"):
            self.assertIn(flag, core)
        for flag in (
            "--min-score-margin",
            "--global-max-candidates",
            "--route-max-poses",
        ):
            self.assertNotIn(flag, core)
            self.assertIn(flag, full.getvalue())
        self.assertIn("--help-all", core)

        # Hidden options still parse.
        args = QUICKSTART.build_arg_parser().parse_args(
            ["--map", "m.pcd", "--global-max-candidates", "16", "--no-g3-recovery"]
        )
        self.assertEqual(args.global_max_candidates, 16)
        self.assertFalse(args.g3_recovery)

    def _dry_run(self, root, extra):
        map_path = root / "site.pcd"
        map_path.write_bytes(b"pcd")
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            result = QUICKSTART.main(
                [
                    "--map",
                    str(map_path),
                    "--output",
                    str(root / "generated.yaml"),
                    "--state-file",
                    str(root / "pose.json"),
                    "--no-discover-topics",
                    "--dry-run",
                    *extra,
                ]
            )
        self.assertEqual(result, 0)
        return map_path, stdout.getvalue()

    def test_occupancy_grid_is_generated_for_a_map_without_one(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / "cache"
            original = QUICKSTART.default_occupancy_dir
            QUICKSTART.default_occupancy_dir = lambda: cache
            try:
                map_path, text = self._dry_run(root, [])
                cached = QUICKSTART.cached_occupancy_map(map_path, cache)
                self.assertIn(f"would generate {cached}", text)
                self.assertNotIn("occupancy_yaml:=", text)

                # A grid cached for the same map contents is used.
                cached.parent.mkdir(parents=True)
                cached.write_text("image: x.pgm\n", encoding="utf-8")
                _, text = self._dry_run(root, [])
                self.assertIn(f"occupancy_yaml:={cached}", text)
                self.assertIn("guarded global search", text)

                _, text = self._dry_run(root, ["--no-auto-occupancy-map"])
                self.assertNotIn("occupancy_yaml:=", text)
                _, text = self._dry_run(
                    root, ["--initial-pose", "0", "0", "0", "0", "0", "0", "1"]
                )
                self.assertNotIn("occupancy_yaml:=", text)
            finally:
                QUICKSTART.default_occupancy_dir = original

    def test_sim_time_follows_a_published_clock(self):
        replay = [
            ("/clock", "rosgraph_msgs/msg/Clock"),
            ("/livox/lidar", "sensor_msgs/msg/PointCloud2"),
        ]
        original = QUICKSTART.discover_ros_graph
        try:
            for topics, extra, expected, note in (
                (replay, [], "true", "clock=sim (/clock published)"),
                (replay[1:], [], "false", "clock=wall (no /clock)"),
                (replay, ["--no-use-sim-time"], "false", None),
                ([], ["--use-sim-time"], "true", None),
            ):
                QUICKSTART.discover_ros_graph = lambda *_, topics=topics: (
                    topics,
                    set(),
                    None,
                )
                with tempfile.TemporaryDirectory() as directory:
                    _, text = self._dry_run(
                        Path(directory),
                        ["--discover-topics", "--no-auto-occupancy-map", *extra],
                    )
                self.assertIn(f"use_sim_time:={expected}", text, extra)
                if note:
                    self.assertIn(note, text)
                else:
                    self.assertNotIn("clock=", text)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_odometry_tf_turns_on_prediction_and_sets_the_base_frame(self):
        topics = [("/tf", "tf2_msgs/msg/TFMessage")]
        original = QUICKSTART.discover_ros_graph
        try:
            for edges, extra, base, prediction in (
                ({("odom", "livox_frame")}, [], "livox_frame", "true"),
                (set(), [], "base_link", "false"),
                (
                    {("odom", "livox_frame")},
                    ["--no-odom-tf-prediction"],
                    "livox_frame",
                    "false",
                ),
                (set(), ["--odom-tf-prediction"], "base_link", "true"),
            ):
                QUICKSTART.discover_ros_graph = lambda *_, edges=edges: (
                    topics,
                    edges,
                    None,
                )
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    _, text = self._dry_run(
                        root, ["--discover-topics", "--no-auto-occupancy-map", *extra]
                    )
                    params = (root / "generated.yaml").read_text(encoding="utf-8")
                self.assertIn(f"base_frame_id:={base}", text, extra)
                self.assertIn(f"use_odom_tf_prediction: {prediction}", params, extra)
                self.assertEqual(
                    "--require-odom-base-tf" in text, prediction == "true", extra
                )
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_lidar_frame_comes_from_the_cloud_header(self):
        topics = [
            ("/points", "sensor_msgs/msg/PointCloud2"),
            ("/tf", "tf2_msgs/msg/TFMessage"),
        ]
        robot = {("odom", "base_link"), ("base_link", "velodyne")}
        original = QUICKSTART.discover_ros_graph
        try:
            for edges, frame, extra, lidar, publish in (
                ({("odom", "livox_frame")}, "livox_frame", [], "livox_frame", "false"),
                (robot, "velodyne", [], "velodyne", "false"),
                ({("odom", "base_link")}, "os_sensor", [], "os_sensor", "true"),
                (robot, "velodyne", ["--publish-lidar-tf"], "velodyne", "true"),
                (robot, "velodyne", ["--lidar-frame", "lidar"], "lidar", "true"),
            ):

                def discover(odom_frame, cloud_topic_for, edges=edges, frame=frame):
                    if cloud_topic_for is None:
                        return topics, edges, None
                    self.assertEqual(cloud_topic_for(topics), "/points")
                    return topics, edges, frame

                QUICKSTART.discover_ros_graph = discover
                with tempfile.TemporaryDirectory() as directory:
                    _, text = self._dry_run(
                        Path(directory),
                        ["--discover-topics", "--no-auto-occupancy-map", *extra],
                    )
                self.assertIn(f"lidar_frame_id:={lidar}", text, extra)
                self.assertIn(f"publish_lidar_tf:={publish}", text, extra)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_generate_occupancy_map_reports_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = Path(directory) / "grids" / "abc.yaml"
            calls = []

            def ok(command, **kwargs):
                calls.append(command)
                yaml_path.write_text("image: abc.pgm\n", encoding="utf-8")
                return QUICKSTART.subprocess.CompletedProcess(command, 0, "", "")

            self.assertIsNone(
                QUICKSTART.generate_occupancy_map(Path("m.pcd"), yaml_path, ok)
            )
            self.assertIn("--max-obstacle-height-m", calls[0])
            self.assertEqual(calls[0][calls[0].index("--map-name") + 1], "abc")

            def fails(command, **kwargs):
                return QUICKSTART.subprocess.CompletedProcess(
                    command, 1, "", "cannot read point cloud: m.pcd\n"
                )

            yaml_path.unlink()
            self.assertEqual(
                QUICKSTART.generate_occupancy_map(Path("m.pcd"), yaml_path, fails),
                "cannot read point cloud: m.pcd",
            )

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
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--occupancy-map",
                        str(occupancy),
                        "--output",
                        str(output),
                        "--state-file",
                        str(state),
                        "--no-discover-topics",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertTrue(output.exists())
            self.assertIn("quickstart.launch.py", text)
            self.assertIn("restore_saved_pose:=true", text)
            self.assertIn("enable_global_initialization:=true", text)
            self.assertIn("g2_use_cpp_backend:=true", text)
            self.assertIn("g2_angular_resolution_deg:=5.0", text)
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
            args = QUICKSTART.build_arg_parser().parse_args(
                [
                    "--map",
                    str(map_path),
                    "--occupancy-map",
                    str(occupancy),
                    "--initial-pose",
                    "1",
                    "2",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "--no-discover-topics",
                ]
            )
            config_args = QUICKSTART._config_args(args, "/cloud", "/imu")
            parts = QUICKSTART.launch_parts(
                args, config_args, root / "config.yaml", root / "pose.json"
            )
            command = shlex.join(parts)
            self.assertIn("restore_saved_pose:=false", command)
            self.assertIn("enable_global_initialization:=false", command)
            self.assertIn("initial_pose_preconfigured:=true", command)

    def test_mid360_records_the_remapped_pose_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            args = QUICKSTART.build_arg_parser().parse_args(
                [
                    "--map",
                    str(map_path),
                    "--profile",
                    "mid360",
                    "--no-discover-topics",
                ]
            )
            config_args = QUICKSTART._config_args(args, "/livox/points", "/livox/imu")
            command = shlex.join(
                QUICKSTART.launch_parts(
                    args, config_args, root / "config.yaml", root / "pose.json"
                )
            )
            self.assertIn("pose_topic:=/localization/pose_with_covariance", command)
            self.assertNotIn("occupancy_yaml:=", command)

    def test_route_crop_reference_csv_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            reference = root / "reference.csv"
            map_path.write_bytes(b"pcd")
            reference.write_text(
                "stamp_sec,position_x,position_y,position_z,"
                "orientation_x,orientation_y,orientation_z,orientation_w\n"
                "1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0\n",
                encoding="utf-8",
            )
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--reference-csv",
                        str(reference),
                        "--no-discover-topics",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertIn("g2_candidate_source:=route_crop", text)
            self.assertIn("reference_csv:=", text)
            self.assertIn("enable_g3_recovery:=true", text)
            self.assertIn("guarded route-crop search", text)
            self.assertIn("enable_global_initialization:=true", text)

    def test_next_step_points_to_rviz_without_global_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--no-discover-topics",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertIn("Next: set 2D Pose Estimate in RViz", text)

    def test_no_rviz_points_to_initialpose_instead_of_rviz(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--no-discover-topics",
                        "--no-rviz",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertIn("Next: publish /initialpose (RViz is off)", text)
            self.assertNotIn("RViz ", text.split("Launch:")[0])

    def test_odom_tf_prediction_is_written_to_generated_params(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            output = root / "generated.yaml"
            map_path.write_bytes(b"pcd")
            with contextlib.redirect_stdout(io.StringIO()):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--output",
                        str(output),
                        "--no-discover-topics",
                        "--odom-tf-prediction",
                        "--dry-run",
                    ]
                )
            self.assertEqual(result, 0)
            generated = output.read_text(encoding="utf-8")
            self.assertIn("use_odom_tf_prediction: true", generated)
            self.assertIn("enable_map_odom_tf: true", generated)

    def test_next_step_mentions_stationary_search_with_route_crop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            reference = root / "reference.csv"
            map_path.write_bytes(b"pcd")
            reference.write_text(
                "stamp_sec,position_x,position_y,position_z,"
                "orientation_x,orientation_y,orientation_z,orientation_w\n"
                "1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0\n",
                encoding="utf-8",
            )
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--reference-csv",
                        str(reference),
                        "--no-discover-topics",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertIn("Next: keep robot stationary", text)

    def test_next_step_verifies_pose_with_explicit_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--initial-pose",
                        "1",
                        "2",
                        "0",
                        "0",
                        "0",
                        "0",
                        "1",
                        "--no-discover-topics",
                        "--dry-run",
                    ]
                )
            text = stdout.getvalue()
            self.assertEqual(result, 0)
            self.assertIn("Next: wait 5s", text)

    def test_missing_map_is_actionable_error(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            result = QUICKSTART.main(
                [
                    "--map",
                    "/definitely/missing/map.pcd",
                    "--no-discover-topics",
                    "--dry-run",
                ]
            )
        self.assertEqual(result, 2)
        self.assertIn("Map file does not exist", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
