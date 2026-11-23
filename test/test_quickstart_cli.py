#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import shlex
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "quickstart", ROOT / "scripts" / "quickstart.py"
)
QUICKSTART = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = QUICKSTART
SPEC.loader.exec_module(QUICKSTART)


class TestQuickstartCli(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        config = patch.object(
            QUICKSTART,
            "default_config_path",
            return_value=Path(directory.name) / "config.yaml",
        )
        config.start()
        self.addCleanup(config.stop)

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

    def test_odometry_discovery_keeps_looking_for_a_late_odometry_node(self):
        # The bag player was discovered first; RKO-LIO's topics and TF arrive
        # 0.5 s later. A single second look would have missed them.
        player = [
            ("/livox/lidar", ["sensor_msgs/msg/PointCloud2"]),
            ("/livox/imu", ["sensor_msgs/msg/Imu"]),
        ]
        rko_lio = [("/rko_lio/odom", ["nav_msgs/msg/Odometry"]), *player]
        now = [0.0]

        def sleep(seconds):
            now[0] += seconds

        typed = QUICKSTART.wait_for_odometry_source(
            lambda: rko_lio if now[0] >= 0.5 else player,
            set(),
            "odom",
            [("/livox/lidar", "sensor_msgs/msg/PointCloud2")],
            3.0,
            lambda: now[0],
            sleep,
        )
        self.assertIn(("/rko_lio/odom", "nav_msgs/msg/Odometry"), typed)
        self.assertIn(("/livox/imu", "sensor_msgs/msg/Imu"), typed)
        self.assertAlmostEqual(now[0], 0.5)

        # An odom TF already seen ends the wait at once.
        now[0] = 0.0
        QUICKSTART.wait_for_odometry_source(
            lambda: player,
            {("odom", "livox_frame")},
            "odom",
            [],
            3.0,
            lambda: now[0],
            sleep,
        )
        self.assertEqual(now[0], 0.0)

        # Without odometry it gives up after the timeout.
        typed = QUICKSTART.wait_for_odometry_source(
            lambda: player, set(), "odom", [], 3.0, lambda: now[0], sleep
        )
        self.assertGreaterEqual(now[0], 3.0)
        self.assertEqual(len(typed), 2)

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
                QUICKSTART.discover_ros_graph = lambda *_, topics=topics, **__: (
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

    def test_topic_hints_show_candidates_without_guessing(self):
        topics = [
            ("/cloud_b", "sensor_msgs/msg/PointCloud2"),
            ("/cloud_a", "sensor_msgs/msg/PointCloud2"),
            ("/imu_b", "sensor_msgs/msg/Imu"),
            ("/imu_a", "sensor_msgs/msg/Imu"),
        ]
        original = QUICKSTART.discover_ros_graph
        QUICKSTART.discover_ros_graph = lambda *_, **__: (topics, set(), None)
        try:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                _, text = self._dry_run(
                    root, ["--discover-topics", "--no-auto-occupancy-map"]
                )
                self.assertIn("/cloud_a, /cloud_b", text)
                self.assertIn("--cloud-topic TOPIC", text)
                self.assertIn("/imu_a, /imu_b", text)
                self.assertIn("--imu-topic TOPIC", text)
                self.assertIn("cloud_topic:=/velodyne_points", text)
                self.assertNotIn("cloud_topic:=/cloud_a", text)

                _, text = self._dry_run(
                    root,
                    [
                        "--discover-topics",
                        "--no-auto-occupancy-map",
                        "--cloud-topic",
                        "/cloud_b",
                        "--imu-topic",
                        "/imu_b",
                    ],
                )
                self.assertNotIn("Input hint:", text)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_explicit_odometry_settings_skip_the_odometry_discovery_wait(self):
        waits = []

        def discover(*_, odometry_discovery_sec, **__):
            waits.append(odometry_discovery_sec)
            return [], set(), None

        with patch.object(QUICKSTART, "discover_ros_graph", discover):
            for extra in (
                [],
                ["--odom-tf-prediction"],
                ["--odom-tf-prediction", "--base-frame", "livox_frame"],
                ["--no-odom-tf-prediction", "--base-frame", "base_link"],
            ):
                with tempfile.TemporaryDirectory() as directory:
                    self._dry_run(
                        Path(directory),
                        ["--discover-topics", "--no-auto-occupancy-map", *extra],
                    )
        self.assertEqual(
            waits,
            [
                QUICKSTART.ODOMETRY_DISCOVERY_SEC,
                QUICKSTART.ODOMETRY_DISCOVERY_SEC,
                0.0,
                0.0,
            ],
        )

    def test_missing_topic_hint_explains_how_to_supply_input(self):
        original = QUICKSTART.discover_ros_graph
        QUICKSTART.discover_ros_graph = lambda *_, **__: ([], set(), None)
        try:
            with tempfile.TemporaryDirectory() as directory:
                _, text = self._dry_run(
                    Path(directory),
                    ["--discover-topics", "--no-auto-occupancy-map"],
                )
            self.assertIn("No PointCloud2 topic detected", text)
            self.assertIn("sensor driver or bag", text)
            self.assertIn("--cloud-topic TOPIC", text)
            self.assertIn("If using IMU", text)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_explicit_cloud_topic_hint_preserves_selection(self):
        original = QUICKSTART.discover_ros_graph
        try:
            for advertised, expected in (
                ([], "not detected"),
                ([("/chosen", "sensor_msgs/msg/LaserScan")], "LaserScan"),
            ):
                with self.subTest(advertised=advertised):
                    topics = [*advertised, ("/available", QUICKSTART.CLOUD_TYPE)]
                    QUICKSTART.discover_ros_graph = lambda *_, topics=topics, **__: (
                        topics,
                        set(),
                        None,
                    )
                    with tempfile.TemporaryDirectory() as directory:
                        _, text = self._dry_run(
                            Path(directory),
                            ["--discover-topics", "--cloud-topic", "/chosen"],
                        )
                    self.assertIn(expected, text)
                    self.assertIn("ros2 topic info -v /chosen", text)
                    self.assertIn("--cloud-topic /available", text)
                    self.assertIn("cloud_topic:=/chosen", text)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_explicit_relative_topic_has_no_false_warning(self):
        self.assertIsNone(
            QUICKSTART.topic_discovery_hint(
                [("/chosen", QUICKSTART.CLOUD_TYPE)],
                QUICKSTART.CLOUD_TYPE,
                "chosen",
                "explicit",
                "--cloud-topic",
            )
        )

    def test_unwritable_configuration_reports_error_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--output",
                        str(root),
                        "--no-discover-topics",
                        "--no-auto-occupancy-map",
                        "--dry-run",
                    ]
                )
            self.assertEqual(result, 2)
            self.assertIn("could not write configuration", stderr.getvalue())
            self.assertIn("--output", stderr.getvalue())

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
                QUICKSTART.discover_ros_graph = lambda *_, edges=edges, **__: (
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

    def test_odometry_without_its_tf_gets_a_hint(self):
        topics = [("/rko_lio/odom", "nav_msgs/msg/Odometry")]
        original = QUICKSTART.discover_ros_graph
        try:
            for extra, expected in (([], True), (["--odom-tf-prediction"], False)):
                QUICKSTART.discover_ros_graph = lambda *_, **__: (topics, set(), None)
                with tempfile.TemporaryDirectory() as directory:
                    _, text = self._dry_run(
                        Path(directory),
                        ["--discover-topics", "--no-auto-occupancy-map", *extra],
                    )
                self.assertEqual(
                    "Odometry is published on /rko_lio/odom" in text, expected, extra
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

                def discover(
                    odom_frame, cloud_topic_for, edges=edges, frame=frame, **_
                ):
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

    def test_tf_hints_match_discovery_and_publication(self):
        original = QUICKSTART.discover_ros_graph
        chain = {("base_link", "mount"), ("mount", "sensor")}
        try:
            for edges, extra, expected in (
                (set(), [], "identity transform"),
                (set(), ["--no-publish-lidar-tf"], "Publish the calibrated transform"),
                (chain, [], None),
                (chain, ["--publish-lidar-tf"], "already connects"),
                (set(), ["--no-discover-topics", "--no-publish-lidar-tf"], None),
                (set(), ["--lidar-frame", "base_link"], None),
            ):
                with self.subTest(extra=extra, edges=edges):
                    QUICKSTART.discover_ros_graph = lambda *_, edges=edges, **__: (
                        [],
                        edges,
                        None,
                    )
                    with tempfile.TemporaryDirectory() as directory:
                        _, text = self._dry_run(
                            Path(directory),
                            [
                                "--discover-topics",
                                "--base-frame",
                                "base_link",
                                "--lidar-frame",
                                "sensor",
                                *extra,
                            ],
                        )
                    if expected is None:
                        self.assertNotIn("TF hint:", text)
                    else:
                        self.assertIn(expected, text)
                        self.assertIn(
                            "ros2 run tf2_ros tf2_echo base_link sensor", text
                        )
                        self.assertIn("publish_lidar_tf:=", text)
        finally:
            QUICKSTART.discover_ros_graph = original

    def test_thinner_generated_grid_is_cached_apart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / "cache"
            original = QUICKSTART.default_occupancy_dir
            QUICKSTART.default_occupancy_dir = lambda: cache
            try:
                map_path, text = self._dry_run(
                    root, ["--occupancy-inflate-radius-m", "0.2"]
                )
                default = QUICKSTART.cached_occupancy_map(map_path, cache)
                thin = QUICKSTART.cached_occupancy_map(map_path, cache, 0.2)
                self.assertNotEqual(default, thin)
                self.assertIn(f"would generate {thin}", text)
                self.assertEqual(
                    QUICKSTART.cached_occupancy_map(map_path, cache, 0.6), default
                )
            finally:
                QUICKSTART.default_occupancy_dir = original

    def test_global_search_height_band_is_passed_to_the_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, text = self._dry_run(root, ["--no-auto-occupancy-map"])
            self.assertIn("g2_z_min_m:=0.5", text)
            self.assertIn("g2_z_max_m:=5.0", text)
            _, text = self._dry_run(
                root,
                [
                    "--no-auto-occupancy-map",
                    "--global-z-min",
                    "0",
                    "--global-z-max",
                    "4",
                ],
            )
            self.assertIn("g2_z_min_m:=0.0", text)
            self.assertIn("g2_z_max_m:=4.0", text)
            self.assertIn("g2_sparse_band_min_points:=512", text)
            _, text = self._dry_run(
                root,
                ["--no-auto-occupancy-map", "--global-sparse-band-min-points", "0"],
            )
            self.assertIn("g2_sparse_band_min_points:=0", text)
            for extra in (
                ["--global-z-min", "2", "--global-z-max", "1"],
                ["--global-z-min", "nan"],
                ["--occupancy-inflate-radius-m", "-0.1"],
            ):
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(
                        QUICKSTART.main(
                            ["--map", str(root / "site.pcd"), "--dry-run", *extra]
                        ),
                        2,
                    )

    def test_global_search_engine_is_passed_only_when_not_the_default(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, text = self._dry_run(root, ["--no-auto-occupancy-map"])
            self.assertNotIn("g2_global_search_engine", text)
            self.assertNotIn("g2_bbs3d_", text)
            _, text = self._dry_run(
                root,
                [
                    "--no-auto-occupancy-map",
                    "--global-search-engine",
                    "bbs3d",
                    "--bbs3d-timeout-sec",
                    "5",
                    "--bbs3d-threads",
                    "2",
                ],
            )
            self.assertIn("g2_global_search_engine:=bbs3d", text)
            self.assertIn("g2_bbs3d_timeout_sec:=5.0", text)
            self.assertIn("g2_bbs3d_num_threads:=2", text)
            unknown = ["--global-search-engine", "bbs4d"]
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(
                SystemExit
            ):
                QUICKSTART.main(
                    ["--map", str(root / "site.pcd"), "--dry-run", *unknown]
                )
            for extra in (
                ["--global-search-engine", "bbs3d", "--bbs3d-timeout-sec", "0"],
                ["--global-search-engine", "bbs3d", "--bbs3d-threads", "0"],
            ):
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(
                        QUICKSTART.main(
                            ["--map", str(root / "site.pcd"), "--dry-run", *extra]
                        ),
                        2,
                    )

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
            self.assertEqual(calls[0][calls[0].index("--inflate-radius-m") + 1], "0.6")
            yaml_path.unlink()
            QUICKSTART.generate_occupancy_map(
                Path("m.pcd"), yaml_path, ok, inflate_radius_m=0.2
            )
            self.assertEqual(calls[1][calls[1].index("--inflate-radius-m") + 1], "0.2")

            def fails(command, **kwargs):
                return QUICKSTART.subprocess.CompletedProcess(
                    command, 1, "", "cannot read point cloud: m.pcd\n"
                )

            yaml_path.unlink()
            self.assertEqual(
                QUICKSTART.generate_occupancy_map(Path("m.pcd"), yaml_path, fails),
                "cannot read point cloud: m.pcd",
            )

    def test_global_candidates_are_seeded_at_the_ground_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for extra, height, seed_z in (
                ([], "1.0", "0.0"),
                (["--sensor-height", "1.3"], "1.3", "0.0"),
                (["--global-seed-z", "-11.4"], "-1.0", "-11.4"),
                (["--global-seed-z", "0"], "-1.0", "0.0"),
            ):
                _, text = self._dry_run(root, ["--no-auto-occupancy-map", *extra])
                self.assertIn(f"g2_registration_sensor_height_m:={height}", text, extra)
                self.assertIn(f"g2_registration_seed_z_m:={seed_z}", text, extra)
            for extra in (
                ["--sensor-height", "1.3", "--global-seed-z", "-11.4"],
                ["--sensor-height", "1.3", "--global-seed-z", "0"],
                ["--sensor-height", "-0.5"],
            ):
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(
                        QUICKSTART.main(
                            ["--map", str(root / "site.pcd"), "--dry-run", *extra]
                        ),
                        2,
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

    def test_odometry_confirmation_mode_reaches_the_supervisor(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            map_path.write_bytes(b"pcd")
            for extra, expected in (
                ([], "window_waiver"),
                (
                    ["--supervisor-odometry-confirmation-mode", "segment_defer"],
                    "segment_defer",
                ),
            ):
                args = QUICKSTART.build_arg_parser().parse_args(
                    ["--map", str(map_path), "--no-discover-topics", *extra]
                )
                command = shlex.join(
                    QUICKSTART.launch_parts(
                        args,
                        QUICKSTART._config_args(args, "/cloud", "/imu"),
                        root / "config.yaml",
                        root / "pose.json",
                    )
                )
                self.assertIn(
                    f"supervisor_odometry_confirmation_mode:={expected}", command
                )

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
            # With odometry prediction the window guard is on by default.
            self.assertIn("enable_seed_correction_window_guard: true", generated)

    def test_seed_correction_window_reaches_generated_params(self):
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
                        "--seed-correction-window-sec=2.0",
                        "--dry-run",
                    ]
                )
            self.assertEqual(result, 0)
            generated = output.read_text(encoding="utf-8")
            self.assertIn("enable_seed_correction_window_guard: true", generated)
            self.assertIn("seed_correction_window_guard_sec: 2.0", generated)
            self.assertIn(
                "seed_correction_guard_release_agreement_scans: 10", generated
            )

    def _settings_run(self, *extra):
        """Dry-run quickstart; return (exit code, stdout, config text, launch command)."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            output = root / "generated.yaml"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--output",
                        str(output),
                        "--no-discover-topics",
                        "--no-auto-occupancy-map",
                        "--dry-run",
                        *extra,
                    ]
                )
            text = stdout.getvalue()
            launch = text.split("Launch:\n", 1)[1].splitlines()[0]
            return result, text, output.read_text(encoding="utf-8"), launch

    def assert_odometry_settings(self, config, launch, window, nms, near_alias):
        if window > 0.0:
            self.assertIn("enable_seed_correction_window_guard: true", config)
            self.assertIn(f"seed_correction_window_guard_sec: {window}", config)
            self.assertIn("seed_correction_guard_release_agreement_scans: 10", config)
        else:
            self.assertIn("enable_seed_correction_window_guard: false", config)
            self.assertIn("seed_correction_guard_release_agreement_scans: 0", config)
        self.assertIn(f"g2_nms_radius_m:={nms}", launch)
        self.assertIn(f"global_consensus_near_alias_samples:={near_alias}", launch)

    def test_defaults_follow_odometry_prediction_under_every_profile(self):
        # The same under every profile: on with odometry prediction, else the
        # values of v1.12.0.
        for profile in ("standalone", "nav2", "mid360"):
            for odometry, expected, source in (
                (
                    "--odom-tf-prediction",
                    (2.0, 1.5, 3),
                    "default with odometry prediction",
                ),
                ("--no-odom-tf-prediction", (0.0, 3.0, 0), "default"),
            ):
                with self.subTest(profile=profile, odometry=odometry):
                    result, text, config, launch = self._settings_run(
                        "--profile", profile, odometry
                    )
                    self.assertEqual(result, 0)
                    self.assert_odometry_settings(config, launch, *expected)
                    window, nms, near_alias = expected
                    settings = (
                        f"seed_correction_window_sec={window:g} ({source}), "
                        f"global_nms_radius_m={nms:g} ({source}), "
                        f"global_consensus_near_alias_samples={near_alias} ({source})"
                    )
                    self.assertIn(f"Settings:      {settings}", text)
                    self.assertIn(
                        f"# Generated by quickstart.py --profile {profile}.", config
                    )
                    state = "on" if odometry == "--odom-tf-prediction" else "off"
                    self.assertIn(
                        f"# Settings that default by odometry prediction ({state})",
                        config,
                    )
                    self.assertIn(
                        f"#   global_nms_radius_m: {nms:g} ({source})", config
                    )
                    # bbs3d and the trusted start stay opt-in everywhere.
                    self.assertNotIn("g2_global_search_engine", launch)
                    self.assertIn("global_pose_position_std_m:=0.0", launch)
                    self.assertIn(
                        "seed_correction_guard_trusted_start_position_std_m: 0.0",
                        config,
                    )

    def test_without_odometry_the_defaults_are_the_old_ones(self):
        # The default profile, no odometry TF given or discovered.
        result, text, config, launch = self._settings_run()
        self.assertEqual(result, 0)
        self.assertIn("use_odom_tf_prediction: false", config)
        self.assert_odometry_settings(config, launch, 0.0, 3.0, 0)
        self.assertIn(
            "Settings:      seed_correction_window_sec=0 (default), "
            "global_nms_radius_m=3 (default), "
            "global_consensus_near_alias_samples=0 (default)",
            text,
        )

    def test_explicit_flags_win_over_the_odometry_defaults(self):
        old = (
            "--seed-correction-window-sec",
            "0",
            "--global-nms-radius-m",
            "3",
            "--global-consensus-near-alias-samples",
            "0",
        )
        for profile in ("standalone", "mid360"):
            with self.subTest(profile=profile):
                result, text, config, launch = self._settings_run(
                    "--profile", profile, "--odom-tf-prediction", *old
                )
                self.assertEqual(result, 0)
                self.assert_odometry_settings(config, launch, 0.0, 3.0, 0)
                self.assertIn(
                    "Settings:      seed_correction_window_sec=0 (command line), "
                    "global_nms_radius_m=3 (command line), "
                    "global_consensus_near_alias_samples=0 (command line)",
                    text,
                )
                self.assertIn(
                    "#   seed_correction_window_sec: 0 (command line)", config
                )

        # One at a time: the others keep their odometry default.
        result, text, config, launch = self._settings_run(
            "--odom-tf-prediction", "--global-nms-radius-m=2.5"
        )
        self.assertEqual(result, 0)
        self.assert_odometry_settings(config, launch, 2.0, 2.5, 3)
        self.assertIn("global_nms_radius_m=2.5 (command line)", text)
        self.assertIn(
            "seed_correction_window_sec=2 (default with odometry prediction)", text
        )

        # And without odometry, explicit values are taken as before.
        result, text, config, launch = self._settings_run(
            "--no-odom-tf-prediction", "--global-nms-radius-m=1.5"
        )
        self.assertEqual(result, 0)
        self.assert_odometry_settings(config, launch, 0.0, 1.5, 0)
        self.assertIn("global_nms_radius_m=1.5 (command line)", text)

    def test_the_odometry_window_brings_the_release_agreement(self):
        result, _, config, _ = self._settings_run("--odom-tf-prediction")
        self.assertEqual(result, 0)
        self.assertIn("seed_correction_guard_release_agreement_scans: 10", config)
        # Turning the window off turns the agreement off with it.
        result, _, config, _ = self._settings_run(
            "--odom-tf-prediction", "--seed-correction-window-sec=0"
        )
        self.assertEqual(result, 0)
        self.assertIn("enable_seed_correction_window_guard: false", config)
        self.assertIn("seed_correction_guard_release_agreement_scans: 0", config)

    def test_explicit_odometry_settings_without_odometry_are_kept(self):
        # An explicit value is kept; the localizer then ignores the window (it is
        # gated by odometry prediction) and the near-alias rule acts as off.
        result, text, config, launch = self._settings_run(
            "--no-odom-tf-prediction",
            "--seed-correction-window-sec=2.0",
            "--global-consensus-near-alias-samples=3",
        )
        self.assertEqual(result, 0)
        self.assertIn("enable_seed_correction_window_guard: false", config)
        self.assertIn("global_consensus_near_alias_samples:=3", launch)
        self.assertIn("seed_correction_window_sec=2 (command line)", text)
        self.assertIn("global_nms_radius_m=3 (default)", text)

    def test_defaults_follow_discovered_odometry(self):
        # With discovery, odometry prediction is known only after the graph is
        # read; the defaults follow what it found, under any profile.
        for profile in ("standalone", "mid360"):
            for odom_live, expected in (
                (True, (2.0, 1.5, 3)),
                (False, (0.0, 3.0, 0)),
            ):
                with self.subTest(profile=profile, odom_live=odom_live):
                    with patch.object(
                        QUICKSTART,
                        "discover_ros_graph",
                        return_value=([], set(), None),
                    ), patch.object(
                        QUICKSTART.model,
                        "select_odometry_frame",
                        return_value=("base_link", odom_live),
                    ):
                        result, _, config, launch = self._settings_run(
                            "--profile", profile, "--discover-topics"
                        )
                    self.assertEqual(result, 0)
                    self.assert_odometry_settings(config, launch, *expected)

    def test_odometry_settings_resolve_without_main(self):
        # Callers that parse and build the launch command directly get the
        # defaults that follow --odom-tf-prediction too.
        for odometry, nms, near_alias in (
            ("--odom-tf-prediction", 1.5, 3),
            ("--no-odom-tf-prediction", 3.0, 0),
        ):
            with self.subTest(
                odometry=odometry
            ), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                map_path = root / "site.pcd"
                map_path.write_bytes(b"pcd")
                args = QUICKSTART.build_arg_parser().parse_args(
                    ["--map", str(map_path), "--no-discover-topics", odometry]
                )
                args.base_frame = "base_link"
                command = shlex.join(
                    QUICKSTART.launch_parts(
                        args,
                        QUICKSTART._config_args(args, "/livox/points", "/livox/imu"),
                        root / "config.yaml",
                        root / "pose.json",
                    )
                )
                self.assertIn(f"g2_nms_radius_m:={nms}", command)
                self.assertIn(
                    f"global_consensus_near_alias_samples:={near_alias}", command
                )

    def test_trusted_global_start_reaches_localizer_and_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            output = root / "generated.yaml"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--output",
                        str(output),
                        "--no-discover-topics",
                        "--odom-tf-prediction",
                        "--trusted-global-start-std-m=0.1",
                        "--dry-run",
                    ]
                )
            self.assertEqual(result, 0)
            generated = output.read_text(encoding="utf-8")
            self.assertIn(
                "seed_correction_guard_trusted_start_position_std_m: 0.1", generated
            )
            self.assertIn("global_pose_position_std_m:=0.1", stdout.getvalue())

    def test_trusted_global_start_needs_odometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_path = root / "site.pcd"
            output = root / "generated.yaml"
            map_path.write_bytes(b"pcd")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = QUICKSTART.main(
                    [
                        "--map",
                        str(map_path),
                        "--output",
                        str(output),
                        "--no-discover-topics",
                        "--trusted-global-start-std-m=0.1",
                        "--dry-run",
                    ]
                )
            self.assertEqual(result, 0)
            generated = output.read_text(encoding="utf-8")
            self.assertIn(
                "seed_correction_guard_trusted_start_position_std_m: 0.0", generated
            )
            self.assertIn("global_pose_position_std_m:=0.0", stdout.getvalue())

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
        self.assertIn("--map /absolute/path/to/map.pcd", stderr.getvalue())

    def test_map_directory_explains_expected_file(self):
        with tempfile.TemporaryDirectory() as directory:
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                result = QUICKSTART.main(["--map", directory, "--dry-run"])
            self.assertEqual(result, 2)
            self.assertIn("directory", stderr.getvalue())
            self.assertIn(".pcd or .ply", stderr.getvalue())

    def test_map_directory_suggests_the_map_inside(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "map.pcd").write_bytes(b"pcd")
            (Path(directory) / "notes.txt").write_text("x")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                result = QUICKSTART.main(["--map", directory, "--dry-run"])
            self.assertEqual(result, 2)
            self.assertIn(
                "Did you mean --map " + str(Path(directory) / "map.pcd") + "?",
                stderr.getvalue(),
            )
            self.assertNotIn("notes.txt", stderr.getvalue())

    def test_missing_map_suggests_a_neighbour(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "site.ply").write_bytes(b"ply")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                result = QUICKSTART.main(
                    ["--map", str(Path(directory) / "site.pcd"), "--dry-run"]
                )
            self.assertEqual(result, 2)
            self.assertIn("Map file does not exist", stderr.getvalue())
            self.assertIn("site.ply", stderr.getvalue())

    def test_wrong_map_extension_and_empty_map_are_named(self):
        with tempfile.TemporaryDirectory() as directory:
            wrong = Path(directory) / "map.bag"
            wrong.write_bytes(b"x")
            empty = Path(directory) / "map.pcd"
            empty.write_bytes(b"")
            for path, expected in ((wrong, "not .bag"), (empty, "Map file is empty")):
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr):
                    result = QUICKSTART.main(["--map", str(path), "--dry-run"])
                self.assertEqual(result, 2)
                self.assertIn(expected, stderr.getvalue())

    def test_status_line_is_on_by_default_and_can_be_turned_off(self):
        with tempfile.TemporaryDirectory() as directory:
            map_path = Path(directory) / "site.pcd"
            map_path.write_bytes(b"pcd")
            for extra, expected in (
                ([], "show_status:=true"),
                (["--no-status"], "show_status:=false"),
            ):
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout):
                    result = QUICKSTART.main(
                        [
                            "--map",
                            str(map_path),
                            "--no-discover-topics",
                            "--dry-run",
                            *extra,
                        ]
                    )
                self.assertEqual(result, 0)
                self.assertIn(expected, stdout.getvalue())
            self.assertIn(
                "'manual_pose_step:=set 2D Pose Estimate in RViz'", stdout.getvalue()
            )


if __name__ == "__main__":
    unittest.main()
