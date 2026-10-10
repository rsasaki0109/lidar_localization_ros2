#!/usr/bin/env python3
"""Generate a safe configuration and start lidar localization in one command."""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import re
import shlex
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def _load_sibling(name: str):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


config_tool = _load_sibling("create_lidar_localization_config")
model = _load_sibling("quickstart_model")


def default_config_path() -> Path:
    return Path.home() / ".config" / "lidar_localization_ros2" / "quickstart.yaml"


def default_state_path(map_path: Path) -> Path:
    safe_stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_path.stem) or "map"
    return (
        Path.home()
        / ".local"
        / "state"
        / "lidar_localization_ros2"
        / f"{safe_stem}.json"
    )


def default_occupancy_dir() -> Path:
    return Path.home() / ".cache" / "lidar_localization_ros2" / "occupancy"


# Generated grids ignore points this high above the ground (a ceiling, tree
# canopy); obstacles still need points from 0.4 m up. Checked indoors (Go2
# aisle), outdoors (Koide campus) and on a driving loop.
AUTO_OCCUPANCY_MAX_OBSTACLE_HEIGHT_M = 2.0

# Global candidates are scored at the map's ground under them plus this height
# unless --global-seed-z fixes one map-frame z. Measured sensor heights were
# 0.51 m (Go2), 1.33 m (handheld) and 1.66 m (car); 1.0 m initialized all three.
DEFAULT_SENSOR_HEIGHT_M = 1.0


def cached_occupancy_map(map_path: Path, cache_dir: Path) -> Path:
    """Occupancy YAML for these map contents (renaming the map keeps it)."""
    identity = model.compute_map_identity(map_path)
    return cache_dir / f"{identity.sha256[:16]}.yaml"


def generate_occupancy_map(
    map_path: Path, yaml_path: Path, run=subprocess.run
) -> str | None:
    """Write yaml_path from the point cloud map; return an error, or None."""
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "generate_occupancy_map_from_pcd",
        "--pcd",
        str(map_path),
        "--output-dir",
        str(yaml_path.parent),
        "--map-name",
        yaml_path.stem,
        "--max-obstacle-height-m",
        str(AUTO_OCCUPANCY_MAX_OBSTACLE_HEIGHT_M),
    ]
    try:
        result = run(command, check=False, capture_output=True, text=True, timeout=600)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    if result.returncode != 0 or not yaml_path.is_file():
        lines = (result.stderr or result.stdout or "").strip().splitlines()
        return lines[-1] if lines else f"exit code {result.returncode}"
    return None


CLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"


def wait_for_typed_topics(
    list_topics, timeout_sec: float, clock=time.monotonic, sleep=time.sleep
):
    """Poll the ROS graph until a cloud and an IMU appear or the timeout passes.

    DDS discovery of an already running publisher takes from 0.1 s to a few
    seconds, so a single fixed wait either misses topics or always waits long.
    """
    deadline = clock() + timeout_sec
    while True:
        typed = [
            (name, type_name)
            for name, type_names in list_topics()
            for type_name in type_names
        ]
        found = {type_name for _, type_name in typed}
        if {CLOUD_TYPE, IMU_TYPE} <= found or clock() >= deadline:
            return typed
        sleep(0.1)


def discover_ros_graph(
    odom_frame: str,
    cloud_topic_for=None,
    timeout_sec: float = 5.0,
    listen_sec: float = 1.5,
):
    """Return live (topic, type) pairs, the (parent, child) TF edges seen, and
    the frame_id of one message on the cloud topic cloud_topic_for(topics) picks.
    """
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage
    except ImportError:
        return [], set(), None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node("lidar_localization_quickstart_discovery")
        edges = set()
        cloud_frames = []

        def record(message):
            for transform in message.transforms:
                edges.add(
                    (
                        transform.header.frame_id.lstrip("/"),
                        transform.child_frame_id.lstrip("/"),
                    )
                )

        node.create_subscription(TFMessage, "/tf", record, 100)
        node.create_subscription(
            TFMessage,
            "/tf_static",
            record,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def spin(seconds):
            rclpy.spin_once(node, timeout_sec=seconds)

        typed = wait_for_typed_topics(
            node.get_topic_names_and_types, timeout_sec, sleep=spin
        )
        cloud_topic = cloud_topic_for(typed) if cloud_topic_for else None
        if (cloud_topic, CLOUD_TYPE) in typed:
            node.create_subscription(
                PointCloud2,
                cloud_topic,
                lambda message: cloud_frames.append(message.header.frame_id),
                qos_profile_sensor_data,
            )
        else:
            cloud_topic = None
        # With TF topics, listen for the whole window: a static base -> LiDAR TF
        # that is missed would be published a second time.
        has_tf = any(name in ("/tf", "/tf_static") for name, _ in typed)
        deadline = time.monotonic() + listen_sec
        while time.monotonic() < deadline and (
            has_tf or (cloud_topic and not cloud_frames)
        ):
            spin(0.1)
        cloud_frame = cloud_frames[0].lstrip("/") if cloud_frames else None
        return typed, edges, cloud_frame or None
    finally:
        rclpy.shutdown()


def build_arg_parser(show_all: bool = False) -> argparse.ArgumentParser:
    """Quickstart options; tuning options are listed only with --help-all."""

    def advanced(text: str | None = None) -> str | None:
        return text if show_all else argparse.SUPPRESS

    parser = argparse.ArgumentParser(
        description="One-command lidar_localization_ros2 setup and guarded startup.",
        epilog=None if show_all else "Tuning options: --help-all.",
    )
    parser.add_argument(
        "--help-all", action="store_true", help="show the tuning options as well"
    )

    start = parser.add_argument_group("map and start")
    start.add_argument(
        "--map",
        "--map-path",
        dest="map_path",
        required=True,
        help="3D point cloud map (.pcd or .ply)",
    )
    start.add_argument(
        "--occupancy-map",
        dest="occupancy_yaml",
        help="2D occupancy grid (map.yaml) of the same map, for start without a pose",
    )
    start.add_argument(
        "--reference-csv",
        dest="reference_csv",
        help="Mapping-run reference trajectory CSV for route-crop G2 candidates "
        "(same format as make_route_grid_relocalization_attempts.py). "
        "Use with --occupancy-map or alone when the route is known.",
    )
    start.add_argument(
        "--initial-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="known start pose in the map frame",
    )
    start.add_argument(
        "--global-seed-z",
        type=float,
        help="score global candidates at this one map-frame sensor z instead of "
        "the map's ground under them plus --sensor-height",
    )
    start.add_argument(
        "--sensor-height",
        type=float,
        help="sensor height above the ground in metres; global candidates are scored "
        f"at the map's ground under them plus this (default: {DEFAULT_SENSOR_HEIGHT_M})",
    )
    start.add_argument(
        "--profile",
        choices=sorted(config_tool.PROFILE_DEFAULTS),
        default="standalone",
        help="sensor and output preset (default: standalone)",
    )
    start.add_argument(
        "--odom-tf-prediction",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="use an external odom -> base TF (e.g. a LIO front end) to predict "
        "motion between scans; needed for fast or handheld motion (default: on "
        "when that TF is published)",
    )
    start.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="follow /clock (default: on when /clock is published, as during "
        "ros2 bag play --clock)",
    )

    sensors = parser.add_argument_group("topics and frames")
    sensors.add_argument("--cloud-topic", help="PointCloud2 topic (default: detected)")
    sensors.add_argument("--imu-topic", help="Imu topic (default: detected)")
    sensors.add_argument(
        "--lidar-frame",
        help="LiDAR frame (default: the cloud's frame_id, else profile)",
    )
    sensors.add_argument("--imu-frame", help="IMU frame (default: profile)")
    sensors.add_argument(
        "--base-frame",
        help="robot base frame (default: the single child of odom in TF, else base_link)",
    )
    sensors.add_argument("--odom-frame", default="odom", help="odometry frame")
    sensors.add_argument("--global-frame", default="map", help="map frame")
    sensors.add_argument(
        "--publish-lidar-tf",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="publish a static identity base -> LiDAR TF (default: on unless the "
        "frames are the same or already linked in TF)",
    )
    sensors.add_argument(
        "--publish-imu-tf",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="publish a static base -> IMU TF",
    )

    session = parser.add_argument_group("session")
    session.add_argument(
        "--rviz",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="start RViz (turn off on a headless robot)",
    )
    session.add_argument(
        "--bringup-check",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="check topics and TF five seconds after launch",
    )
    session.add_argument(
        "--dry-run",
        action="store_true",
        help="write the configuration and print the commands without starting",
    )
    session.add_argument(
        "--output",
        type=Path,
        default=default_config_path(),
        help=advanced("generated parameter file"),
    )
    session.add_argument("--state-file", type=Path, help=advanced("saved pose file"))
    session.add_argument(
        "--auto-initialize",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(
            "Restore a verified saved pose, then use global search when configured."
        ),
    )
    session.add_argument(
        "--restore-saved-pose",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(),
    )
    session.add_argument(
        "--saved-pose-max-age-sec", type=float, default=0.0, help=advanced()
    )
    session.add_argument(
        "--discover-topics",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(),
    )
    session.add_argument(
        "--auto-occupancy-map",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(
            "without a pose or grid, generate (and cache) an occupancy grid from the "
            "map for global search"
        ),
    )

    tuning = parser.add_argument_group("global search and recovery tuning")
    for flag, kwargs in (
        ("--route-time-radius-sec", {"type": float, "default": 20.0}),
        ("--route-min-spacing-m", {"type": float, "default": 8.0}),
        ("--route-max-poses", {"type": int, "default": 32}),
        ("--route-yaw-offsets-deg", {"default": "-15,0,15"}),
        ("--route-lateral-offsets-m", {"default": "-2,0,2"}),
        ("--route-longitudinal-offsets-m", {"default": "-1,0,1"}),
        ("--min-candidate-score", {"type": float, "default": 0.6}),
        ("--min-score-margin", {"type": float, "default": 0.05}),
        ("--max-candidate-age-sec", {"type": float, "default": 30.0}),
        ("--global-query-timeout-sec", {"type": float, "default": 30.0}),
        ("--verification-samples", {"type": int, "default": 3}),
        ("--verification-fitness-threshold", {"type": float, "default": 1.5}),
        ("--max-global-attempts", {"type": int, "default": 6}),
        ("--global-consensus-samples", {"type": int, "default": 2}),
        ("--global-consensus-translation-m", {"type": float, "default": 2.0}),
        ("--global-consensus-yaw-deg", {"type": float, "default": 20.0}),
        ("--global-registration-score-gate", {"type": float, "default": 6.0}),
        ("--global-max-scan-points", {"type": int, "default": 256}),
        ("--global-max-candidates", {"type": int, "default": 8}),
        ("--global-nms-radius-m", {"type": float, "default": 3.0}),
        ("--supervisor-query-timeout-sec", {"type": float, "default": 45.0}),
        ("--supervisor-max-walk-candidates", {"type": int, "default": 4}),
    ):
        tuning.add_argument(flag, help=advanced(), **kwargs)
    tuning.add_argument(
        "--supervisor-odometry-confirmation-mode",
        choices=("window_waiver", "segment_defer"),
        default="window_waiver",
        help=advanced(
            "How G3 confirms answers around odometry dropouts; segment_defer is "
            "experimental."
        ),
    )
    tuning.add_argument(
        "--global-cpp-backend",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(),
    )
    tuning.add_argument(
        "--global-registration-scoring",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced("Use the 3D NDT scorer to validate and rank G2 candidates."),
    )
    tuning.add_argument(
        "--require-global-registration-scoring",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(
            "Refuse automatic global pose publication if 3D scoring is unavailable."
        ),
    )
    tuning.add_argument(
        "--refine-global-candidates",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=advanced(),
    )
    tuning.add_argument(
        "--global-angular-resolution-deg",
        type=float,
        default=5.0,
        help=advanced(
            "yaw step of the global search; coarser steps can miss the true "
            "heading (default: 5)"
        ),
    )
    tuning.add_argument(
        "--g3-recovery",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=advanced(
            "Launch guarded G3 reinitialization when global search is configured."
        ),
    )
    return parser


def manual_pose_step(args) -> str:
    """How the operator supplies a pose when automatic initialization stops."""
    if args.rviz:
        return "set 2D Pose Estimate in RViz"
    return "publish /initialpose (RViz is off)"


def next_step_line(args) -> str:
    """One actionable next step printed after the launch/bringup commands.

    Display only: never changes launch arguments or runtime behavior.
    """
    manual = manual_pose_step(args)
    if args.initial_pose is not None:
        return (
            "wait 5s, then verify pose output "
            "(see Bringup check above); if no pose, echo /alignment_status"
        )
    has_global_asset = bool(args.occupancy_yaml or args.reference_csv)
    if args.auto_initialize and has_global_asset:
        return (
            "keep robot stationary ~30s for guarded search; "
            f"if no_safe_automatic_source, {manual} "
            "or add --reference-csv (see docs/site_setup.md)"
        )
    if args.auto_initialize:
        return (
            f"{manual}, or restart with "
            "--occupancy-map / --reference-csv for automatic search"
        )
    return f"{manual} or pass --initial-pose"


def topic_discovery_hint(typed, message_type, selected, reason, flag) -> str | None:
    """Explain unresolved input selection without changing the selected topic."""
    label = message_type.rsplit("/", 1)[-1]
    if reason == "explicit":
        kinds = sorted(
            {kind for name, kind in typed if name == "/" + selected.lstrip("/")}
        )
        if message_type in kinds:
            return None
        problem = (
            f"has type {', '.join(kinds)}; expected {message_type}"
            if kinds
            else "was not detected; start the sensor driver or bag"
        )
        candidates = sorted({name for name, kind in typed if kind == message_type})
        alternative = (
            f" Available {label} topics: {', '.join(candidates)}. "
            f"Select one with {shlex.join([flag, candidates[0]])}."
            if candidates
            else f" Select a {label} input with {flag} TOPIC."
        )
        return (
            f"Selected topic {selected} {problem}. "
            f"Check with {shlex.join(['ros2', 'topic', 'info', '-v', selected])}."
            + alternative
        )
    if reason == "ambiguous":
        candidates = sorted({name for name, kind in typed if kind == message_type})
        return (
            f"Several {label} topics found: {', '.join(candidates)}; "
            f"keeping {selected}. Select one with {flag} TOPIC."
        )
    if reason == "not_detected":
        action = "If using IMU, start" if message_type == IMU_TYPE else "Start"
        return (
            f"No {label} topic detected; keeping {selected}. "
            f"{action} the sensor driver or bag, or set {flag} TOPIC."
        )
    return None


def lidar_transform_hint(tf_edges, base, lidar, publish, graph_checked) -> str | None:
    """Explain missing or duplicate TF without changing startup configuration."""
    if base == lidar:
        return None
    linked = graph_checked and not model.lidar_tf_needed(tf_edges, base, lidar)
    if linked and publish:
        action = (
            f"TF already connects {base} and {lidar}. "
            "Use --no-publish-lidar-tf to avoid a second transform publisher."
        )
    elif publish:
        action = (
            f"Quickstart will publish an identity transform from {base} to {lidar}. "
            "If the sensor is offset or rotated, publish the calibrated transform "
            "and use --no-publish-lidar-tf."
        )
    elif graph_checked and not linked:
        action = (
            f"No TF link observed between {base} and {lidar}. "
            "Publish the calibrated transform using the robot's TF setup."
        )
    else:
        return None
    check = shlex.join(["ros2", "run", "tf2_ros", "tf2_echo", base, lidar])
    return f"{action} Check with {check}."


def _config_args(args, cloud_topic: str, imu_topic: str):
    argv = [
        "--map-path",
        args.map_path,
        "--output",
        str(args.output),
        "--profile",
        args.profile,
        "--cloud-topic",
        cloud_topic,
        "--imu-topic",
        imu_topic,
        "--global-frame",
        args.global_frame,
        "--odom-frame",
        args.odom_frame,
        "--base-frame",
        args.base_frame,
        "--overwrite",
    ]
    if args.lidar_frame:
        argv.extend(["--lidar-frame", args.lidar_frame])
    if args.imu_frame:
        argv.extend(["--imu-frame", args.imu_frame])
    if args.initial_pose:
        argv.append("--initial-pose")
        argv.extend(str(value) for value in args.initial_pose)
    if args.use_sim_time:
        argv.append("--use-sim-time")
    if args.odom_tf_prediction:
        argv.append("--odom-tf-prediction")
    return config_tool.build_arg_parser().parse_args(argv)


def global_seed(args) -> tuple[float, float]:
    """Seed z and sensor height for scoring global candidates (height < 0: off)."""
    if args.global_seed_z is not None:
        return args.global_seed_z, -1.0
    if args.sensor_height is not None:
        return 0.0, args.sensor_height
    return 0.0, DEFAULT_SENSOR_HEIGHT_M


def launch_parts(args, config_args, config_path: Path, state_path: Path):
    seed_z, sensor_height = global_seed(args)
    cloud_topic = str(config_tool._arg_or_profile(config_args, "cloud_topic"))
    imu_topic = str(config_tool._arg_or_profile(config_args, "imu_topic"))
    lidar_frame = str(config_tool._arg_or_profile(config_args, "lidar_frame"))
    imu_frame = str(config_tool._arg_or_profile(config_args, "imu_frame"))
    pose_topic = (
        "/localization/pose_with_covariance"
        if args.profile in {"nav2", "mid360"}
        else "/pcl_pose"
    )
    explicit_pose = args.initial_pose is not None
    restore = args.auto_initialize and args.restore_saved_pose and not explicit_pose
    route_crop = bool(args.reference_csv)
    global_enabled = (
        args.auto_initialize
        and (args.occupancy_yaml or args.reference_csv)
        and not explicit_pose
    )
    g2_candidate_source = "route_crop" if route_crop else "bbs"
    g2_max_candidates = args.global_max_candidates
    if route_crop and g2_max_candidates == 8:
        g2_max_candidates = 16
    g3_enabled = global_enabled and args.g3_recovery
    supervisor_min_score = (
        0.15 if args.global_registration_scoring else args.min_candidate_score
    )
    supervisor_recovery_threshold = (
        3.5 if route_crop else args.verification_fitness_threshold
    )
    supervisor_max_walk = 1 if route_crop else args.supervisor_max_walk_candidates
    supervisor_confirm_samples = 1 if route_crop else 3
    values = {
        "profile": args.profile,
        "localization_param_dir": str(config_path),
        "map_path": str(Path(args.map_path).expanduser().resolve()),
        "occupancy_yaml": str(Path(args.occupancy_yaml).expanduser().resolve())
        if args.occupancy_yaml
        else "",
        "reference_csv": str(Path(args.reference_csv).expanduser().resolve())
        if args.reference_csv
        else "",
        "g2_candidate_source": g2_candidate_source,
        "pose_state_path": str(state_path),
        "cloud_topic": cloud_topic,
        "imu_topic": imu_topic,
        "pose_topic": pose_topic,
        "global_frame_id": args.global_frame,
        "odom_frame_id": args.odom_frame,
        "base_frame_id": args.base_frame,
        "lidar_frame_id": lidar_frame,
        "imu_frame_id": imu_frame,
        "use_sim_time": str(args.use_sim_time).lower(),
        "publish_lidar_tf": str(args.publish_lidar_tf).lower(),
        "publish_imu_tf": str(args.publish_imu_tf).lower(),
        "restore_saved_pose": str(restore).lower(),
        "initial_pose_preconfigured": str(explicit_pose).lower(),
        "enable_global_initialization": str(global_enabled).lower(),
        "enable_g3_recovery": str(g3_enabled).lower(),
        "start_rviz": str(args.rviz).lower(),
        "run_bringup_check": str(args.bringup_check).lower(),
        "saved_pose_max_age_sec": args.saved_pose_max_age_sec,
        "min_candidate_score": args.min_candidate_score,
        "min_score_margin": args.min_score_margin,
        "max_candidate_age_sec": args.max_candidate_age_sec,
        "global_query_timeout_sec": args.global_query_timeout_sec,
        "verification_samples": args.verification_samples,
        "verification_fitness_threshold": args.verification_fitness_threshold,
        "max_global_attempts": args.max_global_attempts,
        "global_consensus_samples": args.global_consensus_samples,
        "global_consensus_translation_m": args.global_consensus_translation_m,
        "global_consensus_yaw_deg": args.global_consensus_yaw_deg,
        "registration_fitness_high_confidence_threshold": (
            0.5 if route_crop else 1.0e9
        ),
        "g2_use_cpp_backend": str(args.global_cpp_backend).lower(),
        "g2_enable_registration_scoring": str(args.global_registration_scoring).lower(),
        "require_global_registration_scoring": str(
            args.require_global_registration_scoring
        ).lower(),
        "g2_registration_score_gate": args.global_registration_score_gate,
        "g2_registration_refine_candidates": str(args.refine_global_candidates).lower(),
        "g2_registration_seed_z_m": seed_z,
        "g2_registration_sensor_height_m": sensor_height,
        "g2_max_scan_points": args.global_max_scan_points,
        "g2_angular_resolution_deg": args.global_angular_resolution_deg,
        "g2_max_candidates": g2_max_candidates,
        "g2_nms_radius_m": args.global_nms_radius_m,
        "g2_route_time_radius_sec": args.route_time_radius_sec,
        "g2_route_min_spacing_m": args.route_min_spacing_m,
        "g2_route_max_poses": args.route_max_poses,
        "g2_route_yaw_offsets_deg": args.route_yaw_offsets_deg,
        "g2_route_lateral_offsets_m": args.route_lateral_offsets_m,
        "g2_route_longitudinal_offsets_m": args.route_longitudinal_offsets_m,
        "supervisor_min_candidate_score": supervisor_min_score,
        "supervisor_query_timeout_sec": args.supervisor_query_timeout_sec,
        "supervisor_max_walk_candidates": supervisor_max_walk,
        "supervisor_recovery_fitness_threshold": supervisor_recovery_threshold,
        "supervisor_settle_timeout_sec": 25.0 if route_crop else 20.0,
        "supervisor_recovery_confirmation_samples": supervisor_confirm_samples,
        "supervisor_enable_seed_motion_compensation": str(not route_crop).lower(),
        "supervisor_confirm_cross_check": str(not route_crop).lower(),
        "supervisor_prefer_reset_default_z_m": str(abs(seed_z) > 1.0e-9).lower(),
        "supervisor_odometry_confirmation_mode": (
            args.supervisor_odometry_confirmation_mode
        ),
    }
    parts = ["ros2", "launch", "lidar_localization_ros2", "quickstart.launch.py"]
    parts.extend(
        f"{key}:={value}"
        for key, value in values.items()
        if not (key in {"occupancy_yaml", "reference_csv"} and value == "")
    )
    return parts


def _validate(args) -> str | None:
    map_path = Path(args.map_path).expanduser()
    if map_path.is_dir():
        return (
            f"Map path is a directory: {map_path}. Pass a .pcd or .ply file with --map."
        )
    if not map_path.is_file():
        return (
            f"Map file does not exist: {map_path}. "
            "Use --map /absolute/path/to/map.pcd (or .ply)."
        )
    if map_path.suffix.lower() not in config_tool.SUPPORTED_MAP_SUFFIXES:
        return "Map must be a .pcd or .ply file."
    if args.occupancy_yaml:
        occupancy = Path(args.occupancy_yaml).expanduser()
        if not occupancy.is_file():
            return f"Occupancy map YAML does not exist: {occupancy}"
        if occupancy.suffix.lower() not in {".yaml", ".yml"}:
            return "Occupancy map must be a YAML file."
    if args.reference_csv:
        reference = Path(args.reference_csv).expanduser()
        if not reference.is_file():
            return f"Reference CSV does not exist: {reference}"
    if (
        not math.isfinite(args.route_time_radius_sec)
        or args.route_time_radius_sec <= 0.0
        or not math.isfinite(args.route_min_spacing_m)
        or args.route_min_spacing_m < 0.0
        or args.route_max_poses < 1
    ):
        return "Route-crop radius, spacing, and max poses must be valid."
    if (
        args.verification_samples < 1
        or args.max_global_attempts < 1
        or args.global_consensus_samples < 1
        or args.global_max_scan_points < 1
        or args.global_max_candidates < 2
    ):
        return (
            "Verification samples, candidates, points, and attempts must be positive."
        )
    if (
        not math.isfinite(args.global_registration_score_gate)
        or args.global_registration_score_gate <= 0.0
    ):
        return "global_registration_score_gate must be positive."
    if (
        not math.isfinite(args.global_angular_resolution_deg)
        or args.global_angular_resolution_deg <= 0.0
        or not math.isfinite(args.global_nms_radius_m)
        or args.global_nms_radius_m < 0.0
        or not math.isfinite(0.0 if args.global_seed_z is None else args.global_seed_z)
    ):
        return (
            "Global search resolution, NMS radius, and seed z must be finite and valid."
        )
    if args.sensor_height is not None:
        if not math.isfinite(args.sensor_height) or args.sensor_height < 0.0:
            return "--sensor-height must be a finite height above the ground."
        if args.global_seed_z is not None:
            return "Give --sensor-height or --global-seed-z, not both."
    policy_params = model.StartupParams(
        min_candidate_score=args.min_candidate_score,
        min_score_margin=args.min_score_margin,
        max_candidate_age_sec=args.max_candidate_age_sec,
        query_timeout_sec=args.global_query_timeout_sec,
        verification_fitness_threshold=args.verification_fitness_threshold,
        verification_samples=args.verification_samples,
        max_global_attempts=args.max_global_attempts,
        global_consensus_samples=args.global_consensus_samples,
        global_consensus_translation_m=args.global_consensus_translation_m,
        global_consensus_yaw_deg=args.global_consensus_yaw_deg,
        registration_fitness_high_confidence_threshold=(
            0.5 if args.reference_csv else 1.0e9
        ),
    )
    policy_error = model.validate_startup_params(policy_params)
    if policy_error:
        return policy_error
    return None


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--help-all" in argv:
        build_arg_parser(show_all=True).print_help()
        return 0
    args = build_arg_parser().parse_args(argv)
    error = _validate(args)
    if error:
        print(f"quickstart: {error}", file=sys.stderr)
        return 2

    defaults = config_tool.PROFILE_DEFAULTS[args.profile]
    cloud_topic = args.cloud_topic or str(defaults["cloud_topic"])
    imu_topic = args.imu_topic or str(defaults["imu_topic"])
    discovery_notes = []
    input_hints = []
    undecided = (
        args.cloud_topic,
        args.imu_topic,
        args.use_sim_time,
        args.odom_tf_prediction,
        args.base_frame,
        args.lidar_frame,
        args.publish_lidar_tf,
    )
    tf_edges = set()
    graph_checked = False
    if args.discover_topics and None in undecided:

        def cloud_topic_for(typed):
            if args.cloud_topic is not None:
                return args.cloud_topic
            return model.select_discovered_topic(typed, CLOUD_TYPE, cloud_topic)[0]

        typed, tf_edges, cloud_frame = discover_ros_graph(
            args.odom_frame, cloud_topic_for if args.lidar_frame is None else None
        )
        graph_checked = True
        if args.cloud_topic is None:
            cloud_topic, reason = model.select_discovered_topic(
                typed, CLOUD_TYPE, cloud_topic
            )
            discovery_notes.append(f"cloud={cloud_topic} ({reason})")
            hint = topic_discovery_hint(
                typed, CLOUD_TYPE, cloud_topic, reason, "--cloud-topic"
            )
            if hint:
                input_hints.append(hint)
        if args.imu_topic is None:
            imu_topic, reason = model.select_discovered_topic(
                typed, IMU_TYPE, imu_topic
            )
            discovery_notes.append(f"imu={imu_topic} ({reason})")
            hint = topic_discovery_hint(
                typed, IMU_TYPE, imu_topic, reason, "--imu-topic"
            )
            if hint:
                input_hints.append(hint)
        for explicit, kind, flag in (
            (args.cloud_topic, CLOUD_TYPE, "--cloud-topic"),
            (args.imu_topic, IMU_TYPE, "--imu-topic"),
        ):
            if explicit is not None:
                hint = topic_discovery_hint(typed, kind, explicit, "explicit", flag)
                if hint:
                    input_hints.append(hint)
        if args.use_sim_time is None:
            args.use_sim_time = model.detect_sim_time(typed)
            discovery_notes.append(
                "clock=sim (/clock published)"
                if args.use_sim_time
                else "clock=wall (no /clock)"
            )
        if args.odom_tf_prediction is None or args.base_frame is None:
            base_frame, odom_live = model.select_odometry_frame(
                tf_edges, args.odom_frame, args.base_frame
            )
            args.base_frame = base_frame
            if args.odom_tf_prediction is None:
                args.odom_tf_prediction = odom_live
            discovery_notes.append(
                f"odometry={args.odom_frame}->{base_frame} (live TF)"
                if odom_live
                else f"odometry=none (no {args.odom_frame}->{base_frame} TF)"
            )
        if args.lidar_frame is None and cloud_frame:
            args.lidar_frame = cloud_frame
            discovery_notes.append(f"lidar_frame={cloud_frame} (cloud header)")
    for hint in input_hints:
        print(f"Input hint:    {hint}", flush=True)
    args.use_sim_time = bool(args.use_sim_time)
    args.odom_tf_prediction = bool(args.odom_tf_prediction)
    args.base_frame = args.base_frame or "base_link"
    if args.publish_lidar_tf is None:
        lidar_frame = args.lidar_frame or str(defaults["lidar_frame"])
        args.publish_lidar_tf = model.lidar_tf_needed(
            tf_edges, args.base_frame, lidar_frame
        )
    tf_hint = lidar_transform_hint(
        tf_edges,
        args.base_frame,
        args.lidar_frame or str(defaults["lidar_frame"]),
        args.publish_lidar_tf,
        graph_checked,
    )
    if tf_hint:
        print(f"TF hint:       {tf_hint}", flush=True)

    occupancy_note = None
    if (
        args.auto_initialize
        and args.auto_occupancy_map
        and not (args.initial_pose or args.occupancy_yaml)
    ):
        map_path = Path(args.map_path).expanduser().resolve()
        cached = cached_occupancy_map(map_path, default_occupancy_dir())
        if cached.is_file():
            args.occupancy_yaml = str(cached)
            occupancy_note = f"{cached} (cached for this map)"
        elif args.dry_run:
            occupancy_note = f"would generate {cached} from the map"
        else:
            print(f"Generating an occupancy grid for global search: {cached}")
            error = generate_occupancy_map(map_path, cached)
            if error:
                occupancy_note = f"not generated ({error}); set the pose manually"
            else:
                args.occupancy_yaml = str(cached)
                occupancy_note = f"{cached} (generated from the map)"

    config_args = _config_args(args, cloud_topic, imu_topic)
    validation_error = config_tool.validate_args(config_args)
    if validation_error:
        print(f"quickstart: {validation_error}", file=sys.stderr)
        return 2
    output = args.output.expanduser().resolve()
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            config_tool.render_ros_params(config_tool.make_params(config_args)),
            encoding="utf-8",
        )
    except OSError as exc:
        print(
            f"quickstart: could not write configuration {output}: {exc}. "
            "Choose a writable file with --output PATH.",
            file=sys.stderr,
        )
        return 2
    state_path = (
        args.state_file.expanduser().resolve()
        if args.state_file
        else default_state_path(Path(args.map_path)).resolve()
    )
    parts = launch_parts(args, config_args, output, state_path)

    print(f"Configuration: {output}")
    print(f"Pose state:    {state_path}")
    if discovery_notes:
        print("Discovery:     " + ", ".join(discovery_notes))
    if occupancy_note:
        print(f"Occupancy map: {occupancy_note}")
    if args.auto_initialize and not args.initial_pose and args.occupancy_yaml:
        seed_z, sensor_height = global_seed(args)
        print(
            f"Seed height:   map ground + {sensor_height:g} m"
            if sensor_height >= 0.0
            else f"Seed height:   map z {seed_z:g} m"
        )
    fallback = "RViz" if args.rviz else "/initialpose"
    if args.initial_pose:
        print("Initialization: explicit pose")
    elif args.auto_initialize and args.reference_csv:
        suffix = " + guarded G3 recovery" if args.g3_recovery else ""
        print(
            f"Initialization: verified saved pose -> guarded route-crop search{suffix} -> {fallback}"
        )
    elif args.auto_initialize and args.occupancy_yaml:
        suffix = " + guarded G3 recovery" if args.g3_recovery else ""
        print(
            f"Initialization: verified saved pose -> guarded global search{suffix} -> {fallback}"
        )
    elif args.auto_initialize:
        print(
            f"Initialization: verified saved pose -> {fallback} "
            "(add --occupancy-map or --reference-csv for automatic global search)"
        )
    else:
        print(f"Initialization: explicit pose or {fallback}")
    print("Launch:")
    print("  " + shlex.join(parts))
    print("Bringup check:")
    print("  " + config_tool.doctor_command(config_args))
    print(f"Next: {next_step_line(args)}")
    if args.dry_run:
        return 0
    sys.stdout.flush()
    os.execvp(parts[0], parts)
    return 127


if __name__ == "__main__":
    sys.exit(main())
