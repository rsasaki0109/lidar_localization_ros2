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
from pathlib import Path
from typing import Optional, Sequence


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
    return Path.home() / ".local" / "state" / "lidar_localization_ros2" / f"{safe_stem}.json"


def parse_typed_topics(output: str):
    topics = []
    for line in output.splitlines():
        match = re.match(r"^(\S+)\s+\[([^,\]]+)", line.strip())
        if match:
            topics.append((match.group(1), match.group(2)))
    return topics


def discover_typed_topics(timeout_sec: float = 2.0):
    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "--types"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    return parse_typed_topics(result.stdout)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="One-command lidar_localization_ros2 setup and guarded startup.")
    parser.add_argument("--map", "--map-path", dest="map_path", required=True)
    parser.add_argument("--occupancy-map", dest="occupancy_yaml")
    parser.add_argument(
        "--profile", choices=sorted(config_tool.PROFILE_DEFAULTS), default="standalone")
    parser.add_argument("--output", type=Path, default=default_config_path())
    parser.add_argument("--state-file", type=Path)
    parser.add_argument("--cloud-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--lidar-frame")
    parser.add_argument("--imu-frame")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--initial-pose", type=float, nargs=7)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument(
        "--auto-initialize",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Restore a verified saved pose, then use global search when configured.",
    )
    parser.add_argument(
        "--restore-saved-pose", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--saved-pose-max-age-sec", type=float, default=0.0)
    parser.add_argument(
        "--discover-topics", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rviz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--bringup-check", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--publish-lidar-tf", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--publish-imu-tf", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--min-candidate-score", type=float, default=0.6)
    parser.add_argument("--min-score-margin", type=float, default=0.05)
    parser.add_argument("--max-candidate-age-sec", type=float, default=30.0)
    parser.add_argument("--global-query-timeout-sec", type=float, default=30.0)
    parser.add_argument("--verification-samples", type=int, default=3)
    parser.add_argument("--verification-fitness-threshold", type=float, default=1.5)
    parser.add_argument("--max-global-attempts", type=int, default=6)
    parser.add_argument("--global-consensus-samples", type=int, default=2)
    parser.add_argument("--global-consensus-translation-m", type=float, default=2.0)
    parser.add_argument("--global-consensus-yaw-deg", type=float, default=20.0)
    parser.add_argument(
        "--global-cpp-backend", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--global-registration-scoring",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the 3D NDT scorer to validate and rank G2 candidates.",
    )
    parser.add_argument(
        "--require-global-registration-scoring",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Refuse automatic global pose publication if 3D scoring is unavailable.",
    )
    parser.add_argument("--global-registration-score-gate", type=float, default=6.0)
    parser.add_argument(
        "--refine-global-candidates",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--global-seed-z", type=float, default=0.0)
    parser.add_argument("--global-max-scan-points", type=int, default=256)
    parser.add_argument("--global-angular-resolution-deg", type=float, default=10.0)
    parser.add_argument("--global-max-candidates", type=int, default=8)
    parser.add_argument("--global-nms-radius-m", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def _config_args(args, cloud_topic: str, imu_topic: str):
    argv = [
        "--map-path", args.map_path,
        "--output", str(args.output),
        "--profile", args.profile,
        "--cloud-topic", cloud_topic,
        "--imu-topic", imu_topic,
        "--global-frame", args.global_frame,
        "--odom-frame", args.odom_frame,
        "--base-frame", args.base_frame,
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
    return config_tool.build_arg_parser().parse_args(argv)


def launch_parts(args, config_args, config_path: Path, state_path: Path):
    cloud_topic = str(config_tool._arg_or_profile(config_args, "cloud_topic"))
    imu_topic = str(config_tool._arg_or_profile(config_args, "imu_topic"))
    lidar_frame = str(config_tool._arg_or_profile(config_args, "lidar_frame"))
    imu_frame = str(config_tool._arg_or_profile(config_args, "imu_frame"))
    pose_topic = (
        "/localization/pose_with_covariance"
        if args.profile in {"nav2", "mid360"} else "/pcl_pose")
    explicit_pose = args.initial_pose is not None
    restore = args.auto_initialize and args.restore_saved_pose and not explicit_pose
    global_enabled = (
        args.auto_initialize and bool(args.occupancy_yaml) and not explicit_pose)
    values = {
        "profile": args.profile,
        "localization_param_dir": str(config_path),
        "map_path": str(Path(args.map_path).expanduser().resolve()),
        "occupancy_yaml": str(Path(args.occupancy_yaml).expanduser().resolve())
        if args.occupancy_yaml else "",
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
        "g2_use_cpp_backend": str(args.global_cpp_backend).lower(),
        "g2_enable_registration_scoring": str(
            args.global_registration_scoring).lower(),
        "require_global_registration_scoring": str(
            args.require_global_registration_scoring).lower(),
        "g2_registration_score_gate": args.global_registration_score_gate,
        "g2_registration_refine_candidates": str(
            args.refine_global_candidates).lower(),
        "g2_registration_seed_z_m": args.global_seed_z,
        "g2_max_scan_points": args.global_max_scan_points,
        "g2_angular_resolution_deg": args.global_angular_resolution_deg,
        "g2_max_candidates": args.global_max_candidates,
        "g2_nms_radius_m": args.global_nms_radius_m,
    }
    parts = ["ros2", "launch", "lidar_localization_ros2", "quickstart.launch.py"]
    parts.extend(
        f"{key}:={value}"
        for key, value in values.items()
        if not (key == "occupancy_yaml" and value == "")
    )
    return parts


def _validate(args) -> Optional[str]:
    map_path = Path(args.map_path).expanduser()
    if not map_path.is_file():
        return f"Map file does not exist: {map_path}"
    if map_path.suffix.lower() not in config_tool.SUPPORTED_MAP_SUFFIXES:
        return "Map must be a .pcd or .ply file."
    if args.occupancy_yaml:
        occupancy = Path(args.occupancy_yaml).expanduser()
        if not occupancy.is_file():
            return f"Occupancy map YAML does not exist: {occupancy}"
        if occupancy.suffix.lower() not in {".yaml", ".yml"}:
            return "Occupancy map must be a YAML file."
    if (
        args.verification_samples < 1
        or args.max_global_attempts < 1
        or args.global_consensus_samples < 1
        or args.global_max_scan_points < 1
        or args.global_max_candidates < 2
    ):
        return "Verification samples, candidates, points, and attempts must be positive."
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
        or not math.isfinite(args.global_seed_z)
    ):
        return "Global search resolution, NMS radius, and seed z must be finite and valid."
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
    )
    policy_error = model.validate_startup_params(policy_params)
    if policy_error:
        return policy_error
    return None


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    error = _validate(args)
    if error:
        print(f"quickstart: {error}", file=sys.stderr)
        return 2

    defaults = config_tool.PROFILE_DEFAULTS[args.profile]
    cloud_topic = args.cloud_topic or str(defaults["cloud_topic"])
    imu_topic = args.imu_topic or str(defaults["imu_topic"])
    discovery_notes = []
    if args.discover_topics and (args.cloud_topic is None or args.imu_topic is None):
        typed = discover_typed_topics()
        if args.cloud_topic is None:
            cloud_topic, reason = model.select_discovered_topic(
                typed, "sensor_msgs/msg/PointCloud2", cloud_topic)
            discovery_notes.append(f"cloud={cloud_topic} ({reason})")
        if args.imu_topic is None:
            imu_topic, reason = model.select_discovered_topic(
                typed, "sensor_msgs/msg/Imu", imu_topic)
            discovery_notes.append(f"imu={imu_topic} ({reason})")

    config_args = _config_args(args, cloud_topic, imu_topic)
    validation_error = config_tool.validate_args(config_args)
    if validation_error:
        print(f"quickstart: {validation_error}", file=sys.stderr)
        return 2
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        config_tool.render_ros_params(config_tool.make_params(config_args)),
        encoding="utf-8",
    )
    state_path = (
        args.state_file.expanduser().resolve()
        if args.state_file else default_state_path(Path(args.map_path)).resolve())
    parts = launch_parts(args, config_args, output, state_path)

    print(f"Configuration: {output}")
    print(f"Pose state:    {state_path}")
    if discovery_notes:
        print("Discovery:     " + ", ".join(discovery_notes))
    if args.initial_pose:
        print("Initialization: explicit pose")
    elif args.auto_initialize and args.occupancy_yaml:
        print("Initialization: verified saved pose -> guarded global search -> RViz")
    elif args.auto_initialize:
        print(
            "Initialization: verified saved pose -> RViz "
            "(add --occupancy-map for global search)"
        )
    else:
        print("Initialization: explicit pose or RViz")
    print("Launch:")
    print("  " + shlex.join(parts))
    print("Bringup check:")
    print("  " + config_tool.doctor_command(config_args))
    if args.dry_run:
        return 0
    sys.stdout.flush()
    os.execvp(parts[0], parts)
    return 127


if __name__ == "__main__":
    sys.exit(main())
