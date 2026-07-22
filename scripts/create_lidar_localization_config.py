#!/usr/bin/env python3

import argparse
import json
import shlex
import sys
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence


PROFILE_DEFAULTS: Dict[str, Dict[str, object]] = {
    "standalone": {
        "cloud_topic": "/velodyne_points",
        "imu_topic": "/imu",
        "lidar_frame": "velodyne",
        "imu_frame": "imu_link",
        "score_threshold": 6.0,
        "voxel_leaf_size": 0.2,
        "scan_max_range": 100.0,
        "scan_min_range": 1.0,
        "enable_scan_voxel_filter": False,
        "enable_map_odom_tf": False,
        "use_twist_prediction": False,
        "max_twist_prediction_dt": 0.5,
        "local_map_radius": 150.0,
        "imu_mode": "off",
        "imu_preintegration_use_base_frame_transform": False,
    },
    "nav2": {
        "cloud_topic": "/velodyne_points",
        "imu_topic": "/imu/data",
        "lidar_frame": "velodyne",
        "imu_frame": "imu_link",
        "score_threshold": 6.0,
        "voxel_leaf_size": 0.2,
        "scan_max_range": 100.0,
        "scan_min_range": 1.0,
        "enable_scan_voxel_filter": False,
        "enable_map_odom_tf": True,
        "use_twist_prediction": True,
        "max_twist_prediction_dt": 0.5,
        "local_map_radius": 150.0,
        "imu_mode": "off",
        "imu_preintegration_use_base_frame_transform": False,
    },
    "mid360": {
        "cloud_topic": "/livox/points",
        "imu_topic": "/livox/imu",
        "lidar_frame": "livox_frame",
        "imu_frame": "livox_imu_frame",
        "score_threshold": 6.0,
        "voxel_leaf_size": 0.25,
        "scan_max_range": 60.0,
        "scan_min_range": 0.8,
        "enable_scan_voxel_filter": True,
        "enable_map_odom_tf": True,
        "use_twist_prediction": True,
        "max_twist_prediction_dt": 0.25,
        "local_map_radius": 80.0,
        "imu_mode": "preintegration",
        "imu_preintegration_use_base_frame_transform": True,
    },
}
SUPPORTED_MAP_SUFFIXES = {".pcd", ".ply"}


def yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return json.dumps(value)
    return str(value)


def render_ros_params(params: Dict[str, object]) -> str:
    lines = ["/**:", "  ros__parameters:"]
    for key in sorted(params.keys()):
        lines.append(f"    {key}: {yaml_scalar(params[key])}")
    return "\n".join(lines) + "\n"


def command_line(parts: Sequence[object]) -> str:
    return shlex.join(str(part) for part in parts)


def normalized_map_path(map_path: str) -> str:
    return str(Path(map_path).expanduser().resolve(strict=False))


def map_path_warnings(map_path: str) -> List[str]:
    path = Path(normalized_map_path(map_path))
    warnings: List[str] = []
    if not path.exists():
        warnings.append(
            f"Map path does not exist yet: {path}. Check the path before launching."
        )
    suffix = path.suffix.lower()
    if suffix not in SUPPORTED_MAP_SUFFIXES:
        suffix_text = suffix if suffix else "none"
        warnings.append(
            f"Map path extension is {suffix_text}; expected .pcd or .ply."
        )
    return warnings


def _arg_or_profile(args: argparse.Namespace, key: str) -> object:
    value = getattr(args, key, None)
    if value is not None:
        return value
    return PROFILE_DEFAULTS[args.profile][key]


def effective_imu_mode(args: argparse.Namespace) -> str:
    if args.enable_imu:
        return "both"
    if args.imu_mode != "profile":
        return args.imu_mode
    return str(PROFILE_DEFAULTS[args.profile]["imu_mode"])


def use_imu_preintegration_base_frame_transform(args: argparse.Namespace) -> bool:
    mode = effective_imu_mode(args)
    if mode not in {"preintegration", "both"}:
        return False
    return bool(PROFILE_DEFAULTS[args.profile]["imu_preintegration_use_base_frame_transform"])


def use_continuous_time_deskew(args: argparse.Namespace) -> bool:
    configured = getattr(args, "enable_continuous_time_deskew", None)
    if configured is not None:
        return bool(configured)
    # Profiles with IMU preintegration already have the required motion source.
    # No-IMU profiles remain usable and can opt in after selecting an IMU mode.
    return effective_imu_mode(args) in {"preintegration", "both"}


def continuous_time_deskew_requires_imu(args: argparse.Namespace) -> bool:
    return (
        use_continuous_time_deskew(args)
        and getattr(args, "continuous_time_deskew_mode", "relative_motion")
        != "lidar_constant_velocity"
    )


def validate_args(args: argparse.Namespace) -> Optional[str]:
    if continuous_time_deskew_requires_imu(args) and effective_imu_mode(args) == "off":
        return (
            "--enable-continuous-time-deskew requires IMU preintegration. "
            "Pass --imu-mode preintegration or use a profile that enables it."
        )
    return None


def make_params(args: argparse.Namespace) -> Dict[str, object]:
    imu_mode = effective_imu_mode(args)
    params: Dict[str, object] = {
        "registration_method": "NDT_OMP",
        "score_threshold": _arg_or_profile(args, "score_threshold"),
        "ndt_resolution": args.ndt_resolution,
        "ndt_step_size": 0.1,
        "ndt_num_threads": args.ndt_threads,
        "ndt_max_iterations": args.ndt_max_iterations,
        "transform_epsilon": 0.01,
        "voxel_leaf_size": _arg_or_profile(args, "voxel_leaf_size"),
        "enable_scan_voxel_filter": _arg_or_profile(args, "enable_scan_voxel_filter"),
        "scan_max_range": _arg_or_profile(args, "scan_max_range"),
        "scan_min_range": _arg_or_profile(args, "scan_min_range"),
        "scan_period": args.scan_period,
        "scan_time_range_max_duration_ratio": getattr(
            args, "scan_time_range_max_duration_ratio", 2.0),
        "use_pcd_map": True,
        "map_path": normalized_map_path(args.map_path),
        "set_initial_pose": args.initial_pose is not None,
        "initial_pose_x": 0.0,
        "initial_pose_y": 0.0,
        "initial_pose_z": 0.0,
        "initial_pose_qx": 0.0,
        "initial_pose_qy": 0.0,
        "initial_pose_qz": 0.0,
        "initial_pose_qw": 1.0,
        "use_odom": False,
        "use_twist_prediction": _arg_or_profile(args, "use_twist_prediction"),
        "twist_prediction_use_angular_velocity": False,
        "max_twist_prediction_dt": _arg_or_profile(args, "max_twist_prediction_dt"),
        "use_imu": imu_mode in {"legacy", "both"},
        "use_imu_preintegration": imu_mode in {"preintegration", "both"},
        "imu_preintegration_use_base_frame_transform": use_imu_preintegration_base_frame_transform(args),
        "use_continuous_time_deskew": use_continuous_time_deskew(args),
        "continuous_time_deskew_mode": getattr(
            args, "continuous_time_deskew_mode", "relative_motion"),
        "continuous_time_cloud_stamp_reference": getattr(
            args, "continuous_time_cloud_stamp_reference", "start"),
        "continuous_time_deskew_reference_time_sec": args.deskew_reference_time_sec,
        "continuous_time_pose_history_duration_sec": getattr(
            args, "continuous_time_pose_history_duration_sec", 2.0),
        "enable_localizability_guard": getattr(
            args, "enable_localizability_guard", False),
        "localizability_min_xy_eigen_ratio": getattr(
            args, "localizability_min_xy_eigen_ratio", 0.05),
        "enable_registration_localizability_diagnostics": getattr(
            args, "enable_registration_localizability_diagnostics", False),
        "enable_debug": False,
        "predict_pose_from_previous_delta": True,
        "enable_local_map_crop": True,
        "local_map_radius": _arg_or_profile(args, "local_map_radius"),
        "local_map_min_points": 100,
        "reject_above_score_threshold": True,
        "enable_borderline_seed_rejection_gate": True,
        "borderline_seed_gate_score_threshold": 5.25,
        "borderline_seed_gate_min_seed_translation_m": 1.0,
        "enable_open_loop_strict_score_threshold": getattr(
            args, "enable_open_loop_strict_score_threshold", False),
        "open_loop_strict_min_accepted_gap_sec": getattr(
            args, "open_loop_strict_min_accepted_gap_sec", 15.0),
        "open_loop_strict_min_seed_translation_m": getattr(
            args, "open_loop_strict_min_seed_translation_m", 100.0),
        "open_loop_strict_score_threshold": getattr(
            args, "open_loop_strict_score_threshold", 5.25),
        "enable_reinitialization_request_output": True,
        "enable_reinitialization_request_latch": True,
        "reinitialization_trigger_threshold": getattr(
            args, "reinitialization_trigger_threshold", 0.95),
        "reinitialization_trigger_gap_scale_sec": getattr(
            args, "reinitialization_trigger_gap_scale_sec", 30.0),
        "reinitialization_trigger_seed_translation_scale_m":
            getattr(args, "reinitialization_trigger_seed_translation_scale_m", 100.0),
        "reinitialization_trigger_reject_streak_scale":
            getattr(args, "reinitialization_trigger_reject_streak_scale", 200.0),
        "reinitialization_trigger_fitness_explosion_threshold":
            getattr(args, "reinitialization_trigger_fitness_explosion_threshold", 1000.0),
        "enable_map_odom_tf": _arg_or_profile(args, "enable_map_odom_tf"),
        "global_frame_id": args.global_frame,
        "odom_frame_id": args.odom_frame,
        "base_frame_id": args.base_frame,
        "enable_timer_publishing": args.profile in {"nav2", "mid360"},
        "pose_publish_frequency": 20.0 if args.profile == "mid360" else 30.0,
    }
    if args.initial_pose is not None:
        (
            params["initial_pose_x"],
            params["initial_pose_y"],
            params["initial_pose_z"],
            params["initial_pose_qx"],
            params["initial_pose_qy"],
            params["initial_pose_qz"],
            params["initial_pose_qw"],
        ) = args.initial_pose
    return params


def launch_command(args: argparse.Namespace, output_path: Path) -> str:
    cloud_topic = str(_arg_or_profile(args, "cloud_topic"))
    imu_topic = str(_arg_or_profile(args, "imu_topic"))
    lidar_frame = str(_arg_or_profile(args, "lidar_frame"))
    imu_frame = str(_arg_or_profile(args, "imu_frame"))
    if args.profile == "nav2":
        launch_file = "nav2_lidar_localization.launch.py"
        extras = [
            f"cloud_topic:={cloud_topic}",
            f"imu_topic:={imu_topic}",
            f"lidar_frame_id:={lidar_frame}",
        ]
    elif args.profile == "mid360":
        launch_file = "mid360_legged_localization.launch.py"
        extras = [
            f"map_path:={normalized_map_path(args.map_path)}",
            f"cloud_topic:={cloud_topic}",
            f"imu_topic:={imu_topic}",
            f"lidar_frame_id:={lidar_frame}",
        ]
    else:
        launch_file = "lidar_localization.launch.py"
        extras = [
            f"cloud_topic:={cloud_topic}",
            f"imu_topic:={imu_topic}",
            f"lidar_frame_id:={lidar_frame}",
        ]
    extras.extend([
        f"global_frame_id:={args.global_frame}",
        f"odom_frame_id:={args.odom_frame}",
        f"base_frame_id:={args.base_frame}",
    ])
    if getattr(args, "use_sim_time", False):
        extras.append("use_sim_time:=true")
    if args.profile == "mid360" and args.initial_pose is not None:
        initial_pose_keys = [
            "initial_pose_x",
            "initial_pose_y",
            "initial_pose_z",
            "initial_pose_qx",
            "initial_pose_qy",
            "initial_pose_qz",
            "initial_pose_qw",
        ]
        extras.append("set_initial_pose:=true")
        extras.extend(
            f"{key}:={value}" for key, value in zip(initial_pose_keys, args.initial_pose)
        )
    if args.no_publish_lidar_tf or args.base_frame == lidar_frame:
        extras.append("publish_lidar_tf:=false")
    elif args.lidar_tf is not None:
        tf_keys = [
            "lidar_tf_x",
            "lidar_tf_y",
            "lidar_tf_z",
            "lidar_tf_roll",
            "lidar_tf_pitch",
            "lidar_tf_yaw",
        ]
        extras.extend(f"{key}:={value}" for key, value in zip(tf_keys, args.lidar_tf))
    if args.no_publish_imu_tf or args.base_frame == imu_frame:
        extras.append("publish_imu_tf:=false")
    elif args.imu_tf is not None:
        extras.extend([
            "publish_imu_tf:=true",
            f"imu_frame_id:={imu_frame}",
        ])
        tf_keys = [
            "imu_tf_x",
            "imu_tf_y",
            "imu_tf_z",
            "imu_tf_roll",
            "imu_tf_pitch",
            "imu_tf_yaw",
        ]
        extras.extend(f"{key}:={value}" for key, value in zip(tf_keys, args.imu_tf))
    elif args.imu_frame is not None:
        extras.append(f"imu_frame_id:={imu_frame}")
    extras.extend([
        f"use_imu_preintegration:={str(effective_imu_mode(args) in {'preintegration', 'both'}).lower()}",
        (
            "imu_preintegration_use_base_frame_transform:="
            f"{str(use_imu_preintegration_base_frame_transform(args)).lower()}"
        ),
    ])
    if use_continuous_time_deskew(args):
        extras.extend([
            "use_continuous_time_deskew:=true",
            f"continuous_time_deskew_reference_time_sec:={args.deskew_reference_time_sec}",
        ])
    parts = [
        "ros2",
        "launch",
        "lidar_localization_ros2",
        launch_file,
        f"localization_param_dir:={output_path}",
    ]
    parts.extend(extras)
    return command_line(parts)


def doctor_command(args: argparse.Namespace) -> str:
    cloud_topic = str(_arg_or_profile(args, "cloud_topic"))
    imu_topic = str(_arg_or_profile(args, "imu_topic"))
    lidar_frame = str(_arg_or_profile(args, "lidar_frame"))
    imu_frame = str(_arg_or_profile(args, "imu_frame"))
    parts = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "check_lidar_localization_bringup.py",
        "--profile",
        args.profile,
        "--cloud-topic",
        cloud_topic,
        "--imu-topic",
        imu_topic,
        "--lidar-frame",
        lidar_frame,
        "--imu-frame",
        imu_frame,
        "--global-frame",
        args.global_frame,
        "--odom-frame",
        args.odom_frame,
        "--base-frame",
        args.base_frame,
    ]
    if args.profile in {"nav2", "mid360"}:
        parts.append("--require-odom-base-tf")
    if args.profile == "nav2":
        parts.append("--require-map-odom-tf")
    if effective_imu_mode(args) != "off" or continuous_time_deskew_requires_imu(args):
        parts.append("--require-imu")
    if args.require_cloud_time_field or use_continuous_time_deskew(args):
        parts.append("--require-cloud-time-field")
    if use_imu_preintegration_base_frame_transform(args):
        parts.append("--require-imu-base-tf")
    return command_line(parts)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a small lidar_localization_ros2 parameter file for first bringup."
    )
    parser.add_argument("--map-path", required=True, help="Absolute or workspace-local PCD/PLY map path.")
    parser.add_argument("--output", default="lidar_localization.generated.yaml")
    parser.add_argument("--profile", choices=sorted(PROFILE_DEFAULTS), default="standalone")
    parser.add_argument("--cloud-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--lidar-frame")
    parser.add_argument("--imu-frame")
    parser.add_argument(
        "--require-cloud-time-field",
        action="store_true",
        help="Add a doctor check for per-point timing fields used by continuous-time deskew.",
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="Add use_sim_time:=true to the printed launch command for rosbag playback.",
    )
    parser.add_argument(
        "--lidar-tf",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Static base_frame -> lidar_frame transform used by the launch command.",
    )
    parser.add_argument(
        "--no-publish-lidar-tf",
        action="store_true",
        help="Do not publish a static lidar TF; use this when robot_state_publisher or the bag already provides it.",
    )
    parser.add_argument(
        "--imu-tf",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Static base_frame -> imu_frame transform used by the launch command.",
    )
    parser.add_argument(
        "--no-publish-imu-tf",
        action="store_true",
        help="Do not publish a static IMU TF; use this when robot_state_publisher or the bag already provides it.",
    )
    parser.add_argument(
        "--initial-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Set an initial map pose in the generated YAML.",
    )
    parser.add_argument(
        "--imu-mode",
        choices=["profile", "off", "preintegration", "legacy", "both"],
        default="profile",
        help=(
            "IMU usage in the generated YAML. profile uses the selected profile default; "
            "preintegration enables only the guarded IMU preintegration path; legacy enables "
            "the older use_imu path; both enables both paths."
        ),
    )
    parser.add_argument(
        "--enable-imu",
        action="store_true",
        help="Compatibility alias for --imu-mode both.",
    )
    parser.add_argument(
        "--enable-continuous-time-deskew",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Enable continuous-time deskew (default for IMU preintegration profiles). "
            "It requires per-point PointCloud2 timing and falls back safely until "
            "the required motion data is ready."
        ),
    )
    parser.add_argument(
        "--deskew-reference-time-sec",
        type=float,
        default=0.0,
        help="Scan-relative reference time for continuous-time deskew; 0.0 uses the earliest point.",
    )
    parser.add_argument("--score-threshold", type=float)
    parser.add_argument(
        "--enable-open-loop-strict-score-threshold",
        action="store_true",
        help=(
            "Tighten the fitness gate when the localizer has been open-loop "
            "for a while and the seed has moved away from the last accepted pose."
        ),
    )
    parser.add_argument("--open-loop-strict-min-accepted-gap-sec", type=float, default=15.0)
    parser.add_argument("--open-loop-strict-min-seed-translation-m", type=float, default=100.0)
    parser.add_argument("--open-loop-strict-score-threshold", type=float, default=5.25)
    parser.add_argument(
        "--reinitialization-trigger-threshold",
        type=float,
        default=0.95,
        help="Score threshold for publishing /reinitialization_requested.",
    )
    parser.add_argument(
        "--reinitialization-trigger-gap-scale-sec",
        type=float,
        default=30.0,
        help="Open-loop accepted-gap scale used by the reinitialization trigger score.",
    )
    parser.add_argument(
        "--reinitialization-trigger-seed-translation-scale-m",
        type=float,
        default=100.0,
        help="Seed drift scale used by the reinitialization trigger score.",
    )
    parser.add_argument(
        "--reinitialization-trigger-reject-streak-scale",
        type=float,
        default=200.0,
        help="Rejected-update streak scale used by the reinitialization trigger score.",
    )
    parser.add_argument(
        "--reinitialization-trigger-fitness-explosion-threshold",
        type=float,
        default=1000.0,
        help="Fitness score that adds the trigger's fitness-explosion component.",
    )
    parser.add_argument("--voxel-leaf-size", type=float)
    parser.add_argument("--scan-max-range", type=float)
    parser.add_argument("--scan-min-range", type=float)
    parser.add_argument("--scan-period", type=float, default=0.1)
    parser.add_argument(
        "--scan-time-range-max-duration-ratio", type=float, default=2.0)
    parser.add_argument("--ndt-resolution", type=float, default=1.0)
    parser.add_argument("--ndt-threads", type=int, default=4)
    parser.add_argument("--ndt-max-iterations", type=int, default=35)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    validation_error = validate_args(args)
    if validation_error is not None:
        print(validation_error, file=sys.stderr)
        return 2
    output_path = Path(args.output).expanduser()
    if output_path.exists() and not args.overwrite:
        print(f"{output_path} already exists. Pass --overwrite to replace it.", file=sys.stderr)
        return 2
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_ros_params(make_params(args)), encoding="utf-8")
    resolved_output = output_path.resolve()
    print(f"Wrote {resolved_output}")
    warnings = map_path_warnings(args.map_path)
    if warnings:
        print("")
        print("Warnings:")
        for warning in warnings:
            print(f"  - {warning}")
    print("")
    print("Launch:")
    print(f"  {launch_command(args, resolved_output)}")
    print("")
    print("Bringup check:")
    print(f"  {doctor_command(args)}")
    if args.initial_pose is None:
        print("")
        print("Initial pose is not embedded. Publish /initialpose from RViz or pass --initial-pose.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
