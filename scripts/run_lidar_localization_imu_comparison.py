#!/usr/bin/env python3

import argparse
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

import create_lidar_localization_config as config_tool


@dataclass(frozen=True)
class ComparisonMode:
    name: str
    imu_mode: str
    enable_continuous_time_deskew: bool = False
    validate_imu: bool = False
    require_deskew_applied: bool = False
    continuous_time_deskew_mode: str = "relative_motion"
    enable_localizability_guard: bool = False
    enable_registration_localizability_diagnostics: bool = False


MODES: Dict[str, ComparisonMode] = {
    "lidar_only": ComparisonMode("lidar_only", "off"),
    "imu_preintegration": ComparisonMode(
        "imu_preintegration",
        "preintegration",
        validate_imu=True,
    ),
    "deskew": ComparisonMode(
        "deskew",
        "preintegration",
        enable_continuous_time_deskew=True,
        validate_imu=True,
        require_deskew_applied=True,
    ),
    "imu_pose_history": ComparisonMode(
        "imu_pose_history",
        "preintegration",
        enable_continuous_time_deskew=True,
        validate_imu=True,
        require_deskew_applied=True,
        continuous_time_deskew_mode="imu_pose_history",
    ),
    "lidar_constant_velocity": ComparisonMode(
        "lidar_constant_velocity",
        "off",
        enable_continuous_time_deskew=True,
        require_deskew_applied=True,
        continuous_time_deskew_mode="lidar_constant_velocity",
    ),
    "localizability_guard": ComparisonMode(
        "localizability_guard",
        "off",
        enable_localizability_guard=True,
    ),
    "registration_localizability": ComparisonMode(
        "registration_localizability",
        "off",
        enable_registration_localizability_diagnostics=True,
    ),
}
BAG_FILE_SUFFIXES = {".db3", ".mcap"}
POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run lidar_localization_ros2 LiDAR-only / IMU preintegration / "
            "continuous-time deskew comparisons on one rosbag2 input."
        )
    )
    parser.add_argument("--bag-path", default="")
    parser.add_argument("--map-path", default="")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--profile", choices=sorted(config_tool.PROFILE_DEFAULTS), default="mid360")
    parser.add_argument(
        "--mode",
        action="append",
        choices=sorted(MODES),
        help="Mode to run. Repeat to choose a subset. Default: all three modes.",
    )
    parser.add_argument("--cloud-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--lidar-frame")
    parser.add_argument("--imu-frame")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument(
        "--lidar-tf",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
    )
    parser.add_argument("--no-publish-lidar-tf", action="store_true")
    parser.add_argument(
        "--imu-tf",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
    )
    parser.add_argument("--no-publish-imu-tf", action="store_true")
    parser.add_argument(
        "--initial-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
    )
    parser.add_argument("--deskew-reference-time-sec", type=float, default=0.0)
    parser.add_argument("--score-threshold", type=float)
    parser.add_argument("--enable-open-loop-strict-score-threshold", action="store_true")
    parser.add_argument("--open-loop-strict-min-accepted-gap-sec", type=float, default=15.0)
    parser.add_argument("--open-loop-strict-min-seed-translation-m", type=float, default=100.0)
    parser.add_argument("--open-loop-strict-score-threshold", type=float, default=5.25)
    parser.add_argument("--voxel-leaf-size", type=float)
    parser.add_argument("--scan-max-range", type=float)
    parser.add_argument("--scan-min-range", type=float)
    parser.add_argument("--scan-period", type=float, default=0.1)
    parser.add_argument(
        "--scan-time-range-max-duration-ratio", type=float, default=2.0)
    parser.add_argument("--ndt-resolution", type=float, default=1.0)
    parser.add_argument("--ndt-threads", type=int, default=4)
    parser.add_argument("--ndt-max-iterations", type=int, default=35)
    parser.add_argument("--record-topic")
    parser.add_argument("--bag-rate", type=float, default=1.0)
    parser.add_argument("--bag-start-offset", type=float, default=0.0)
    parser.add_argument("--bag-duration", type=float, default=0.0)
    parser.add_argument(
        "--bag-command",
        default="",
        help="Explicit ros2 bag play command. When set, --bag-path is kept for reports but not input-checked.",
    )
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    parser.add_argument("--post-roll-seconds", type=float, default=3.0)
    parser.add_argument("--ros-domain-id", type=int, default=-1)
    parser.add_argument("--reference-csv", default="")
    parser.add_argument("--max-time-diff", type=float, default=0.1)
    parser.add_argument("--min-imu-active-ratio", type=float, default=0.5)
    parser.add_argument("--min-imu-integrated-samples", type=int, default=1)
    parser.add_argument("--max-imu-fallback-count", type=int, default=0)
    parser.add_argument("--min-imu-seed-source-ratio", type=float, default=0.5)
    parser.add_argument("--min-deskew-applied-ratio", type=float, default=0.1)
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="Extra launch argument appended to every generated system command, e.g. name:=value.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Keep running later modes even if one benchmark or validation step fails.",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Write configs and run_plan.json, print commands, but do not execute benchmarks.",
    )
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="Regenerate comparison.md from an existing output directory without running benchmarks.",
    )
    return parser


def selected_modes(args: argparse.Namespace) -> List[ComparisonMode]:
    names = args.mode if args.mode else ["lidar_only", "imu_preintegration", "deskew"]
    return [MODES[name] for name in names]


def arg_or_profile(args: argparse.Namespace, key: str) -> object:
    value = getattr(args, key, None)
    if value is not None:
        return value
    return config_tool.PROFILE_DEFAULTS[args.profile][key]


def bag_path_error(path_text: str) -> Optional[str]:
    path = Path(path_text).expanduser()
    if not path.exists():
        return f"Bag path does not exist: {path}"
    if path.is_dir() and not (path / "metadata.yaml").exists():
        return f"Bag directory has no metadata.yaml: {path}"
    if path.is_file() and path.name != "metadata.yaml" and path.suffix.lower() not in BAG_FILE_SUFFIXES:
        return f"Bag path should be a rosbag2 directory, metadata.yaml, .db3, or .mcap file: {path}"
    return None


def metadata_path_for_bag(path_text: str) -> Optional[Path]:
    path = Path(path_text).expanduser()
    if path.is_file() and path.name == "metadata.yaml":
        return path
    if path.is_dir():
        metadata_path = path / "metadata.yaml"
        return metadata_path if metadata_path.exists() else None
    if path.is_file() and path.suffix.lower() in BAG_FILE_SUFFIXES:
        metadata_path = path.parent / "metadata.yaml"
        return metadata_path if metadata_path.exists() else None
    return None


def _yaml_scalar_text(value: str) -> str:
    return value.strip().strip("'\"")


def bag_topics_from_metadata(metadata_path: Path) -> Dict[str, str]:
    topics: Dict[str, str] = {}
    pending_name: Optional[str] = None
    with metadata_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped.startswith("name:"):
                pending_name = _yaml_scalar_text(stripped.split(":", 1)[1])
            elif stripped.startswith("type:") and pending_name:
                topics[pending_name] = _yaml_scalar_text(stripped.split(":", 1)[1])
                pending_name = None
    return topics


def _topics_of_type(topics: Dict[str, str], message_type: str) -> str:
    matches = sorted(topic for topic, topic_type in topics.items() if topic_type == message_type)
    return ", ".join(matches) if matches else "none"


def _topic_type_error(
    topics: Dict[str, str],
    topic: str,
    expected_type: str,
    topic_label: str,
    flag_name: str,
    extra_hint: str = "",
) -> Optional[str]:
    actual_type = topics.get(topic)
    if actual_type == expected_type:
        return None
    if actual_type is not None:
        return (
            f"Bag metadata topic {topic} has type {actual_type}, expected {expected_type}. "
            f"Available {topic_label} topics: {_topics_of_type(topics, expected_type)}. "
            f"Pass {flag_name} to match the bag."
            + (f" {extra_hint}" if extra_hint else "")
        )
    return (
        f"Bag metadata has no {expected_type} on {topic}. "
        f"Available {topic_label} topics: {_topics_of_type(topics, expected_type)}. "
        f"Pass {flag_name} to match the bag."
        + (f" {extra_hint}" if extra_hint else "")
    )


def bag_topic_check_errors(args: argparse.Namespace, topics: Dict[str, str]) -> List[str]:
    errors: List[str] = []
    cloud_topic = str(arg_or_profile(args, "cloud_topic"))
    imu_topic = str(arg_or_profile(args, "imu_topic"))
    modes = selected_modes(args)
    require_imu = any(mode.imu_mode != "off" or mode.enable_continuous_time_deskew for mode in modes)

    cloud_error = _topic_type_error(
        topics,
        cloud_topic,
        POINTCLOUD2_TYPE,
        "pointcloud",
        "--cloud-topic",
    )
    if cloud_error is not None:
        errors.append(cloud_error)
    if require_imu:
        imu_error = _topic_type_error(
            topics,
            imu_topic,
            IMU_TYPE,
            "IMU",
            "--imu-topic",
            "Or run only --mode lidar_only.",
        )
        if imu_error is not None:
            errors.append(imu_error)
    return errors


def input_check_errors(args: argparse.Namespace) -> List[str]:
    errors: List[str] = []
    if not args.bag_path:
        errors.append("--bag-path is required unless --report-only is used")
    if not args.map_path:
        errors.append("--map-path is required unless --report-only is used")
    if errors:
        return errors

    if not args.bag_command:
        error = bag_path_error(args.bag_path)
        if error is not None:
            errors.append(error)
        else:
            metadata_path = metadata_path_for_bag(args.bag_path)
            if metadata_path is not None:
                errors.extend(bag_topic_check_errors(args, bag_topics_from_metadata(metadata_path)))

    map_path = Path(config_tool.normalized_map_path(args.map_path))
    if not map_path.exists():
        errors.append(f"Map path does not exist: {map_path}")
    elif map_path.suffix.lower() not in config_tool.SUPPORTED_MAP_SUFFIXES:
        errors.append(f"Map path extension must be .pcd or .ply: {map_path}")

    if args.reference_csv:
        reference_path = Path(args.reference_csv).expanduser()
        if not reference_path.exists():
            errors.append(f"Reference CSV does not exist: {reference_path}")
        elif reference_path.suffix.lower() != ".csv":
            errors.append(f"Reference path extension must be .csv: {reference_path}")
    return errors


def default_record_topic(profile: str) -> str:
    if profile in {"nav2", "mid360"}:
        return "/localization/pose_with_covariance"
    return "/pcl_pose"


def config_args_for_mode(
    args: argparse.Namespace,
    mode: ComparisonMode,
    output_path: Path,
) -> argparse.Namespace:
    return argparse.Namespace(
        map_path=args.map_path,
        output=str(output_path),
        profile=args.profile,
        cloud_topic=args.cloud_topic,
        imu_topic=args.imu_topic,
        lidar_frame=args.lidar_frame,
        imu_frame=args.imu_frame,
        require_cloud_time_field=mode.enable_continuous_time_deskew,
        base_frame=args.base_frame,
        odom_frame=args.odom_frame,
        global_frame=args.global_frame,
        lidar_tf=args.lidar_tf,
        no_publish_lidar_tf=args.no_publish_lidar_tf,
        imu_tf=args.imu_tf,
        no_publish_imu_tf=args.no_publish_imu_tf,
        initial_pose=args.initial_pose,
        imu_mode=mode.imu_mode,
        enable_imu=False,
        enable_continuous_time_deskew=mode.enable_continuous_time_deskew,
        continuous_time_deskew_mode=mode.continuous_time_deskew_mode,
        continuous_time_cloud_stamp_reference="start",
        continuous_time_pose_history_duration_sec=2.0,
        enable_localizability_guard=mode.enable_localizability_guard,
        localizability_min_xy_eigen_ratio=0.05,
        enable_registration_localizability_diagnostics=(
            mode.enable_registration_localizability_diagnostics),
        deskew_reference_time_sec=args.deskew_reference_time_sec,
        score_threshold=args.score_threshold,
        enable_open_loop_strict_score_threshold=args.enable_open_loop_strict_score_threshold,
        open_loop_strict_min_accepted_gap_sec=args.open_loop_strict_min_accepted_gap_sec,
        open_loop_strict_min_seed_translation_m=args.open_loop_strict_min_seed_translation_m,
        open_loop_strict_score_threshold=args.open_loop_strict_score_threshold,
        voxel_leaf_size=args.voxel_leaf_size,
        scan_max_range=args.scan_max_range,
        scan_min_range=args.scan_min_range,
        scan_period=args.scan_period,
        scan_time_range_max_duration_ratio=args.scan_time_range_max_duration_ratio,
        ndt_resolution=args.ndt_resolution,
        ndt_threads=args.ndt_threads,
        ndt_max_iterations=args.ndt_max_iterations,
        overwrite=True,
    )


def write_config(tool_args: argparse.Namespace, output_path: Path) -> None:
    validation_error = config_tool.validate_args(tool_args)
    if validation_error is not None:
        raise ValueError(validation_error)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        config_tool.render_ros_params(config_tool.make_params(tool_args)),
        encoding="utf-8",
    )


def system_command(tool_args: argparse.Namespace, config_path: Path, extra_launch_args: Sequence[str]) -> str:
    parts = shlex.split(config_tool.launch_command(tool_args, config_path.resolve()))
    parts.append("use_sim_time:=true")
    parts.extend(extra_launch_args)
    return config_tool.command_line(parts)


def benchmark_command(
    args: argparse.Namespace,
    run_dir: Path,
    command: str,
) -> str:
    parts: List[object] = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "benchmark_runner",
        "--bag-path",
        args.bag_path,
        "--output-dir",
        run_dir,
        "--system-command",
        command,
        "--record-topic",
        args.record_topic or default_record_topic(args.profile),
        "--bag-rate",
        args.bag_rate,
        "--bag-start-offset",
        args.bag_start_offset,
        "--bag-duration",
        args.bag_duration,
        "--settle-seconds",
        args.settle_seconds,
        "--post-roll-seconds",
        args.post_roll_seconds,
    ]
    if args.bag_command:
        parts.extend(["--bag-command", args.bag_command])
    if args.ros_domain_id >= 0:
        parts.extend(["--ros-domain-id", args.ros_domain_id])
    return config_tool.command_line(parts)


def imu_validation_command(args: argparse.Namespace, mode: ComparisonMode, run_dir: Path) -> Optional[str]:
    if not mode.validate_imu and not mode.require_deskew_applied:
        return None
    parts: List[object] = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "validate_lidar_localization_imu.py",
        "--alignment-csv",
        run_dir / "alignment_status.csv",
        "--output-json",
        run_dir / "imu_validation.json",
        "--output-md",
        run_dir / "imu_validation.md",
    ]
    if mode.validate_imu:
        parts.extend([
            "--min-imu-active-ratio",
            args.min_imu_active_ratio,
            "--min-imu-integrated-samples",
            args.min_imu_integrated_samples,
            "--max-imu-fallback-count",
            args.max_imu_fallback_count,
            "--require-imu-seed-source",
            "--min-imu-seed-source-ratio",
            args.min_imu_seed_source_ratio,
        ])
    else:
        parts.extend([
            "--min-imu-active-ratio", 0.0,
            "--min-imu-integrated-samples", 0,
            "--max-imu-fallback-count", 0,
        ])
    if mode.require_deskew_applied:
        parts.append("--require-deskew-applied")
        parts.extend(["--min-deskew-applied-ratio", args.min_deskew_applied_ratio])
    return config_tool.command_line(parts)


def trajectory_eval_command(args: argparse.Namespace, run_dir: Path) -> Optional[str]:
    if not args.reference_csv:
        return None
    parts: List[object] = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "benchmark_eval_trajectory",
        "--estimated-csv",
        run_dir / "pose_trace.csv",
        "--reference-csv",
        args.reference_csv,
        "--output-json",
        run_dir / "trajectory_eval.json",
        "--max-time-diff",
        args.max_time_diff,
    ]
    return config_tool.command_line(parts)


def compare_command(output_dir: Path) -> str:
    return config_tool.command_line([
        "ros2",
        "run",
        "lidar_localization_ros2",
        "benchmark_compare_runs",
        "--run-dir",
        output_dir,
        "--output-json",
        output_dir / "comparison.json",
    ])


def run_shell(command: str) -> int:
    print(f"$ {command}", flush=True)
    return subprocess.run(["bash", "-lc", command], check=False).returncode


def write_plan(output_dir: Path, plan: Dict[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "run_plan.json").write_text(
        json.dumps(plan, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_command_script(output_dir: Path, plan: Dict[str, object]) -> Path:
    script_path = output_dir / "run_commands.sh"
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        "# Generated by run_lidar_localization_imu_comparison.py",
        "# Run from a shell where ROS 2 and this workspace are sourced.",
        "",
    ]
    for run in plan.get("runs", []):
        if not isinstance(run, dict):
            continue
        mode = str(run.get("mode", ""))
        lines.extend([
            f"echo '== {mode}: benchmark =='",
            str(run.get("benchmark_command", "")),
        ])
        if run.get("imu_validation_command"):
            lines.extend([
                f"echo '== {mode}: IMU validation =='",
                str(run["imu_validation_command"]),
            ])
        if run.get("trajectory_eval_command"):
            lines.extend([
                f"echo '== {mode}: trajectory evaluation =='",
                str(run["trajectory_eval_command"]),
            ])
        lines.append("")
    lines.extend([
        "echo '== aggregate comparison =='",
        str(plan.get("compare_command", "")),
        "echo '== markdown report =='",
        config_tool.command_line([
            "ros2",
            "run",
            "lidar_localization_ros2",
            "run_lidar_localization_imu_comparison.py",
            "--output-dir",
            plan.get("output_dir", output_dir),
            "--report-only",
        ]),
        "",
    ])
    script_path.write_text("\n".join(lines), encoding="utf-8")
    script_path.chmod(0o755)
    return script_path


def load_json_if_exists(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    return data if isinstance(data, dict) else {}


def format_value(value: Any, digits: int = 3) -> str:
    if value is None or value == "":
        return "n/a"
    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}"
    try:
        return f"{float(str(value)):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def format_percent(value: Any) -> str:
    if value is None or value == "":
        return "n/a"
    try:
        return f"{100.0 * float(value):.1f}%"
    except (TypeError, ValueError):
        return str(value)


def numeric_value(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def format_delta(value: Any, baseline: Any) -> str:
    current = numeric_value(value)
    base = numeric_value(baseline)
    if current is None or base is None:
        return "n/a"
    delta = current - base
    sign = "+" if delta > 0.0 else ""
    return f"{sign}{delta:.3f}"


def pose_coverage_warning(mode: str, run: Dict[str, Any]) -> str:
    pose_trace = run.get("pose_trace", {}) if isinstance(run.get("pose_trace"), dict) else {}
    ratio = numeric_value(pose_trace.get("requested_duration_ratio"))
    last_elapsed = numeric_value(pose_trace.get("last_elapsed_sec"))
    timing = run.get("timing_sec", {}) if isinstance(run.get("timing_sec"), dict) else {}
    requested = numeric_value(timing.get("requested_bag_duration_seconds"))
    if ratio is None or requested is None or requested <= 0.0 or ratio >= 0.8:
        return ""
    return (
        f"`{mode}` published poses through {last_elapsed:.1f}s of the requested "
        f"{requested:.1f}s replay; RMSE only covers those published pose samples."
    )


def imu_validation_level(imu_validation: Dict[str, Any]) -> str:
    checks = imu_validation.get("checks", [])
    if not checks:
        return "n/a"
    if any(isinstance(check, dict) and check.get("level") == "FAIL" for check in checks):
        return "FAIL"
    if any(isinstance(check, dict) and check.get("level") == "WARN" for check in checks):
        return "WARN"
    return "OK"


def comparison_run_by_mode(comparison: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    indexed: Dict[str, Dict[str, Any]] = {}
    for run in comparison.get("runs", []):
        if not isinstance(run, dict):
            continue
        run_dir = Path(str(run.get("run_dir", ""))).name
        backend_hint = str(run.get("backend_hint", ""))
        if run_dir:
            indexed[run_dir] = run
        if backend_hint:
            indexed[backend_hint] = run
    return indexed


def render_markdown_report(
    output_dir: Path,
    plan: Dict[str, object],
    comparison: Dict[str, Any],
) -> str:
    by_mode = comparison_run_by_mode(comparison)
    baseline_trajectory = (
        by_mode.get("lidar_only", {}).get("trajectory_eval", {})
        if isinstance(by_mode.get("lidar_only", {}).get("trajectory_eval"), dict)
        else {}
    )
    has_reference_eval = any(
        isinstance(run.get("trajectory_eval"), dict) and bool(run.get("trajectory_eval"))
        for run in by_mode.values()
    )
    rows = [
        "| Mode | Trans RMSE m | Delta trans | Rot RMSE deg | Delta rot | Matched | Pose rows | Last pose s | IMU check | IMU active | IMU fallback | Deskew applied | Align rows | Non-OK | Align median ms | Fitness median | CPU max % | RSS max MB |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    coverage_warnings: List[str] = []
    for planned_run in plan.get("runs", []):
        if not isinstance(planned_run, dict):
            continue
        mode = str(planned_run.get("mode", ""))
        run = by_mode.get(mode, {})
        trajectory = run.get("trajectory_eval", {}) if isinstance(run.get("trajectory_eval"), dict) else {}
        pose_trace = run.get("pose_trace", {}) if isinstance(run.get("pose_trace"), dict) else {}
        alignment = run.get("alignment", {}) if isinstance(run.get("alignment"), dict) else {}
        resource = (
            run.get("resource_monitor", {})
            if isinstance(run.get("resource_monitor"), dict)
            else {}
        )
        imu_validation = load_json_if_exists(output_dir / mode / "imu_validation.json")
        imu_summary = (
            imu_validation.get("summary", {})
            if isinstance(imu_validation.get("summary"), dict)
            else {}
        )
        rows.append(
            "| "
            + " | ".join([
                mode,
                format_value(trajectory.get("translation_rmse_m")),
                format_delta(
                    trajectory.get("translation_rmse_m"),
                    baseline_trajectory.get("translation_rmse_m"),
                ),
                format_value(trajectory.get("rotation_rmse_deg")),
                format_delta(
                    trajectory.get("rotation_rmse_deg"),
                    baseline_trajectory.get("rotation_rmse_deg"),
                ),
                format_value(trajectory.get("matched_sample_count"), digits=0),
                format_value(pose_trace.get("sample_count"), digits=0),
                format_value(pose_trace.get("last_elapsed_sec"), digits=1),
                imu_validation_level(imu_validation),
                format_percent(imu_summary.get("imu_active_ratio")),
                format_value(imu_summary.get("imu_fallback_count"), digits=0),
                format_percent(imu_summary.get("continuous_time_deskew_applied_ratio")),
                format_value(alignment.get("row_count"), digits=0),
                format_value(alignment.get("non_ok_row_count"), digits=0),
                format_value(
                    (
                        1000.0 * float(alignment["alignment_time_median_sec"])
                        if alignment.get("alignment_time_median_sec") is not None
                        else None
                    )
                ),
                format_value(alignment.get("fitness_median")),
                format_value(resource.get("cpu_percent_max")),
                format_value(resource.get("rss_mb_max")),
            ])
            + " |"
        )
        warning = pose_coverage_warning(mode, run)
        if warning:
            coverage_warnings.append(warning)

    interpretation = [
        "## Interpretation",
        "",
    ]
    if has_reference_eval:
        interpretation.append(
            "Negative delta RMSE means the mode improved against `lidar_only`; positive delta means it regressed."
        )
    else:
        interpretation.append(
            "No `reference_csv` trajectory evaluation was found, so this report is runtime health only. It cannot prove accuracy improvement."
        )
    if "imu_preintegration" in by_mode:
        imu_json = load_json_if_exists(output_dir / "imu_preintegration" / "imu_validation.json")
        interpretation.append(
            f"IMU preintegration runtime check: `{imu_validation_level(imu_json)}`."
        )
    if "deskew" in by_mode:
        deskew_json = load_json_if_exists(output_dir / "deskew" / "imu_validation.json")
        interpretation.append(
            f"Continuous-time deskew runtime check: `{imu_validation_level(deskew_json)}`."
        )
    if coverage_warnings:
        interpretation.extend(["", "Coverage warnings:"])
        interpretation.extend(f"- {warning}" for warning in coverage_warnings)

    lines = [
        "# LiDAR Localization IMU Comparison",
        "",
        f"- Bag: `{plan.get('bag_path', 'n/a')}`",
        f"- Map: `{plan.get('map_path', 'n/a')}`",
        f"- Profile: `{plan.get('profile', 'n/a')}`",
        f"- Output: `{plan.get('output_dir', output_dir)}`",
        "",
        "This report checks runtime behavior and benchmark metrics. Treat the deskew row as experimental until validated against ground truth.",
        "",
        *rows,
        "",
        *interpretation,
        "",
        "## Artifacts",
        "",
    ]
    for planned_run in plan.get("runs", []):
        if not isinstance(planned_run, dict):
            continue
        mode = str(planned_run.get("mode", ""))
        lines.extend([
            f"- `{mode}`: `{planned_run.get('run_dir', '')}`",
        ])
    lines.extend([
        "",
        "Generated files:",
        "",
        f"- `run_plan.json`",
        f"- `comparison.json`",
        f"- `comparison.md`",
        "",
    ])
    return "\n".join(lines)


def write_markdown_report(output_dir: Path, plan: Dict[str, object]) -> Path:
    comparison = load_json_if_exists(output_dir / "comparison.json")
    report_path = output_dir / "comparison.md"
    report_path.write_text(render_markdown_report(output_dir, plan, comparison), encoding="utf-8")
    return report_path


def load_plan(output_dir: Path) -> Dict[str, object]:
    plan_path = output_dir / "run_plan.json"
    if not plan_path.exists():
        raise FileNotFoundError(f"run_plan.json not found: {plan_path}")
    with plan_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"run_plan.json is not an object: {plan_path}")
    return data


def print_input_errors(errors: Sequence[str]) -> None:
    for error in errors:
        print(f"input error: {error}", file=sys.stderr)
    print("Use --print-only to inspect commands without checking inputs.", file=sys.stderr)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.report_only:
        try:
            plan = load_plan(output_dir)
            report_path = write_markdown_report(output_dir, plan)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            print(f"report error: {error}", file=sys.stderr)
            return 2
        print(f"Wrote {report_path}")
        return 0

    required_errors = input_check_errors(args)
    if required_errors and not args.print_only:
        print_input_errors(required_errors)
        return 2

    runs = []
    for mode in selected_modes(args):
        run_dir = output_dir / mode.name
        config_path = run_dir / "localization.yaml"
        tool_args = config_args_for_mode(args, mode, config_path)
        write_config(tool_args, config_path)
        launch = system_command(tool_args, config_path, args.launch_arg)
        run = {
            "mode": mode.name,
            "run_dir": str(run_dir),
            "config_path": str(config_path),
            "system_command": launch,
            "benchmark_command": benchmark_command(args, run_dir, launch),
            "imu_validation_command": imu_validation_command(args, mode, run_dir),
            "trajectory_eval_command": trajectory_eval_command(args, run_dir),
        }
        runs.append(run)

    plan = {
        "bag_path": args.bag_path,
        "map_path": args.map_path,
        "profile": args.profile,
        "output_dir": str(output_dir),
        "runs": runs,
        "compare_command": compare_command(output_dir),
        "report_path": str(output_dir / "comparison.md"),
        "command_script_path": str(output_dir / "run_commands.sh"),
    }
    write_plan(output_dir, plan)
    write_command_script(output_dir, plan)

    if args.print_only:
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 0

    input_errors = input_check_errors(args)
    if input_errors:
        print_input_errors(input_errors)
        return 2

    failed = False
    for run in runs:
        run_dir = Path(str(run["run_dir"]))
        run_dir.mkdir(parents=True, exist_ok=True)
        for key in ("benchmark_command", "imu_validation_command", "trajectory_eval_command"):
            command = run.get(key)
            if not command:
                continue
            code = run_shell(str(command))
            if code != 0:
                failed = True
                if not args.continue_on_failure:
                    return code

    compare_code = run_shell(compare_command(output_dir))
    if compare_code != 0:
        failed = True
    report_path = write_markdown_report(output_dir, plan)
    print(f"Wrote {report_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
