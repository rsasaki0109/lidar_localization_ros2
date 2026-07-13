#!/usr/bin/env python3
"""Prepare 30 s benchmark manifests for every Koide dataset bag."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sqlite3

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SEQUENCES = {
    "indoor_easy_01": ("indoor_easy_01", "indoor_easy", True),
    "indoor_easy_02": ("indoor_easy_02", "indoor_easy", True),
    "indoor_hard_01": ("indoor_hard_01", "indoor_hard", True),
    "indoor_kidnap_01": ("indoor_kidnap_01", "indoor_hard", True),
    "indoor_kidnap_02": ("indoor_kidnap_02", "indoor_hard", True),
    "outdoor_hard_01a": ("outdoor_hard_01", "outdoor_hard", False),
    "outdoor_hard_01b": ("outdoor_hard_01", "outdoor_hard", False),
    "outdoor_hard_02a": ("outdoor_hard_02", "outdoor_hard", False),
    "outdoor_hard_02b": ("outdoor_hard_02", "outdoor_hard", False),
    "outdoor_kidnap_a": ("outdoor_kidnap", "outdoor_kidnap", False),
    "outdoor_kidnap_b": ("outdoor_kidnap", "outdoor_kidnap", False),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument(
        "--indoor-imu",
        action="store_true",
        help="Enable indoor IMU preintegration with the published depth-camera-to-IMU extrinsic",
    )
    return parser.parse_args()


def load_tum(path: Path) -> list[tuple[float, ...]]:
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        values = tuple(float(value) for value in line.split()[:8])
        if len(values) == 8:
            rows.append(values)
    if not rows:
        raise RuntimeError(f"no trajectory poses: {path}")
    return rows


def write_reference(path: Path, rows: list[tuple[float, ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "message_index", "stamp_sec", "frame_id",
                "position_x", "position_y", "position_z",
                "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                "covariance",
            ]
        )
        for index, row in enumerate(rows):
            stamp, x, y, z, qx, qy, qz, qw = row
            writer.writerow(
                [
                    index, f"{stamp:.9f}", "map",
                    f"{x:.10f}", f"{y:.10f}", f"{z:.10f}",
                    f"{qx:.10f}", f"{qy:.10f}", f"{qz:.10f}", f"{qw:.10f}", "",
                ]
            )


def write_initial_pose(path: Path, pose: tuple[float, ...]) -> None:
    _, x, y, z, qx, qy, qz, qw = pose
    data = {
        "/**": {
            "ros__parameters": {
                "set_initial_pose": True,
                "initial_pose_x": x,
                "initial_pose_y": y,
                "initial_pose_z": z,
                "initial_pose_qx": qx,
                "initial_pose_qy": qy,
                "initial_pose_qz": qz,
                "initial_pose_qw": qw,
            }
        }
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def bag_info(metadata_path: Path) -> tuple[float, float]:
    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = data["rosbag2_bagfile_information"]
    start = float(info["starting_time"]["nanoseconds_since_epoch"]) / 1e9
    duration = float(info["duration"]["nanoseconds"]) / 1e9
    return start, duration


def find_metadata(sequence_dir: Path) -> Path:
    candidates = list(sequence_dir.glob("metadata.yaml")) + list(sequence_dir.glob("*/metadata.yaml"))
    if len(candidates) != 1:
        raise RuntimeError(f"expected one metadata.yaml under {sequence_dir}, got {candidates}")
    return candidates[0]


def first_cloud_header_stamp(metadata_path: Path, topic: str, record_start: float) -> float:
    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = data["rosbag2_bagfile_information"]
    database = metadata_path.parent / info["relative_file_paths"][0]
    with sqlite3.connect(database) as connection:
        type_row = connection.execute(
            "SELECT type FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if type_row is None:
            raise RuntimeError(f"missing topic {topic} in {database}")
        message_row = connection.execute(
            "SELECT data FROM messages JOIN topics ON messages.topic_id = topics.id "
            "WHERE topics.name = ? AND messages.timestamp >= ? "
            "ORDER BY messages.timestamp LIMIT 1",
            (topic, round(record_start * 1e9)),
        ).fetchone()
    if message_row is None:
        raise RuntimeError(f"no {topic} message at or after {record_start:.9f}")
    message = deserialize_message(message_row[0], get_message(type_row[0]))
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9


def main() -> int:
    args = parse_args()
    data_dir = args.data_dir.resolve()
    output_root = args.output_root.resolve()
    repo_root = Path(__file__).resolve().parents[1]
    assets_dir = output_root / "assets"
    manifests_dir = output_root / "manifests"
    runs_dir = output_root / "runs"
    assets_dir.mkdir(parents=True, exist_ok=True)
    manifests_dir.mkdir(parents=True, exist_ok=True)
    runs_dir.mkdir(parents=True, exist_ok=True)

    for index, (sequence, (gt_name, map_name, indoor)) in enumerate(SEQUENCES.items()):
        metadata = find_metadata(data_dir / "sequences" / sequence)
        bag_start, bag_duration = bag_info(metadata)
        start_offset = 0.6 if sequence == "outdoor_kidnap_a" else 0.0
        if bag_duration < start_offset + args.duration:
            raise RuntimeError(f"{sequence}: bag is too short for {args.duration:.1f} s")

        poses = load_tum(data_dir / "gt" / f"traj_lidar_{gt_name}.txt")
        play_start = bag_start + start_offset
        play_end = play_start + args.duration
        if poses[0][0] > play_start + 0.1 or poses[-1][0] < play_end - 0.1:
            raise RuntimeError(f"{sequence}: GT does not cover [{play_start}, {play_end}]")
        reference = assets_dir / f"{sequence}_reference.csv"
        initial_yaml = assets_dir / f"{sequence}_initial_pose.yaml"
        write_reference(reference, poses)

        if indoor:
            base_param = repo_root / "param" / "koide_indoor_ndt.yaml"
            cloud_topic, imu_topic = "/points2/decompressed", "/imu"
            overrides = {
                "use_imu": False,
                "use_imu_preintegration": args.indoor_imu,
                "imu_preintegration_use_base_frame_transform": args.indoor_imu,
                "enable_scan_voxel_filter": True,
                "voxel_leaf_size": 0.7,
                "base_frame_id": "depth_camera_link",
                "score_threshold": 15.0,
            }
            if args.indoor_imu:
                overrides.update(
                    {
                        "imu_prediction_correction_guard_warmup_accepts": 50,
                        "imu_prediction_correction_guard_translation_m": 0.5,
                        "imu_prediction_correction_guard_yaw_deg": 4.0,
                    }
                )
            launch_args = [
                "use_sim_time:=true",
                "use_dataset_tf_tree:=false",
                "base_frame_id:=depth_camera_link",
                "lidar_frame_id:=depth_camera_link",
                "publish_lidar_tf:=false",
                f"use_imu_preintegration:={'true' if args.indoor_imu else 'false'}",
            ]
            if args.indoor_imu:
                launch_args.extend(
                    [
                        "imu_preintegration_use_base_frame_transform:=true",
                        "publish_imu_tf:=true",
                        "imu_frame_id:=imu_link",
                        "imu_tf_x:=0.003463566434548747",
                        "imu_tf_y:=0.0041740033449125195",
                        "imu_tf_z:=-0.05071645628165228",
                        "imu_tf_roll:=-0.030374946857",
                        "imu_tf_pitch:=1.471526122045",
                        "imu_tf_yaw:=1.541169478742",
                    ]
                )
        else:
            base_param = repo_root / "param" / "nav2_ndt_urban.yaml"
            cloud_topic, imu_topic = "/livox/points", "/livox/imu"
            overrides = {
                "use_imu": False,
                "use_imu_preintegration": True,
                "enable_scan_voxel_filter": True,
                "voxel_leaf_size": 0.5,
                "base_frame_id": "livox_frame",
                "scan_min_range": 1.0,
                "scan_max_range": 100.0,
                "score_threshold": 6.0,
            }
            launch_args = [
                "use_sim_time:=true",
                "base_frame_id:=livox_frame",
                "lidar_frame_id:=livox_frame",
                "use_imu_preintegration:=true",
            ]

        first_cloud_stamp = first_cloud_header_stamp(
            metadata, cloud_topic, bag_start + start_offset
        )
        initial_stamp = first_cloud_stamp + 0.05
        initial_pose = next((pose for pose in poses if pose[0] >= initial_stamp), None)
        if initial_pose is None:
            raise RuntimeError(f"{sequence}: no initial pose at {initial_stamp}")
        write_initial_pose(initial_yaml, initial_pose)

        manifest = {
            "mode": "run",
            "dataset": {
                "bag_path": str(metadata.parent),
                "map_path": str(data_dir / f"map_{map_name}.ply"),
                "reference_csv": str(reference),
                "initial_pose_yaml": str(initial_yaml),
                "cloud_topic": cloud_topic,
                "imu_topic": imu_topic,
            },
            "localizer": {
                "base_param_yaml": str(base_param),
                "param_overrides": overrides,
                "launch_args": launch_args,
            },
            "benchmark": {
                "output_dir": str(runs_dir / sequence),
                "ros_domain_id": 190 + index,
                "bag_duration": args.duration,
                "bag_start_offset": start_offset,
                "settle_seconds": 5,
                "post_roll_seconds": 1,
                "max_time_diff": 0.05,
                "record_qos_reliability": "reliable",
                "record_qos_durability": "volatile",
                "record_use_sim_time": True,
            },
        }
        manifest_path = manifests_dir / f"{sequence}.yaml"
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        mode = (
            "extrinsic-corrected IMU preintegration"
            if indoor and args.indoor_imu
            else "LiDAR-only"
            if indoor
            else "guarded IMU preintegration"
        )
        print(
            f"{sequence}: {mode}, offset={start_offset:.1f}s, "
            f"first_cloud={first_cloud_stamp:.3f}, initial={initial_pose[0]:.3f}, "
            f"manifest={manifest_path}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
