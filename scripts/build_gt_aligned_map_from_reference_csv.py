#!/usr/bin/env python3

"""Build a GT-aligned point cloud map from a rosbag2 PointCloud2 topic."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
import open3d as o3d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores
from rosbags.typesys import get_typestore


TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
SUPPORTED_OUTPUT_SUFFIXES = {".ply", ".pcd"}


@dataclass
class BuildStats:
    scan_count_seen: int = 0
    used_scans: int = 0
    skipped_no_pose: int = 0
    raw_points: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a GT-aligned map by transforming PointCloud2 scans with reference poses."
    )
    parser.add_argument("--bag-path", required=True, help="Path to the rosbag2 directory")
    parser.add_argument("--reference-csv", required=True, help="Reference CSV with pose rows in map frame")
    parser.add_argument(
        "--output-map",
        required=True,
        help=(
            "Destination .ply/.pcd path. Prefer .ply for benchmark/runtime use; this helper writes "
            "float32 binary PLY instead of Open3D's default double-precision PLY fields."
        ),
    )
    parser.add_argument("--cloud-topic", default="/velodyne_points", help="PointCloud2 topic name")
    parser.add_argument("--point-stride", type=int, default=10, help="Keep every Nth scan")
    parser.add_argument("--voxel-size", type=float, default=0.5, help="Voxel downsample size in meters")
    parser.add_argument("--min-range", type=float, default=1.0, help="Minimum sensor-frame range")
    parser.add_argument("--max-range", type=float, default=100.0, help="Maximum sensor-frame range")
    parser.add_argument("--max-time-diff", type=float, default=0.05, help="Pose matching tolerance")
    parser.add_argument("--print-every", type=int, default=20, help="Print progress every N used scans")
    return parser.parse_args()


def load_reference_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    stamps = []
    positions = []
    quaternions = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            stamps.append(float(row["stamp_sec"]))
            positions.append(
                [float(row["position_x"]), float(row["position_y"]), float(row["position_z"])]
            )
            quaternions.append(
                [
                    float(row["orientation_x"]),
                    float(row["orientation_y"]),
                    float(row["orientation_z"]),
                    float(row["orientation_w"]),
                ]
            )
    if not stamps:
        raise RuntimeError(f"No rows found in reference CSV: {path}")
    return (
        np.asarray(stamps, dtype=np.float64),
        np.asarray(positions, dtype=np.float64),
        np.asarray(quaternions, dtype=np.float64),
    )


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def nearest_pose(
    stamp_sec: float,
    reference_stamps: np.ndarray,
    reference_positions: np.ndarray,
    reference_quaternions: np.ndarray,
    max_time_diff: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    index = int(np.searchsorted(reference_stamps, stamp_sec))
    candidates = []
    if index < len(reference_stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None
    best_index = min(candidates, key=lambda idx: abs(reference_stamps[idx] - stamp_sec))
    if abs(reference_stamps[best_index] - stamp_sec) > max_time_diff:
        return None
    return reference_positions[best_index], reference_quaternions[best_index]


def pointcloud2_xyz_array(message) -> np.ndarray:
    field_offsets = {field.name: int(field.offset) for field in message.fields}
    for field_name in ("x", "y", "z"):
        if field_name not in field_offsets:
            raise RuntimeError(f"PointCloud2 is missing required field: {field_name}")

    endian = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
            "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
            "itemsize": int(message.point_step),
        }
    )
    array = np.frombuffer(bytes(message.data), dtype=dtype, count=int(message.width * message.height))
    return np.column_stack([array["x"], array["y"], array["z"]]).astype(np.float32, copy=False)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(points) == 0:
        return points
    quantized = np.floor(points / voxel_size).astype(np.int64)
    _, keep_indices = np.unique(quantized, axis=0, return_index=True)
    keep_indices.sort()
    return points[keep_indices]


def filter_sensor_points(xyz_sensor: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    if len(xyz_sensor) == 0:
        return xyz_sensor
    ranges = np.linalg.norm(xyz_sensor, axis=1)
    valid = np.isfinite(xyz_sensor).all(axis=1)
    valid &= ranges >= min_range
    valid &= ranges <= max_range
    return xyz_sensor[valid]


def write_binary_float32_ply(path: Path, points: np.ndarray) -> None:
    xyz = np.asarray(points, dtype=np.float32, order="C")
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment Generated by build_gt_aligned_map_from_reference_csv.py\n"
        f"element vertex {xyz.shape[0]}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(xyz.astype("<f4", copy=False).tobytes())


def write_output_map(path: Path, points: np.ndarray) -> None:
    suffix = path.suffix.lower()
    if suffix not in SUPPORTED_OUTPUT_SUFFIXES:
        supported = ", ".join(sorted(SUPPORTED_OUTPUT_SUFFIXES))
        raise RuntimeError(f"Unsupported output suffix {suffix!r}; expected one of: {supported}")

    if suffix == ".ply":
        # Open3D writes x/y/z as double-precision PLY properties, but this stack's PCL loader only
        # behaves reliably with float32 PLY fields (`property float x/y/z`), like the CloudCompare
        # maps already used elsewhere in the repo.
        write_binary_float32_ply(path, points)
        return

    print(
        "warning: .pcd output is kept for inspection tools, but current benchmark/runtime validation "
        "in this repo prefers float32 .ply maps",
        file=sys.stderr,
    )
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    if not o3d.io.write_point_cloud(str(path), point_cloud):
        raise RuntimeError(f"Failed to write point cloud: {path}")


def print_summary(
    bag_path: Path,
    reference_csv: Path,
    cloud_topic: str,
    voxel_size: float,
    output_map: Path,
    points: np.ndarray,
    stats: BuildStats,
) -> None:
    print(f"bag_path: {bag_path}")
    print(f"reference_csv: {reference_csv}")
    print(f"cloud_topic: {cloud_topic}")
    print(f"scan_count_seen: {stats.scan_count_seen}")
    print(f"used_scans: {stats.used_scans}")
    print(f"skipped_no_pose: {stats.skipped_no_pose}")
    print(f"raw_points: {stats.raw_points}")
    print(f"voxel_size: {voxel_size}")
    print(f"points_after_voxel: {int(points.shape[0])}")
    print(f"map_bounds_min: {points.min(axis=0).tolist()}")
    print(f"map_bounds_max: {points.max(axis=0).tolist()}")
    print(f"output_map: {output_map}")


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag_path).expanduser().resolve()
    reference_csv = Path(args.reference_csv).expanduser().resolve()
    output_map = Path(args.output_map).expanduser().resolve()

    reference_stamps, reference_positions, reference_quaternions = load_reference_csv(reference_csv)

    reader = Reader(str(bag_path))
    reader.open()

    stats = BuildStats()
    chunks: list[np.ndarray] = []

    for connection, _, rawdata in reader.messages():
        if connection.topic != args.cloud_topic:
            continue

        stats.scan_count_seen += 1
        if args.point_stride > 1 and (stats.scan_count_seen - 1) % args.point_stride != 0:
            continue

        message = TYPESTORE.deserialize_cdr(rawdata, connection.msgtype)
        stamp_sec = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

        pose = nearest_pose(
            stamp_sec,
            reference_stamps,
            reference_positions,
            reference_quaternions,
            float(args.max_time_diff),
        )
        if pose is None:
            stats.skipped_no_pose += 1
            continue

        xyz_sensor = pointcloud2_xyz_array(message)
        xyz_sensor = filter_sensor_points(
            xyz_sensor,
            float(args.min_range),
            float(args.max_range),
        )
        if len(xyz_sensor) == 0:
            continue

        position, quaternion = pose
        rotation = quaternion_to_rotation_matrix(quaternion)
        xyz_map = (xyz_sensor @ rotation.T) + position
        xyz_map = xyz_map.astype(np.float32, copy=False)
        chunks.append(xyz_map)
        stats.used_scans += 1
        stats.raw_points += int(xyz_map.shape[0])

        if args.print_every > 0 and stats.used_scans % args.print_every == 0:
            print(f"used_scans={stats.used_scans} raw_points={stats.raw_points}")

    reader.close()

    if not chunks:
        raise RuntimeError("No scans were transformed into the map.")

    points = np.vstack(chunks)
    points = voxel_downsample(points, float(args.voxel_size))

    output_map.parent.mkdir(parents=True, exist_ok=True)
    write_output_map(output_map, points)
    print_summary(
        bag_path,
        reference_csv,
        args.cloud_topic,
        float(args.voxel_size),
        output_map,
        points,
        stats,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
