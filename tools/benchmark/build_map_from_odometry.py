#!/usr/bin/env python3
"""Accumulate a PointCloud2 bag into a point cloud map along an odometry trajectory.

Builds the JEPLO self maps of the Go2 suite, which the dataset no longer
publishes (it shipped ``maps_loc/<sequence>/map.pcd``). Every scan is deskewed
with the trajectory (poses of the cloud frame in a gravity-aligned odometry
frame, TUM) interpolated at each point's absolute ``timestamp``, then the cloud
is voxel-filtered. Move the result
into the ground-truth frame with tools/benchmark/align_map_to_ground_truth.py.

usage: build_map_from_odometry.py --bag BAG --trajectory odom.tum --out map.pcd
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

POINT = np.dtype(
    {
        "names": ["x", "y", "z", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f8"],
        "offsets": [0, 4, 8, 16],
        "itemsize": 24,
    }
)


def load_tum(path: Path) -> np.ndarray:
    data = np.loadtxt(path, ndmin=2)
    return data[np.argsort(data[:, 0])]


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrices of (N, 4) quaternions in x, y, z, w order."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    return np.stack(
        [
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y),
        ],
        axis=1,
    ).reshape(-1, 3, 3)


def slerp(q0: np.ndarray, q1: np.ndarray, t: np.ndarray) -> np.ndarray:
    dot = np.sum(q0 * q1, axis=1)
    q1 = np.where(dot[:, None] < 0.0, -q1, q1)
    dot = np.abs(dot)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    small = theta < 1.0e-6
    sin = np.where(small, 1.0, np.sin(theta))
    w0 = np.where(small, 1.0 - t, np.sin((1.0 - t) * theta) / sin)
    w1 = np.where(small, t, np.sin(t * theta) / sin)
    return w0[:, None] * q0 + w1[:, None] * q1


def interpolate_poses(trajectory: np.ndarray, stamps: np.ndarray):
    """Rotations and translations at stamps (clamped to the trajectory ends)."""
    times = trajectory[:, 0]
    stamps = np.clip(stamps, times[0], times[-1])
    upper = np.clip(np.searchsorted(times, stamps), 1, len(times) - 1)
    lower = upper - 1
    span = np.maximum(times[upper] - times[lower], 1.0e-9)
    t = (stamps - times[lower]) / span
    translation = trajectory[lower, 1:4] + t[:, None] * (
        trajectory[upper, 1:4] - trajectory[lower, 1:4]
    )
    quaternion = slerp(trajectory[lower, 4:8], trajectory[upper, 4:8], t)
    return quat_to_matrix(quaternion), translation


def voxel_filter(points: np.ndarray, leaf: float) -> np.ndarray:
    keys = np.floor(points / leaf).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.zeros((counts.shape[0], 3))
    np.add.at(sums, inverse.ravel(), points)
    return sums / counts[:, None]


def write_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n"
        "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--every", type=int, default=2, help="use every N-th cloud")
    parser.add_argument("--min-range", type=float, default=0.8)
    parser.add_argument("--max-range", type=float, default=40.0)
    parser.add_argument("--voxel", type=float, default=0.05)
    args = parser.parse_args(argv)

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    trajectory = load_tum(args.trajectory)
    reader = rosbag2_py.SequentialReader()
    info = rosbag2_py.Info().read_metadata(str(args.bag), "")
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(args.bag), storage_id=info.storage_identifier
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    chunks = []
    chunk_points = 0
    index = used = 0
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        index += 1
        if (index - 1) % max(1, args.every):
            continue
        msg = deserialize_message(data, PointCloud2)
        cloud = np.frombuffer(bytes(msg.data), dtype=POINT)
        if cloud.size == 0:
            continue
        times = cloud["timestamp"]
        if times.max() < trajectory[0, 0] or times.min() > trajectory[-1, 0]:
            continue
        xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float64)
        distance = np.linalg.norm(xyz, axis=1)
        keep = (distance >= args.min_range) & (distance <= args.max_range)
        xyz, times = xyz[keep], times[keep]
        rotation, translation = interpolate_poses(trajectory, times)
        world = np.einsum("nij,nj->ni", rotation, xyz) + translation
        chunks.append(world)
        chunk_points += len(world)
        used += 1
        if chunk_points > 5_000_000:
            chunks = [voxel_filter(np.concatenate(chunks), args.voxel)]
            chunk_points = len(chunks[0])
    points = voxel_filter(np.concatenate(chunks), args.voxel)
    write_pcd(args.out, points)
    print(f"{args.out}: {len(points)} points from {used} clouds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
