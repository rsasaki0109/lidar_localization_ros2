#!/usr/bin/env python3
"""Prepare a Boreas localization fixture from the public bucket (s3://boreas).

Writes into OUT_DIR:
  bag/               rosbag2 (sqlite3) of one window of the localization sequence:
                     /velodyne_points (frame velodyne), voxel-downsampled per scan, and
                     /imu/data (frame velodyne, 200 Hz, with gravity) from imu_raw.csv
  map.ply            binary little-endian float32 x y z intensity: scans of another
                     sequence of the same route at their ground-truth poses
  reference.tum      ground-truth lidar poses of the window (t x y z qx qy qz qw)
  preparation.json   the inputs, counts and conventions

The ground truth is applanix/lidar_poses.csv: the lidar pose in a local ENU frame shared
by the sequences. Its rotation is (Rz(heading) Ry(pitch) Rx(roll))^T: two scans 3 s apart
overlap at 0.54 with it, at 0.41 with the next best convention, and at 0.06 with the
active Rz Ry Rx of the converter removed in 6825f05.

applanix/imu_raw.csv holds the raw IMU (gyro rad/s, accelerometer m/s^2 with gravity) in
the Applanix frame turned by 180 deg about (1, -1, 0) / sqrt(2): x' = -y, y' = -x,
z' = -z. In the lidar frame (calib/T_applanix_lidar.txt) its yaw rate follows the
ground-truth heading rate (correlation 1.0 in a U-turn) and, parked, its acceleration
is (0.094, 0.323, 9.794) m/s^2 against (0.092, 0.327, 9.801) from the ground-truth
attitude. imu.csv is post-processed and has gravity removed.

usage: boreas_prepare.py OUT_DIR [--loc-seq SEQ] [--start S] [--duration S] [--map-seq SEQ]
"""

import argparse
import json
import math
import re
import time
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

BUCKET = "https://boreas.s3.amazonaws.com"
POINTS = "/velodyne_points"
IMU = "/imu/data"
FRAME = "velodyne"
# imu_raw.csv axes in the Applanix frame (see the module docstring).
RAW_TO_APPLANIX = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], dtype=float)


def fetch(url, attempts=5):
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(url, timeout=120) as response:
                return response.read()
        except OSError:
            if attempt == attempts - 1:
                raise
            time.sleep(2**attempt)
    raise AssertionError("unreachable")


def list_keys(prefix):
    keys, token = [], None
    while True:
        query = {"list-type": "2", "prefix": prefix}
        if token:
            query["continuation-token"] = token
        body = fetch(f"{BUCKET}/?{urllib.parse.urlencode(query)}").decode()
        keys += re.findall(r"<Key>([^<]*)</Key>", body)
        match = re.search(
            r"<NextContinuationToken>([^<]*)</NextContinuationToken>", body
        )
        if not match:
            return keys
        token = match.group(1)


def cached(cache, key):
    path = cache / key
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(fetch(f"{BUCKET}/{key}"))
    return path


def lidar_poses(cache, sequence):
    """Rows of applanix/lidar_poses.csv with the lidar scan key of each row."""
    rows = np.loadtxt(
        cached(cache, f"{sequence}/applanix/lidar_poses.csv"), delimiter=",", skiprows=1
    )
    keys = sorted(
        (k for k in list_keys(f"{sequence}/lidar/") if k.endswith(".bin")),
        key=lambda k: int(k.rsplit("/", 1)[1][:-4]),
    )
    stamps = [int(k.rsplit("/", 1)[1][:-4]) for k in keys]
    if stamps != [int(t) for t in rows[:, 0]]:
        raise ValueError(f"{sequence}: lidar scans and lidar_poses.csv rows differ")
    return rows, keys


def imu_in_lidar_frame(cache, sequence, begin_sec, end_sec):
    """t wx wy wz ax ay az of imu_raw.csv in the lidar frame, within [begin, end]."""
    raw = np.loadtxt(
        cached(cache, f"{sequence}/applanix/imu_raw.csv"), delimiter=",", skiprows=1
    )
    raw = raw[(raw[:, 0] >= begin_sec) & (raw[:, 0] <= end_sec)]
    applanix_lidar = np.loadtxt(cached(cache, f"{sequence}/calib/T_applanix_lidar.txt"))
    to_lidar = applanix_lidar[:3, :3].T @ RAW_TO_APPLANIX
    # Columns: GPSTime, angvel_z, angvel_y, angvel_x, accelz, accely, accelx.
    gyro = raw[:, [3, 2, 1]] @ to_lidar.T
    accel = raw[:, [6, 5, 4]] @ to_lidar.T
    return np.column_stack([raw[:, 0], gyro, accel])


def rotation(roll, pitch, heading):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(heading), math.sin(heading)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return (rz @ ry @ rx).T


def quaternion(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(
        math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2]
    )
    y = math.copysign(
        math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0]
    )
    z = math.copysign(
        math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1]
    )
    return x, y, z, w


def voxel_first(points, leaf):
    """The first point of every occupied voxel."""
    keys = np.floor(points[:, :3] / leaf).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def scan(key):
    """x y z intensity ring time; time is seconds from the scan stamp (mid-sweep)."""
    return np.frombuffer(fetch(f"{BUCKET}/{key}"), dtype=np.float32).reshape(-1, 6)


def write_ply(path, points):
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nproperty float intensity\n"
        "end_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode())
        stream.write(np.ascontiguousarray(points[:, :4], dtype="<f4").tobytes())


def build_map(cache, sequence, path_xy, radius, spacing, leaf, max_range, jobs):
    rows, keys = lidar_poses(cache, sequence)
    near = np.zeros(len(rows), dtype=bool)
    for start in range(0, len(path_xy), 200):
        chunk = path_xy[start : start + 200]
        d2 = ((rows[:, None, 1:3] - chunk[None, :, :]) ** 2).sum(-1).min(1)
        near |= d2 <= radius * radius
    selected, last = [], None
    for i in np.flatnonzero(near):
        if last is None or np.linalg.norm(rows[i, 1:4] - rows[last, 1:4]) >= spacing:
            selected.append(i)
            last = i

    def transformed(i):
        points = scan(keys[i]).astype(np.float64)
        r = np.linalg.norm(points[:, :3], axis=1)
        points = voxel_first(points[(r >= 2.0) & (r <= max_range)], leaf)
        row = rows[i]
        xyz = points[:, :3] @ rotation(row[7], row[8], row[9]).T + row[1:4]
        return np.column_stack([xyz, points[:, 3]])

    with ThreadPoolExecutor(jobs) as pool:
        clouds = list(pool.map(transformed, selected))
    return voxel_first(np.concatenate(clouds), leaf), len(selected)


def write_bag(path, rows, keys, leaf, jobs, imu):
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Vector3
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import Header

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic_id, (name, kind) in enumerate(
        ((POINTS, "sensor_msgs/msg/PointCloud2"), (IMU, "sensor_msgs/msg/Imu"))
    ):
        topic = {"name": name, "type": kind, "serialization_format": "cdr"}
        try:
            metadata = rosbag2_py.TopicMetadata(**topic)  # Humble
        except TypeError:
            metadata = rosbag2_py.TopicMetadata(id=topic_id, **topic)  # Jazzy
        writer.create_topic(metadata)

    def stamp(ns):
        return Time(sec=ns // 10**9, nanosec=ns % 10**9)

    imu_ns = np.round(imu[:, 0] * 1e9).astype(np.int64)
    next_imu = [0]

    def write_imu_until(ns):
        while next_imu[0] < len(imu) and imu_ns[next_imu[0]] <= ns:
            i = next_imu[0]
            message = Imu(
                header=Header(stamp=stamp(int(imu_ns[i])), frame_id=FRAME),
                angular_velocity=Vector3(x=imu[i, 1], y=imu[i, 2], z=imu[i, 3]),
                linear_acceleration=Vector3(x=imu[i, 4], y=imu[i, 5], z=imu[i, 6]),
            )
            message.orientation_covariance[0] = -1.0  # no orientation
            writer.write(IMU, serialize_message(message), int(imu_ns[i]))
            next_imu[0] += 1

    dtype = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("ring", "<u2"),
            ("pad", "<u2"),
            ("time", "<f4"),
        ]
    )
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]

    def downsampled(key):
        return voxel_first(scan(key), leaf)

    points_total = 0
    with ThreadPoolExecutor(jobs) as pool:
        for row, points in zip(rows, pool.map(downsampled, keys), strict=True):
            cloud = np.zeros(len(points), dtype=dtype)
            for name, column in (
                ("x", 0),
                ("y", 1),
                ("z", 2),
                ("intensity", 3),
                ("time", 5),
            ):
                cloud[name] = points[:, column]
            cloud["ring"] = np.clip(points[:, 4], 0, 65535).astype(np.uint16)
            stamp_ns = int(row[0]) * 1000
            write_imu_until(stamp_ns)
            message = PointCloud2(
                header=Header(stamp=stamp(stamp_ns), frame_id=FRAME),
                height=1,
                width=len(cloud),
                fields=fields,
                is_bigendian=False,
                point_step=dtype.itemsize,
                row_step=dtype.itemsize * len(cloud),
                data=cloud.tobytes(),
                is_dense=True,
            )
            writer.write(POINTS, serialize_message(message), stamp_ns)
            points_total += len(cloud)
    write_imu_until(np.iinfo(np.int64).max)
    # The writer closes the bag when it goes out of scope here.
    return points_total


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", type=Path)
    parser.add_argument("--loc-seq", default="boreas-2021-09-02-11-42")
    parser.add_argument(
        "--start", type=float, default=0.0, help="s from the first scan"
    )
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--map-seq", default="boreas-2020-11-26-13-58")
    parser.add_argument(
        "--map-radius",
        type=float,
        default=60.0,
        help="map scans within this distance (m) of the window's path",
    )
    parser.add_argument(
        "--map-spacing", type=float, default=2.0, help="m travelled between map scans"
    )
    # 0.3 m keeps over 6 points (the NDT minimum) on 1 m of surface; 0.2 m doubles
    # the map and the time to build a target.
    parser.add_argument("--map-leaf", type=float, default=0.3)
    parser.add_argument("--map-max-range", type=float, default=80.0)
    parser.add_argument("--scan-leaf", type=float, default=0.25)
    parser.add_argument("--jobs", type=int, default=8)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    cache = args.out / "cache"
    rows, keys = lidar_poses(cache, args.loc_seq)
    t = (rows[:, 0] - rows[0, 0]) * 1e-6
    window = (t >= args.start) & (t < args.start + args.duration)
    rows, keys = rows[window], [k for k, w in zip(keys, window, strict=True) if w]

    with (args.out / "reference.tum").open("w") as stream:
        for row in rows:
            q = quaternion(rotation(row[7], row[8], row[9]))
            values = [row[0] * 1e-6, *row[1:4], *q]
            stream.write(" ".join(f"{v:.9f}" for v in values) + "\n")

    print(
        f"map from {args.map_seq} within {args.map_radius} m of {len(rows)} poses",
        flush=True,
    )
    map_points, map_scans = build_map(
        cache,
        args.map_seq,
        rows[:, 1:3],
        args.map_radius,
        args.map_spacing,
        args.map_leaf,
        args.map_max_range,
        args.jobs,
    )
    write_ply(args.out / "map.ply", map_points)
    print(f"map.ply: {len(map_points)} points from {map_scans} scans", flush=True)

    bag = args.out / "bag"
    if bag.exists():
        raise SystemExit(f"{bag} exists")
    # IMU from 1 s before the first scan, so a filter can settle.
    imu = imu_in_lidar_frame(
        cache, args.loc_seq, rows[0, 0] * 1e-6 - 1.0, rows[-1, 0] * 1e-6 + 0.1
    )
    print(
        f"bag: {len(keys)} scans and {len(imu)} IMU samples of {args.loc_seq}",
        flush=True,
    )
    points_total = write_bag(bag, rows, keys, args.scan_leaf, args.jobs, imu)

    (args.out / "preparation.json").write_text(
        json.dumps(
            {
                "source": "s3://boreas (Boreas dataset, Burnett et al. 2023)",
                "localization_sequence": args.loc_seq,
                "window_sec": [args.start, args.start + args.duration],
                "scans": len(keys),
                "imu_samples": len(imu),
                "scan_leaf_m": args.scan_leaf,
                "mean_points_per_scan": points_total / max(1, len(keys)),
                "map_sequence": args.map_seq,
                "map_scans": map_scans,
                "map_points": len(map_points),
                "map_radius_m": args.map_radius,
                "map_spacing_m": args.map_spacing,
                "map_leaf_m": args.map_leaf,
                "map_max_range_m": args.map_max_range,
                "pose_convention": "R_enu_lidar = (Rz(heading) Ry(pitch) Rx(roll))^T",
            },
            indent=2,
        )
        + "\n"
    )
    print(f"done: {args.out}", flush=True)


if __name__ == "__main__":
    main()
