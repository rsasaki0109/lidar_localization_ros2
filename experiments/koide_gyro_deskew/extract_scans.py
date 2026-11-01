#!/usr/bin/env python3
"""Write Koide outdoor_hard_02b clouds three ways for batch_align.cpp: as recorded
(raw), deskewed with the GT trajectory (desk) and rotated with the integrated gyro
(gyro). All three are in the sensor frame at the header stamp, the sweep start.

usage: extract_scans.py BAG GT FIRST_EXPECTED_NS OUT_DIR

BAG is the SI-converted bag, GT traj_lidar_outdoor_hard_02.txt (TUM, one pose per
cloud header stamp), FIRST_EXPECTED_NS the stamp of the first cloud the online
check expects. OUT_DIR gets NAME_{raw,desk,gyro}.pcd and index.txt with, per cloud:
name, window, seconds from the first expected cloud, GT rotation rate (deg/s) and the
GT pose at the header stamp (x y z qx qy qz qw).
"""

import itertools
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

# Seconds from the first expected cloud, and every how many clouds to keep.
WINDOWS = {"surge": (96.0, 134.0, 1), "calm": (40.0, 80.0, 4)}


def open_topic(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quat_to_rot(q):
    x, y, z, w = q.T
    return np.stack(
        [
            np.stack(
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1
            ),
            np.stack(
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1
            ),
            np.stack(
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1
            ),
        ],
        -2,
    )


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def slerp(q0, q1, a):
    dot = np.sum(q0 * q1, -1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)
    theta = np.arccos(np.clip(np.abs(dot), -1, 1))
    sin = np.sin(theta)
    safe = np.where(sin > 1e-6, sin, 1.0)
    w0 = np.where(sin > 1e-6, np.sin((1 - a) * theta) / safe, 1 - a)
    w1 = np.where(sin > 1e-6, np.sin(a * theta) / safe, a)
    q = w0 * q0 + w1 * q1
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def write_pcd(path, xyz):
    xyzi = np.zeros((len(xyz), 4), np.float32)
    xyzi[:, :3] = xyz
    head = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        f"COUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(xyz)}\nDATA binary\n"
    )
    path.write_bytes(head.encode() + xyzi.tobytes())


def read_gyro(bag):
    times, rates = [], []
    reader = open_topic(bag, "/livox/imu")
    while reader.has_next():
        msg = deserialize_message(reader.read_next()[1], Imu)
        times.append(stamp_of(msg))
        w = msg.angular_velocity
        rates.append([w.x, w.y, w.z])
    return np.array(times), np.array(rates)


def gyro_rotations(imu_t, imu_w, start, end):
    """Knot times from START to END and the rotation (x, y, z, w) from the sensor
    at each knot to the sensor at START, integrated from the gyro."""
    knots = np.concatenate([[start], imu_t[(imu_t > start) & (imu_t < end)], [end]])
    quats = [np.array([0.0, 0.0, 0.0, 1.0])]
    for a, b in itertools.pairwise(knots):
        omega = np.array(
            [np.interp(0.5 * (a + b), imu_t, imu_w[:, i]) for i in range(3)]
        )
        angle = np.linalg.norm(omega) * (b - a)
        axis = omega / np.linalg.norm(omega) if angle > 1e-12 else np.array([0.0, 0, 1])
        quats.append(
            quat_mul(
                quats[-1],
                np.concatenate([axis * np.sin(angle / 2), [np.cos(angle / 2)]]),
            )
        )
    return knots, np.array(quats)


def main(bag, gt_path, first_ns, out):
    out.mkdir(parents=True, exist_ok=True)
    gt = np.loadtxt(gt_path)
    imu_t, imu_w = read_gyro(bag)
    t0 = first_ns * 1e-9
    seen = dict.fromkeys(WINDOWS, 0)
    index = []
    reader = open_topic(bag, "/livox/points")
    while reader.has_next():
        msg = deserialize_message(reader.read_next()[1], PointCloud2)
        stamp = stamp_of(msg)
        window = next(
            (w for w, (a, b, _) in WINDOWS.items() if a <= stamp - t0 < b), None
        )
        if window is None:
            continue
        seen[window] += 1
        if (seen[window] - 1) % WINDOWS[window][2]:
            continue
        k = int(np.searchsorted(gt[:, 0], stamp - 1e-4))
        if k + 1 >= len(gt) or abs(gt[k, 0] - stamp) > 1e-3:
            continue  # no GT pose at this header stamp
        points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "t"), skip_nans=True
        )
        xyz = np.stack([points["x"], points["y"], points["z"]], 1).astype(np.float64)
        t = stamp + np.asarray(points["t"], np.float64) * 1e-9

        # GT: interpolate the pose at each point time between the samples around it.
        j = np.clip(np.searchsorted(gt[:, 0], t, side="right") - 1, 0, len(gt) - 2)
        a = ((t - gt[j, 0]) / (gt[j + 1, 0] - gt[j, 0]))[:, None]
        world = np.einsum(
            "nij,nj->ni", quat_to_rot(slerp(gt[j, 4:8], gt[j + 1, 4:8], a)), xyz
        ) + ((1 - a) * gt[j, 1:4] + a * gt[j + 1, 1:4])
        desk = (world - gt[k, 1:4]) @ quat_to_rot(gt[k : k + 1, 4:8])[0]

        # Gyro: rotate each point into the sensor frame at the sweep start.
        knots, quats = gyro_rotations(imu_t, imu_w, stamp, t.max() + 1e-6)
        seg = np.clip(np.searchsorted(knots, t, side="right") - 1, 0, len(knots) - 2)
        frac = ((t - knots[seg]) / (knots[seg + 1] - knots[seg]))[:, None]
        gyro = np.einsum(
            "nij,nj->ni", quat_to_rot(slerp(quats[seg], quats[seg + 1], frac)), xyz
        )

        name = f"{window}_{len(index):04d}"
        for kind, cloud in (("raw", xyz), ("desk", desk), ("gyro", gyro)):
            write_pcd(out / f"{name}_{kind}.pcd", cloud)
        dq = abs(np.sum(gt[k, 4:8] * gt[k + 1, 4:8]))
        rate = np.degrees(2 * np.arccos(np.clip(dq, 0, 1)) / (gt[k + 1, 0] - gt[k, 0]))
        index.append(
            f"{name} {window} {stamp - t0:.3f} {rate:.2f} "
            + " ".join(f"{v:.9f}" for v in gt[k, 1:8])
        )
    (out / "index.txt").write_text("\n".join(index) + "\n")
    print(seen, len(index), "clouds written")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]), Path(sys.argv[4]))
