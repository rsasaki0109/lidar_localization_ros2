#!/usr/bin/env python3
"""Extract what replay.py needs from the Autoware Istanbul localization bag.

Writes into OUT_DIR:
  gnss_reference.csv  /sensing/gnss/pose_with_covariance at ~20 Hz (t, x, y, z, quaternion)
  time_map.txt        bag receive time and header stamp of every cloud
  bag_start.txt       the bag's start time, which `ros2 bag play --start-offset` counts from

usage: prepare.py BAG_DIR OUT_DIR
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CLOUD = "/localization/util/downsample/pointcloud"
GNSS = "/sensing/gnss/pose_with_covariance"


def open_bag(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main(bag, out):
    out.mkdir(parents=True, exist_ok=True)
    metadata = rosbag2_py.Info().read_metadata(str(bag), "sqlite3")
    (out / "bag_start.txt").write_text(
        f"{metadata.starting_time.nanoseconds * 1e-9:.9f}\n"
    )

    reader = open_bag(bag, [GNSS])
    pose_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")
    rows, last = [], -1.0
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, pose_type)
        t = stamp(msg.header)
        if t - last < 0.05:
            continue
        last = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    with open(out / "gnss_reference.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(rows)

    reader = open_bag(bag, [CLOUD])
    cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    pairs = []
    while reader.has_next():
        _, data, received = reader.read_next()
        pairs.append(
            (received * 1e-9, stamp(deserialize_message(data, cloud_type).header))
        )
    np.savetxt(out / "time_map.txt", np.array(pairs), fmt="%.6f")

    # Where the route climbs: 10 s segments with the steepest grade.
    step = 200
    grades = []
    for i in range(0, len(rows) - step, step):
        a, b = rows[i], rows[i + step]
        run = math.hypot(b[1] - a[1], b[2] - a[2])
        if run > 20.0:
            grades.append((a[0] - rows[0][0], (b[3] - a[3]) / run * 100.0))
    print(f"{len(rows)} reference poses, {len(pairs)} clouds")
    for offset, grade in sorted(sorted(grades, key=lambda g: -abs(g[1]))[:8]):
        print(f"  GNSS t+{offset:5.0f} s grade {grade:+5.1f} %")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2]))
