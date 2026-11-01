#!/usr/bin/env python3
"""Write one cloud of the Autoware Istanbul localization bag as scan.pcd (x y z,
zero intensity) and the GNSS pose at its stamp as pose.txt (x y z qx qy qz qw),
for ndt_align_bench.cpp.

usage: extract_scan.py BAG FIXTURE START_SEC OUT_DIR

FIXTURE is tools/benchmark/istanbul_prepare.py's output; START_SEC counts from the
bag start, as istanbul_replay.py --start does.
"""

import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

CLOUD = "/localization/util/downsample/pointcloud"


def main(bag, fixture, start, out):
    bag_start = float((fixture / "bag_start.txt").read_text())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[CLOUD]))
    reader.seek(int((bag_start + start) * 1e9))
    _, data, _ = reader.read_next()
    msg = deserialize_message(data, PointCloud2)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    xyz = point_cloud2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    out.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        f"TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA ascii\n"
    )
    rows = "\n".join(f"{x:.6f} {y:.6f} {z:.6f} 0" for x, y, z in xyz)
    (out / "scan.pcd").write_text(header + rows + "\n")
    ref = np.loadtxt(fixture / "gnss_reference.csv", delimiter=",", skiprows=1)
    k = min(int(np.searchsorted(ref[:, 0], stamp)), len(ref) - 1)
    (out / "pose.txt").write_text(" ".join(f"{v:.9f}" for v in ref[k, 1:8]) + "\n")
    print(f"{msg.header.frame_id} stamp {stamp:.3f}: {len(xyz)} points")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    main(sys.argv[1], Path(sys.argv[2]), float(sys.argv[3]), Path(sys.argv[4]))
