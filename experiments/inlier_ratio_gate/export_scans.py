"""Export Koide outdoor_hard_01a clouds for koide_probe.

The clouds of a window, in seconds from the first scan the replays align (bag + 4 s,
1694532829.4995 s), are written as float32 x, y, z in scan_NNNN.bin. index.txt holds,
per scan, its stamp and the nearest ground-truth pose within 0.05 s (TUM order).

usage: export_scans.py DATA_DIR OUT_DIR START_SEC END_SEC
  DATA_DIR holds outdoor_hard_01a_si/ (koide_public_bag.prepare_si_bag) and gt/
"""

import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FIRST_ALIGNED_STAMP = 1694532829.4995492


def main():
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    window = float(sys.argv[3]), float(sys.argv[4])
    out.mkdir(parents=True, exist_ok=True)
    gt = np.loadtxt(data / "gt" / "traj_lidar_outdoor_hard_01.txt")
    gt = gt[np.argsort(gt[:, 0])]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(data / "outdoor_hard_01a_si"), storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/points"]))
    rows = []
    while reader.has_next():
        _, payload, _ = reader.read_next()
        message = deserialize_message(payload, PointCloud2)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        since_first = stamp - FIRST_ALIGNED_STAMP
        if since_first < window[0]:
            continue
        if since_first >= window[1]:
            break
        i = int(np.argmin(abs(gt[:, 0] - stamp)))
        if abs(gt[i, 0] - stamp) > 0.05:
            continue
        points = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        points.astype(np.float32).tofile(out / f"scan_{len(rows):04d}.bin")
        rows.append([stamp, *gt[i, 1:8]])
    np.savetxt(out / "index.txt", np.array(rows), fmt="%.9f")
    print(len(rows), "scans")


if __name__ == "__main__":
    main()
