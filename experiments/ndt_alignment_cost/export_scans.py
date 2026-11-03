"""Export Boreas fixture scans for boreas_ndt_bench.

Every second scan of 84-116 s of a boreas_prepare.py fixture, as float32 x, y, z,
intensity in scan_NNNN.bin, and index.txt with, per scan, its stamp and ground-truth
pose (TUM order) followed by those of the two reference poses before it.

usage: export_scans.py FIXTURE_DIR OUT_DIR
"""

import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

WINDOW_SEC = (84.0, 116.0)


def main():
    fixture, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    reference = np.loadtxt(fixture / "reference.tum")
    reference = reference[np.argsort(reference[:, 0])]
    start = reference[0, 0]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(fixture / "bag"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/velodyne_points"]))
    rows = []
    seen = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        message = deserialize_message(data, PointCloud2)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp - start < WINDOW_SEC[0]:
            continue
        if stamp - start >= WINDOW_SEC[1]:
            break
        seen += 1
        if seen % 2:
            continue
        i = int(np.argmin(abs(reference[:, 0] - stamp)))
        if abs(reference[i, 0] - stamp) > 0.01:
            continue
        points = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        points.astype(np.float32).tofile(out / f"scan_{len(rows):04d}.bin")
        rows.append([stamp, *reference[i, 1:8], *reference[i - 1], *reference[i - 2]])
    np.savetxt(out / "index.txt", np.array(rows), fmt="%.9f")
    print(len(rows), "scans")


if __name__ == "__main__":
    main()
