#!/usr/bin/env python3
"""Report map-frame position error of recorded /pcl_pose against a TUM reference.

No alignment is applied: localization runs in the reference map frame. Each
pose is matched to the nearest reference sample within 0.15 s. usage:
  evaluate_pose_against_tum.py <recorded_bag> <reference.tum> [--tum-out est.tum]
"""

import argparse

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message

ap = argparse.ArgumentParser()
ap.add_argument("recorded_bag", help="rosbag2 directory with /pcl_pose")
ap.add_argument("reference_tum", help="t x y z qx qy qz qw per line")
ap.add_argument("--topic", default="/pcl_pose")
ap.add_argument("--tum-out", help="also write the estimate as TUM")
args = ap.parse_args()

storage = rosbag2_py.Info().read_metadata(args.recorded_bag, "").storage_identifier
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=args.recorded_bag, storage_id=storage),
    rosbag2_py.ConverterOptions("", ""),
)
reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
rows = []
while reader.has_next():
    _, data, _ = reader.read_next()
    msg = deserialize_message(data, PoseWithCovarianceStamped)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if stamp < 1.0:  # an echo of /initialpose without a sensor stamp
        continue
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    rows.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
if not rows:
    raise SystemExit(f"no {args.topic} messages in {args.recorded_bag}")
estimate = np.array(sorted(rows))
if args.tum_out:
    np.savetxt(args.tum_out, estimate, fmt="%.9f")

reference = np.loadtxt(args.reference_tum)
reference = reference[np.argsort(reference[:, 0])]
index = np.clip(np.searchsorted(reference[:, 0], estimate[:, 0]), 1, len(reference) - 1)
earlier = np.abs(reference[index - 1, 0] - estimate[:, 0]) < np.abs(
    reference[index, 0] - estimate[:, 0]
)
index = np.where(earlier, index - 1, index)
matched = np.abs(reference[index, 0] - estimate[:, 0]) < 0.15
error = np.linalg.norm(estimate[matched, 1:4] - reference[index[matched], 1:4], axis=1)
print(
    f"poses {len(estimate)} over {estimate[-1, 0] - estimate[0, 0]:.1f} s, matched {matched.sum()}, "
    f"longest output gap {np.diff(estimate[:, 0]).max():.2f} s"
)
print(
    f"map-frame position error: RMSE {np.sqrt(np.mean(error**2)):.3f} m, median {np.median(error):.3f} m, "
    f"p95 {np.percentile(error, 95):.3f} m, max {error.max():.3f} m"
)
