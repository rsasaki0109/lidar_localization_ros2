#!/usr/bin/env python3
"""Print a `ros2 topic pub` command that sends a ground-truth /initialpose.

The pose is the TUM reference sample nearest to `bag start + --offset` seconds
(the README run uses 4 s, while the handheld sensor is still). usage:
  initial_pose_from_tum.py <reference.tum> <bag> [--offset 4.0] [--frame map]
"""

import argparse

import numpy as np
import rosbag2_py

ap = argparse.ArgumentParser()
ap.add_argument("reference_tum", help="t x y z qx qy qz qw per line")
ap.add_argument("bag", help="rosbag2 directory the pose is for")
ap.add_argument("--offset", type=float, default=4.0, help="seconds after the bag start")
ap.add_argument("--frame", default="map")
ap.add_argument("--xy-variance", type=float, default=0.25)
ap.add_argument("--yaw-variance", type=float, default=0.07)
args = ap.parse_args()

info = rosbag2_py.Info().read_metadata(args.bag, "")
start = info.starting_time.nanoseconds * 1e-9
reference = np.loadtxt(args.reference_tum)
row = reference[np.argmin(np.abs(reference[:, 0] - (start + args.offset)))]
if abs(row[0] - (start + args.offset)) > 1.0:
    raise SystemExit("reference has no sample within 1 s of the requested time")

sec = int(row[0])
nanosec = round((row[0] - sec) * 1e9)
covariance = [0.0] * 36
covariance[0] = covariance[7] = args.xy_variance
covariance[35] = args.yaw_variance
x, y, z, qx, qy, qz, qw = row[1:8]
message = (
    f"{{header: {{stamp: {{sec: {sec}, nanosec: {nanosec}}}, frame_id: {args.frame}}}, "
    f"pose: {{pose: {{position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, "
    f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}, "
    f"covariance: {covariance}}}}}"
)
print(
    f"ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{message}'"
)
