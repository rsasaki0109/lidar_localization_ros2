#!/usr/bin/env python3
"""Publish the ground-truth pose on /initialpose once the replay reaches a chosen bag time.

Starts tracking from a known-good pose at a controlled moment, so a replay can test
what the localizer does in the first seconds after an initial pose without the
global search deciding when that is. The ground truth is the pose of the cloud
frame (TUM: stamp x y z qx qy qz qw), which the JEPLO suites track as the base frame.

usage: gt_initial_pose.py --ground-truth gt.txt --bag BAG --start-sec 12.0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml


def bag_start_sec(bag: Path) -> float:
    info = yaml.safe_load((bag / "metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    return info["starting_time"]["nanoseconds_since_epoch"] * 1.0e-9


def pose_at(truth: np.ndarray, stamp: float) -> np.ndarray:
    """Interpolated x y z and the nearer sample's quaternion at stamp."""
    index = int(np.clip(np.searchsorted(truth[:, 0], stamp), 1, len(truth) - 1))
    before, after = truth[index - 1], truth[index]
    span = max(after[0] - before[0], 1.0e-9)
    t = float(np.clip((stamp - before[0]) / span, 0.0, 1.0))
    position = before[1:4] + t * (after[1:4] - before[1:4])
    quaternion = before[4:8] if t < 0.5 else after[4:8]
    return np.concatenate([position, quaternion])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument(
        "--start-sec", type=float, required=True, help="seconds into the bag"
    )
    parser.add_argument("--frame", default="map")
    parser.add_argument(
        "--offset-m",
        type=float,
        default=0.0,
        help="shift the pose this far along map x, to test pulling in a coarse pose",
    )
    args = parser.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    truth = np.loadtxt(args.ground_truth, ndmin=2)
    truth = truth[np.argsort(truth[:, 0])]
    target = bag_start_sec(args.bag) + args.start_sec

    rclpy.init()
    node = Node(
        "gt_initial_pose",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
    done = {"published": False}

    def tick():
        now = node.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        if done["published"] or now_sec < target:
            return
        pose = pose_at(truth, now_sec)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = args.frame
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y, p.z = (float(v) for v in pose[:3])
        p.x += args.offset_m
        q.x, q.y, q.z, q.w = (float(v) for v in pose[3:])
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.001
        publisher.publish(msg)
        done["published"] = True
        node.get_logger().info(
            f"published ground-truth initial pose (+{args.offset_m:g} m in x) at bag "
            f"t={now_sec - target + args.start_sec:.2f} s"
        )

    node.create_timer(0.02, tick)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
