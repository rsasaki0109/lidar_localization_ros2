#!/usr/bin/env python3
"""Wait until the odometry publishes its odom -> base TF, and print the base frame.

run_benchmark.py starts quickstart only once this succeeds, so a run cannot start
before the odometry runs.

usage: wait_for_odometry.py [--odom-frame odom] [--timeout SEC]
exit 0 and the child frame of the first odom TF on stdout, or exit 1 on timeout.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    try:
        node = rclpy.create_node("benchmark_odometry_wait")
        children = []

        def record(message):
            for transform in message.transforms:
                if transform.header.frame_id.lstrip("/") == args.odom_frame:
                    children.append(transform.child_frame_id.lstrip("/"))

        node.create_subscription(TFMessage, "/tf", record, 100)
        deadline = time.monotonic() + args.timeout
        while not children and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rclpy.shutdown()
    if not children:
        print(
            f"no {args.odom_frame} TF on /tf within {args.timeout:g} s",
            file=sys.stderr,
        )
        return 1
    print(children[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
