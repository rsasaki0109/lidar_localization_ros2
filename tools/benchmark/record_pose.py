#!/usr/bin/env python3
"""Record localization poses as TUM and the startup status as JSON lines.

usage: record_pose.py <est.tum> <status.jsonl> [pose_topic] [status_topic]
"""

from __future__ import annotations

import contextlib
import json
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


def main() -> int:
    tum_path, status_path = sys.argv[1], sys.argv[2]
    pose_topic = sys.argv[3] if len(sys.argv) > 3 else "/pcl_pose"
    status_topic = (
        sys.argv[4] if len(sys.argv) > 4 else "/startup_initialization/status"
    )
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = rclpy.create_node("benchmark_pose_recorder")
    with (
        open(tum_path, "w", encoding="utf-8") as tum,
        open(status_path, "w", encoding="utf-8") as status,
    ):

        def on_pose(msg: PoseWithCovarianceStamped) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            tum.write(
                f"{stamp:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
            )
            tum.flush()

        def on_status(msg: String) -> None:
            now = node.get_clock().now().nanoseconds * 1.0e-9
            status.write(json.dumps({"sim_time_sec": now, "status": msg.data}) + "\n")
            status.flush()

        node.create_subscription(PoseWithCovarianceStamped, pose_topic, on_pose, 100)
        node.create_subscription(String, status_topic, on_status, 100)
        with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
            rclpy.spin(node)
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
