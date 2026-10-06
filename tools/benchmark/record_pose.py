#!/usr/bin/env python3
"""Record what a localization run reports, for scoring against ground truth.

Writes into <run_dir>:
  est.tum           /pcl_pose as TUM
  covariance.txt    stamp, xx, yy and yaw-yaw variance of each pose
  status.jsonl      the startup status stream
  alignment.jsonl   the localizer's per-scan /alignment_status level and values

usage: record_pose.py <run_dir> [pose_topic]
"""

from __future__ import annotations

import contextlib
import json
import sys
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# /alignment_status values kept per scan (the launch log keeps the rest).
ALIGNMENT_KEYS = (
    "fitness_score",
    "correction_translation_m",
    "consecutive_rejected_updates",
    "accepted_gap_sec",
    "seed_translation_since_accept_m",
    "recovery_state",
    "failure_category",
    "bad_match_active",
    "weak_overlap_active",
    "horizontal_localizability_eigenvalue_ratio",
    "registration_localizability_weak_ratio",
)


def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1.0e-9


def main() -> int:
    run_dir = Path(sys.argv[1])
    pose_topic = sys.argv[2] if len(sys.argv) > 2 else "/pcl_pose"
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = rclpy.create_node("benchmark_pose_recorder")
    latched = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    with (
        (run_dir / "est.tum").open("w", encoding="utf-8") as tum,
        (run_dir / "covariance.txt").open("w", encoding="utf-8") as covariance,
        (run_dir / "status.jsonl").open("w", encoding="utf-8") as status,
        (run_dir / "alignment.jsonl").open("w", encoding="utf-8") as alignment,
    ):

        def on_pose(msg: PoseWithCovarianceStamped) -> None:
            stamp = _stamp(msg.header)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            c = msg.pose.covariance
            tum.write(
                f"{stamp:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
            )
            covariance.write(f"{stamp:.9f} {c[0]:.6g} {c[7]:.6g} {c[35]:.6g}\n")
            tum.flush()
            covariance.flush()

        def on_status(msg: String) -> None:
            now = node.get_clock().now().nanoseconds * 1.0e-9
            status.write(json.dumps({"sim_time_sec": now, "status": msg.data}) + "\n")
            status.flush()

        def on_alignment(msg: DiagnosticArray) -> None:
            for entry in msg.status:
                level = entry.level
                record = {
                    "stamp": _stamp(msg.header),
                    "level": level[0]
                    if isinstance(level, (bytes, bytearray))
                    else int(level),
                    "message": entry.message,
                }
                record.update(
                    {
                        kv.key: kv.value
                        for kv in entry.values
                        if kv.key in ALIGNMENT_KEYS
                    }
                )
                alignment.write(json.dumps(record) + "\n")
            alignment.flush()

        node.create_subscription(PoseWithCovarianceStamped, pose_topic, on_pose, 100)
        node.create_subscription(
            String, "/startup_initialization/status", on_status, latched
        )
        node.create_subscription(
            DiagnosticArray, "/alignment_status", on_alignment, latched
        )
        with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
            rclpy.spin(node)
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
