#!/usr/bin/env python3
"""Watch /alignment_status in one human-readable colored line per update.

Diagnostics only: subscribes diagnostic_msgs/DiagnosticArray, prints
failure_category + fitness + reject streak + seed drift + reinit flag,
plus the next action from troubleshooting.md. Never changes acceptance.

Usage:
    ros2 run lidar_localization_ros2 watch_alignment.py
    ros2 run lidar_localization_ros2 watch_alignment.py --no-color --once
    ros2 run lidar_localization_ros2 watch_alignment.py --csv /tmp/align.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, Tuple

CATEGORY_COLORS = {
    "healthy": "32",
    "missing_map": "36",
    "missing_initial_pose": "36",
    "weak_overlap": "33",
    "bad_match": "33",
    "stale_prediction": "31",
    "overload": "35",
}

NEXT_ACTIONS = {
    "healthy": "tracking OK",
    "missing_map": "check map_path / /map topic (troubleshooting: Map Not Visible)",
    "missing_initial_pose": "RViz 2D Pose Estimate or --initial-pose (frame_contract.md)",
    "weak_overlap": "check scan filters/FOV/initial pose vs map crop",
    "bad_match": "re-seed in RViz; check map_alignment.md / sensor frame",
    "stale_prediction": "tracking lost; G3 queries if --g3-recovery, else re-seed",
    "overload": "reduce voxel / ndt threads; check CPU load",
}


def suggest_next_action(values: Dict[str, str], message: str) -> str:
    """Return the one-line next action for a status sample."""
    category = str(values.get("failure_category", "") or "").strip()
    base = NEXT_ACTIONS.get(category, f"message={message}")
    if str(values.get("reinitialization_requested", "")).lower() == "true":
        base += " | reinit_requested=true -> publish /initialpose"
    return base


def format_alignment_line(message: str, values: Dict[str, str]) -> Tuple[str, str]:
    """Format one status sample as (plain_line, color_code).

    Pure function (no rclpy) so unit tests run without a ROS env.
    """
    category = str(values.get("failure_category", "?") or "?")
    fitness = values.get("fitness_score", "?")
    threshold = values.get(
        "effective_score_threshold", values.get("score_threshold", "?"))
    reject = values.get("consecutive_rejected_updates", "?")
    seed_m = values.get("seed_translation_since_accept_m", "?")
    seed_yaw = values.get("seed_yaw_since_accept_deg", "?")
    reinit = values.get("reinitialization_requested", "?")
    action = suggest_next_action(values, message)
    line = (
        f"[{category}] msg={message} fitness={fitness}/{threshold} "
        f"reject={reject} seed={seed_m}m/{seed_yaw}deg "
        f"reinit={reinit} -> {action}"
    )
    return line, CATEGORY_COLORS.get(category, "0")


def colorize(line: str, color_code: str, use_color: bool) -> str:
    if not use_color or color_code in ("0", ""):
        return line
    return f"\033[{color_code}m{line}\033[0m"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Watch /alignment_status with colored one-line updates.")
    parser.add_argument("--topic", default="/alignment_status")
    parser.add_argument("--csv", type=Path, default=None,
                        help="Append raw key fields to this CSV file.")
    parser.add_argument("--once", action="store_true",
                        help="Print one sample and exit.")
    parser.add_argument("--no-color", action="store_true")
    return parser


def _csv_header() -> list:
    return ["stamp_sec", "message", "failure_category", "fitness_score",
            "effective_score_threshold", "consecutive_rejected_updates",
            "seed_translation_since_accept_m",
            "seed_yaw_since_accept_deg", "reinitialization_requested"]


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
    except ImportError as exc:
        print(f"watch_alignment: ROS env not sourced ({exc})", file=sys.stderr)
        return 2

    writer = None
    csv_file = None
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        new_file = not args.csv.exists()
        csv_file = open(args.csv, "a", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=_csv_header())
        if new_file:
            writer.writeheader()

    rclpy.init()
    node = rclpy.create_node("watch_alignment")
    use_color = not args.no_color and sys.stdout.isatty()
    done = {"flag": False}

    def callback(msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            if "failure_category" not in values and "fitness_score" not in values:
                continue
            line, color = format_alignment_line(status.message, values)
            print(colorize(line, color, use_color), flush=True)
            if writer is not None:
                stamp = msg.header.stamp
                writer.writerow({
                    "stamp_sec": f"{stamp.sec}.{stamp.nanosec:09d}",
                    "message": status.message,
                    "failure_category": values.get("failure_category", ""),
                    "fitness_score": values.get("fitness_score", ""),
                    "effective_score_threshold": values.get(
                        "effective_score_threshold",
                        values.get("score_threshold", "")),
                    "consecutive_rejected_updates": values.get(
                        "consecutive_rejected_updates", ""),
                    "seed_translation_since_accept_m": values.get(
                        "seed_translation_since_accept_m", ""),
                    "seed_yaw_since_accept_deg": values.get(
                        "seed_yaw_since_accept_deg", ""),
                    "reinitialization_requested": values.get(
                        "reinitialization_requested", ""),
                })
                csv_file.flush()
            if args.once:
                done["flag"] = True
            return

    node.create_subscription(DiagnosticArray, args.topic, callback, 10)
    try:
        while rclpy.ok() and not done["flag"]:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if csv_file is not None:
            csv_file.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
