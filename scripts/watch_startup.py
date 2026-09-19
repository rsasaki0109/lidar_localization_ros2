#!/usr/bin/env python3
"""Watch /startup_initialization/status in one human-readable line per update.

Diagnostics only: subscribes std_msgs/String (transient JSON published by
startup_initialization_node.py), prints state + source + attempts + reason
plus the next action from quickstart.md/site_setup.md. Never changes behavior.

Usage:
    ros2 run lidar_localization_ros2 watch_startup.py
    ros2 run lidar_localization_ros2 watch_startup.py --no-color --once
"""

from __future__ import annotations

import argparse
import json
import sys

STATE_COLORS = {
    "active": "32",
    "needs_operator": "31",
    "querying_global": "33",
    "verifying": "33",
    "waiting_for_scan": "36",
}


def parse_status_json(text: str) -> dict:
    """Parse a status payload safely; return {} on invalid JSON."""
    try:
        payload = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError):
        return {}
    return payload if isinstance(payload, dict) else {}


def suggest_startup_action(payload: dict) -> str:
    """Return the one-line next action for a startup status payload."""
    state = str(payload.get("state", "") or "")
    reason = str(payload.get("reason", "") or "")
    if state == "active":
        return "startup complete; tracking via localizer"
    if state == "needs_operator":
        if reason == "no_safe_automatic_source":
            return (
                "set 2D Pose Estimate in RViz or restart with "
                "--occupancy-map / --reference-csv"
            )
        if reason == "global_attempts_exhausted":
            return "automatic publication stopped; set pose in RViz"
        if reason.startswith("ambiguous"):
            return (
                "similar places indistinguishable; do not loosen margin "
                "without replay evidence"
            )
        if reason == "map_mismatch":
            return "stored pose belongs to different map; delete state file or ignore"
        return f"operator input needed ({reason or 'unknown reason'})"
    if state == "querying_global":
        return "global search running; keep robot stationary"
    if state == "verifying":
        return "verifying candidate with localizer fitness"
    if state == "waiting_for_scan":
        return "waiting for first scan; start driver/bag"
    return f"state={state or 'unknown'}"


def format_startup_line(payload: dict) -> tuple[str, str]:
    """Format a status payload as (plain_line, color_code).

    Pure function (no rclpy) so unit tests run without a ROS env.
    """
    state = str(payload.get("state", "?") or "?")
    source = str(payload.get("source", "?") or "?")
    attempts = payload.get("global_attempts", "?")
    reason = str(payload.get("reason", "") or "")
    action = suggest_startup_action(payload)
    line = f"[{state}] source={source} attempts={attempts} reason={reason} -> {action}"
    return line, STATE_COLORS.get(state, "0")


def colorize(line: str, color_code: str, use_color: bool) -> str:
    if not use_color or color_code in ("0", ""):
        return line
    return f"\033[{color_code}m{line}\033[0m"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Watch startup initialization status with one-line updates."
    )
    parser.add_argument("--topic", default="/startup_initialization/status")
    parser.add_argument(
        "--once", action="store_true", help="Print one sample and exit."
    )
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=0.0,
        help="Exit after N seconds without a message (0 = wait).",
    )
    return parser


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"watch_startup: ROS env not sourced ({exc})", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("watch_startup")
    use_color = not args.no_color and sys.stdout.isatty()
    done = {"flag": False}
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    def callback(msg: String) -> None:
        payload = parse_status_json(msg.data)
        if not payload:
            return
        line, color = format_startup_line(payload)
        print(colorize(line, color, use_color), flush=True)
        if args.once:
            done["flag"] = True

    node.create_subscription(String, args.topic, callback, qos)
    import time

    start = time.monotonic()
    try:
        while rclpy.ok() and not done["flag"]:
            rclpy.spin_once(node, timeout_sec=0.5)
            if args.timeout_sec > 0.0 and (time.monotonic() - start) > args.timeout_sec:
                print("watch_startup: timeout waiting for status", file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
