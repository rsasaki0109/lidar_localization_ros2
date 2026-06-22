#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

from lidar_localization_mid360.validation_model import AlignmentDiagnosticSample
from lidar_localization_mid360.validation_model import RuntimeValidationConfig
from lidar_localization_mid360.validation_model import evaluate_runtime_summary
from lidar_localization_mid360.validation_model import load_alignment_csv
from lidar_localization_mid360.validation_model import markdown_report
from lidar_localization_mid360.validation_model import render_runtime_report
from lidar_localization_mid360.validation_model import summarize_runtime
from lidar_localization_mid360.validation_model import validation_exit_code


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate that lidar_localization_ros2 is actually using IMU "
            "preintegration, and optionally the experimental continuous-time deskew hook."
        )
    )
    parser.add_argument(
        "--alignment-csv",
        default="",
        help=(
            "Analyze an existing alignment_status.csv from benchmark_diagnostic_recorder "
            "instead of subscribing to a live ROS topic."
        ),
    )
    parser.add_argument("--alignment-status-topic", default="/alignment_status")
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--min-imu-active-ratio", type=float, default=0.5)
    parser.add_argument("--min-imu-integrated-samples", type=int, default=1)
    parser.add_argument("--max-imu-fallback-count", type=int, default=0)
    parser.add_argument(
        "--require-imu-seed-source",
        action="store_true",
        help="Fail unless registration_seed_source reports imu_preintegration often enough.",
    )
    parser.add_argument("--min-imu-seed-source-ratio", type=float, default=0.5)
    parser.add_argument(
        "--require-deskew-applied",
        action="store_true",
        help="Fail unless continuous-time deskew is applied for enough status samples.",
    )
    parser.add_argument("--min-deskew-applied-ratio", type=float, default=0.1)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-md", default="")
    return parser


def config_from_args(args: argparse.Namespace) -> RuntimeValidationConfig:
    return RuntimeValidationConfig(
        min_samples=max(0, args.min_samples),
        min_imu_active_ratio=max(0.0, min(1.0, args.min_imu_active_ratio)),
        min_imu_integrated_samples=max(0, args.min_imu_integrated_samples),
        max_imu_fallback_count=max(0, args.max_imu_fallback_count),
        require_imu_seed_source=args.require_imu_seed_source,
        min_imu_seed_source_ratio=max(0.0, min(1.0, args.min_imu_seed_source_ratio)),
        require_deskew_applied=args.require_deskew_applied,
        min_deskew_applied_ratio=max(0.0, min(1.0, args.min_deskew_applied_ratio)),
    )


def _samples_from_ros(args: argparse.Namespace) -> List[AlignmentDiagnosticSample]:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    class AlignmentStatusCollector(Node):
        def __init__(self) -> None:
            super().__init__("lidar_localization_imu_validator")
            self.samples: List[AlignmentDiagnosticSample] = []
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                DiagnosticArray,
                args.alignment_status_topic,
                self._on_status,
                qos,
            )

        def _on_status(self, msg: DiagnosticArray) -> None:
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            for status in msg.status:
                self.samples.append(
                    AlignmentDiagnosticSample(
                        stamp_sec=stamp_sec,
                        level=int(status.level),
                        message=str(status.message),
                        values={str(entry.key): str(entry.value) for entry in status.values},
                    )
                )

    rclpy.init()
    node = AlignmentStatusCollector()
    try:
        deadline = time.monotonic() + max(args.duration_sec, 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return list(node.samples)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _write_outputs(
    args: argparse.Namespace,
    summary: Dict[str, object],
    checks: Sequence[object],
) -> None:
    if args.output_json:
        output = Path(args.output_json).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps({"summary": summary, "checks": [check.__dict__ for check in checks]}, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
    if args.output_md:
        output = Path(args.output_md).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(markdown_report(summary, checks), encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = config_from_args(args)
    try:
        if args.alignment_csv:
            samples = load_alignment_csv(args.alignment_csv)
        else:
            samples = _samples_from_ros(args)
    except (ImportError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"input error: {error}", file=sys.stderr)
        return 2

    summary = summarize_runtime(samples)
    checks = evaluate_runtime_summary(summary, config)
    for line in render_runtime_report(summary, checks):
        print(line)
    _write_outputs(args, summary, checks)
    return validation_exit_code(checks)


if __name__ == "__main__":
    sys.exit(main())
