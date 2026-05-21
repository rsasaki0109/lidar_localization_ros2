import argparse
import sys
import time
from typing import Optional
from typing import Sequence

import rclpy

from lidar_localization_mid360.bringup_model import BringupCheckConfig
from lidar_localization_mid360.bringup_model import evaluate_snapshot
from lidar_localization_mid360.bringup_model import exit_code
from lidar_localization_mid360.bringup_model import report_lines
from lidar_localization_mid360.ros_doctor import Mid360BringupDoctor


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check MID-360 legged-robot localization bringup topics and TF."
    )
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--cloud-topic", default="/livox/points")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--pose-topic", default="/localization/pose_with_covariance")
    parser.add_argument("--alignment-status-topic", default="/alignment_status")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--lidar-frame", default="livox_frame")
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument("--require-map-odom-tf", action="store_true")
    parser.add_argument("--require-localization-output", action="store_true")
    return parser


def config_from_args(args: argparse.Namespace) -> BringupCheckConfig:
    return BringupCheckConfig(
        cloud_topic=args.cloud_topic,
        imu_topic=args.imu_topic,
        pose_topic=args.pose_topic,
        alignment_status_topic=args.alignment_status_topic,
        global_frame=args.global_frame,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        lidar_frame=args.lidar_frame,
        require_imu=args.require_imu,
        require_map_odom_tf=args.require_map_odom_tf,
        require_localization_output=args.require_localization_output,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = config_from_args(args)
    rclpy.init()
    node = Mid360BringupDoctor(config)
    try:
        deadline = time.monotonic() + max(args.duration_sec, 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        snapshot = node.snapshot()
        results = evaluate_snapshot(config, snapshot)
        for line in report_lines(config, snapshot, results):
            print(line)
        return exit_code(results)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
