#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Dict
from typing import Optional
from typing import Sequence

from lidar_localization_mid360.bringup_model import BringupCheckConfig
from lidar_localization_mid360.bringup_model import evaluate_snapshot
from lidar_localization_mid360.bringup_model import exit_code
from lidar_localization_mid360.bringup_model import report_lines


PROFILE_DEFAULTS: Dict[str, Dict[str, object]] = {
    "standalone": {
        "cloud_topic": "/velodyne_points",
        "imu_topic": "/imu",
        "pose_topic": "/pcl_pose",
        "lidar_frame": "velodyne",
        "imu_frame": "imu_link",
        "require_cloud_time_field": False,
        "require_imu_base_tf": False,
        "require_odom_base_tf": False,
        "require_map_odom_tf": False,
    },
    "nav2": {
        "cloud_topic": "/velodyne_points",
        "imu_topic": "/imu/data",
        "pose_topic": "/localization/pose_with_covariance",
        "lidar_frame": "velodyne",
        "imu_frame": "imu_link",
        "require_cloud_time_field": False,
        "require_imu_base_tf": False,
        "require_odom_base_tf": True,
        "require_map_odom_tf": True,
    },
    "mid360": {
        "cloud_topic": "/livox/points",
        "imu_topic": "/livox/imu",
        "pose_topic": "/localization/pose_with_covariance",
        "lidar_frame": "livox_frame",
        "imu_frame": "livox_imu_frame",
        "require_cloud_time_field": False,
        "require_imu_base_tf": False,
        "require_odom_base_tf": True,
        "require_map_odom_tf": False,
    },
}


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check lidar_localization_ros2 bringup topics, TF, pose, and diagnostics."
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_DEFAULTS.keys()),
        default="standalone",
        help="Preset defaults for common bringup modes.",
    )
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--cloud-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--pose-topic")
    parser.add_argument("--alignment-status-topic", default="/alignment_status")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--lidar-frame")
    parser.add_argument("--imu-frame")
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument(
        "--require-cloud-time-field",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Require a per-point timing field on PointCloud2 for continuous-time "
            "deskew readiness checks."
        ),
    )
    parser.add_argument(
        "--require-imu-base-tf",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Require base_frame <- imu_frame TF. Enable when IMU preintegration transforms samples into base_frame.",
    )
    parser.add_argument(
        "--require-odom-base-tf",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Require odom <- base_link TF. Disable for standalone map -> base_link mode.",
    )
    parser.add_argument(
        "--require-map-odom-tf",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Require map <- odom TF. Enable for Nav2 map->odom integration.",
    )
    parser.add_argument("--require-localization-output", action="store_true")
    return parser


def _arg_or_profile(args: argparse.Namespace, key: str):
    value = getattr(args, key)
    if value is not None:
        return value
    return PROFILE_DEFAULTS[args.profile][key]


def config_from_args(args: argparse.Namespace) -> BringupCheckConfig:
    return BringupCheckConfig(
        cloud_topic=str(_arg_or_profile(args, "cloud_topic")),
        imu_topic=str(_arg_or_profile(args, "imu_topic")),
        pose_topic=str(_arg_or_profile(args, "pose_topic")),
        alignment_status_topic=args.alignment_status_topic,
        global_frame=args.global_frame,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        lidar_frame=str(_arg_or_profile(args, "lidar_frame")),
        imu_frame=str(_arg_or_profile(args, "imu_frame")),
        require_imu=args.require_imu,
        require_cloud_time_field=bool(_arg_or_profile(args, "require_cloud_time_field")),
        require_imu_base_tf=bool(_arg_or_profile(args, "require_imu_base_tf")),
        require_odom_base_tf=bool(_arg_or_profile(args, "require_odom_base_tf")),
        require_map_odom_tf=bool(_arg_or_profile(args, "require_map_odom_tf")),
        require_localization_output=args.require_localization_output,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    import rclpy

    from lidar_localization_mid360.ros_doctor import Mid360BringupDoctor

    config = config_from_args(args)
    rclpy.init()
    node = Mid360BringupDoctor(
        config,
        node_name="lidar_localization_bringup_doctor",
    )
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
