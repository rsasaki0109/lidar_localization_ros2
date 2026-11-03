#!/usr/bin/env python3
"""Replay a Koide sequence online with RKO-LIO odometry, as the outdoor_hard_02b check does.

The localizer runs the online profile (tools/readme_media/koide_outdoor_hard_02b_online.yaml),
seeded from RKO-LIO's odometry, with one ground-truth initial pose at bag + 4 s and the bag
played at 1x. Scored like the 02b check (koide_public_bag.evaluate), plus how long the
error stays within 1 m. Unlike scripts/run_koide_public_bag.py this takes any sequence of
the Zenodo record and parameter overrides, for experiments.

usage: online.py DATA_DIR SEQUENCE OUT_DIR --ws WS --rko RKO_ONLINE_NODE [--set KEY=VALUE ...]
"""

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
PROFILES = REPO / "tools" / "readme_media"
START_OFFSET_SEC = 4.0


def ground_truth(data, sequence):
    """outdoor_hard_01a and _01b share traj_lidar_outdoor_hard_01.txt."""
    name = sequence[:-1] if sequence[-1] in "ab" else sequence
    return data / "gt" / f"traj_lidar_{name}.txt"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data", type=Path)
    parser.add_argument("sequence")
    parser.add_argument("out", type=Path)
    parser.add_argument(
        "--ws", type=Path, required=True, help="colcon workspace with install/"
    )
    parser.add_argument("--rko", type=Path, required=True, help="RKO-LIO online_node")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override a localizer parameter (YAML value); repeatable",
    )
    parser.add_argument("--ros-domain-id", type=int, default=84)
    args = parser.parse_args()

    import koide_public_bag as data_tool
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from replay_session import ReplaySession, apply_overrides
    from rosgraph_msgs.msg import Clock
    from start_lifecycle_node import activate

    raw = args.data / args.sequence
    if not raw.is_dir():
        data_tool.extract_archive(args.data / f"{args.sequence}.zip", args.data)
    bag = data_tool.prepare_si_bag(raw, args.data / f"{args.sequence}_si")
    start_ns, clouds = data_tool.bag_cloud_times(bag)
    first_ns = start_ns + round(START_OFFSET_SEC * 1e9)
    expected = {stamp for stamp in clouds if stamp >= first_ns}
    reference = np.loadtxt(ground_truth(args.data, args.sequence))
    reference = reference[np.argsort(reference[:, 0])]
    row = reference[np.argmin(abs(reference[:, 0] - first_ns * 1e-9))]
    if abs(row[0] - first_ns * 1e-9) > 0.05:
        raise SystemExit("ground truth has no pose within 0.05 s of bag + 4 s")
    args.out.mkdir(parents=True, exist_ok=False)

    config = yaml.safe_load(
        (PROFILES / "koide_outdoor_hard_02b_online.yaml").read_text()
    )
    params = config["/**"]["ros__parameters"]
    params["map_path"] = str((args.data / "map_outdoor_hard.ply").resolve())
    apply_overrides(params, args.set)
    config_path = args.out / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))
    rko_config = args.out / "rko.yaml"
    qos_config = args.out / "points-qos.yaml"
    shutil.copyfile(PROFILES / "koide_outdoor_hard_02b_online_rko.yaml", rko_config)
    shutil.copyfile(PROFILES / "koide_reliable_points_qos.yaml", qos_config)

    localizer = (
        args.ws
        / "install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
    )
    rko = args.rko.expanduser().resolve()
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    environment["ROS_LOG_DIR"] = str(args.out / "ros-logs")
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [
            str(rko.parent.parent),
            str(localizer.parent.parent),
            environment.get("LD_LIBRARY_PATH", ""),
        ]
    )
    os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
    os.environ["ROS_LOG_DIR"] = environment["ROS_LOG_DIR"]

    clock_ns = [0]
    with ReplaySession(args.out, environment, "koide_imu_ab_online") as session:
        session.require_free_domain()
        session.record(accept_stamp=lambda stamp: stamp >= first_ns)
        node = session.node
        node.create_subscription(
            Clock,
            "/clock",
            lambda message: clock_ns.__setitem__(0, data_tool.stamp_ns(message.clock)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        initialpose = node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        session.spawn([str(rko), "--ros-args", "--params-file", str(rko_config)], "rko")
        session.spawn(
            [
                str(localizer),
                "--ros-args",
                "--params-file",
                str(config_path),
                "-r",
                f"cloud:={data_tool.POINTS}",
                "-r",
                f"imu:={data_tool.IMU}",
            ],
            "localizer",
        )
        activate(node, "/lidar_localization", 120.0)
        deadline = time.monotonic() + 30
        while True:
            session.require_running("rko", "localizer")
            reliable = {
                item.node_name
                for item in node.get_subscriptions_info_by_topic(data_tool.POINTS)
                if item.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            }
            if initialpose.get_subscription_count() and len(reliable) >= 2:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "RKO/localizer point subscriptions not ready; see logs"
                )
            session.spin_for(0.1)
        session.spin_for(2.0)
        print(f"Replaying {args.sequence} at 1x with RKO-LIO", flush=True)
        player = session.spawn(
            [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--clock",
                "--rate",
                "1",
                "--qos-profile-overrides-path",
                str(qos_config),
                "--topics",
                data_tool.POINTS,
                data_tool.IMU,
            ],
            "player",
        )
        sent = False
        while player.poll() is None:
            session.require_running("rko", "localizer")
            session.spin_for(0.05)
            if not sent and clock_ns[0] >= first_ns:
                message = PoseWithCovarianceStamped()
                message.header.frame_id = "map"
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                    round(row[0] * 1e9), 10**9
                )
                p, q = message.pose.pose.position, message.pose.pose.orientation
                p.x, p.y, p.z = map(float, row[1:4])
                q.x, q.y, q.z, q.w = map(float, row[4:8])
                message.pose.covariance[0] = message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = 0.07
                initialpose.publish(message)
                sent = True
        if player.returncode != 0 or not sent:
            raise RuntimeError(
                f"playback failed (exit {player.returncode}); see player.log"
            )
        session.spin_for(15.0)

    result = data_tool.evaluate(
        session.poses, reference, expected, session.diagnostic_stamps
    )
    if args.sequence != "outdoor_hard_02b":
        for key in (
            "off_map_window_sec",
            "off_map_matched_poses",
            "rmse_on_map_m",
            "rmse_off_map_m",
        ):
            result.pop(key, None)
    poses = np.asarray(session.poses)
    index = np.clip(
        np.searchsorted(reference[:, 0], poses[:, 0]), 0, len(reference) - 1
    )
    errors = np.linalg.norm(poses[:, 1:4] - reference[index, 1:4], axis=1)
    within = poses[errors < 1.0, 0]
    result["within_1m_until_sec"] = (
        float(within.max() - first_ns * 1e-9) if len(within) else None
    )
    result["max_error_m"] = float(errors.max())
    result["share_within_1m"] = float((errors < 1.0).mean())
    result.update({"sequence": args.sequence, "overrides": args.set})
    (args.out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"RMSE {result['rmse_m']:.3f} m, max {result['max_error_m']:.2f} m, coverage "
        f"{result['diagnostic_coverage']:.1%}, within 1 m until {result['within_1m_until_sec']} s; "
        f"{args.out}",
        flush=True,
    )


if __name__ == "__main__":
    main()
