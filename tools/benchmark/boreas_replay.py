#!/usr/bin/env python3
"""Replay a Boreas fixture (boreas_prepare.py) through the localizer and score it.

The localizer starts at the ground-truth pose of the first scan and runs with a
preset (default param/boreas_ndt_velodyne.yaml) and its own map; the bag is played
at --rate with /clock. The published /pcl_pose is matched to the ground-truth lidar
pose with the same stamp (within 0.05 s), without any alignment.

Writes into OUT_DIR: localizer.yaml, localizer.log, player.log, estimate.tum,
alignment.jsonl and summary.json.

usage: boreas_replay.py FIXTURE_DIR OUT_DIR --ws WS [--preset YAML] [--set KEY=VALUE]
"""

import argparse
import contextlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
POINTS = "/velodyne_points"
IMU = "/imu/data"
MATCH_SEC = 0.05


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def score(poses, reference, scans, diagnostics, accepted):
    poses = np.asarray(poses, dtype=float).reshape(-1, 8)
    reference = reference[np.argsort(reference[:, 0])]
    result = {
        "scans": scans,
        "poses": len(poses),
        "diagnostic_coverage": len(diagnostics) / scans,
        "accepted_share": accepted / scans,
    }
    if not len(poses):
        return result
    index = np.clip(
        np.searchsorted(reference[:, 0], poses[:, 0]), 1, len(reference) - 1
    )
    earlier = abs(reference[index - 1, 0] - poses[:, 0]) < abs(
        reference[index, 0] - poses[:, 0]
    )
    index = np.where(earlier, index - 1, index)
    matched = abs(reference[index, 0] - poses[:, 0]) < MATCH_SEC
    diff = poses[matched, 1:4] - reference[index[matched], 1:4]
    error = np.linalg.norm(diff, axis=1)
    horizontal = np.hypot(diff[:, 0], diff[:, 1])
    t = poses[matched, 0] - reference[0, 0]
    bins = {}
    for start in range(0, int(t.max()) + 1, 20) if len(t) else []:
        inside = (t >= start) & (t < start + 20)
        if inside.any():
            bins[f"{start}-{start + 20}"] = round(
                float(np.sqrt(np.mean(error[inside] ** 2))), 3
            )
    result.update(
        {
            "matched_poses": int(matched.sum()),
            "rmse_m": float(np.sqrt(np.mean(error**2))),
            "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal**2))),
            "z_rmse_m": float(np.sqrt(np.mean(diff[:, 2] ** 2))),
            "p95_error_m": float(np.percentile(error, 95)),
            "max_error_m": float(error.max()),
            "rmse_by_20_s": bins,
        }
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("fixture", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument(
        "--ws", type=Path, required=True, help="colcon workspace with install/"
    )
    parser.add_argument(
        "--preset", type=Path, default=REPO / "param/boreas_ndt_velodyne.yaml"
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override a preset parameter (YAML value); repeatable",
    )
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--ros-domain-id", type=int, default=87)
    args = parser.parse_args()

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from start_lifecycle_node import activate

    reference = np.loadtxt(args.fixture / "reference.tum")
    scans = len(reference)
    first = reference[0]
    args.out.mkdir(parents=True, exist_ok=False)

    config = yaml.safe_load(args.preset.read_text())
    params = config["/**"]["ros__parameters"]
    params.update(
        {
            "map_path": str((args.fixture / "map.ply").resolve()),
            "use_pcd_map": True,
            "use_sim_time": True,
            "base_frame_id": "velodyne",
            # The fixture has no odometry or twist; the post-processed GT velocity is
            # not a runtime input.
            "use_twist_prediction": False,
            "use_odom": False,
            # The ~1 MB clouds lose fragments on the best-effort sensor QoS: with it only
            # 38 of 97 scans of the 10 s window arrived.
            f"qos_overrides.{POINTS}.subscription.reliability": "reliable",
            "set_initial_pose": True,
            "initial_pose_x": float(first[1]),
            "initial_pose_y": float(first[2]),
            "initial_pose_z": float(first[3]),
            "initial_pose_qx": float(first[4]),
            "initial_pose_qy": float(first[5]),
            "initial_pose_qz": float(first[6]),
            "initial_pose_qw": float(first[7]),
        }
    )
    for item in args.set:
        key, value = item.split("=", 1)
        params[key] = yaml.safe_load(value)
    config_path = args.out / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))

    localizer = (
        args.ws
        / "install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
    )
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    environment["ROS_LOG_DIR"] = str(args.out / "ros-logs")
    os.environ.update({k: environment[k] for k in ("ROS_DOMAIN_ID", "ROS_LOG_DIR")})

    poses, diagnostics, accepted = [], set(), [0]
    processes = []
    rclpy.init(args=[])
    node = rclpy.create_node(f"boreas_replay_{os.getpid()}")
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if set(node.get_node_names()) - {node.get_name()}:
            raise SystemExit(
                "ROS domain is already in use; pass another --ros-domain-id"
            )
        with contextlib.ExitStack() as stack:
            tum = stack.enter_context((args.out / "estimate.tum").open("w"))
            diagnostic_log = stack.enter_context(
                (args.out / "alignment.jsonl").open("w")
            )

            def pose_received(message):
                p, q = message.pose.pose.position, message.pose.pose.orientation
                values = [
                    stamp_ns(message.header.stamp) * 1e-9,
                    p.x,
                    p.y,
                    p.z,
                    q.x,
                    q.y,
                    q.z,
                    q.w,
                ]
                poses.append(values)
                tum.write(" ".join(f"{v:.9f}" for v in values) + "\n")

            def diagnostic_received(message):
                diagnostics.add(stamp_ns(message.header.stamp))
                for entry in message.status:
                    values = {v.key: v.value for v in entry.values}
                    if values.get("recovery_action", "").startswith("accept"):
                        accepted[0] += 1
                    diagnostic_log.write(
                        json.dumps(
                            {
                                "stamp_ns": stamp_ns(message.header.stamp),
                                "message": entry.message,
                                "values": values,
                            }
                        )
                        + "\n"
                    )

            reliable_latched = QoSProfile(
                depth=1000,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(
                PoseWithCovarianceStamped, "/pcl_pose", pose_received, reliable_latched
            )
            node.create_subscription(
                DiagnosticArray,
                "/alignment_status",
                diagnostic_received,
                reliable_latched,
            )

            def spawn(command, name):
                log = stack.enter_context((args.out / f"{name}.log").open("w"))
                process = subprocess.Popen(
                    command,
                    env=environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append(process)
                return process

            localizer_process = spawn(
                [
                    str(localizer),
                    "--ros-args",
                    "--params-file",
                    str(config_path),
                    "-r",
                    f"cloud:={POINTS}",
                    "-r",
                    f"imu:={IMU}",
                ],
                "localizer",
            )
            activate(node, "/lidar_localization", 180.0)
            deadline = time.monotonic() + 30
            while not node.get_subscriptions_info_by_topic(POINTS):
                if localizer_process.poll() is not None or time.monotonic() > deadline:
                    raise SystemExit("localizer did not subscribe; see localizer.log")
                rclpy.spin_once(node, timeout_sec=0.1)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(f"Replaying {scans} scans at {args.rate}x", flush=True)
            player = spawn(
                [
                    "ros2",
                    "bag",
                    "play",
                    str(args.fixture / "bag"),
                    "--clock",
                    "--rate",
                    str(args.rate),
                    "--topics",
                    POINTS,
                    IMU,
                ],
                "player",
            )
            while player.poll() is None:
                if localizer_process.poll() is not None:
                    raise SystemExit("localizer exited; see localizer.log")
                rclpy.spin_once(node, timeout_sec=0.05)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for process in processes:
            if process.poll() is None:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(process.pid, 2)
        for process in processes:
            with contextlib.suppress(subprocess.TimeoutExpired):
                process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()

    result = score(poses, reference, scans, diagnostics, accepted[0])
    result.update(
        {
            "fixture": str(args.fixture),
            "preset": str(args.preset),
            "overrides": args.set,
            "rate": args.rate,
        }
    )
    (args.out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"RMSE {result.get('rmse_m', float('nan')):.3f} m (horizontal "
        f"{result.get('horizontal_rmse_m', float('nan')):.3f}), max "
        f"{result.get('max_error_m', float('nan')):.2f} m, {result['poses']} poses for "
        f"{scans} scans, accepted {result['accepted_share']:.0%}; {args.out}",
        flush=True,
    )


if __name__ == "__main__":
    main()
