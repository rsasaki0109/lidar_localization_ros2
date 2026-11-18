#!/usr/bin/env python3
"""Replay a Boreas fixture (boreas_prepare.py) through the localizer and score it.

The localizer starts at the ground-truth pose of the first scan and runs with a
preset (default param/boreas_ndt_velodyne.yaml) and its own map; the bag is played
at --rate with /clock. The published /pcl_pose is matched to the ground-truth lidar
pose with the same stamp (within 0.05 s), without any alignment.

Any fixture with the same layout replays the same way: FIXTURE_DIR/bag, map.ply (or
map.pcd) and reference.tum, the ground-truth pose of the cloud frame; pass its topics
and frame with --points-topic, --imu-topic and --base-frame. A JEPLO Go2 fixture is a
converted bag (jeplo_to_pc2_bag.py), a map in the motion-capture frame and gt/<run>.txt.

While the bag plays, the localizer's resident memory, peak memory (VmHWM), anonymous
memory (RssAnon, without the shared libraries' pages) and CPU time (utime + stime) are
sampled from /proc every --sample-sec into resources.csv, and each --companion's into
companionN_resources.csv. summary.json adds the CPU in cores (mean and p95 of the
intervals), CPU seconds per second of data and per scan, the peak and steady memory,
and the pose latency: from the control node receiving a cloud to receiving its pose.

Writes into OUT_DIR: localizer.yaml, localizer.log, player.log, estimate.tum,
alignment.jsonl, resources.csv and summary.json.

usage: boreas_replay.py FIXTURE_DIR OUT_DIR --ws WS [--preset YAML] [--set KEY=VALUE]
                        [--points-topic T] [--imu-topic T] [--base-frame F]
                        [--wrap COMMAND] [--companion COMMAND ...]

--wrap runs the localizer under COMMAND, e.g. 'taskset -c 0,1' or a script that puts
it into a memory cgroup and execs it. Each --companion COMMAND (e.g. an odometry node)
starts once the localizer is active, logging to companionN.log.
"""

import argparse
import json
import os
import shlex
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
    result["poses_over_1_m"] = int((error > 1.0).sum())
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
            "median_error_m": float(np.median(error)),
            "max_error_m": float(error.max()),
            "rmse_by_20_s": bins,
        }
    )
    return result


def bag_message_count(bag, topic):
    """The number of TOPIC messages in BAG, from its metadata, or None."""
    import rosbag2_py

    try:
        metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    except RuntimeError:
        return None
    for entry in metadata.topics_with_message_count:
        if entry.topic_metadata.name == topic:
            return entry.message_count
    return None


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
    parser.add_argument("--points-topic", default=POINTS)
    parser.add_argument("--imu-topic", default=IMU)
    parser.add_argument("--base-frame", default="velodyne")
    parser.add_argument("--wrap", default="", metavar="COMMAND")
    parser.add_argument("--companion", action="append", default=[], metavar="COMMAND")
    parser.add_argument("--sample-sec", type=float, default=1.0)
    args = parser.parse_args()
    points, imu = args.points_topic, args.imu_topic

    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from replay_session import (
        ReplaySession,
        ResourceMonitor,
        apply_overrides,
        find_process,
        initial_pose_parameters,
    )
    from sensor_msgs.msg import PointCloud2
    from soak_replay import latency_summary, pose_latencies
    from start_lifecycle_node import activate

    reference = np.loadtxt(args.fixture / "reference.tum")
    scans = len(reference)
    args.out.mkdir(parents=True, exist_ok=False)

    map_path = args.fixture / "map.ply"
    if not map_path.exists():
        map_path = args.fixture / "map.pcd"
    config = yaml.safe_load(args.preset.read_text())
    params = config["/**"]["ros__parameters"]
    params.update(
        {
            "map_path": str(map_path.resolve()),
            "use_pcd_map": True,
            "use_sim_time": True,
            "base_frame_id": args.base_frame,
            # The fixture has no odometry or twist; the post-processed GT velocity is
            # not a runtime input.
            "use_twist_prediction": False,
            "use_odom": False,
            # The ~1 MB clouds lose fragments on the best-effort sensor QoS: with it only
            # 38 of 97 scans of the 10 s window arrived.
            f"qos_overrides.{points}.subscription.reliability": "reliable",
        }
    )
    params.update(initial_pose_parameters(reference[0, 1:8]))
    apply_overrides(params, args.set)
    config_path = args.out / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))

    localizer = (
        args.ws
        / "install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOG_DIR"] = str(args.out / "ros-logs")
    accepted = [0]

    def count_accepted(values):
        if values.get("recovery_action", "").startswith("accept"):
            accepted[0] += 1

    # Monotonic times the control node received a cloud and its pose, by stamp in ns.
    cloud_wall, pose_wall = {}, {}

    def note_pose(stamp):
        pose_wall.setdefault(stamp, time.monotonic())
        return True

    def note_cloud(data):
        # A raw CDR PointCloud2: 4 bytes of encapsulation, then the header stamp.
        sec = int.from_bytes(data[4:8], "little", signed=True)
        nanosec = int.from_bytes(data[8:12], "little")
        cloud_wall.setdefault(sec * 10**9 + nanosec, time.monotonic())

    monitors = {}
    with ReplaySession(args.out, os.environ.copy(), "boreas_replay") as session:
        session.require_free_domain()
        session.record(accept_stamp=note_pose, on_status=count_accepted)
        session.node.create_subscription(
            PointCloud2,
            points,
            note_cloud,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE),
            raw=True,
        )
        process = session.spawn(
            [
                *shlex.split(args.wrap),
                str(localizer),
                "--ros-args",
                "--params-file",
                str(config_path),
                "-r",
                f"cloud:={points}",
                "-r",
                f"imu:={imu}",
            ],
            "localizer",
        )
        activate(session.node, "/lidar_localization", 180.0)
        monitors["localizer"] = ResourceMonitor(
            find_process(process.pid, "lidar_localization_node"),
            args.out / "resources.csv",
            args.sample_sec,
        )
        for index, companion in enumerate(args.companion):
            name = f"companion{index}"
            started = session.spawn(shlex.split(companion), name)
            monitors[name] = ResourceMonitor(
                started.pid, args.out / f"{name}_resources.csv", args.sample_sec
            )
        own = session.node.get_name()

        def subscribers():
            return {
                info.node_name
                for info in session.node.get_subscriptions_info_by_topic(points)
                if info.node_name != own
            }

        deadline = time.monotonic() + 30
        while len(subscribers()) < 1 + len(args.companion):
            session.require_running("localizer")
            if time.monotonic() > deadline:
                if "lidar_localization" in subscribers():
                    break
                raise RuntimeError("localizer did not subscribe; see localizer.log")
            session.spin_for(0.1)

        def poll():
            for monitor in monitors.values():
                monitor.poll()

        session.spin_for(2.0, poll)
        began = {name: monitor.elapsed() for name, monitor in monitors.items()}
        print(f"Replaying {scans} scans at {args.rate}x", flush=True)
        player = session.spawn(
            [
                "ros2",
                "bag",
                "play",
                str(args.fixture / "bag"),
                "--clock",
                "--rate",
                str(args.rate),
                "--topics",
                points,
                imu,
            ],
            "player",
        )
        while player.poll() is None:
            session.require_running("localizer")
            session.spin_for(0.05, poll)
        # The CPU is averaged over the playback and the 2 s after it, not the wait for
        # the last poses.
        windows = {
            name: (began[name], monitor.elapsed() + 2.0)
            for name, monitor in monitors.items()
        }
        session.spin_for(10.0, poll)
        for monitor in monitors.values():
            monitor.close()
    poses, diagnostics = session.poses, session.diagnostic_stamps
    scans = bag_message_count(args.fixture / "bag", points) or scans
    data_sec = (max(cloud_wall) - min(cloud_wall)) * 1e-9 if cloud_wall else None

    result = score(poses, reference, scans, diagnostics, accepted[0])
    result.update(
        {
            "fixture": str(args.fixture),
            "preset": str(args.preset),
            "overrides": args.set,
            "rate": args.rate,
            "wrap": args.wrap,
            "companions": args.companion,
            "clouds_received": len(cloud_wall),
            "processed_scans": len(diagnostics),
            "accepted_scans": accepted[0],
            "resources": monitors["localizer"].summary(
                data_sec, scans, windows["localizer"]
            ),
            **{
                f"{name}_resources": monitor.summary(data_sec, scans, windows[name])
                for name, monitor in monitors.items()
                if name != "localizer"
            },
            **latency_summary(
                [
                    1000.0 * latency
                    for _, latency in pose_latencies(cloud_wall, pose_wall)
                ]
            ),
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
