#!/usr/bin/env python3
"""Run the localizer for a long time on a looped Boreas fixture, and watch it.

One localizer runs for --minutes while the fixture's bag (boreas_prepare.py) plays
lap after lap at 1x. Each lap's clouds, IMU samples and /clock are shifted by the
laps before it plus a 1 s gap, so time only moves forward. At the start of every lap
after the first, /initialpose puts the localizer back at the ground-truth start pose,
as an operator re-localizing it would. Every 5 s the localizer process's resident
memory and CPU use are sampled from /proc.

Writes into OUT_DIR: localizer.yaml, localizer.log, estimate.tum, alignment.jsonl,
resources.csv (wall_sec, sim_sec, rss_mb, cpu_percent, threads) and summary.json, with
each lap's RMSE, acceptance and alignment time, and the memory trend.

usage: soak_replay.py FIXTURE_DIR OUT_DIR --ws WS [--minutes 60] [--preset YAML]
                      [--set KEY=VALUE ...]
"""

import argparse
import json
import os
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
LAP_GAP_SEC = 1.0
SAMPLE_SEC = 5.0


def lap_of(stamp_sec, start_sec, period_sec):
    """The lap a shifted stamp falls in."""
    return int(np.floor((stamp_sec - start_sec + 0.5 * LAP_GAP_SEC) / period_sec))


def score_laps(poses, reference, period_sec, statuses, lap_count):
    """Per lap: poses, RMSE and max error against the reference, accepted share and
    alignment time. POSES are (stamp, x, y, z, ...) rows with shifted stamps;
    STATUSES are (stamp_sec, values) of /alignment_status. Stamps outside the laps
    (the initial pose's, published before /clock starts) are left out."""
    reference = reference[np.argsort(reference[:, 0])]
    start = reference[0, 0]
    laps = {}
    for row in poses:
        lap = lap_of(row[0], start, period_sec)
        if not 0 <= lap < lap_count:
            continue
        bag_stamp = row[0] - lap * period_sec
        i = int(np.argmin(abs(reference[:, 0] - bag_stamp)))
        entry = laps.setdefault(lap, {"errors": [], "poses": 0, "status": []})
        entry["poses"] += 1
        if abs(reference[i, 0] - bag_stamp) < MATCH_SEC:
            entry["errors"].append(float(np.linalg.norm(row[1:4] - reference[i, 1:4])))
    for stamp, values in statuses:
        lap = lap_of(stamp, start, period_sec)
        if not 0 <= lap < lap_count:
            continue
        laps.setdefault(lap, {"errors": [], "poses": 0, "status": []})["status"].append(
            values
        )
    result = []
    for lap in sorted(laps):
        entry = laps[lap]
        errors = np.array(entry["errors"])
        accepted = sum(
            1
            for v in entry["status"]
            if v.get("recovery_action", "").startswith("accept")
        )
        align_ms = [
            1000.0 * float(v["alignment_time_sec"])
            for v in entry["status"]
            if float(v.get("alignment_time_sec", 0.0)) > 0.0
        ]
        result.append(
            {
                "lap": lap,
                "poses": entry["poses"],
                "matched": len(errors),
                "rmse_m": round(float(np.sqrt(np.mean(errors**2))), 3)
                if len(errors)
                else None,
                "max_error_m": round(float(errors.max()), 3) if len(errors) else None,
                "accepted": accepted,
                "align_ms_median": round(float(np.median(align_ms)), 1)
                if align_ms
                else None,
                "align_ms_p95": round(float(np.percentile(align_ms, 95)), 1)
                if align_ms
                else None,
            }
        )
    return result


def memory_trend(samples, skip_sec):
    """Resident memory at the start and the end, and its slope in MB per hour over
    the samples after SKIP_SEC (the warm-up). SAMPLES are (wall_sec, rss_mb) rows."""
    samples = np.asarray(samples, dtype=float).reshape(-1, 2)
    after = samples[samples[:, 0] >= skip_sec]
    if len(after) < 2:
        return {}
    slope = np.polyfit(after[:, 0] / 3600.0, after[:, 1], 1)[0]
    return {
        "rss_mb_first": round(float(after[0, 1]), 1),
        "rss_mb_last": round(float(after[-1, 1]), 1),
        "rss_mb_max": round(float(samples[:, 1].max()), 1),
        "rss_mb_per_hour": round(float(slope), 2),
    }


def process_sample(pid, previous):
    """(rss_mb, cpu_percent, threads, ticks) of PID; cpu over the time since PREVIOUS,
    a (wall_sec, ticks) pair or None."""
    status = Path(f"/proc/{pid}/status").read_text()
    fields = dict(line.split(":", 1) for line in status.splitlines() if ":" in line)
    rss_mb = int(fields["VmRSS"].split()[0]) / 1024.0
    threads = int(fields["Threads"])
    stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    ticks = int(stat[11]) + int(stat[12])
    cpu = None
    if previous is not None:
        wall = time.monotonic() - previous[0]
        cpu = 100.0 * (ticks - previous[1]) / os.sysconf("SC_CLK_TCK") / wall
    return rss_mb, cpu, threads, ticks


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("fixture", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--ws", type=Path, required=True)
    parser.add_argument("--minutes", type=float, default=60.0)
    parser.add_argument(
        "--preset", type=Path, default=REPO / "param/boreas_ndt_velodyne.yaml"
    )
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--ros-domain-id", type=int, default=89)
    args = parser.parse_args()

    import rclpy
    import rosbag2_py
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from replay_session import (
        ReplaySession,
        apply_overrides,
        initial_pose_parameters,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from start_lifecycle_node import activate

    reference = np.loadtxt(args.fixture / "reference.tum")
    args.out.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.preset.read_text())
    params = config["/**"]["ros__parameters"]
    params.update(
        {
            "map_path": str((args.fixture / "map.ply").resolve()),
            "use_pcd_map": True,
            "use_sim_time": True,
            "base_frame_id": "velodyne",
            "use_twist_prediction": False,
            "use_odom": False,
            f"qos_overrides.{POINTS}.subscription.reliability": "reliable",
        }
        | initial_pose_parameters(reference[0, 1:8])
    )
    apply_overrides(params, args.set)
    config_path = args.out / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))
    global_frame = params.get("global_frame_id", "map")

    types = {POINTS: PointCloud2, IMU: Imu}

    def lap_messages(deserialize=True):
        """The bag's messages in order, read afresh each lap so the clouds (about
        1.7 GB) are not held in this process: (bag_ns, topic, message)."""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(args.fixture / "bag"), storage_id=""),
            rosbag2_py.ConverterOptions("", ""),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=[POINTS, IMU]))
        while reader.has_next():
            topic, data, bag_ns = reader.read_next()
            yield (
                bag_ns,
                topic,
                (deserialize_message(data, types[topic]) if deserialize else None),
            )

    stamps = [bag_ns for bag_ns, _, _ in lap_messages(deserialize=False)]
    first_ns, last_ns = stamps[0], stamps[-1]
    period_ns = last_ns - first_ns + int(LAP_GAP_SEC * 1e9)
    lap_count = max(1, int(args.minutes * 60e9 // period_ns))
    start_sec = reference[0, 0]

    localizer = (
        args.ws
        / "install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOG_DIR"] = str(args.out / "ros-logs")
    samples = []
    with ReplaySession(args.out, os.environ.copy(), "soak_replay") as session:
        session.require_free_domain()
        session.record()
        process = session.spawn(
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
        activate(session.node, "/lidar_localization", 180.0)
        node = session.node
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        publishers = {
            POINTS: node.create_publisher(PointCloud2, POINTS, reliable),
            IMU: node.create_publisher(Imu, IMU, QoSProfile(depth=500)),
        }
        clock_pub = node.create_publisher(Clock, "/clock", 10)
        pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        deadline = time.monotonic() + 30
        while not node.get_subscriptions_info_by_topic(POINTS):
            session.require_running("localizer")
            if time.monotonic() > deadline:
                raise RuntimeError("localizer did not subscribe; see localizer.log")
            session.spin_for(0.1)
        session.spin_for(2.0)

        resources = (args.out / "resources.csv").open("w")
        resources.write("wall_sec,sim_sec,rss_mb,cpu_percent,threads\n")
        began = time.monotonic()
        previous = None
        next_sample = began

        def stamp_of(ns):
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(int(ns), 10**9)
            return msg.clock

        def wait_until(wall, sim_ns):
            nonlocal previous, next_sample
            while True:
                now = time.monotonic()
                if now >= next_sample:
                    rss, cpu, threads, ticks = process_sample(process.pid, previous)
                    previous = (now, ticks)
                    elapsed = now - began
                    samples.append((elapsed, rss))
                    resources.write(
                        f"{elapsed:.1f},{sim_ns * 1e-9:.3f},{rss:.1f},"
                        f"{'' if cpu is None else f'{cpu:.1f}'},{threads}\n"
                    )
                    resources.flush()
                    next_sample += SAMPLE_SEC
                if now >= wall:
                    return
                session.require_running("localizer")
                rclpy.spin_once(node, timeout_sec=min(0.005, wall - now))

        print(
            f"Soaking {lap_count} laps of {period_ns * 1e-9:.1f} s "
            f"({lap_count * period_ns / 60e9:.1f} min)",
            flush=True,
        )
        for lap in range(lap_count):
            offset = lap * period_ns
            lap_wall = began + offset * 1e-9
            if lap:
                # Put the localizer back at the start pose, as an operator would.
                wait_until(lap_wall - 0.5, first_ns + offset)
                clock_pub.publish(Clock(clock=stamp_of(first_ns + offset)))
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = global_frame
                pose.header.stamp = stamp_of(int(start_sec * 1e9) + offset)
                x, y, z, qx, qy, qz, qw = reference[0, 1:8]
                pose.pose.pose.position.x = float(x)
                pose.pose.pose.position.y = float(y)
                pose.pose.pose.position.z = float(z)
                pose.pose.pose.orientation.x = float(qx)
                pose.pose.pose.orientation.y = float(qy)
                pose.pose.pose.orientation.z = float(qz)
                pose.pose.pose.orientation.w = float(qw)
                pose_pub.publish(pose)
            for bag_ns, topic, message in lap_messages():
                sim_ns = bag_ns + offset
                wait_until(lap_wall + (bag_ns - first_ns) * 1e-9, sim_ns)
                clock_pub.publish(Clock(clock=stamp_of(sim_ns)))
                stamp = message.header.stamp
                shifted = stamp.sec * 10**9 + stamp.nanosec + offset
                message.header.stamp = stamp_of(shifted)
                publishers[topic].publish(message)
            print(f"lap {lap + 1}/{lap_count} done", flush=True)
        wait_until(time.monotonic() + 5.0, last_ns + offset)
        alive = process.poll() is None
        resources.close()

    poses = np.asarray(session.poses, dtype=float).reshape(-1, 8)
    statuses = []
    for line in (args.out / "alignment.jsonl").read_text().splitlines():
        record = json.loads(line)
        statuses.append((record["stamp_ns"] * 1e-9, record["values"]))
    laps = score_laps(poses, reference, period_ns * 1e-9, statuses, lap_count)
    summary = {
        "fixture": str(args.fixture),
        "preset": str(args.preset),
        "overrides": args.set,
        "laps": lap_count,
        "lap_sec": round(period_ns * 1e-9, 1),
        "localizer_alive_at_end": alive,
        "memory": memory_trend(samples, skip_sec=period_ns * 1e-9),
        "per_lap": laps,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    rmse = [lap["rmse_m"] for lap in laps if lap["rmse_m"] is not None]
    print(
        f"{lap_count} laps, alive {alive}; RMSE per lap {min(rmse):.3f}-{max(rmse):.3f} m; "
        f"memory {summary['memory']}; {args.out}",
        flush=True,
    )


if __name__ == "__main__":
    main()
