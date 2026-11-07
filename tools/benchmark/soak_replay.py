#!/usr/bin/env python3
"""Run the localizer for a long time on a looped Boreas fixture, and watch it.

One localizer runs for --minutes while the fixture's bag (boreas_prepare.py) plays
lap after lap at 1x. Each lap's clouds, IMU samples and /clock are shifted by the
laps before it plus a 1 s gap, so time only moves forward. At the start of every lap
after the first, /initialpose puts the localizer back at the ground-truth start pose,
as an operator re-localizing it would. Every 5 s the localizer process's resident
memory and CPU use are sampled from /proc.

Each --fault KIND@LAP:START[+DURATION] injects one fault START seconds into lap LAP
(counted from 0) for DURATION seconds: lidar or imu drops those messages, stall stops
the localizer process (SIGSTOP, then SIGCONT), and reinit sends /initialpose at the
ground-truth pose there, while moving. The laps without one are the control.

Writes into OUT_DIR: localizer.yaml, localizer.log, estimate.tum, alignment.jsonl,
resources.csv (wall_sec, sim_sec, rss_mb, cpu_percent, threads) and summary.json, with
each lap's RMSE, acceptance and alignment time, the memory trend, and per fault the
poses published during it, the largest error from its start to SETTLE_SEC after its
end, the time from its end until the error stays under RECOVERED_M for the rest of
the lap, and the RMSE after the settling time.

usage: soak_replay.py FIXTURE_DIR OUT_DIR --ws WS [--minutes 60] [--preset YAML]
                      [--set KEY=VALUE ...] [--fault KIND@LAP:START[+DURATION] ...]
                      [--wrap COMMAND]

--wrap runs the localizer under COMMAND, e.g. --wrap 'heaptrack -o OUT_DIR/heap';
the samples are still taken from the localizer process itself.
"""

import argparse
import contextlib
import json
import os
import shlex
import signal
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
FAULT_KINDS = ("lidar", "imu", "stall", "reinit")
SETTLE_SEC = 10.0
RECOVERED_M = 0.5


def lap_of(stamp_sec, start_sec, period_sec):
    """The lap a shifted stamp falls in."""
    return int(np.floor((stamp_sec - start_sec + 0.5 * LAP_GAP_SEC) / period_sec))


def parse_fault(text):
    """A KIND@LAP:START[+DURATION] fault as a dict."""
    kind, _, rest = text.partition("@")
    lap, _, window = rest.partition(":")
    start, _, duration = window.partition("+")
    try:
        fault = {
            "kind": kind,
            "lap": int(lap),
            "start_sec": float(start),
            "duration_sec": float(duration or 0.0),
        }
    except ValueError:
        fault = None
    if (
        fault is None
        or kind not in FAULT_KINDS
        or fault["lap"] < 0
        or fault["start_sec"] < 0
        or fault["duration_sec"] < 0
    ):
        raise argparse.ArgumentTypeError(
            f"expected KIND@LAP:START[+DURATION] with KIND one of "
            f"{', '.join(FAULT_KINDS)}, got {text!r}"
        )
    return fault


def pose_errors(poses, reference, period_sec, lap_count):
    """(lap, stamp_sec, error_m) of each pose in the laps, against the reference
    shifted to that lap; the error is None with no reference stamp within MATCH_SEC.
    POSES are (stamp, x, y, z, ...) rows with shifted stamps. Stamps outside the laps
    (the initial pose's, published before /clock starts) are left out."""
    reference = reference[np.argsort(reference[:, 0])]
    start = reference[0, 0]
    rows = []
    for row in poses:
        lap = lap_of(row[0], start, period_sec)
        if not 0 <= lap < lap_count:
            continue
        bag_stamp = row[0] - lap * period_sec
        i = int(np.argmin(abs(reference[:, 0] - bag_stamp)))
        error = None
        if abs(reference[i, 0] - bag_stamp) < MATCH_SEC:
            error = float(np.linalg.norm(row[1:4] - reference[i, 1:4]))
        rows.append((lap, float(row[0]), error))
    return rows


def score_laps(poses, reference, period_sec, statuses, lap_count):
    """Per lap: poses, RMSE and max error against the reference, accepted share and
    alignment time. STATUSES are (stamp_sec, values) of /alignment_status."""
    start = reference[:, 0].min()
    laps = {}
    for lap, _, error in pose_errors(poses, reference, period_sec, lap_count):
        entry = laps.setdefault(lap, {"errors": [], "poses": 0, "status": []})
        entry["poses"] += 1
        if error is not None:
            entry["errors"].append(error)
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


def score_faults(errors, start_sec, period_sec, faults):
    """Per fault: the poses published during it, the largest error from its start to
    SETTLE_SEC after its end, the seconds from its end until the error stays under
    RECOVERED_M for the rest of the lap (None if it does not), and the RMSE after the
    settling time. ERRORS are pose_errors() rows; START_SEC is the first lap's start."""
    result = []
    for fault in faults:
        begin = start_sec + fault["lap"] * period_sec + fault["start_sec"]
        end = begin + fault["duration_sec"]
        lap = [(t, e) for n, t, e in errors if n == fault["lap"]]
        matched = [(t, e) for t, e in lap if e is not None and t >= begin]
        window = [e for t, e in matched if t < end + SETTLE_SEC]
        settled = np.array([e for t, e in matched if t >= end + SETTLE_SEC])
        after = [(t, e) for t, e in matched if t >= end]
        bad = [t for t, e in after if e >= RECOVERED_M]
        later = [t for t, _ in after if not bad or t > bad[-1]]
        result.append(
            fault
            | {
                "poses_during": sum(1 for t, _ in lap if begin <= t < end),
                "max_error_m": round(max(window), 3) if window else None,
                "recovered_after_sec": round(later[0] - end, 2) if later else None,
                "rmse_settled_m": round(float(np.sqrt(np.mean(settled**2))), 3)
                if len(settled)
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


def localizer_pid(group, name="lidar_localization_node"):
    """The process in process group GROUP whose name is NAME (as /proc shows it, cut
    to 15 characters), or GROUP itself when there is none."""
    for stat in Path("/proc").glob("[0-9]*/stat"):
        try:
            head, tail = stat.read_text().rsplit(")", 1)
        except OSError:
            continue
        comm = head.split("(", 1)[1]
        if int(tail.split()[2]) == group and comm == name[:15]:
            return int(stat.parent.name)
    return group


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
    parser.add_argument(
        "--fault",
        action="append",
        default=[],
        type=parse_fault,
        metavar="KIND@LAP:START[+DURATION]",
    )
    parser.add_argument("--wrap", default="", metavar="COMMAND")
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
    reference = reference[np.argsort(reference[:, 0])]
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
    for fault in args.fault:
        if fault["lap"] >= lap_count:
            parser.error(f"a fault is in lap {fault['lap']} of laps 0-{lap_count - 1}")
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
                *shlex.split(args.wrap),
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
        pid = localizer_pid(process.pid)
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

        def publish_pose(row, offset):
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = global_frame
            pose.header.stamp = stamp_of(round(row[0] * 1e9) + offset)
            position, orientation = pose.pose.pose.position, pose.pose.pose.orientation
            position.x, position.y, position.z = (float(v) for v in row[1:4])
            orientation.x, orientation.y, orientation.z, orientation.w = (
                float(v) for v in row[4:8]
            )
            pose_pub.publish(pose)

        # (wall_sec, action) of the faults that act at a moment, soonest first.
        pending = []

        def wait_until(wall, sim_ns):
            nonlocal previous, next_sample
            while True:
                now = time.monotonic()
                while pending and pending[0][0] <= now:
                    pending.pop(0)[1]()
                if now >= next_sample:
                    rss, cpu, threads, ticks = process_sample(pid, previous)
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

        def signal_localizer(sig):
            with contextlib.suppress(ProcessLookupError):
                os.kill(pid, sig)

        start_ns = round(start_sec * 1e9)
        try:
            for lap in range(lap_count):
                offset = lap * period_ns
                lap_wall = began + offset * 1e-9
                if lap:
                    # Put the localizer back at the start pose, as an operator would.
                    wait_until(lap_wall - 0.5, first_ns + offset)
                    clock_pub.publish(Clock(clock=stamp_of(first_ns + offset)))
                    publish_pose(reference[0], offset)
                faults = [fault for fault in args.fault if fault["lap"] == lap]
                for fault in faults:
                    at = lap_wall + fault["start_sec"]
                    if fault["kind"] == "stall":
                        pending.append((at, lambda: signal_localizer(signal.SIGSTOP)))
                        pending.append(
                            (
                                at + fault["duration_sec"],
                                lambda: signal_localizer(signal.SIGCONT),
                            )
                        )
                    elif fault["kind"] == "reinit":
                        i = int(
                            np.argmin(
                                abs(reference[:, 0] - start_sec - fault["start_sec"])
                            )
                        )
                        pending.append(
                            (
                                at,
                                lambda row=reference[i], o=offset: publish_pose(row, o),
                            )
                        )
                pending.sort(key=lambda event: event[0])
                dropped = {
                    POINTS: [f for f in faults if f["kind"] == "lidar"],
                    IMU: [f for f in faults if f["kind"] == "imu"],
                }
                for bag_ns, topic, message in lap_messages():
                    sim_ns = bag_ns + offset
                    wait_until(lap_wall + (bag_ns - first_ns) * 1e-9, sim_ns)
                    clock_pub.publish(Clock(clock=stamp_of(sim_ns)))
                    stamp = message.header.stamp
                    stamp_ns = stamp.sec * 10**9 + stamp.nanosec
                    into_lap = (stamp_ns - start_ns) * 1e-9
                    if any(
                        f["start_sec"] <= into_lap < f["start_sec"] + f["duration_sec"]
                        for f in dropped[topic]
                    ):
                        continue
                    message.header.stamp = stamp_of(stamp_ns + offset)
                    publishers[topic].publish(message)
                print(f"lap {lap + 1}/{lap_count} done", flush=True)
            wait_until(time.monotonic() + 5.0, last_ns + offset)
        finally:
            signal_localizer(signal.SIGCONT)
        alive = process.poll() is None
        resources.close()

    poses = np.asarray(session.poses, dtype=float).reshape(-1, 8)
    statuses = []
    for line in (args.out / "alignment.jsonl").read_text().splitlines():
        record = json.loads(line)
        statuses.append((record["stamp_ns"] * 1e-9, record["values"]))
    laps = score_laps(poses, reference, period_ns * 1e-9, statuses, lap_count)
    faults = score_faults(
        pose_errors(poses, reference, period_ns * 1e-9, lap_count),
        start_sec,
        period_ns * 1e-9,
        args.fault,
    )
    summary = {
        "fixture": str(args.fixture),
        "preset": str(args.preset),
        "overrides": args.set,
        "wrap": args.wrap,
        "laps": lap_count,
        "lap_sec": round(period_ns * 1e-9, 1),
        "localizer_alive_at_end": alive,
        "memory": memory_trend(samples, skip_sec=period_ns * 1e-9),
        "per_lap": laps,
        "faults": faults,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    rmse = [lap["rmse_m"] for lap in laps if lap["rmse_m"] is not None]
    print(
        f"{lap_count} laps, alive {alive}; RMSE per lap {min(rmse):.3f}-{max(rmse):.3f} m; "
        f"memory {summary['memory']}; {args.out}",
        flush=True,
    )
    for fault in faults:
        print(json.dumps(fault), flush=True)


if __name__ == "__main__":
    main()
