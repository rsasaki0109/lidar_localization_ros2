#!/usr/bin/env python3
"""Replay one window of the Autoware Istanbul localization bag through the
localizer of one colcon workspace and score /pcl_pose against the bag's GNSS
pose (horizontal, z, 3D and yaw errors).

usage: replay.py --ws WS --fixture DIR --bag BAG --map PCD --start SEC --duration SEC
                 --out DIR [--preset FILE] [--domain N]

FIXTURE is prepare.py's output directory. The run directory gets params.yaml,
node.log, est.tum (record_pose.py) and score.json.
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
CLOUD = "/localization/util/downsample/pointcloud"
TWIST = "/localization/twist_estimator/twist_with_covariance"
# The ROS 2 CLI and Python the distro was built for (override when `python3` differs).
ROS2 = os.environ.get("ROS2_CLI", "ros2")
PYTHON = os.environ.get("ROS_PYTHON", "python3")


def load_reference(fixture):
    with open(fixture / "gnss_reference.csv") as f:
        ref = np.array([[float(v) for v in row] for row in list(csv.reader(f))[1:]])
    # The first GNSS messages repeat a stale pose 1.9 m and 146 deg off the
    # next ones; drop everything before that jump.
    steps = np.hypot(np.diff(ref[:, 1]), np.diff(ref[:, 2]))
    early = np.nonzero((steps > 1.0) & (ref[1:, 0] < ref[0, 0] + 1.0))[0]
    return ref[early[-1] + 1 :] if len(early) else ref


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def shell(ws, command):
    return f"source {ws}/install/setup.bash && {command}"


def run_window(args):
    out, fixture = Path(args.out), Path(args.fixture)
    out.mkdir(parents=True, exist_ok=True)
    ref = load_reference(fixture)
    # Header stamps (and the bag's own /clock) are 2371531.4 s before the bag's
    # receive time, so map the window start through the clouds' stamps.
    time_map = np.loadtxt(fixture / "time_map.txt")
    bag_start = float((fixture / "bag_start.txt").read_text())
    t_start = float(np.interp(bag_start + args.start, time_map[:, 0], time_map[:, 1]))
    _, x, y, z, qx, qy, qz, qw = ref[
        min(int(np.searchsorted(ref[:, 0], t_start)), len(ref) - 1)
    ]

    params = yaml.safe_load((REPO / "param" / args.preset).read_text())
    p = params["/**"]["ros__parameters"]
    p.update(
        {
            "map_path": str(args.map),
            "use_sim_time": True,
            "set_initial_pose": True,
            "initial_pose_x": float(x),
            "initial_pose_y": float(y),
            "initial_pose_z": float(z),
            "initial_pose_qx": float(qx),
            "initial_pose_qy": float(qy),
            "initial_pose_qz": float(qz),
            "initial_pose_qw": float(qw),
        }
    )
    for item in args.set:
        key, value = item.split("=", 1)
        p[key] = yaml.safe_load(value)
    (out / "params.yaml").write_text(yaml.safe_dump(params))

    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), RCUTILS_COLORIZED_OUTPUT="0")
    with (
        open(out / "node.log", "w") as node_log,
        open(out / "recorder.log", "w") as recorder_log,
    ):
        play_sec = replay(args, out, env, node_log, recorder_log)

    score = evaluate(out / "est.tum", ref, t_start, t_start + args.duration)
    score.update(
        {
            "ws": str(args.ws),
            "preset": args.preset,
            "overrides": args.set,
            "start": args.start,
            "duration": args.duration,
            "wall_play_sec": round(play_sec, 1),
        }
    )
    (out / "score.json").write_text(json.dumps(score, indent=1))
    print(json.dumps(score))


def replay(args, out, env, node_log, recorder_log):
    """Start the localizer, play the window, stop; return the play wall time."""
    node_bin = f"{args.ws}/install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
    node = subprocess.Popen(
        [
            "bash",
            "-c",
            shell(
                args.ws,
                f"exec {node_bin} --ros-args --params-file {out}/params.yaml "
                f"-r cloud:={CLOUD} -r twist:={TWIST}",
            ),
        ],
        stdout=node_log,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )
    try:
        deadline = time.time() + 120.0
        while (
            subprocess.run(
                [
                    "bash",
                    "-c",
                    shell(args.ws, f"{ROS2} lifecycle get /lidar_localization"),
                ],
                env=env,
                capture_output=True,
                text=True,
            ).returncode
            != 0
        ):
            if time.time() > deadline or node.poll() is not None:
                raise RuntimeError("localizer did not come up; see node.log")
            time.sleep(2.0)
        for transition in ("configure", "activate"):
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    shell(
                        args.ws,
                        f"{ROS2} lifecycle set /lidar_localization {transition}",
                    ),
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=900,
            )
            if result.returncode != 0 or "successful" not in result.stdout:
                raise RuntimeError(
                    f"{transition} failed: {result.stdout} {result.stderr}"
                )
    except BaseException:
        os.killpg(node.pid, signal.SIGINT)
        raise
    recorder = subprocess.Popen(
        [
            "bash",
            "-c",
            shell(
                args.ws, f"exec {PYTHON} {REPO}/tools/benchmark/record_pose.py {out}"
            ),
        ],
        stdout=recorder_log,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )
    time.sleep(2.0)
    began = time.time()
    # Play the bag's own /clock: it is in the header stamps' time base.
    subprocess.run(
        [
            "bash",
            "-c",
            shell(
                args.ws,
                f"{ROS2} bag play {args.bag} --start-offset {args.start} "
                f"--playback-duration {args.duration} --topics /clock {CLOUD} {TWIST}",
            ),
        ],
        env=env,
        check=True,
        capture_output=True,
    )
    play_sec = time.time() - began
    time.sleep(3.0)
    for proc in (recorder, node):
        os.killpg(proc.pid, signal.SIGINT)
    for proc in (recorder, node):
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    return play_sec


def evaluate(est_path, ref, t0, t1):
    est = np.zeros((0, 8))
    if est_path.exists() and est_path.stat().st_size:
        est = np.loadtxt(est_path, ndmin=2)
        est = est[(est[:, 0] >= t0) & (est[:, 0] <= t1)]
    if len(est) == 0:
        return {"poses": 0}
    rx = np.interp(est[:, 0], ref[:, 0], ref[:, 1])
    ry = np.interp(est[:, 0], ref[:, 0], ref[:, 2])
    rz = np.interp(est[:, 0], ref[:, 0], ref[:, 3])
    nearest = np.clip(np.searchsorted(ref[:, 0], est[:, 0]), 0, len(ref) - 1)
    ref_yaw = np.array([yaw_of(*ref[k, 4:8]) for k in nearest])
    est_yaw = np.array([yaw_of(*row[4:8]) for row in est])
    dh = np.hypot(est[:, 1] - rx, est[:, 2] - ry)
    dz = est[:, 3] - rz
    d3 = np.sqrt(dh**2 + dz**2)
    dyaw = np.degrees(np.abs((est_yaw - ref_yaw + np.pi) % (2 * np.pi) - np.pi))

    def stats(v):
        return {
            "rmse": round(float(np.sqrt(np.mean(v**2))), 3),
            "median": round(float(np.median(np.abs(v))), 3),
            "p95": round(float(np.percentile(np.abs(v), 95)), 3),
            "max": round(float(np.max(np.abs(v))), 3),
        }

    return {
        "poses": len(est),
        "poses_per_sec": round(len(est) / (t1 - t0), 2),
        "horizontal_m": stats(dh),
        "z_m": stats(dz),
        "trans3d_m": stats(d3),
        "yaw_deg": stats(dyaw),
        "final_trans3d_m": round(float(d3[-1]), 3),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ws", required=True, help="colcon workspace with install/")
    parser.add_argument("--fixture", required=True, help="prepare.py output directory")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument(
        "--start", type=float, required=True, help="seconds from bag start"
    )
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--preset", default="istanbul_gtsam_smoother.yaml")
    parser.add_argument("--domain", type=int, default=73)
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override a preset parameter (YAML value); repeatable",
    )
    run_window(parser.parse_args())
