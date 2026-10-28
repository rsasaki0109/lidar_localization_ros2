#!/usr/bin/env python3
"""Replay a Koide hard-localization sequence through the localizer of one colcon
workspace, with a quickstart-generated config and no external odometry, and score
/pcl_pose against the ground truth (tools/benchmark/benchmark_eval.py).

usage: replay_koide.py --ws WS --bag SEQ_DIR --map PLY --gt TUM --config YAML --out DIR
                       [--start SEC] [--duration SEC] [--set KEY=VALUE ...] [--domain N]

The localizer starts from the ground-truth pose at the replay start, in the LiDAR
frame (the ground truth is the LiDAR trajectory). The run directory gets
params.yaml, node.log, est.tum, alignment.jsonl and score.json.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "benchmark"))

import benchmark_eval  # noqa: E402

CLOUD = "/livox/points"
IMU = "/livox/imu"
LIDAR_FRAME = "livox_frame"
ROS2 = os.environ.get("ROS2_CLI", "ros2")
PYTHON = os.environ.get("ROS_PYTHON", "python3")


def shell(ws, command):
    return f"source {ws}/install/setup.bash && {command}"


def bag_start_and_duration(bag):
    info = yaml.safe_load((Path(bag) / "metadata.yaml").read_text())
    info = info["rosbag2_bagfile_information"]
    return (
        info["starting_time"]["nanoseconds_since_epoch"] * 1e-9,
        info["duration"]["nanoseconds"] * 1e-9,
    )


def write_params(args, out, truth, start_sec):
    params = yaml.safe_load(Path(args.config).read_text())
    p = params["/**"]["ros__parameters"]
    row = truth[np.searchsorted(truth[:, 0], start_sec)]
    p.update(
        {
            "map_path": str(Path(args.map).resolve()),
            "use_sim_time": True,
            "base_frame_id": LIDAR_FRAME,
            "set_initial_pose": True,
            "initial_pose_x": float(row[1]),
            "initial_pose_y": float(row[2]),
            "initial_pose_z": float(row[3]),
            "initial_pose_qx": float(row[4]),
            "initial_pose_qy": float(row[5]),
            "initial_pose_qz": float(row[6]),
            "initial_pose_qw": float(row[7]),
        }
    )
    for item in args.set:
        key, value = item.split("=", 1)
        p[key] = yaml.safe_load(value)
    (out / "params.yaml").write_text(yaml.safe_dump(params))


def lifecycle(ws, env):
    deadline = time.time() + 120.0
    while (
        subprocess.run(
            ["bash", "-c", shell(ws, f"{ROS2} lifecycle get /lidar_localization")],
            env=env,
            capture_output=True,
        ).returncode
        != 0
    ):
        if time.time() > deadline:
            raise RuntimeError("localizer did not come up; see node.log")
        time.sleep(2.0)
    for transition in ("configure", "activate"):
        result = subprocess.run(
            [
                "bash",
                "-c",
                shell(ws, f"{ROS2} lifecycle set /lidar_localization {transition}"),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=900,
        )
        if result.returncode != 0 or "successful" not in result.stdout:
            raise RuntimeError(f"{transition} failed: {result.stdout} {result.stderr}")


def main(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    truth = benchmark_eval.load_tum(args.gt)
    bag_start, bag_duration = bag_start_and_duration(args.bag)
    start_sec = bag_start + args.start
    duration = args.duration or bag_duration - args.start
    write_params(args, out, truth, start_sec)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    node_bin = (
        f"{args.ws}/install/lidar_localization_ros2/lib/lidar_localization_ros2/"
        "lidar_localization_node"
    )
    with (
        (out / "node.log").open("w") as node_log,
        (out / "recorder.log").open("w") as recorder_log,
    ):
        node = subprocess.Popen(
            [
                "bash",
                "-c",
                shell(
                    args.ws,
                    f"exec {node_bin} --ros-args --params-file {out}/params.yaml "
                    f"-r cloud:={CLOUD} -r imu:={IMU}",
                ),
            ],
            stdout=node_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        recorder = None
        try:
            lifecycle(args.ws, env)
            recorder = subprocess.Popen(
                [
                    "bash",
                    "-c",
                    shell(
                        args.ws,
                        f"exec {PYTHON} {REPO}/tools/benchmark/record_pose.py {out}",
                    ),
                ],
                stdout=recorder_log,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            time.sleep(2.0)
            began = time.time()
            subprocess.run(
                [
                    "bash",
                    "-c",
                    shell(
                        args.ws,
                        f"{ROS2} bag play {args.bag} --clock --start-offset {args.start} "
                        f"--playback-duration {duration} --topics {CLOUD} {IMU}",
                    ),
                ],
                env=env,
                check=True,
                capture_output=True,
            )
            play_sec = time.time() - began
            time.sleep(3.0)
        finally:
            for proc in (recorder, node):
                if proc is not None:
                    os.killpg(proc.pid, signal.SIGINT)
            for proc in (recorder, node):
                if proc is None:
                    continue
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)

    est_path = out / "est.tum"
    estimate = (
        benchmark_eval.load_tum(est_path)
        if est_path.exists() and est_path.stat().st_size
        else np.zeros((0, 8))
    )
    result = benchmark_eval.score_run(
        estimate,
        truth,
        start_sec,
        benchmark_eval.load_alignment_levels(out / "alignment.jsonl"),
        end_sec=start_sec + duration,
    ).as_dict()
    est, gt = benchmark_eval.match_to_ground_truth(estimate, truth)
    xy = np.hypot(est[:, 1] - gt[:, 1], est[:, 2] - gt[:, 2]) if len(est) else []
    result.update(
        {
            "poses": len(estimate),
            "poses_per_sec": round(len(estimate) / duration, 2),
            "rmse_xy_m": round(float(np.sqrt(np.mean(np.square(xy)))), 3)
            if len(xy)
            else None,
            "start_sec": args.start,
            "duration_sec": round(duration, 1),
            "wall_play_sec": round(play_sec, 1),
            "overrides": args.set,
        }
    )
    (out / "score.json").write_text(json.dumps(result, indent=1))
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ws", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--config", required=True, help="quickstart-generated YAML")
    parser.add_argument("--out", required=True)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = to the end")
    parser.add_argument("--domain", type=int, default=60)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    main(parser.parse_args())
