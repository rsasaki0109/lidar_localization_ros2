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

    from replay_session import (
        ReplaySession,
        apply_overrides,
        initial_pose_parameters,
    )
    from start_lifecycle_node import activate

    reference = np.loadtxt(args.fixture / "reference.tum")
    scans = len(reference)
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
        }
        | initial_pose_parameters(reference[0, 1:8])
    )
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

    with ReplaySession(args.out, os.environ.copy(), "boreas_replay") as session:
        session.require_free_domain()
        session.record(on_status=count_accepted)
        session.spawn(
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
        deadline = time.monotonic() + 30
        while not session.node.get_subscriptions_info_by_topic(POINTS):
            session.require_running("localizer")
            if time.monotonic() > deadline:
                raise RuntimeError("localizer did not subscribe; see localizer.log")
            session.spin_for(0.1)
        session.spin_for(2.0)
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
                POINTS,
                IMU,
            ],
            "player",
        )
        while player.poll() is None:
            session.require_running("localizer")
            session.spin_for(0.05)
        session.spin_for(10.0)
    poses, diagnostics = session.poses, session.diagnostic_stamps

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
