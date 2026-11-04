#!/usr/bin/env python3
"""Replay a Koide sequence through the localizer alone (no odometry) and score it.

The A/B prepared before 6825f05 for the IMU preintegration: param/nav2_ndt_urban.yaml
with the Koide overrides of its manifests (NDT_OMP, 0.5 m scan voxels, livox_frame,
1-100 m, score threshold 6, recovery retry after 3 rejections), started at the
ground-truth pose 4 s into the bag and played from there with /clock. The published
/pcl_pose is matched to the ground truth by stamp (within 0.05 s), without alignment.

The sequence is extracted from DATA_DIR/SEQUENCE.zip (the Zenodo record 10122133) and
its IMU converted from g to SI once (koide_public_bag.prepare_si_bag).

usage: replay.py DATA_DIR SEQUENCE OUT_DIR --ws WS [--set KEY=VALUE ...]
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
sys.path.insert(0, str(REPO / "tools" / "benchmark"))
POINTS = "/livox/points"
IMU = "/livox/imu"
START_OFFSET_SEC = 4.0
# The overrides of param/benchmark/koide_hard_localization_outdoor_hard_01a_full380_*
# (git show 6825f05^:param/benchmark/...), without use_imu_preintegration.
OVERRIDES = {
    "use_imu": False,
    "enable_scan_voxel_filter": True,
    "voxel_leaf_size": 0.5,
    "base_frame_id": "livox_frame",
    "scan_min_range": 1.0,
    "scan_max_range": 100.0,
    "score_threshold": 6.0,
    "enable_recovery_retry_from_last_pose": True,
    "recovery_retry_from_last_pose_min_rejections": 3,
    "recovery_retry_from_last_pose_max_accepted_gap_sec": 1.0,
    "recovery_retry_from_last_pose_max_seed_translation_m": 15.0,
    # Since 2026-11-04. Rebuilt in the callback, the 150 m crop of the 2M-point map
    # stalls the node: 10% of the clouds got no diagnostic, 1.1 s of them at 21.4 s,
    # and a gap over 1 s drops the IMU prediction. The online profile rebuilds in the
    # background too.
    "local_map_async_rebuild": True,
}


def ground_truth(data, sequence):
    """outdoor_hard_01a and _01b share traj_lidar_outdoor_hard_01.txt."""
    name = sequence[:-1] if sequence[-1] in "ab" else sequence
    reference = np.loadtxt(data / "gt" / f"traj_lidar_{name}.txt")
    return reference[np.argsort(reference[:, 0])]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data", type=Path)
    parser.add_argument("sequence")
    parser.add_argument("out", type=Path)
    parser.add_argument(
        "--ws", type=Path, required=True, help="colcon workspace with install/"
    )
    parser.add_argument(
        "--preset", type=Path, default=REPO / "param/nav2_ndt_urban.yaml"
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override a parameter (YAML value); repeatable",
    )
    parser.add_argument("--ros-domain-id", type=int, default=89)
    args = parser.parse_args()

    import koide_public_bag as data_tool
    from boreas_replay import score
    from replay_session import ReplaySession, apply_overrides, initial_pose_parameters
    from start_lifecycle_node import activate

    raw = args.data / args.sequence
    if not raw.is_dir():
        data_tool.extract_archive(args.data / f"{args.sequence}.zip", args.data)
    bag = data_tool.prepare_si_bag(raw, args.data / f"{args.sequence}_si")
    start_ns, clouds = data_tool.bag_cloud_times(bag)
    first_ns = start_ns + round(START_OFFSET_SEC * 1e9)
    reference = ground_truth(args.data, args.sequence)
    k = int(np.argmin(abs(reference[:, 0] - first_ns * 1e-9)))
    if abs(reference[k, 0] - first_ns * 1e-9) > 0.05:
        raise SystemExit("ground truth has no pose within 0.05 s of bag + 4 s")
    expected = sorted(stamp for stamp in clouds if stamp >= first_ns)
    args.out.mkdir(parents=True, exist_ok=False)

    config = yaml.safe_load(args.preset.read_text())
    params = config["/**"]["ros__parameters"]
    params.update(OVERRIDES)
    params.update(
        {
            "map_path": str((args.data / "map_outdoor_hard.ply").resolve()),
            "use_pcd_map": True,
            "use_sim_time": True,
            # No odometry or twist in the bag: the localizer runs alone.
            "use_twist_prediction": False,
            "use_odom": False,
            f"qos_overrides.{POINTS}.subscription.reliability": "reliable",
        }
        | initial_pose_parameters(reference[k, 1:8])
    )
    apply_overrides(params, args.set)
    config_path = args.out / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))
    qos_path = REPO / "tools" / "readme_media" / "koide_reliable_points_qos.yaml"

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

    with ReplaySession(args.out, os.environ.copy(), "koide_imu_ab") as session:
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
        print(f"Replaying {len(expected)} clouds of {args.sequence}", flush=True)
        player = session.spawn(
            [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--clock",
                "--start-offset",
                str(START_OFFSET_SEC),
                "--qos-profile-overrides-path",
                str(qos_path),
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

    result = score(
        session.poses, reference, len(expected), session.diagnostic_stamps, accepted[0]
    )
    result.update(
        {
            "sequence": args.sequence,
            "source": data_tool.SOURCE,
            "preset": str(args.preset),
            "overrides": args.set,
            "start_offset_sec": START_OFFSET_SEC,
        }
    )
    (args.out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"RMSE {result.get('rmse_m', float('nan')):.3f} m (horizontal "
        f"{result.get('horizontal_rmse_m', float('nan')):.3f}), max "
        f"{result.get('max_error_m', float('nan')):.2f} m, {result['poses']} poses for "
        f"{len(expected)} clouds, accepted {result['accepted_share']:.0%}; {args.out}",
        flush=True,
    )


if __name__ == "__main__":
    main()
