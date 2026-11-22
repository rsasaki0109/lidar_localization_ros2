#!/usr/bin/env python3
"""Reference poses for a run on another run's map, beyond the motion-capture area.

The odometry of the run is moved into the ground-truth frame with the SE(3) that
align_map_to_ground_truth.py fitted for the run's own map (map.align.json), and
then registered (NDT, the G2 scorer) against the target map once per --interval
seconds, seeded at that pose. The correction (x, y, z, yaw) is held between
registrations and applied to every odometry pose; roll and pitch stay the
odometry's. Registrations that do not converge, or move the pose by more than
--max-correction-m, keep the previous correction and are counted.

usage: make_reference.py --bag BAG --odometry odom.tum --align map.align.json
           --map target_map.pcd --out reference.tum
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "tools" / "benchmark"))
sys.path.insert(0, str(REPO / "experiments" / "go2_wide_map_global_init"))

import build_map_from_odometry as map_builder  # noqa: E402
import g2_offline_probe as probe  # noqa: E402
import global_localization_query as glq  # noqa: E402


def rotation_to_quaternion(r: np.ndarray) -> list:
    """(qx, qy, qz, qw) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(
        math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2]
    )
    y = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0]
    )
    z = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1]
    )
    return [x, y, z, w]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--align", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--end-sec", type=float, default=1.0e9)
    parser.add_argument("--max-correction-m", type=float, default=1.0)
    parser.add_argument("--topic", default="/livox/lidar")
    args = parser.parse_args(argv)

    align = json.loads(args.align.read_text())
    rotation = np.array(align["rotation"])
    translation = np.array(align["translation"])
    odometry = map_builder.load_tum(args.odometry)
    rotations, translations = map_builder.interpolate_poses(odometry, odometry[:, 0])
    world_r = np.einsum("ij,njk->nik", rotation, rotations)
    world_t = translations @ rotation.T + translation

    config = glq.GlobalLocalizationConfig(
        enable_registration_scoring=True, map_path=str(args.map)
    )
    engine = glq.GlobalLocalizationEngine.__new__(glq.GlobalLocalizationEngine)
    engine.config = config
    engine._init_registration_scorer(config)
    if engine.registration_scorer is None:
        raise RuntimeError(engine.registration_scoring_error)

    t0 = odometry[0, 0]
    stamps = list(np.arange(t0, min(odometry[-1, 0], t0 + args.end_sec), args.interval))
    scans = probe.read_scans(args.bag, stamps, args.topic)
    corrections = []  # (stamp, dx, dy, dz, dyaw)
    current = (0.0, 0.0, 0.0, 0.0)
    rejected = 0
    for stamp, xyz in scans:
        index = min(int(np.searchsorted(odometry[:, 0], stamp)), len(odometry) - 1)
        r, t = world_r[index], world_t[index]
        yaw = probe.yaw_of(r)
        levelled = probe.level_scan(xyz, r)
        seed = (
            t[0] + current[0],
            t[1] + current[1],
            t[2] + current[2],
            yaw + current[3],
        )
        result = engine.registration_scorer.score_candidates(
            glq.registration_scan_points(levelled, 0.0), [seed]
        )[0]
        moved = math.hypot(result.refined_x - seed[0], result.refined_y - seed[1])
        if (
            result.converged
            and math.isfinite(result.fitness)
            and moved <= args.max_correction_m
        ):
            current = (
                result.refined_x - t[0],
                result.refined_y - t[1],
                result.refined_z - t[2],
                probe.wrap(result.refined_yaw - yaw),
            )
        else:
            rejected += 1
        corrections.append((stamp, *current, float(result.fitness)))
    corrections = np.array(corrections)

    rows = []
    for index, stamp in enumerate(odometry[:, 0]):
        if stamp > t0 + args.end_sec:
            break
        k = max(0, int(np.searchsorted(corrections[:, 0], stamp, side="right")) - 1)
        dx, dy, dz, dyaw = corrections[k, 1:5]
        r = world_r[index]
        c, s = math.cos(dyaw), math.sin(dyaw)
        r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r
        t = world_t[index] + np.array([dx, dy, dz])
        rows.append([stamp, *t.tolist(), *rotation_to_quaternion(r)])
    np.savetxt(args.out, np.array(rows), fmt="%.6f")
    shift = np.hypot(corrections[:, 1], corrections[:, 2])
    print(
        json.dumps(
            {
                "registrations": len(corrections),
                "rejected": rejected,
                "correction_xy_median_m": float(np.median(shift)),
                "correction_xy_max_m": float(np.max(shift)),
                "correction_z_max_m": float(np.max(np.abs(corrections[:, 3]))),
                "correction_yaw_max_deg": float(
                    np.degrees(np.max(np.abs(corrections[:, 4])))
                ),
                "fitness_median": float(np.median(corrections[:, 5])),
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
