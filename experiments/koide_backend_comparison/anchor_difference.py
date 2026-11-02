#!/usr/bin/env python3
"""Compare two run_koide_public_bag.py runs over the stretch where neither accepted
a scan: is one trajectory the other moved by a single rigid transform, a different
map -> odom anchor on the same odometry?

usage: anchor_difference.py RUN_A RUN_B

The stretch is the longest one without an accepted scan in RUN_A, minus 7 s at its
end, where a registration already pulls the pose back. Poses are paired by stamp.
"""

import argparse
import json
from pathlib import Path

import numpy as np

END_MARGIN_SEC = 7.0


def rotation(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def roll_pitch_yaw_deg(m):
    return np.degrees(
        [
            np.arctan2(m[2, 1], m[2, 2]),
            np.arcsin(np.clip(-m[2, 0], -1.0, 1.0)),
            np.arctan2(m[1, 0], m[0, 0]),
        ]
    )


def from_roll_pitch_yaw_deg(rpy):
    r, p, y = np.radians(rpy)
    cr, sr, cp, sp, cy, sy = (
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def no_accept_stretch(run):
    stamps = []
    for line in (run / "alignment.jsonl").read_text().splitlines():
        record = json.loads(line)
        if record["values"]["recovery_action"].startswith("accept"):
            stamps.append(record["stamp_ns"] * 1e-9)
    stamps = np.array(stamps)
    i = int(np.argmax(np.diff(stamps)))
    return stamps[i], stamps[i + 1]


def poses(run):
    return {round(row[0], 3): row for row in np.loadtxt(run / "estimate.tum")}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    args = parser.parse_args()
    start, end = no_accept_stretch(args.run_a)
    a, b = poses(args.run_a), poses(args.run_b)
    common = sorted(s for s in a if s in b and start < s < end - END_MARGIN_SEC)
    if not common:
        raise SystemExit("no common poses in the stretch")
    deltas_rpy, deltas_t = [], []
    for s in common:
        delta = rotation(b[s][4:8]) @ rotation(a[s][4:8]).T
        deltas_rpy.append(roll_pitch_yaw_deg(delta))
        deltas_t.append(b[s][1:4] - delta @ a[s][1:4])
    rpy = np.median(deltas_rpy, axis=0)
    t = np.median(deltas_t, axis=0)
    pa = np.array([a[s][1:4] for s in common])
    pb = np.array([b[s][1:4] for s in common])
    residual = np.linalg.norm((from_roll_pitch_yaw_deg(rpy) @ pa.T).T + t - pb, axis=1)
    gap = np.linalg.norm(pa - pb, axis=1)
    print(
        f"{args.run_a.name} -> {args.run_b.name}: {len(common)} poses over "
        f"{common[-1] - common[0]:.0f} s"
    )
    print(f"  anchor change roll/pitch/yaw deg: {np.round(rpy, 3)}")
    print(
        f"  position gap max {gap.max():.3f} m; after the anchor change "
        f"median {np.median(residual):.3f} m, max {residual.max():.3f} m"
    )


if __name__ == "__main__":
    main()
