#!/usr/bin/env python3
"""Would a better map -> odom anchor shrink the error off the Koide map?

Off the map (OFF_MAP_SEC after the first expected cloud) the published pose is the
map -> odom of the last accepted scan times the RKO-LIO odometry. For each run this
prints the roll/pitch error of the accepted scans before the stretch against the
ground truth, and the off-map RMSE the run would have had with the last scan's tilt
error swapped for the mean tilt error of its last N accepted scans, or with none: the
off-map poses are turned about the anchor position by the difference.

usage: anchor_tilt.py DATA_DIR RUN_DIR [RUN_DIR ...]

DATA_DIR is the run_koide_public_bag.py data directory (koide-data).
"""

import argparse
import json
from pathlib import Path

import numpy as np

OFF_MAP_SEC = (163.3, 259.0)
MATCH_SEC = 0.05
WINDOWS = (5, 10, 20, 30)


def rotation(q):
    x, y, z, w = np.asarray(q) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def roll_pitch_deg(r):
    return (
        np.degrees(np.arctan2(r[2, 1], r[2, 2])),
        np.degrees(-np.arcsin(np.clip(r[2, 0], -1.0, 1.0))),
    )


def mean_rotation(rotations):
    u, _, vt = np.linalg.svd(sum(rotations))
    return u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt


def tilt(r):
    """The roll/pitch part of a world-frame rotation: its yaw about z removed."""
    yaw = np.arctan2(r[1, 0], r[0, 0])
    c, s = np.cos(-yaw), np.sin(-yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r


def analyze(run, gt):
    rows = [json.loads(line) for line in (run / "alignment.jsonl").open()]
    first = min(row["stamp_ns"] for row in rows) * 1e-9
    accepted = sorted(
        row["stamp_ns"] * 1e-9
        for row in rows
        if row["values"].get("recovery_action", "").startswith("accept")
    )
    estimate = np.loadtxt(run / "estimate.tum", ndmin=2)
    estimate = estimate[np.argsort(estimate[:, 0])]
    by_stamp = {round(row[0], 4): row for row in estimate}

    def nearest(stamp):
        k = int(np.argmin(abs(gt[:, 0] - stamp)))
        return k if abs(gt[k, 0] - stamp) < MATCH_SEC else None

    start, end = first + OFF_MAP_SEC[0], first + OFF_MAP_SEC[1]
    before = [
        t
        for t in accepted
        if t < start and round(t, 4) in by_stamp and nearest(t) is not None
    ][-max(WINDOWS) :]
    # World-frame rotation errors, and roll/pitch in the body frame for the printout.
    errors, body = [], []
    for stamp in before:
        r_est = rotation(by_stamp[round(stamp, 4)][4:8])
        r_gt = rotation(gt[nearest(stamp), 4:8])
        errors.append(r_est @ r_gt.T)
        body.append(roll_pitch_deg(r_gt.T @ r_est))
    body = np.array(body)
    anchor = by_stamp[round(before[-1], 4)][1:4]

    off = estimate[(estimate[:, 0] >= start) & (estimate[:, 0] <= end)]
    matches = [nearest(t) for t in off[:, 0]]
    keep = np.array([k is not None for k in matches])
    off, matches = off[keep], [k for k in matches if k is not None]

    def rmse(points):
        return float(np.sqrt(np.mean(np.sum((points - gt[matches, 1:4]) ** 2, axis=1))))

    def turned(correction):
        return anchor + (off[:, 1:4] - anchor) @ correction.T

    last_tilt = tilt(errors[-1])
    result = {
        "last_accept_sec": round(before[-1] - first, 1),
        "last_roll_pitch_deg": [round(v, 2) for v in body[-1]],
        "roll_pitch_std_deg": [round(v, 2) for v in body.std(axis=0)],
        "off_map_rmse_m": round(rmse(off[:, 1:4]), 3),
        "no_tilt_error_m": round(rmse(turned(last_tilt.T)), 3),
    }
    for n in WINDOWS:
        result[f"mean_of_last_{n}"] = {
            "roll_pitch_deg": [round(v, 2) for v in body[-n:].mean(axis=0)],
            "off_map_rmse_m": round(
                rmse(turned(tilt(mean_rotation(errors[-n:])) @ last_tilt.T)), 3
            ),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data_dir", type=Path)
    parser.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args()
    gt = np.loadtxt(args.data_dir / "gt" / "traj_lidar_outdoor_hard_02.txt")
    gt = gt[np.argsort(gt[:, 0])]
    for run in args.runs:
        print(json.dumps({"run": run.name} | analyze(run, gt)))


if __name__ == "__main__":
    main()
