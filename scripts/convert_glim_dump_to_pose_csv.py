#!/usr/bin/env python3
"""Convert GLIM dump trajectories into the benchmark pose CSV format."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


FIELDS = [
    "message_index", "stamp_sec", "receive_stamp_sec", "frame_id",
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "covariance",
]


def _quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate(q, vector):
    norm = math.sqrt(sum(value * value for value in q))
    q = tuple(value / norm for value in q)
    rotated = _quat_multiply(_quat_multiply(q, (*vector, 0.0)), _quat_conjugate(q))
    return rotated[:3]


def _load_tum(path: Path):
    poses = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            values = tuple(float(value) for value in line.split())
            if len(values) == 8 and all(math.isfinite(value) for value in values):
                poses.append(values)
    return poses


def _global_correction(dump_dir: Path, stamp: float):
    odometry = _load_tum(dump_dir / "odom_lidar.txt")
    trajectory = _load_tum(dump_dir / "traj_lidar.txt")
    if not odometry or len(odometry) != len(trajectory):
        raise ValueError("global correction requires matching odom_lidar.txt/traj_lidar.txt")
    index = min(range(len(odometry)), key=lambda i: abs(odometry[i][0] - stamp))
    if abs(odometry[index][0] - stamp) > 0.05:
        raise ValueError(f"no global trajectory anchor near submap stamp {stamp:.9f}")
    odom = odometry[index]
    world = trajectory[index]
    rotation = _quat_multiply(world[4:8], _quat_conjugate(odom[4:8]))
    rotated_origin = _quat_rotate(rotation, odom[1:4])
    translation = tuple(a - b for a, b in zip(world[1:4], rotated_origin))
    return translation, rotation


def load_imu_rate_trajectory(
    dump_dir: Path, apply_global_correction: bool = False, planarize_z: bool = False
):
    rows = []
    invalid_rows = 0
    for path in sorted(dump_dir.glob("[0-9][0-9][0-9][0-9][0-9][0-9]/imu_rate.txt")):
        path_rows = []
        with path.open(encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                try:
                    values = tuple(float(value) for value in line.split())
                    if len(values) != 8 or not all(math.isfinite(value) for value in values):
                        raise ValueError("expected eight finite TUM values")
                    if sum(value * value for value in values[4:]) <= 1e-12:
                        raise ValueError("zero quaternion")
                    path_rows.append((values, path, line_number))
                except ValueError:
                    invalid_rows += 1
        if apply_global_correction and path_rows:
            translation, rotation = _global_correction(dump_dir, path_rows[0][0][0])
            corrected = []
            for values, source, line_number in path_rows:
                position = tuple(
                    a + b for a, b in zip(translation, _quat_rotate(rotation, values[1:4])))
                quaternion = _quat_multiply(rotation, values[4:8])
                corrected.append(((values[0], *position, *quaternion), source, line_number))
            path_rows = corrected
        rows.extend(path_rows)

    rows.sort(key=lambda item: item[0][0])
    unique = []
    duplicate_rows = 0
    conflicting_duplicates = 0
    for item in rows:
        if unique and item[0][0] == unique[-1][0][0]:
            duplicate_rows += 1
            delta = max(abs(a - b) for a, b in zip(item[0][1:], unique[-1][0][1:]))
            if delta > 1e-5 and not apply_global_correction:
                conflicting_duplicates += 1
            unique[-1] = item
        else:
            unique.append(item)

    if not unique:
        raise ValueError(f"no valid */imu_rate.txt poses found below {dump_dir}")
    if conflicting_duplicates:
        raise ValueError(
            f"found {conflicting_duplicates} duplicate timestamps with conflicting poses")
    if planarize_z:
        origin_z = unique[0][0][3]
        unique = [
            ((values[0], values[1], values[2], origin_z, *values[4:]), source, line_number)
            for values, source, line_number in unique
        ]
    return unique, invalid_rows, duplicate_rows


def convert(
    dump_dir: Path,
    output_csv: Path,
    frame_id: str,
    apply_global_correction: bool = False,
    planarize_z: bool = False,
):
    rows, invalid_rows, duplicate_rows = load_imu_rate_trajectory(
        dump_dir, apply_global_correction, planarize_z)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for index, (values, _path, _line_number) in enumerate(rows):
            stamp, x, y, z, qx, qy, qz, qw = values
            writer.writerow({
                "message_index": index,
                "stamp_sec": f"{stamp:.9f}",
                "receive_stamp_sec": "",
                "frame_id": frame_id,
                "position_x": f"{x:.9f}",
                "position_y": f"{y:.9f}",
                "position_z": f"{z:.9f}",
                "orientation_x": f"{qx:.9f}",
                "orientation_y": f"{qy:.9f}",
                "orientation_z": f"{qz:.9f}",
                "orientation_w": f"{qw:.9f}",
                "covariance": "",
            })
    return {
        "source_dump_dir": str(dump_dir.resolve()),
        "output_csv": str(output_csv.resolve()),
        "pose_count": len(rows),
        "invalid_source_row_count": invalid_rows,
        "duplicate_source_row_count": duplicate_rows,
        "global_correction_applied": apply_global_correction,
        "planar_z_constraint_applied": planarize_z,
        "first_stamp_sec": rows[0][0][0],
        "last_stamp_sec": rows[-1][0][0],
        "max_pose_gap_sec": max(
            (current[0][0] - previous[0][0]
             for previous, current in zip(rows, rows[1:])),
            default=0.0,
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dump-dir", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json")
    parser.add_argument("--frame-id", default="glim_map")
    parser.add_argument("--apply-global-correction", action="store_true")
    parser.add_argument("--planarize-z", action="store_true")
    args = parser.parse_args()
    try:
        summary = convert(
            Path(args.dump_dir),
            Path(args.output_csv),
            args.frame_id,
            args.apply_global_correction,
            args.planarize_z,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if args.summary_json:
        path = Path(args.summary_json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
