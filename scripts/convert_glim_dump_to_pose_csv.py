#!/usr/bin/env python3
"""Convert GLIM dump trajectories into the benchmark pose CSV format."""

from __future__ import annotations

import argparse
import bisect
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


def _poses_equivalent(a, b, translation_tolerance_m=1.0e-3,
                      rotation_tolerance_rad=1.0e-3):
    translation_delta = math.sqrt(sum(
        (left - right) ** 2 for left, right in zip(a[1:4], b[1:4])))
    if translation_delta > translation_tolerance_m:
        return False
    qa = a[4:8]
    qb = b[4:8]
    norm_a = math.sqrt(sum(value * value for value in qa))
    norm_b = math.sqrt(sum(value * value for value in qb))
    cosine = abs(sum(left * right for left, right in zip(qa, qb)) / (norm_a * norm_b))
    rotation_delta = 2.0 * math.acos(min(1.0, max(0.0, cosine)))
    return rotation_delta <= rotation_tolerance_rad


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
            if not _poses_equivalent(item[0], unique[-1][0]) and not apply_global_correction:
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


def load_tum_trajectory(path: Path, planarize_z: bool = False):
    rows = []
    invalid_rows = 0
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                values = tuple(float(value) for value in line.split())
                if len(values) != 8 or not all(math.isfinite(value) for value in values):
                    raise ValueError("expected eight finite TUM values")
                if sum(value * value for value in values[4:]) <= 1e-12:
                    raise ValueError("zero quaternion")
                rows.append((values, path, line_number))
            except ValueError:
                invalid_rows += 1

    rows.sort(key=lambda item: item[0][0])
    unique = []
    duplicate_rows = 0
    for item in rows:
        if unique and item[0][0] == unique[-1][0][0]:
            duplicate_rows += 1
            unique[-1] = item
        else:
            unique.append(item)
    if not unique:
        raise ValueError(f"no valid poses found in {path}")
    if planarize_z:
        origin_z = unique[0][0][3]
        unique = [
            ((values[0], values[1], values[2], origin_z, *values[4:]), source, line_number)
            for values, source, line_number in unique
        ]
    return unique, invalid_rows, duplicate_rows


def _interpolate_tum_transform(samples, stamps, stamp):
    upper = bisect.bisect_right(stamps, stamp)
    if upper == 0:
        return samples[0][1:4], samples[0][4:8]
    if upper >= len(samples):
        return samples[-1][1:4], samples[-1][4:8]

    before = samples[upper - 1]
    after = samples[upper]
    span = after[0] - before[0]
    alpha = 0.0 if span <= 0.0 else (stamp - before[0]) / span
    translation = tuple(
        (1.0 - alpha) * a + alpha * b for a, b in zip(before[1:4], after[1:4]))
    q0 = before[4:8]
    q1 = after[4:8]
    if sum(a * b for a, b in zip(q0, q1)) < 0.0:
        q1 = tuple(-value for value in q1)
    quaternion = tuple((1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1))
    norm = math.sqrt(sum(value * value for value in quaternion))
    quaternion = tuple(value / norm for value in quaternion)
    return translation, quaternion


def load_external_map_odom_trajectory(
    dump_dir: Path, map_odom_tum: Path, planarize_z: bool = False
):
    rows, invalid_rows, duplicate_rows = load_imu_rate_trajectory(
        dump_dir, apply_global_correction=False, planarize_z=False)
    corrections = _load_tum(map_odom_tum)
    if not corrections:
        raise ValueError(f"no valid map-to-odom transforms found in {map_odom_tum}")
    corrections.sort(key=lambda values: values[0])
    correction_stamps = [values[0] for values in corrections]

    corrected = []
    for values, source, line_number in rows:
        stamp = values[0]
        translation, rotation = _interpolate_tum_transform(
            corrections, correction_stamps, stamp)
        position = tuple(
            a + b for a, b in zip(translation, _quat_rotate(rotation, values[1:4])))
        quaternion = _quat_multiply(rotation, values[4:8])
        corrected.append(((stamp, *position, *quaternion), source, line_number))
    if planarize_z:
        origin_z = corrected[0][0][3]
        corrected = [
            ((values[0], values[1], values[2], origin_z, *values[4:]), source, line_number)
            for values, source, line_number in corrected
        ]
    return corrected, invalid_rows, duplicate_rows


def convert(
    dump_dir: Path,
    output_csv: Path,
    frame_id: str,
    apply_global_correction: bool = False,
    planarize_z: bool = False,
    trajectory_tum: Path | None = None,
    map_odom_tum: Path | None = None,
):
    if trajectory_tum is not None and map_odom_tum is not None:
        raise ValueError("trajectory_tum and map_odom_tum are mutually exclusive")
    if map_odom_tum is not None:
        rows, invalid_rows, duplicate_rows = load_external_map_odom_trajectory(
            dump_dir, map_odom_tum, planarize_z)
    elif trajectory_tum is None:
        rows, invalid_rows, duplicate_rows = load_imu_rate_trajectory(
            dump_dir, apply_global_correction, planarize_z)
    else:
        rows, invalid_rows, duplicate_rows = load_tum_trajectory(
            trajectory_tum, planarize_z)
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
        "source_trajectory_tum": (
            str(trajectory_tum.resolve()) if trajectory_tum is not None else None),
        "source_map_odom_tum": (
            str(map_odom_tum.resolve()) if map_odom_tum is not None else None),
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
    parser.add_argument(
        "--trajectory-tum",
        help="Use this live TUM trajectory instead of reconstructing poses from the dump.",
    )
    parser.add_argument(
        "--map-odom-tum",
        help="Compose this recorded live map-to-odom TUM history with raw dump poses.",
    )
    args = parser.parse_args()
    try:
        summary = convert(
            Path(args.dump_dir),
            Path(args.output_csv),
            args.frame_id,
            args.apply_global_correction,
            args.planarize_z,
            Path(args.trajectory_tum) if args.trajectory_tum else None,
            Path(args.map_odom_tum) if args.map_odom_tum else None,
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
