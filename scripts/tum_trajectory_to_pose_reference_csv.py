#!/usr/bin/env python3
"""Convert a space-separated TUM trajectory (t x y z qx qy qz qw) to benchmark reference CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterator
from typing import Tuple

import yaml


def iter_tum_poses(path: Path) -> Iterator[Tuple[float, float, float, float, float, float, float]]:
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            yield t, x, y, z, qx, qy, qz, qw


def export_initial_pose_yaml(
    path: Path,
    position: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> None:
    data = {
        "/**": {
            "ros__parameters": {
                "set_initial_pose": True,
                "initial_pose_x": position[0],
                "initial_pose_y": position[1],
                "initial_pose_z": position[2],
                "initial_pose_qx": quaternion[0],
                "initial_pose_qy": quaternion[1],
                "initial_pose_qz": quaternion[2],
                "initial_pose_qw": quaternion[3],
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="TUM text file, no header")
    parser.add_argument("--output-csv", required=True, type=Path, help="Destination pose reference CSV")
    parser.add_argument(
        "--output-initial-pose-yaml",
        default="",
        type=Path,
        help="Optional parameter YAML (set_initial_pose + pose) from the chosen initial row",
    )
    parser.add_argument(
        "--initial-pose-skip-sec",
        type=float,
        default=0.0,
        help="If > 0, initial pose is the first row with t >= (first_t + this)",
    )
    parser.add_argument(
        "--time-start",
        type=float,
        default=None,
        help="Drop rows with stamp < this (seconds, same timebase as TUM file)",
    )
    parser.add_argument(
        "--time-end",
        type=float,
        default=None,
        help="Drop rows with stamp > this",
    )
    args = parser.parse_args()

    rows = list(iter_tum_poses(args.input))
    if not rows:
        raise SystemExit(f"No poses parsed from {args.input}")

    if args.time_start is not None:
        rows = [r for r in rows if r[0] >= args.time_start]
    if args.time_end is not None:
        rows = [r for r in rows if r[0] <= args.time_end]
    if not rows:
        raise SystemExit("No poses left after time window filter")

    first_t = rows[0][0]

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "message_index",
                "stamp_sec",
                "frame_id",
                "position_x",
                "position_y",
                "position_z",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "covariance",
            ]
        )
        for i, (t, x, y, z, qx, qy, qz, qw) in enumerate(rows):
            writer.writerow(
                [
                    i,
                    f"{t:.9f}",
                    "map",
                    f"{x:.10f}",
                    f"{y:.10f}",
                    f"{z:.10f}",
                    f"{qx:.10f}",
                    f"{qy:.10f}",
                    f"{qz:.10f}",
                    f"{qw:.10f}",
                    "",
                ]
            )

    if args.output_initial_pose_yaml:
        if args.initial_pose_skip_sec and args.initial_pose_skip_sec > 0.0:
            thresh = first_t + args.initial_pose_skip_sec
            chosen = next((r for r in rows if r[0] >= thresh), None)
            if chosen is None:
                raise SystemExit(
                    f"No row with t >= {thresh:.6f} for initial pose (first_t={first_t:.6f})"
                )
        else:
            chosen = rows[0]
        _, x, y, z, qx, qy, qz, qw = chosen
        export_initial_pose_yaml(
            args.output_initial_pose_yaml,
            (x, y, z),
            (qx, qy, qz, qw),
        )
        print(f"initial_pose_stamp_sec: {chosen[0]:.9f}")
        print(f"output_initial_pose_yaml: {args.output_initial_pose_yaml}")

    print(f"rows_written: {len(rows)}")
    print(f"output_csv: {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
