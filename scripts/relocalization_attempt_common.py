"""Shared contract and helpers for relocalization attempt artifact generators.

The candidate generators and the runtime global-localization engine share the
same ``relocalization_attempts.csv`` / candidate CSV contract. Keeping the
fieldnames and window parsing here prevents the callers from drifting apart.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

ATTEMPT_FIELDNAMES = [
    "attempt_id",
    "trigger_stamp_sec",
    "start_stamp_sec",
    "end_stamp_sec",
    "source",
    "mode",
    "roi_type",
    "candidate_count",
    "accepted",
    "accepted_candidate_rank",
    "rejection_reason",
    "best_score",
    "second_score",
    "candidate_margin",
    "overlap",
    "converged",
    "refinement_delta_m",
    "refinement_delta_yaw_rad",
    "runtime_sec",
    "post_reset_ok_rows",
    "post_reset_window_sec",
    "false_recovery",
    "request_reason",
    "request_score",
    "request_window_rows",
    "request_window_duration_sec",
    "generated_at",
    "reference_pose_count",
    "route_time_radius_sec",
    "route_min_spacing_m",
    "route_window_start_stamp_sec",
    "route_window_end_stamp_sec",
    "nearest_reference_stamp_sec",
    "nearest_reference_time_delta_sec",
    "yaw_offsets_deg",
    "lateral_offsets_m",
    "longitudinal_offsets_m",
    "candidates_csv",
]


CANDIDATE_FIELDNAMES = [
    "attempt_id",
    "candidate_index",
    "source",
    "pose_x",
    "pose_y",
    "pose_z",
    "yaw_rad",
    "route_stamp_sec",
    "route_time_delta_sec",
    "route_position_x",
    "route_position_y",
    "route_position_z",
    "route_yaw_rad",
    "longitudinal_offset_m",
    "lateral_offset_m",
    "yaw_offset_deg",
]


def as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def as_float(value: Any) -> float | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def parse_float_list(raw: str) -> list[float]:
    values: list[float] = []
    for token in str(raw).split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    if not values:
        raise ValueError(f"expected at least one numeric value in {raw!r}")
    return values


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def generated_at() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load_alignment_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "requested": as_bool(values.get("reinitialization_requested")),
                    "reason": values.get("reinitialization_request_reason"),
                    "score": as_float(values.get("reinitialization_request_score")),
                }
            )
    return rows


def request_windows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    windows: list[dict[str, Any]] = []
    index = 0
    while index < len(rows):
        if not rows[index]["requested"]:
            index += 1
            continue
        start = index
        while index < len(rows) and rows[index]["requested"]:
            index += 1
        end = index - 1
        windows.append(
            {
                "start_index": start,
                "end_index": end,
                "start_stamp_sec": rows[start]["stamp_sec"],
                "end_stamp_sec": rows[end]["stamp_sec"],
                "row_count": end - start + 1,
                "reason": rows[start]["reason"],
                "score": rows[start]["score"],
            }
        )
    return windows


def write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    fieldnames: Sequence[str],
    overwrite: bool,
) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
