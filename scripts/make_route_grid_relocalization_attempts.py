#!/usr/bin/env python3

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a route-grid relocalization candidate artifact from "
            "alignment_status.csv and a reference trajectory. This helper does not "
            "score candidates or reset pose; it records candidate coverage so global "
            "relocalization can be developed against a stable CSV contract."
        )
    )
    parser.add_argument("--alignment-csv", required=True, help="Input alignment_status.csv")
    parser.add_argument("--reference-csv", required=True, help="Reference trajectory CSV")
    parser.add_argument("--output-csv", required=True, help="Output relocalization_attempts.csv")
    parser.add_argument(
        "--output-candidates-csv",
        default="",
        help="Optional per-candidate CSV. Defaults to relocalization_candidates.csv next to output CSV.",
    )
    parser.add_argument("--source", default="route_grid", help="Attempt source label.")
    parser.add_argument(
        "--mode",
        default="offline_route_roi",
        help="Attempt mode label.",
    )
    parser.add_argument(
        "--roi-type",
        default="route_corridor",
        help="ROI type label.",
    )
    parser.add_argument(
        "--route-time-radius-sec",
        type=float,
        default=15.0,
        help="Reference poses within +/- this many seconds around the trigger become route seeds.",
    )
    parser.add_argument(
        "--route-min-spacing-m",
        type=float,
        default=10.0,
        help="Minimum XY spacing between route seed poses after time-window selection.",
    )
    parser.add_argument(
        "--max-route-poses",
        type=int,
        default=32,
        help="Maximum route seed poses per request window after spacing.",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=256,
        help="Maximum generated candidate poses per request window.",
    )
    parser.add_argument(
        "--yaw-offsets-deg",
        default="0",
        help="Comma-separated yaw offsets in degrees, for example '-15,0,15'.",
    )
    parser.add_argument(
        "--lateral-offsets-m",
        default="0",
        help="Comma-separated lateral offsets in meters.",
    )
    parser.add_argument(
        "--longitudinal-offsets-m",
        default="0",
        help="Comma-separated longitudinal offsets in meters.",
    )
    parser.add_argument(
        "--rejection-reason",
        default="candidate_scoring_not_implemented",
        help="Rejection reason written for every generated attempt.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output CSV files.")
    return parser.parse_args()


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _parse_float_list(raw: str) -> List[float]:
    values: List[float] = []
    for token in str(raw).split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    if not values:
        raise ValueError(f"expected at least one numeric value in {raw!r}")
    return values


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            values = json.loads(record["values_json"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "requested": _as_bool(values.get("reinitialization_requested")),
                    "reason": values.get("reinitialization_request_reason"),
                    "score": _as_float(values.get("reinitialization_request_score")),
                }
            )
    return rows


def load_reference_rows(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream):
            qx = float(record["orientation_x"])
            qy = float(record["orientation_y"])
            qz = float(record["orientation_z"])
            qw = float(record["orientation_w"])
            rows.append(
                {
                    "stamp_sec": float(record["stamp_sec"]),
                    "x": float(record["position_x"]),
                    "y": float(record["position_y"]),
                    "z": float(record["position_z"]),
                    "yaw": _yaw_from_quaternion(qx, qy, qz, qw),
                }
            )
    if not rows:
        raise RuntimeError(f"reference CSV has no rows: {path}")
    rows.sort(key=lambda row: row["stamp_sec"])
    return rows


def request_windows(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    windows: List[Dict[str, Any]] = []
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


def _distance_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def _spaced_reference_rows(
    rows: Iterable[Dict[str, float]],
    min_spacing_m: float,
    max_rows: int,
) -> List[Dict[str, float]]:
    selected: List[Dict[str, float]] = []
    for row in rows:
        if selected and min_spacing_m > 0.0 and _distance_xy(row, selected[-1]) < min_spacing_m:
            continue
        selected.append(row)
        if max_rows > 0 and len(selected) >= max_rows:
            break
    return selected


def select_route_rows(
    reference_rows: List[Dict[str, float]],
    trigger_stamp_sec: float,
    time_radius_sec: float,
    min_spacing_m: float,
    max_route_poses: int,
) -> Tuple[List[Dict[str, float]], Dict[str, float]]:
    nearest = min(
        reference_rows,
        key=lambda row: abs(row["stamp_sec"] - trigger_stamp_sec),
    )
    start_stamp = trigger_stamp_sec - time_radius_sec
    end_stamp = trigger_stamp_sec + time_radius_sec
    window_rows = [
        row for row in reference_rows if start_stamp <= row["stamp_sec"] <= end_stamp
    ]
    if not window_rows:
        window_rows = [nearest]
    selected = _spaced_reference_rows(window_rows, min_spacing_m, max_route_poses)
    if not selected:
        selected = [nearest]
    if all(row["stamp_sec"] != nearest["stamp_sec"] for row in selected):
        selected.append(nearest)
        selected.sort(key=lambda row: row["stamp_sec"])
        if max_route_poses > 0 and len(selected) > max_route_poses:
            selected = sorted(
                selected,
                key=lambda row: (row["stamp_sec"] != nearest["stamp_sec"], abs(row["stamp_sec"] - nearest["stamp_sec"])),
            )[:max_route_poses]
            selected.sort(key=lambda row: row["stamp_sec"])
    return selected, nearest


def build_candidates(
    attempt_id: str,
    route_rows: List[Dict[str, float]],
    trigger_stamp_sec: float,
    source: str,
    longitudinal_offsets_m: List[float],
    lateral_offsets_m: List[float],
    yaw_offsets_deg: List[float],
    max_candidates: int,
) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    for route_row in route_rows:
        cos_yaw = math.cos(route_row["yaw"])
        sin_yaw = math.sin(route_row["yaw"])
        for longitudinal in longitudinal_offsets_m:
            for lateral in lateral_offsets_m:
                for yaw_offset_deg in yaw_offsets_deg:
                    x = route_row["x"] + longitudinal * cos_yaw - lateral * sin_yaw
                    y = route_row["y"] + longitudinal * sin_yaw + lateral * cos_yaw
                    yaw = _wrap_angle(route_row["yaw"] + math.radians(yaw_offset_deg))
                    candidates.append(
                        {
                            "attempt_id": attempt_id,
                            "candidate_index": str(len(candidates)),
                            "source": source,
                            "pose_x": f"{x:.9f}",
                            "pose_y": f"{y:.9f}",
                            "pose_z": f"{route_row['z']:.9f}",
                            "yaw_rad": f"{yaw:.9f}",
                            "route_stamp_sec": f"{route_row['stamp_sec']:.9f}",
                            "route_time_delta_sec": f"{route_row['stamp_sec'] - trigger_stamp_sec:.9f}",
                            "route_position_x": f"{route_row['x']:.9f}",
                            "route_position_y": f"{route_row['y']:.9f}",
                            "route_position_z": f"{route_row['z']:.9f}",
                            "route_yaw_rad": f"{route_row['yaw']:.9f}",
                            "longitudinal_offset_m": f"{longitudinal:.9f}",
                            "lateral_offset_m": f"{lateral:.9f}",
                            "yaw_offset_deg": f"{yaw_offset_deg:.9f}",
                        }
                    )
                    if max_candidates > 0 and len(candidates) >= max_candidates:
                        return candidates
    return candidates


def build_attempt_artifacts(
    alignment_rows: List[Dict[str, Any]],
    reference_rows: List[Dict[str, float]],
    candidates_csv: Path,
    source: str,
    mode: str,
    roi_type: str,
    route_time_radius_sec: float,
    route_min_spacing_m: float,
    max_route_poses: int,
    max_candidates: int,
    yaw_offsets_deg: List[float],
    lateral_offsets_m: List[float],
    longitudinal_offsets_m: List[float],
    rejection_reason: str,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    attempts: List[Dict[str, Any]] = []
    candidate_rows: List[Dict[str, Any]] = []
    for window in request_windows(alignment_rows):
        started = time.monotonic()
        attempt_id = f"{source}_{len(attempts) + 1:04d}"
        trigger_stamp_sec = float(window["start_stamp_sec"])
        route_rows, nearest = select_route_rows(
            reference_rows,
            trigger_stamp_sec=trigger_stamp_sec,
            time_radius_sec=route_time_radius_sec,
            min_spacing_m=route_min_spacing_m,
            max_route_poses=max_route_poses,
        )
        candidates = build_candidates(
            attempt_id=attempt_id,
            route_rows=route_rows,
            trigger_stamp_sec=trigger_stamp_sec,
            source=source,
            longitudinal_offsets_m=longitudinal_offsets_m,
            lateral_offsets_m=lateral_offsets_m,
            yaw_offsets_deg=yaw_offsets_deg,
            max_candidates=max_candidates,
        )
        candidate_rows.extend(candidates)
        route_start = min(row["stamp_sec"] for row in route_rows)
        route_end = max(row["stamp_sec"] for row in route_rows)
        attempts.append(
            {
                "attempt_id": attempt_id,
                "trigger_stamp_sec": f"{trigger_stamp_sec:.9f}",
                "start_stamp_sec": f"{trigger_stamp_sec:.9f}",
                "end_stamp_sec": f"{trigger_stamp_sec:.9f}",
                "source": source,
                "mode": mode,
                "roi_type": roi_type,
                "candidate_count": str(len(candidates)),
                "accepted": "false",
                "accepted_candidate_rank": "",
                "rejection_reason": rejection_reason,
                "best_score": "",
                "second_score": "",
                "candidate_margin": "",
                "overlap": "",
                "converged": "false",
                "refinement_delta_m": "",
                "refinement_delta_yaw_rad": "",
                "runtime_sec": f"{time.monotonic() - started:.6f}",
                "post_reset_ok_rows": "0",
                "post_reset_window_sec": "",
                "false_recovery": "false",
                "request_reason": window["reason"] or "",
                "request_score": "" if window["score"] is None else f"{window['score']:.10g}",
                "request_window_rows": str(window["row_count"]),
                "request_window_duration_sec": f"{max(0.0, window['end_stamp_sec'] - window['start_stamp_sec']):.9f}",
                "generated_at": generated_at,
                "reference_pose_count": str(len(route_rows)),
                "route_time_radius_sec": f"{route_time_radius_sec:.9f}",
                "route_min_spacing_m": f"{route_min_spacing_m:.9f}",
                "route_window_start_stamp_sec": f"{route_start:.9f}",
                "route_window_end_stamp_sec": f"{route_end:.9f}",
                "nearest_reference_stamp_sec": f"{nearest['stamp_sec']:.9f}",
                "nearest_reference_time_delta_sec": f"{nearest['stamp_sec'] - trigger_stamp_sec:.9f}",
                "yaw_offsets_deg": ",".join(f"{value:.9f}" for value in yaw_offsets_deg),
                "lateral_offsets_m": ",".join(f"{value:.9f}" for value in lateral_offsets_m),
                "longitudinal_offsets_m": ",".join(
                    f"{value:.9f}" for value in longitudinal_offsets_m
                ),
                "candidates_csv": str(candidates_csv),
            }
        )
    return attempts, candidate_rows


def write_csv(
    path: Path,
    rows: List[Dict[str, Any]],
    fieldnames: List[str],
    overwrite: bool,
) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    alignment_csv = Path(args.alignment_csv).expanduser().resolve()
    reference_csv = Path(args.reference_csv).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    candidates_csv = (
        Path(args.output_candidates_csv).expanduser().resolve()
        if args.output_candidates_csv
        else output_csv.with_name("relocalization_candidates.csv")
    )
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")
    if not reference_csv.exists():
        raise FileNotFoundError(f"reference CSV does not exist: {reference_csv}")

    attempts, candidates = build_attempt_artifacts(
        alignment_rows=load_alignment_rows(alignment_csv),
        reference_rows=load_reference_rows(reference_csv),
        candidates_csv=candidates_csv,
        source=args.source,
        mode=args.mode,
        roi_type=args.roi_type,
        route_time_radius_sec=args.route_time_radius_sec,
        route_min_spacing_m=args.route_min_spacing_m,
        max_route_poses=args.max_route_poses,
        max_candidates=args.max_candidates,
        yaw_offsets_deg=_parse_float_list(args.yaw_offsets_deg),
        lateral_offsets_m=_parse_float_list(args.lateral_offsets_m),
        longitudinal_offsets_m=_parse_float_list(args.longitudinal_offsets_m),
        rejection_reason=args.rejection_reason,
    )
    write_csv(output_csv, attempts, ATTEMPT_FIELDNAMES, overwrite=args.overwrite)
    write_csv(candidates_csv, candidates, CANDIDATE_FIELDNAMES, overwrite=args.overwrite)
    print(
        json.dumps(
            {
                "output_csv": str(output_csv),
                "output_candidates_csv": str(candidates_csv),
                "attempt_count": len(attempts),
                "candidate_count": len(candidates),
            }
        )
    )


if __name__ == "__main__":
    main()
