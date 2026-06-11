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
            "Create a map-wide grid relocalization candidate artifact from "
            "alignment_status.csv and the pointcloud map. Unlike the route-grid "
            "helper, this needs no reference trajectory or route prior: candidate "
            "seeds are sampled from occupied map cells. This helper does not score "
            "candidates or reset pose; registration scoring orders the candidates."
        )
    )
    parser.add_argument("--alignment-csv", required=True, help="Input alignment_status.csv")
    parser.add_argument("--map-pcd", required=True, help="Pointcloud map (.pcd or .ply)")
    parser.add_argument("--output-csv", required=True, help="Output relocalization_attempts.csv")
    parser.add_argument(
        "--output-candidates-csv",
        default="",
        help="Optional per-candidate CSV. Defaults to relocalization_candidates.csv next to output CSV.",
    )
    parser.add_argument("--source", default="map_grid", help="Attempt source label.")
    parser.add_argument("--mode", default="offline_map_roi", help="Attempt mode label.")
    parser.add_argument("--roi-type", default="map_grid", help="ROI type label.")
    parser.add_argument(
        "--grid-spacing-m",
        type=float,
        default=10.0,
        help="XY spacing of candidate seed cells sampled from the map.",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=int,
        default=20,
        help="Cells with fewer map points are skipped as unreliable seed locations.",
    )
    parser.add_argument(
        "--z-percentile",
        type=float,
        default=5.0,
        help="Per-cell z percentile used as the ground estimate for the seed pose.",
    )
    parser.add_argument(
        "--z-offset-m",
        type=float,
        default=0.0,
        help="Offset added to the per-cell ground estimate, for example sensor height.",
    )
    parser.add_argument(
        "--yaw-count",
        type=int,
        default=8,
        help="Number of evenly spaced yaw hypotheses per seed cell.",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=1024,
        help=(
            "Maximum candidate poses per request window. When the grid exceeds this, "
            "candidates are strided deterministically to keep map-wide coverage."
        ),
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


def load_map_points(path: Path):
    import numpy as np
    import open3d as o3d

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    if points.size == 0:
        raise RuntimeError(f"map has no points: {path}")
    return points


def build_grid_cells(
    points,
    grid_spacing_m: float,
    min_points_per_cell: int,
    z_percentile: float,
    z_offset_m: float,
) -> List[Dict[str, float]]:
    """Sample one seed location per occupied XY cell of the map."""
    import numpy as np

    if grid_spacing_m <= 0.0:
        raise ValueError("grid spacing must be positive")
    cell_x = np.floor(points[:, 0] / grid_spacing_m).astype(np.int64)
    cell_y = np.floor(points[:, 1] / grid_spacing_m).astype(np.int64)
    keys = cell_x * np.int64(1 << 32) + cell_y
    order = np.argsort(keys, kind="stable")
    sorted_keys = keys[order]
    boundaries = np.flatnonzero(np.diff(sorted_keys)) + 1
    groups = np.split(order, boundaries)
    cells: List[Dict[str, float]] = []
    for group in groups:
        if len(group) < max(1, min_points_per_cell):
            continue
        cell_points = points[group]
        cells.append(
            {
                "x": float(np.mean(cell_points[:, 0])),
                "y": float(np.mean(cell_points[:, 1])),
                "z": float(np.percentile(cell_points[:, 2], z_percentile) + z_offset_m),
                "point_count": float(len(group)),
            }
        )
    cells.sort(key=lambda cell: (cell["x"], cell["y"]))
    return cells


def build_candidates(
    attempt_id: str,
    cells: List[Dict[str, float]],
    source: str,
    yaw_count: int,
    max_candidates: int,
) -> Tuple[List[Dict[str, Any]], List[float]]:
    if yaw_count <= 0:
        raise ValueError("yaw count must be positive")
    yaws = [_wrap_angle(2.0 * math.pi * index / yaw_count) for index in range(yaw_count)]
    # Cap by striding CELLS and keeping every yaw of each kept cell. Striding the
    # flat (cell, yaw) list aliases with yaw_count and silently collapses the yaw
    # dimension to a single heading.
    kept_cells = cells
    if max_candidates > 0 and len(cells) * yaw_count > max_candidates:
        max_cells = max(1, max_candidates // yaw_count)
        stride = math.ceil(len(cells) / max_cells)
        kept_cells = cells[::stride]
    raw: List[Tuple[Dict[str, float], float]] = [
        (cell, yaw) for cell in kept_cells for yaw in yaws
    ]
    if max_candidates > 0 and len(raw) > max_candidates:
        raw = raw[:max_candidates]
    candidates: List[Dict[str, Any]] = []
    for cell, yaw in raw:
        candidates.append(
            {
                "attempt_id": attempt_id,
                "candidate_index": str(len(candidates)),
                "source": source,
                "pose_x": f"{cell['x']:.9f}",
                "pose_y": f"{cell['y']:.9f}",
                "pose_z": f"{cell['z']:.9f}",
                "yaw_rad": f"{yaw:.9f}",
                "route_stamp_sec": "",
                "route_time_delta_sec": "",
                "route_position_x": "",
                "route_position_y": "",
                "route_position_z": "",
                "route_yaw_rad": "",
                "longitudinal_offset_m": "",
                "lateral_offset_m": "",
                "yaw_offset_deg": f"{math.degrees(yaw):.9f}",
            }
        )
    return candidates, yaws


def build_attempt_artifacts(
    alignment_rows: List[Dict[str, Any]],
    cells: List[Dict[str, float]],
    candidates_csv: Path,
    source: str,
    mode: str,
    roi_type: str,
    yaw_count: int,
    max_candidates: int,
    grid_spacing_m: float,
    rejection_reason: str,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    attempts: List[Dict[str, Any]] = []
    candidate_rows: List[Dict[str, Any]] = []
    for window in request_windows(alignment_rows):
        started = time.monotonic()
        attempt_id = f"{source}_{len(attempts) + 1:04d}"
        trigger_stamp_sec = float(window["start_stamp_sec"])
        candidates, yaws = build_candidates(
            attempt_id=attempt_id,
            cells=cells,
            source=source,
            yaw_count=yaw_count,
            max_candidates=max_candidates,
        )
        candidate_rows.extend(candidates)
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
                "reference_pose_count": str(len(cells)),
                "route_time_radius_sec": "",
                "route_min_spacing_m": f"{grid_spacing_m:.9f}",
                "route_window_start_stamp_sec": "",
                "route_window_end_stamp_sec": "",
                "nearest_reference_stamp_sec": "",
                "nearest_reference_time_delta_sec": "",
                "yaw_offsets_deg": ",".join(f"{math.degrees(yaw):.9f}" for yaw in yaws),
                "lateral_offsets_m": "",
                "longitudinal_offsets_m": "",
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
    map_pcd = Path(args.map_pcd).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    candidates_csv = (
        Path(args.output_candidates_csv).expanduser().resolve()
        if args.output_candidates_csv
        else output_csv.with_name("relocalization_candidates.csv")
    )
    if not alignment_csv.exists():
        raise FileNotFoundError(f"alignment CSV does not exist: {alignment_csv}")
    if not map_pcd.exists():
        raise FileNotFoundError(f"map does not exist: {map_pcd}")

    cells = build_grid_cells(
        load_map_points(map_pcd),
        grid_spacing_m=args.grid_spacing_m,
        min_points_per_cell=args.min_points_per_cell,
        z_percentile=args.z_percentile,
        z_offset_m=args.z_offset_m,
    )
    if not cells:
        raise RuntimeError("no occupied map cells survived the grid filters")

    attempts, candidates = build_attempt_artifacts(
        alignment_rows=load_alignment_rows(alignment_csv),
        cells=cells,
        candidates_csv=candidates_csv,
        source=args.source,
        mode=args.mode,
        roi_type=args.roi_type,
        yaw_count=args.yaw_count,
        max_candidates=args.max_candidates,
        grid_spacing_m=args.grid_spacing_m,
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
                "grid_cell_count": len(cells),
            }
        )
    )


if __name__ == "__main__":
    main()
