#!/usr/bin/env python3

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores
from rosbags.typesys import get_typestore


TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

SCORE_FIELDNAMES = [
    "job_id",
    "attempt_id",
    "candidate_index",
    "selection_source",
    "selection_rank",
    "oracle_rank",
    "oracle_score",
    "trigger_stamp_sec",
    "scan_stamp_sec",
    "scan_time_delta_sec",
    "scan_point_count_raw",
    "scan_point_count_valid",
    "scan_nearest_range_m",
    "scan_farthest_range_m",
    "scan_bounds_min_x",
    "scan_bounds_min_y",
    "scan_bounds_min_z",
    "scan_bounds_max_x",
    "scan_bounds_max_y",
    "scan_bounds_max_z",
    "scan_pcd_path",
    "map_path",
    "registration_method",
    "initial_pose_x",
    "initial_pose_y",
    "initial_pose_z",
    "initial_yaw_rad",
    "status",
    "rejection_reason",
    "converged",
    "score",
    "overlap",
    "refinement_delta_m",
    "refinement_delta_yaw_rad",
    "runtime_sec",
]


@dataclass(frozen=True)
class ScanSummary:
    stamp_sec: float
    raw_point_count: int
    valid_point_count: int
    nearest_range_m: Optional[float]
    farthest_range_m: Optional[float]
    bounds_min: Tuple[Optional[float], Optional[float], Optional[float]]
    bounds_max: Tuple[Optional[float], Optional[float], Optional[float]]
    valid_points: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Resolve the nearest PointCloud2 scan for each relocalization registration job. "
            "This validates scorer inputs and writes a score-shaped CSV, but does not run "
            "registration or reset pose."
        )
    )
    parser.add_argument("--jobs-csv", required=True, help="Input relocalization_registration_jobs.csv")
    parser.add_argument(
        "--output-csv",
        required=True,
        help="Output relocalization_registration_scores.csv",
    )
    parser.add_argument(
        "--max-scan-time-diff",
        type=float,
        default=0.25,
        help="Maximum allowed absolute trigger-to-scan time difference in seconds.",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=1.0,
        help="Minimum range for valid point count and bounds.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=120.0,
        help="Maximum range for valid point count and bounds.",
    )
    parser.add_argument(
        "--max-jobs",
        type=int,
        default=0,
        help="Optional cap for smoke testing. 0 means all jobs.",
    )
    parser.add_argument(
        "--export-scan-pcd-dir",
        default="",
        help="Optional directory for one PCD file per unique resolved scan.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output CSV.")
    return parser.parse_args()


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _fmt(value: Optional[float]) -> str:
    return "" if value is None else f"{value:.9f}"


def _read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, Any]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def pointcloud2_xyz_array(message: object) -> np.ndarray:
    field_offsets = {field.name: int(field.offset) for field in message.fields}
    for field_name in ("x", "y", "z"):
        if field_name not in field_offsets:
            raise RuntimeError(f"PointCloud2 is missing required field: {field_name}")

    endian = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
            "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
            "itemsize": int(message.point_step),
        }
    )
    count = int(message.width * message.height)
    array = np.frombuffer(bytes(message.data), dtype=dtype, count=count)
    return np.column_stack([array["x"], array["y"], array["z"]]).astype(
        np.float32, copy=False
    )


def scan_stamp_sec(message: object, fallback_record_time_ns: int) -> float:
    if hasattr(message, "header") and hasattr(message.header, "stamp"):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(fallback_record_time_ns) * 1e-9


def summarize_scan(
    message: object,
    record_time_ns: int,
    min_range: float,
    max_range: float,
) -> ScanSummary:
    xyz = pointcloud2_xyz_array(message)
    raw_count = int(xyz.shape[0])
    finite = np.isfinite(xyz).all(axis=1)
    ranges = np.linalg.norm(xyz, axis=1)
    valid_mask = finite & (ranges >= min_range) & (ranges <= max_range)
    valid = xyz[valid_mask]
    valid_ranges = ranges[valid_mask]
    if valid.shape[0] == 0:
        bounds_min: Tuple[Optional[float], Optional[float], Optional[float]] = (
            None,
            None,
            None,
        )
        bounds_max: Tuple[Optional[float], Optional[float], Optional[float]] = (
            None,
            None,
            None,
        )
        nearest = None
        farthest = None
    else:
        bounds_min_values = valid.min(axis=0)
        bounds_max_values = valid.max(axis=0)
        bounds_min = tuple(float(value) for value in bounds_min_values)
        bounds_max = tuple(float(value) for value in bounds_max_values)
        nearest = float(valid_ranges.min())
        farthest = float(valid_ranges.max())
    return ScanSummary(
        stamp_sec=scan_stamp_sec(message, record_time_ns),
        raw_point_count=raw_count,
        valid_point_count=int(valid.shape[0]),
        nearest_range_m=nearest,
        farthest_range_m=farthest,
        bounds_min=bounds_min,
        bounds_max=bounds_max,
        valid_points=valid.astype(np.float32, copy=False),
    )


def write_xyz_intensity_pcd(path: Path, points: np.ndarray) -> None:
    xyz = np.asarray(points, dtype=np.float32, order="C")
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise RuntimeError(f"expected Nx3 point array, got {xyz.shape}")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {xyz.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\n"
        "DATA binary\n"
    ).encode("ascii")
    intensity = np.zeros((xyz.shape[0], 1), dtype=np.float32)
    xyzi = np.hstack([xyz, intensity]).astype("<f4", copy=False)
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(xyzi.tobytes())


def load_scan_summaries(
    bag_path: Path,
    cloud_topic: str,
    min_range: float,
    max_range: float,
) -> List[ScanSummary]:
    reader = Reader(str(bag_path))
    reader.open()
    summaries: List[ScanSummary] = []
    try:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != cloud_topic:
                continue
            message = TYPESTORE.deserialize_cdr(rawdata, connection.msgtype)
            summaries.append(
                summarize_scan(
                    message,
                    record_time_ns=int(timestamp),
                    min_range=min_range,
                    max_range=max_range,
                )
            )
    finally:
        reader.close()
    if not summaries:
        raise RuntimeError(f"no PointCloud2 scans found for topic {cloud_topic} in {bag_path}")
    summaries.sort(key=lambda scan: scan.stamp_sec)
    return summaries


def nearest_scan(
    scans: List[ScanSummary],
    target_stamp_sec: float,
) -> ScanSummary:
    # A linear scan is acceptable here because this helper is an artifact validator; the bag
    # is loaded once per unique bag/topic and job counts are intentionally capped.
    return min(scans, key=lambda scan: abs(scan.stamp_sec - target_stamp_sec))


def grouped_jobs(jobs: Iterable[Dict[str, Any]]) -> Dict[Tuple[str, str], List[Dict[str, Any]]]:
    groups: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for job in jobs:
        groups.setdefault((str(job["bag_path"]), str(job["cloud_topic"])), []).append(job)
    return groups


def build_score_row(
    job: Dict[str, Any],
    scan: Optional[ScanSummary],
    max_scan_time_diff: float,
    scan_pcd_path: Optional[Path],
    started: float,
) -> Dict[str, Any]:
    trigger_stamp = _as_float(job.get("trigger_stamp_sec"))
    if scan is None or trigger_stamp is None:
        status = "missing_scan"
        rejection_reason = "scan_not_resolved"
        scan_delta: Optional[float] = None
    else:
        scan_delta = scan.stamp_sec - trigger_stamp
        if abs(scan_delta) > max_scan_time_diff:
            status = "scan_time_mismatch"
            rejection_reason = "scan_time_delta_exceeds_threshold"
        else:
            status = "scan_resolved_no_registration"
            rejection_reason = "registration_not_implemented"

    bounds_min = (None, None, None) if scan is None else scan.bounds_min
    bounds_max = (None, None, None) if scan is None else scan.bounds_max
    return {
        "job_id": str(job.get("job_id", "")),
        "attempt_id": str(job.get("attempt_id", "")),
        "candidate_index": str(job.get("candidate_index", "")),
        "selection_source": str(job.get("selection_source", "")),
        "selection_rank": str(job.get("selection_rank", "")),
        "oracle_rank": str(job.get("oracle_rank", "")),
        "oracle_score": str(job.get("oracle_score", "")),
        "trigger_stamp_sec": str(job.get("trigger_stamp_sec", "")),
        "scan_stamp_sec": "" if scan is None else f"{scan.stamp_sec:.9f}",
        "scan_time_delta_sec": _fmt(scan_delta),
        "scan_point_count_raw": "" if scan is None else str(scan.raw_point_count),
        "scan_point_count_valid": "" if scan is None else str(scan.valid_point_count),
        "scan_nearest_range_m": "" if scan is None else _fmt(scan.nearest_range_m),
        "scan_farthest_range_m": "" if scan is None else _fmt(scan.farthest_range_m),
        "scan_bounds_min_x": _fmt(bounds_min[0]),
        "scan_bounds_min_y": _fmt(bounds_min[1]),
        "scan_bounds_min_z": _fmt(bounds_min[2]),
        "scan_bounds_max_x": _fmt(bounds_max[0]),
        "scan_bounds_max_y": _fmt(bounds_max[1]),
        "scan_bounds_max_z": _fmt(bounds_max[2]),
        "scan_pcd_path": "" if scan_pcd_path is None else str(scan_pcd_path),
        "map_path": str(job.get("map_path", "")),
        "registration_method": str(job.get("registration_method", "")),
        "initial_pose_x": str(job.get("initial_pose_x", "")),
        "initial_pose_y": str(job.get("initial_pose_y", "")),
        "initial_pose_z": str(job.get("initial_pose_z", "")),
        "initial_yaw_rad": str(job.get("initial_yaw_rad", "")),
        "status": status,
        "rejection_reason": rejection_reason,
        "converged": "false",
        "score": "",
        "overlap": "",
        "refinement_delta_m": "",
        "refinement_delta_yaw_rad": "",
        "runtime_sec": f"{time.monotonic() - started:.6f}",
    }


def resolve_jobs(
    jobs: List[Dict[str, Any]],
    max_scan_time_diff: float,
    min_range: float,
    max_range: float,
    export_scan_pcd_dir: Optional[Path],
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    exported_scans: Dict[Tuple[str, str, str], Path] = {}
    for (bag_path_raw, cloud_topic), group in grouped_jobs(jobs).items():
        bag_path = Path(bag_path_raw)
        scans = load_scan_summaries(
            bag_path,
            cloud_topic=cloud_topic,
            min_range=min_range,
            max_range=max_range,
        )
        for job in group:
            started = time.monotonic()
            trigger_stamp = _as_float(job.get("trigger_stamp_sec"))
            scan = None if trigger_stamp is None else nearest_scan(scans, trigger_stamp)
            scan_pcd_path = None
            if scan is not None and export_scan_pcd_dir is not None:
                key = (str(bag_path), cloud_topic, f"{scan.stamp_sec:.9f}")
                scan_pcd_path = exported_scans.get(key)
                if scan_pcd_path is None:
                    safe_topic = cloud_topic.strip("/").replace("/", "_") or "cloud"
                    scan_pcd_path = (
                        export_scan_pcd_dir
                        / f"{safe_topic}_{scan.stamp_sec:.9f}.pcd"
                    )
                    write_xyz_intensity_pcd(scan_pcd_path, scan.valid_points)
                    exported_scans[key] = scan_pcd_path
            rows.append(build_score_row(job, scan, max_scan_time_diff, scan_pcd_path, started))
    return rows


def write_scores(path: Path, rows: List[Dict[str, Any]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCORE_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    jobs_csv = Path(args.jobs_csv).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    if not jobs_csv.exists():
        raise FileNotFoundError(f"jobs CSV does not exist: {jobs_csv}")

    _, jobs = _read_csv_rows(jobs_csv)
    if args.max_jobs > 0:
        jobs = jobs[: args.max_jobs]
    rows = resolve_jobs(
        jobs,
        max_scan_time_diff=float(args.max_scan_time_diff),
        min_range=float(args.min_range),
        max_range=float(args.max_range),
        export_scan_pcd_dir=(
            Path(args.export_scan_pcd_dir).expanduser().resolve()
            if args.export_scan_pcd_dir
            else None
        ),
    )
    write_scores(output_csv, rows, overwrite=args.overwrite)
    status_counts: Dict[str, int] = {}
    for row in rows:
        status_counts[row["status"]] = status_counts.get(row["status"], 0) + 1
    print(
        json.dumps(
            {
                "output_csv": str(output_csv),
                "job_count": len(jobs),
                "score_row_count": len(rows),
                "status_counts": status_counts,
            }
        )
    )


if __name__ == "__main__":
    main()
