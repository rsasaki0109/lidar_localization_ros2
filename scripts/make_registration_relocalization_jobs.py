#!/usr/bin/env python3

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple


JOB_FIELDNAMES = [
    "job_id",
    "attempt_id",
    "candidate_index",
    "trigger_stamp_sec",
    "bag_path",
    "cloud_topic",
    "map_path",
    "registration_method",
    "initial_pose_x",
    "initial_pose_y",
    "initial_pose_z",
    "initial_yaw_rad",
    "route_stamp_sec",
    "route_time_delta_sec",
    "candidate_source",
    "selection_source",
    "selection_rank",
    "oracle_rank",
    "oracle_score",
    "oracle_translation_error_m",
    "oracle_yaw_error_deg",
    "voxel_leaf_size",
    "local_map_radius",
    "timeout_sec",
    "generated_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create registration-scoring job rows from relocalization candidate artifacts. "
            "This is a contract artifact only; it does not run registration or reset pose."
        )
    )
    parser.add_argument("--attempts-csv", required=True, help="Input relocalization_attempts.csv")
    parser.add_argument(
        "--candidates-csv",
        required=True,
        help="Input relocalization_candidates.csv or relocalization_candidate_scores.csv",
    )
    parser.add_argument("--bag-path", required=True, help="Rosbag2 directory used by the run")
    parser.add_argument("--map-path", required=True, help="Point cloud map path used by the run")
    parser.add_argument("--cloud-topic", required=True, help="PointCloud2 topic to score")
    parser.add_argument("--output-csv", required=True, help="Output registration jobs CSV")
    parser.add_argument(
        "--registration-method",
        default="NDT_OMP",
        help="Registration backend label for the future scorer.",
    )
    parser.add_argument(
        "--max-candidates-per-attempt",
        type=int,
        default=32,
        help="Maximum candidates to emit per attempt. 0 means all candidates.",
    )
    parser.add_argument(
        "--selection-source",
        choices=["candidate_index", "route_proximity", "oracle_rank"],
        default="candidate_index",
        help=(
            "How to order candidates before truncation. route_proximity is an oracle-free "
            "route-centered order. oracle_rank is an offline diagnostic mode and must not be "
            "used as a runtime claim."
        ),
    )
    parser.add_argument("--voxel-leaf-size", type=float, default=1.0)
    parser.add_argument("--local-map-radius", type=float, default=150.0)
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output CSV.")
    return parser.parse_args()


def _read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, Any]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        return float(str(value))
    except (TypeError, ValueError):
        return None


def _as_int(value: Any) -> Optional[int]:
    if value is None or str(value).strip() == "":
        return None
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return None


def _abs_float_or_large(row: Dict[str, Any], key: str) -> float:
    value = _as_float(row.get(key))
    if value is None:
        return float("inf")
    return abs(value)


def _candidate_sort_key(row: Dict[str, Any], selection_source: str) -> Tuple[float, ...]:
    candidate_index = _as_int(row.get("candidate_index"))
    if candidate_index is None:
        candidate_index = 10**9
    if selection_source == "route_proximity":
        return (
            _abs_float_or_large(row, "route_time_delta_sec"),
            _abs_float_or_large(row, "longitudinal_offset_m"),
            _abs_float_or_large(row, "lateral_offset_m"),
            _abs_float_or_large(row, "yaw_offset_deg"),
            float(candidate_index),
        )
    if selection_source == "oracle_rank":
        oracle_rank = _as_int(row.get("oracle_rank"))
        if oracle_rank is not None:
            return (float(oracle_rank), float(candidate_index))
    return (float(candidate_index), float(candidate_index))


def build_jobs(
    attempts: List[Dict[str, Any]],
    candidates: List[Dict[str, Any]],
    bag_path: Path,
    map_path: Path,
    cloud_topic: str,
    registration_method: str,
    max_candidates_per_attempt: int,
    selection_source: str,
    voxel_leaf_size: float,
    local_map_radius: float,
    timeout_sec: float,
) -> List[Dict[str, Any]]:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    attempts_by_id = {str(row.get("attempt_id", "")): row for row in attempts}
    candidates_by_attempt: Dict[str, List[Dict[str, Any]]] = {}
    for candidate in candidates:
        candidates_by_attempt.setdefault(str(candidate.get("attempt_id", "")), []).append(
            candidate
        )

    jobs: List[Dict[str, Any]] = []
    for attempt_id, attempt in attempts_by_id.items():
        attempt_candidates = sorted(
            candidates_by_attempt.get(attempt_id, []),
            key=lambda row: _candidate_sort_key(row, selection_source),
        )
        if max_candidates_per_attempt > 0:
            attempt_candidates = attempt_candidates[:max_candidates_per_attempt]
        trigger_stamp_sec = str(attempt.get("trigger_stamp_sec", ""))
        for selection_index, candidate in enumerate(attempt_candidates):
            candidate_index = str(candidate.get("candidate_index", ""))
            jobs.append(
                {
                    "job_id": f"registration_job_{len(jobs) + 1:06d}",
                    "attempt_id": attempt_id,
                    "candidate_index": candidate_index,
                    "trigger_stamp_sec": trigger_stamp_sec,
                    "bag_path": str(bag_path),
                    "cloud_topic": cloud_topic,
                    "map_path": str(map_path),
                    "registration_method": registration_method,
                    "initial_pose_x": str(candidate.get("pose_x", "")),
                    "initial_pose_y": str(candidate.get("pose_y", "")),
                    "initial_pose_z": str(candidate.get("pose_z", "")),
                    "initial_yaw_rad": str(candidate.get("yaw_rad", "")),
                    "route_stamp_sec": str(candidate.get("route_stamp_sec", "")),
                    "route_time_delta_sec": str(candidate.get("route_time_delta_sec", "")),
                    "candidate_source": str(candidate.get("source", "")),
                    "selection_source": selection_source,
                    "selection_rank": str(selection_index + 1),
                    "oracle_rank": str(candidate.get("oracle_rank", "")),
                    "oracle_score": str(candidate.get("oracle_score", "")),
                    "oracle_translation_error_m": str(
                        candidate.get("oracle_translation_error_m", "")
                    ),
                    "oracle_yaw_error_deg": str(candidate.get("oracle_yaw_error_deg", "")),
                    "voxel_leaf_size": f"{voxel_leaf_size:.9f}",
                    "local_map_radius": f"{local_map_radius:.9f}",
                    "timeout_sec": f"{timeout_sec:.9f}",
                    "generated_at": generated_at,
                }
            )
    return jobs


def write_jobs(path: Path, jobs: List[Dict[str, Any]], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"output CSV already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=JOB_FIELDNAMES)
        writer.writeheader()
        writer.writerows(jobs)


def main() -> None:
    args = parse_args()
    attempts_csv = Path(args.attempts_csv).expanduser().resolve()
    candidates_csv = Path(args.candidates_csv).expanduser().resolve()
    bag_path = Path(args.bag_path).expanduser().resolve()
    map_path = Path(args.map_path).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    for path, label in [
        (attempts_csv, "attempts CSV"),
        (candidates_csv, "candidates CSV"),
        (bag_path, "bag path"),
        (map_path, "map path"),
    ]:
        if not path.exists():
            raise FileNotFoundError(f"{label} does not exist: {path}")

    _, attempts = _read_csv_rows(attempts_csv)
    _, candidates = _read_csv_rows(candidates_csv)
    jobs = build_jobs(
        attempts=attempts,
        candidates=candidates,
        bag_path=bag_path,
        map_path=map_path,
        cloud_topic=args.cloud_topic,
        registration_method=args.registration_method,
        max_candidates_per_attempt=args.max_candidates_per_attempt,
        selection_source=args.selection_source,
        voxel_leaf_size=args.voxel_leaf_size,
        local_map_radius=args.local_map_radius,
        timeout_sec=args.timeout_sec,
    )
    write_jobs(output_csv, jobs, overwrite=args.overwrite)
    print(
        json.dumps(
            {
                "output_csv": str(output_csv),
                "attempt_count": len(attempts),
                "candidate_count": len(candidates),
                "job_count": len(jobs),
                "selection_source": args.selection_source,
            }
        )
    )


if __name__ == "__main__":
    main()
