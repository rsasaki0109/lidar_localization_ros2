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


SCORED_CANDIDATE_EXTRA_FIELDS = [
    "target_reference_stamp_sec",
    "target_reference_time_delta_sec",
    "target_reference_x",
    "target_reference_y",
    "target_reference_z",
    "target_reference_yaw_rad",
    "oracle_translation_error_m",
    "oracle_xy_error_m",
    "oracle_z_error_m",
    "oracle_yaw_error_rad",
    "oracle_yaw_error_deg",
    "oracle_score",
    "oracle_recoverable",
    "oracle_rank",
    "oracle_is_best",
]


ATTEMPT_EXTRA_FIELDS = [
    "oracle_scored_candidate_count",
    "oracle_best_candidate_index",
    "oracle_best_translation_error_m",
    "oracle_best_xy_error_m",
    "oracle_best_yaw_error_deg",
    "oracle_recoverable",
    "oracle_translation_threshold_m",
    "oracle_yaw_threshold_deg",
    "oracle_yaw_weight_m_per_rad",
    "scored_candidates_csv",
    "scored_at",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Score relocalization candidates against a reference trajectory as an "
            "offline oracle. This does not call registration and does not accept or "
            "reset poses; it measures whether the candidate set contains a plausible "
            "ground-truth-near pose."
        )
    )
    parser.add_argument("--attempts-csv", required=True, help="Input relocalization_attempts.csv")
    parser.add_argument(
        "--candidates-csv",
        required=True,
        help="Input relocalization_candidates.csv from a candidate generator.",
    )
    parser.add_argument("--reference-csv", required=True, help="Reference trajectory CSV")
    parser.add_argument(
        "--output-attempts-csv",
        required=True,
        help="Output attempts CSV with oracle score fields filled.",
    )
    parser.add_argument(
        "--output-scored-candidates-csv",
        default="",
        help="Output per-candidate score CSV. Defaults to relocalization_candidate_scores.csv.",
    )
    parser.add_argument(
        "--translation-threshold-m",
        type=float,
        default=2.0,
        help="Translation error threshold used only for oracle_recoverable.",
    )
    parser.add_argument(
        "--yaw-threshold-deg",
        type=float,
        default=15.0,
        help="Yaw error threshold used only for oracle_recoverable.",
    )
    parser.add_argument(
        "--yaw-weight-m-per-rad",
        type=float,
        default=1.0,
        help="Score = translation_error_m + yaw_weight_m_per_rad * yaw_error_rad.",
    )
    parser.add_argument(
        "--rejection-reason",
        default="offline_oracle_scored_no_reset",
        help="Rejection reason written for scored attempts. Attempts remain accepted=false.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output CSV files.")
    return parser.parse_args()


def _as_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, Any]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


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


def nearest_reference(
    reference_rows: List[Dict[str, float]],
    stamp_sec: float,
) -> Dict[str, float]:
    return min(reference_rows, key=lambda row: abs(row["stamp_sec"] - stamp_sec))


def score_candidate(
    candidate: Dict[str, Any],
    target: Dict[str, float],
    trigger_stamp_sec: float,
    translation_threshold_m: float,
    yaw_threshold_rad: float,
    yaw_weight_m_per_rad: float,
) -> Dict[str, Any]:
    pose_x = float(candidate["pose_x"])
    pose_y = float(candidate["pose_y"])
    pose_z = float(candidate["pose_z"])
    yaw = float(candidate["yaw_rad"])
    xy_error = math.hypot(pose_x - target["x"], pose_y - target["y"])
    z_error = abs(pose_z - target["z"])
    translation_error = math.sqrt(xy_error * xy_error + z_error * z_error)
    yaw_error = _angle_error(yaw, target["yaw"])
    score = translation_error + yaw_weight_m_per_rad * yaw_error
    recoverable = (
        translation_error <= translation_threshold_m and yaw_error <= yaw_threshold_rad
    )
    scored = dict(candidate)
    scored.update(
        {
            "target_reference_stamp_sec": f"{target['stamp_sec']:.9f}",
            "target_reference_time_delta_sec": f"{target['stamp_sec'] - trigger_stamp_sec:.9f}",
            "target_reference_x": f"{target['x']:.9f}",
            "target_reference_y": f"{target['y']:.9f}",
            "target_reference_z": f"{target['z']:.9f}",
            "target_reference_yaw_rad": f"{target['yaw']:.9f}",
            "oracle_translation_error_m": f"{translation_error:.9f}",
            "oracle_xy_error_m": f"{xy_error:.9f}",
            "oracle_z_error_m": f"{z_error:.9f}",
            "oracle_yaw_error_rad": f"{yaw_error:.9f}",
            "oracle_yaw_error_deg": f"{math.degrees(yaw_error):.9f}",
            "oracle_score": f"{score:.9f}",
            "oracle_recoverable": "true" if recoverable else "false",
            "oracle_rank": "",
            "oracle_is_best": "false",
        }
    )
    return scored


def build_scored_artifacts(
    attempts: List[Dict[str, Any]],
    candidates: List[Dict[str, Any]],
    reference_rows: List[Dict[str, float]],
    scored_candidates_csv: Path,
    translation_threshold_m: float,
    yaw_threshold_deg: float,
    yaw_weight_m_per_rad: float,
    rejection_reason: str,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    yaw_threshold_rad = math.radians(yaw_threshold_deg)
    scored_at = datetime.now().astimezone().isoformat(timespec="seconds")
    candidates_by_attempt: Dict[str, List[Dict[str, Any]]] = {}
    for candidate in candidates:
        candidates_by_attempt.setdefault(str(candidate["attempt_id"]), []).append(candidate)

    scored_attempts: List[Dict[str, Any]] = []
    scored_candidates: List[Dict[str, Any]] = []
    for attempt in attempts:
        started = time.monotonic()
        updated = dict(attempt)
        attempt_id = str(updated.get("attempt_id", ""))
        trigger_stamp = _as_float(updated.get("trigger_stamp_sec"))
        attempt_candidates = candidates_by_attempt.get(attempt_id, [])
        if trigger_stamp is None or not attempt_candidates:
            updated.update(
                {
                    "accepted": "false",
                    "rejection_reason": "no_oracle_candidates",
                    "oracle_scored_candidate_count": str(len(attempt_candidates)),
                    "oracle_recoverable": "false",
                    "scored_candidates_csv": str(scored_candidates_csv),
                    "scored_at": scored_at,
                }
            )
            scored_attempts.append(updated)
            continue

        target = nearest_reference(reference_rows, trigger_stamp)
        scored_for_attempt = [
            score_candidate(
                candidate,
                target=target,
                trigger_stamp_sec=trigger_stamp,
                translation_threshold_m=translation_threshold_m,
                yaw_threshold_rad=yaw_threshold_rad,
                yaw_weight_m_per_rad=yaw_weight_m_per_rad,
            )
            for candidate in attempt_candidates
        ]
        scored_for_attempt.sort(key=lambda row: float(row["oracle_score"]))
        for rank, candidate in enumerate(scored_for_attempt, start=1):
            candidate["oracle_rank"] = str(rank)
            candidate["oracle_is_best"] = "true" if rank == 1 else "false"
        scored_candidates.extend(scored_for_attempt)

        best = scored_for_attempt[0]
        second_score = (
            float(scored_for_attempt[1]["oracle_score"])
            if len(scored_for_attempt) > 1
            else None
        )
        best_score = float(best["oracle_score"])
        margin = "" if second_score is None else f"{second_score - best_score:.9f}"
        recoverable = best["oracle_recoverable"] == "true"
        runtime = _as_float(updated.get("runtime_sec")) or 0.0
        updated.update(
            {
                "candidate_count": str(len(scored_for_attempt)),
                "accepted": "false",
                "accepted_candidate_rank": "",
                "rejection_reason": rejection_reason,
                "best_score": f"{best_score:.9f}",
                "second_score": "" if second_score is None else f"{second_score:.9f}",
                "candidate_margin": margin,
                "converged": "false",
                "post_reset_ok_rows": "0",
                "false_recovery": "false",
                "runtime_sec": f"{runtime + (time.monotonic() - started):.6f}",
                "oracle_scored_candidate_count": str(len(scored_for_attempt)),
                "oracle_best_candidate_index": str(best.get("candidate_index", "")),
                "oracle_best_translation_error_m": best["oracle_translation_error_m"],
                "oracle_best_xy_error_m": best["oracle_xy_error_m"],
                "oracle_best_yaw_error_deg": best["oracle_yaw_error_deg"],
                "oracle_recoverable": "true" if recoverable else "false",
                "oracle_translation_threshold_m": f"{translation_threshold_m:.9f}",
                "oracle_yaw_threshold_deg": f"{yaw_threshold_deg:.9f}",
                "oracle_yaw_weight_m_per_rad": f"{yaw_weight_m_per_rad:.9f}",
                "scored_candidates_csv": str(scored_candidates_csv),
                "scored_at": scored_at,
            }
        )
        scored_attempts.append(updated)
    return scored_attempts, scored_candidates


def _fieldnames(base: List[str], extra: List[str]) -> List[str]:
    fields = list(base)
    for field in extra:
        if field not in fields:
            fields.append(field)
    return fields


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
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    attempts_csv = Path(args.attempts_csv).expanduser().resolve()
    candidates_csv = Path(args.candidates_csv).expanduser().resolve()
    reference_csv = Path(args.reference_csv).expanduser().resolve()
    output_attempts_csv = Path(args.output_attempts_csv).expanduser().resolve()
    output_scored_candidates_csv = (
        Path(args.output_scored_candidates_csv).expanduser().resolve()
        if args.output_scored_candidates_csv
        else output_attempts_csv.with_name("relocalization_candidate_scores.csv")
    )
    for path, label in [
        (attempts_csv, "attempts CSV"),
        (candidates_csv, "candidates CSV"),
        (reference_csv, "reference CSV"),
    ]:
        if not path.exists():
            raise FileNotFoundError(f"{label} does not exist: {path}")

    attempt_fields, attempts = _read_csv_rows(attempts_csv)
    candidate_fields, candidates = _read_csv_rows(candidates_csv)
    scored_attempts, scored_candidates = build_scored_artifacts(
        attempts=attempts,
        candidates=candidates,
        reference_rows=load_reference_rows(reference_csv),
        scored_candidates_csv=output_scored_candidates_csv,
        translation_threshold_m=args.translation_threshold_m,
        yaw_threshold_deg=args.yaw_threshold_deg,
        yaw_weight_m_per_rad=args.yaw_weight_m_per_rad,
        rejection_reason=args.rejection_reason,
    )
    write_csv(
        output_attempts_csv,
        scored_attempts,
        _fieldnames(attempt_fields, ATTEMPT_EXTRA_FIELDS),
        overwrite=args.overwrite or output_attempts_csv == attempts_csv,
    )
    write_csv(
        output_scored_candidates_csv,
        scored_candidates,
        _fieldnames(candidate_fields, SCORED_CANDIDATE_EXTRA_FIELDS),
        overwrite=args.overwrite,
    )
    recoverable_count = sum(
        1 for attempt in scored_attempts if attempt.get("oracle_recoverable") == "true"
    )
    print(
        json.dumps(
            {
                "output_attempts_csv": str(output_attempts_csv),
                "output_scored_candidates_csv": str(output_scored_candidates_csv),
                "attempt_count": len(scored_attempts),
                "candidate_count": len(scored_candidates),
                "oracle_recoverable_attempt_count": recoverable_count,
            }
        )
    )


if __name__ == "__main__":
    main()
