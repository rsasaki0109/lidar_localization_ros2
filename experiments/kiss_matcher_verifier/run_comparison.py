#!/usr/bin/env python3
"""Compare BBS, bounded NDT, and KISS-Matcher on one top-K job artifact.

KISS-Matcher is optional research tooling. Install it in an isolated environment
with ``python -m pip install kiss-matcher``; it is deliberately not a runtime ROS
dependency. The script never publishes a reset pose.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any

import numpy as np

from experiments.point_cloud_io import read_xyz_pcd
from experiments.point_cloud_io import read_xyz_ply
from experiments.kiss_matcher_verifier.interface import Candidate
from experiments.kiss_matcher_verifier.interface import GlobalEstimate
from experiments.kiss_matcher_verifier.interface import angle_error_rad
from experiments.kiss_matcher_verifier.variants import BbsFirstSelector
from experiments.kiss_matcher_verifier.variants import KissNearestSelector


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes"}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _nearest_reference(reference: list[dict[str, str]], stamp: float) -> dict[str, str]:
    return min(reference, key=lambda row: abs(float(row["stamp_sec"]) - stamp))


def _reference_yaw(row: dict[str, str]) -> float:
    x = float(row["orientation_x"])
    y = float(row["orientation_y"])
    z = float(row["orientation_z"])
    w = float(row["orientation_w"])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _estimate_medoid(estimates: list[GlobalEstimate]) -> GlobalEstimate:
    def distance(lhs: GlobalEstimate, rhs: GlobalEstimate) -> float:
        return math.hypot(lhs.x - rhs.x, lhs.y - rhs.y) + angle_error_rad(
            lhs.yaw_rad, rhs.yaw_rad
        )

    return min(estimates, key=lambda item: sum(distance(item, other) for other in estimates))


def _repeat_spread(estimates: list[GlobalEstimate]) -> tuple[float, float]:
    xy_spread = 0.0
    yaw_spread = 0.0
    for index, lhs in enumerate(estimates):
        for rhs in estimates[index + 1 :]:
            xy_spread = max(xy_spread, math.hypot(lhs.x - rhs.x, lhs.y - rhs.y))
            yaw_spread = max(yaw_spread, angle_error_rad(lhs.yaw_rad, rhs.yaw_rad))
    return xy_spread, yaw_spread


def _summarize_variant(rows: list[dict[str, Any]]) -> dict[str, Any]:
    accepted = [row for row in rows if row["accepted"]]
    correct = [row for row in accepted if row["selected_oracle_recoverable"]]
    return {
        "attempt_count": len(rows),
        "accepted_count": len(accepted),
        "correct_accept_count": len(correct),
        "false_accept_count": len(accepted) - len(correct),
        "recovery_rate": len(correct) / len(rows) if rows else 0.0,
        "precision": len(correct) / len(accepted) if accepted else None,
        "median_runtime_sec": _median([float(row["runtime_sec"]) for row in rows]),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-jobs-csv", required=True)
    parser.add_argument("--ndt-scores-csv", required=True)
    parser.add_argument("--candidates-csv", required=True)
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--min-final-inliers", type=int, default=5)
    parser.add_argument("--max-candidate-delta-m", type=float, default=2.0)
    parser.add_argument("--max-candidate-yaw-delta-deg", type=float, default=15.0)
    parser.add_argument("--max-repeat-spread-m", type=float, default=2.0)
    parser.add_argument("--max-repeat-yaw-spread-deg", type=float, default=15.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        import kiss_matcher
    except ImportError as error:
        raise SystemExit(
            "kiss-matcher is required only for this experiment; install it in a venv"
        ) from error

    jobs = _read_csv(Path(args.scan_jobs_csv))
    ndt_rows = _read_csv(Path(args.ndt_scores_csv))
    oracle_rows = _read_csv(Path(args.candidates_csv))
    oracle_by_candidate = {
        (row["attempt_id"], int(row["candidate_index"])): row for row in oracle_rows
    }
    reference = _read_csv(Path(args.reference_csv))
    by_attempt: dict[str, list[dict[str, str]]] = {}
    for row in jobs:
        by_attempt.setdefault(row["attempt_id"], []).append(row)

    map_paths = {row["map_path"] for row in jobs}
    if len(map_paths) != 1:
        raise RuntimeError("comparison requires exactly one map")
    target = read_xyz_ply(Path(next(iter(map_paths))))
    target = target[np.isfinite(target).all(axis=1)]
    bbs_selector = BbsFirstSelector()
    kiss_selector = KissNearestSelector(
        min_final_inliers=args.min_final_inliers,
        max_translation_delta_m=args.max_candidate_delta_m,
        max_yaw_delta_rad=math.radians(args.max_candidate_yaw_delta_deg),
    )
    output_rows: dict[str, list[dict[str, Any]]] = {
        "bbs_first": [],
        "ndt_bounded": [],
        "kiss_nearest": [],
    }
    kiss_estimates: list[dict[str, Any]] = []

    for attempt_id, attempt_rows in sorted(by_attempt.items()):
        def is_recoverable(candidate_index: int) -> bool:
            oracle = oracle_by_candidate[(attempt_id, candidate_index)]
            return (
                float(oracle["oracle_translation_error_m"]) <= 2.0
                and float(oracle["oracle_yaw_error_deg"]) <= 15.0
            )

        candidates = [
            Candidate(
                attempt_id=attempt_id,
                candidate_index=int(row["candidate_index"]),
                x=float(row["initial_pose_x"]),
                y=float(row["initial_pose_y"]),
                yaw_rad=float(row["initial_yaw_rad"]),
                oracle_recoverable=is_recoverable(int(row["candidate_index"])),
            )
            for row in attempt_rows
        ]
        bbs = bbs_selector.select(candidates)
        output_rows["bbs_first"].append(
            {
                "attempt_id": attempt_id,
                "accepted": bbs.accepted,
                "selected_candidate_index": bbs.candidate.candidate_index if bbs.candidate else None,
                "selected_oracle_recoverable": bool(
                    bbs.candidate and bbs.candidate.oracle_recoverable
                ),
                "reason": bbs.reason,
                "runtime_sec": 0.0,
            }
        )

        attempt_ndt = [row for row in ndt_rows if row["attempt_id"] == attempt_id]
        gated_ndt = [row for row in attempt_ndt if _as_bool(row["registration_gate_passed"])]
        selected_ndt = min(gated_ndt, key=lambda row: float(row["score"])) if gated_ndt else None
        output_rows["ndt_bounded"].append(
            {
                "attempt_id": attempt_id,
                "accepted": selected_ndt is not None,
                "selected_candidate_index": (
                    int(selected_ndt["candidate_index"]) if selected_ndt else None
                ),
                "selected_oracle_recoverable": bool(
                    selected_ndt and is_recoverable(int(selected_ndt["candidate_index"]))
                ),
                "reason": "passed_reset_disabled" if selected_ndt else "all_candidates_rejected",
                "runtime_sec": sum(float(row["runtime_sec"]) for row in attempt_ndt),
            }
        )

        scan = read_xyz_pcd(Path(attempt_rows[0]["scan_pcd_path"]))
        scan = scan[np.isfinite(scan).all(axis=1)]
        estimates: list[GlobalEstimate] = []
        for repeat in range(args.repeats):
            config = kiss_matcher.KISSMatcherConfig(args.voxel_size)
            matcher = kiss_matcher.KISSMatcher(config)
            started = time.monotonic()
            result = matcher.estimate(scan, target)
            runtime_sec = time.monotonic() - started
            rotation = np.asarray(result.rotation)
            translation = np.asarray(result.translation).reshape(3)
            estimates.append(
                GlobalEstimate(
                    valid=bool(result.valid),
                    x=float(translation[0]),
                    y=float(translation[1]),
                    yaw_rad=_yaw_from_rotation(rotation),
                    final_inliers=int(matcher.get_num_final_inliers()),
                    runtime_sec=runtime_sec,
                )
            )
        medoid = _estimate_medoid(estimates)
        repeat_spread_m, repeat_yaw_spread_rad = _repeat_spread(estimates)
        repeat_consistent = (
            all(item.valid for item in estimates)
            and min(item.final_inliers for item in estimates) >= args.min_final_inliers
            and repeat_spread_m <= args.max_repeat_spread_m
            and repeat_yaw_spread_rad <= math.radians(args.max_repeat_yaw_spread_deg)
        )
        estimate = GlobalEstimate(
            valid=medoid.valid and repeat_consistent,
            x=medoid.x,
            y=medoid.y,
            yaw_rad=medoid.yaw_rad,
            final_inliers=medoid.final_inliers,
            runtime_sec=statistics.median(item.runtime_sec for item in estimates),
        )
        selection = kiss_selector.select(candidates, estimate)
        target_reference = _nearest_reference(reference, float(attempt_rows[0]["trigger_stamp_sec"]))
        gt_x = float(target_reference["position_x"])
        gt_y = float(target_reference["position_y"])
        gt_yaw = _reference_yaw(target_reference)
        kiss_estimates.append(
            {
                "attempt_id": attempt_id,
                "valid": estimate.valid,
                "final_inliers": estimate.final_inliers,
                "estimated_x": estimate.x,
                "estimated_y": estimate.y,
                "estimated_yaw_rad": estimate.yaw_rad,
                "oracle_xy_error_m": math.hypot(estimate.x - gt_x, estimate.y - gt_y),
                "oracle_yaw_error_deg": math.degrees(angle_error_rad(estimate.yaw_rad, gt_yaw)),
                "repeat_runtime_sec": [item.runtime_sec for item in estimates],
                "repeat_estimated_xy": [[item.x, item.y] for item in estimates],
                "repeat_final_inliers": [item.final_inliers for item in estimates],
                "repeat_consistent": repeat_consistent,
                "repeat_xy_spread_m": repeat_spread_m,
                "repeat_yaw_spread_deg": math.degrees(repeat_yaw_spread_rad),
            }
        )
        output_rows["kiss_nearest"].append(
            {
                "attempt_id": attempt_id,
                "accepted": selection.accepted,
                "selected_candidate_index": (
                    selection.candidate.candidate_index if selection.candidate else None
                ),
                "selected_oracle_recoverable": bool(
                    selection.candidate and selection.candidate.oracle_recoverable
                ),
                "reason": selection.reason,
                "runtime_sec": statistics.median(item.runtime_sec for item in estimates),
                "candidate_translation_delta_m": selection.translation_delta_m,
                "candidate_yaw_delta_deg": math.degrees(selection.yaw_delta_rad),
            }
        )

    coverage_count = sum(
        any(candidate.oracle_recoverable for candidate in [
            Candidate(
                attempt_id=attempt_id,
                candidate_index=int(row["candidate_index"]),
                x=float(row["initial_pose_x"]),
                y=float(row["initial_pose_y"]),
                yaw_rad=float(row["initial_yaw_rad"]),
                oracle_recoverable=(
                    float(oracle_by_candidate[(attempt_id, int(row["candidate_index"]))][
                        "oracle_translation_error_m"
                    ])
                    <= 2.0
                    and float(oracle_by_candidate[(attempt_id, int(row["candidate_index"]))][
                        "oracle_yaw_error_deg"
                    ])
                    <= 15.0
                ),
            )
            for row in rows
        ])
        for attempt_id, rows in by_attempt.items()
    )
    summary = {
        "schema_version": 1,
        "reset_published": False,
        "attempt_count": len(by_attempt),
        "candidate_set_oracle_coverage_count": coverage_count,
        "candidate_set_oracle_coverage_rate": coverage_count / len(by_attempt),
        "config": vars(args),
        "variants": {name: _summarize_variant(rows) for name, rows in output_rows.items()},
        "attempts": output_rows,
        "kiss_global_estimates": kiss_estimates,
    }
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output_json": str(output), "variants": summary["variants"]}))


if __name__ == "__main__":
    main()
