#!/usr/bin/env python3
"""Check the Hard Point Cloud Localization full-sequence odometry goal.

The checker intentionally treats coverage, final arrival, continuity, accuracy,
runtime, and process safety as independent mandatory evidence.  A short, accurate
trajectory cannot pass the full-sequence gate.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Pose:
    stamp_sec: float
    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]


@dataclass(frozen=True)
class CompletionThresholds:
    requested_duration_sec: float = 380.0
    min_coverage_ratio: float = 0.99
    max_final_lag_sec: float = 1.0
    max_pose_gap_sec: float = 1.0
    max_translation_ate_rmse_m: float = 2.0
    max_translation_end_error_m: float = 5.0
    rpe_distance_m: float = 10.0
    max_rpe_translation_median_m: float = 0.20
    max_rpe_rotation_median_deg: float = 1.0
    max_pose_jump_m: float = 5.0
    max_pose_jump_rotation_deg: float = 45.0
    max_time_diff_sec: float = 0.05


@dataclass(frozen=True)
class LoadedTrajectory:
    poses: Tuple[Pose, ...]
    row_count: int
    non_finite_row_count: int
    timestamp_backwards_count: int
    duplicate_timestamp_count: int


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _normalize_quaternion(
    quaternion: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid zero/non-finite quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _quat_conjugate(
    quaternion: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quat_multiply(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quat_rotate(
    quaternion: Tuple[float, float, float, float],
    vector: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    q = _normalize_quaternion(quaternion)
    rotated = _quat_multiply(_quat_multiply(q, (*vector, 0.0)), _quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


def _quat_angle_deg(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> float:
    delta = _quat_multiply(
        _quat_conjugate(_normalize_quaternion(lhs)),
        _normalize_quaternion(rhs),
    )
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(delta[3])))))


def _distance(
    lhs: Tuple[float, float, float], rhs: Tuple[float, float, float]
) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))


def load_trajectory(path: Path) -> LoadedTrajectory:
    poses: List[Pose] = []
    row_count = 0
    non_finite = 0
    backwards = 0
    duplicates = 0
    previous_stamp: Optional[float] = None
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            row_count += 1
            try:
                stamp = float(row["stamp_sec"])
                position = (
                    float(row["position_x"]),
                    float(row["position_y"]),
                    float(row["position_z"]),
                )
                quaternion = (
                    float(row["orientation_x"]),
                    float(row["orientation_y"]),
                    float(row["orientation_z"]),
                    float(row["orientation_w"]),
                )
                if not _finite((stamp, *position, *quaternion)):
                    raise ValueError("non-finite pose")
                quaternion = _normalize_quaternion(quaternion)
            except (KeyError, TypeError, ValueError):
                non_finite += 1
                continue
            if previous_stamp is not None:
                if stamp < previous_stamp:
                    backwards += 1
                elif stamp == previous_stamp:
                    duplicates += 1
            previous_stamp = stamp
            poses.append(Pose(stamp, position, quaternion))
    return LoadedTrajectory(
        poses=tuple(poses),
        row_count=row_count,
        non_finite_row_count=non_finite,
        timestamp_backwards_count=backwards,
        duplicate_timestamp_count=duplicates,
    )


def _align_estimates_to_reference_start(
    estimates: Sequence[Pose], reference_start: Pose
) -> List[Pose]:
    if not estimates:
        return []
    estimate_start = estimates[0]
    rotation = _quat_multiply(
        reference_start.quaternion, _quat_conjugate(estimate_start.quaternion))
    aligned: List[Pose] = []
    for pose in estimates:
        delta = tuple(
            value - origin
            for value, origin in zip(pose.position, estimate_start.position)
        )
        rotated = _quat_rotate(rotation, delta)  # type: ignore[arg-type]
        position = tuple(
            origin + value
            for origin, value in zip(reference_start.position, rotated)
        )
        quaternion = _normalize_quaternion(_quat_multiply(rotation, pose.quaternion))
        aligned.append(Pose(pose.stamp_sec, position, quaternion))  # type: ignore[arg-type]
    return aligned


def _nearest_pose(
    poses: Sequence[Pose], stamps: Sequence[float], stamp: float
) -> Optional[Pose]:
    if not poses:
        return None
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    return min(candidates, key=lambda pose: abs(pose.stamp_sec - stamp))


def _relative_pose(origin: Pose, target: Pose) -> Tuple[
    Tuple[float, float, float], Tuple[float, float, float, float]
]:
    inverse_rotation = _quat_conjugate(origin.quaternion)
    delta = tuple(a - b for a, b in zip(target.position, origin.position))
    translation = _quat_rotate(inverse_rotation, delta)  # type: ignore[arg-type]
    rotation = _normalize_quaternion(
        _quat_multiply(inverse_rotation, target.quaternion))
    return translation, rotation


def _median(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _trajectory_metrics(
    estimated: LoadedTrajectory,
    reference: LoadedTrajectory,
    thresholds: CompletionThresholds,
) -> Dict[str, Any]:
    estimates = list(estimated.poses)
    references = sorted(reference.poses, key=lambda pose: pose.stamp_sec)
    if not estimates or not references:
        return {"matched_sample_count": 0}
    reference_stamps = [pose.stamp_sec for pose in references]
    estimates_by_time = sorted(estimates, key=lambda pose: pose.stamp_sec)
    reference_at_estimate_start = _nearest_pose(
        references, reference_stamps, estimates_by_time[0].stamp_sec)
    if reference_at_estimate_start is None:
        return {"matched_sample_count": 0}
    aligned = _align_estimates_to_reference_start(
        estimates, reference_at_estimate_start)
    matched: List[Tuple[Pose, Pose]] = []
    for pose in aligned:
        ref = _nearest_pose(references, reference_stamps, pose.stamp_sec)
        if ref is not None and abs(ref.stamp_sec - pose.stamp_sec) <= thresholds.max_time_diff_sec:
            matched.append((pose, ref))

    translation_errors = [
        _distance(estimate.position, ref.position) for estimate, ref in matched]
    rotation_errors = [
        _quat_angle_deg(estimate.quaternion, ref.quaternion)
        for estimate, ref in matched]

    sorted_estimates = sorted(aligned, key=lambda pose: pose.stamp_sec)
    gaps = [
        current.stamp_sec - previous.stamp_sec
        for previous, current in zip(sorted_estimates, sorted_estimates[1:])]
    translation_jumps = [
        _distance(previous.position, current.position)
        for previous, current in zip(sorted_estimates, sorted_estimates[1:])]
    rotation_jumps = [
        _quat_angle_deg(previous.quaternion, current.quaternion)
        for previous, current in zip(sorted_estimates, sorted_estimates[1:])]

    expected_start = references[0].stamp_sec
    expected_end = expected_start + thresholds.requested_duration_sec
    overlap_start = max(expected_start, sorted_estimates[0].stamp_sec)
    overlap_end = min(expected_end, sorted_estimates[-1].stamp_sec)
    covered_duration = max(0.0, overlap_end - overlap_start)
    coverage_ratio = covered_duration / thresholds.requested_duration_sec
    final_lag = max(0.0, expected_end - sorted_estimates[-1].stamp_sec)

    cumulative_distance = [0.0]
    for (_, previous), (_, current) in zip(matched, matched[1:]):
        cumulative_distance.append(
            cumulative_distance[-1] + _distance(previous.position, current.position))
    rpe_translation: List[float] = []
    rpe_rotation: List[float] = []
    for start_index in range(len(matched)):
        target_distance = cumulative_distance[start_index] + thresholds.rpe_distance_m
        end_index = bisect.bisect_left(cumulative_distance, target_distance, start_index + 1)
        if end_index >= len(matched):
            continue
        estimate_start, reference_start = matched[start_index]
        estimate_end, reference_end = matched[end_index]
        est_translation, est_rotation = _relative_pose(estimate_start, estimate_end)
        ref_translation, ref_rotation = _relative_pose(reference_start, reference_end)
        rpe_translation.append(_distance(est_translation, ref_translation))
        rpe_rotation.append(_quat_angle_deg(est_rotation, ref_rotation))

    return {
        "estimated_row_count": estimated.row_count,
        "estimated_valid_sample_count": len(estimates),
        "reference_valid_sample_count": len(references),
        "matched_sample_count": len(matched),
        "non_finite_row_count": estimated.non_finite_row_count,
        "timestamp_backwards_count": estimated.timestamp_backwards_count,
        "duplicate_timestamp_count": estimated.duplicate_timestamp_count,
        "first_stamp_sec": sorted_estimates[0].stamp_sec,
        "last_stamp_sec": sorted_estimates[-1].stamp_sec,
        "covered_duration_sec": covered_duration,
        "coverage_ratio": coverage_ratio,
        "final_lag_sec": final_lag,
        "max_pose_gap_sec": max(gaps) if gaps else None,
        "max_pose_jump_m": max(translation_jumps) if translation_jumps else None,
        "max_pose_jump_rotation_deg": max(rotation_jumps) if rotation_jumps else None,
        "translation_ate_rmse_m": (
            math.sqrt(sum(error * error for error in translation_errors) / len(translation_errors))
            if translation_errors else None),
        "translation_end_error_m": translation_errors[-1] if translation_errors else None,
        "rotation_ate_rmse_deg": (
            math.sqrt(sum(error * error for error in rotation_errors) / len(rotation_errors))
            if rotation_errors else None),
        "rpe_distance_m": thresholds.rpe_distance_m,
        "rpe_sample_count": len(rpe_translation),
        "rpe_translation_median_m": _median(rpe_translation),
        "rpe_rotation_median_deg": _median(rpe_rotation),
    }


def _read_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def evaluate_completion(
    estimated_csv: Path,
    reference_csv: Path,
    runtime_json: Path,
    run_summary_json: Path,
    thresholds: CompletionThresholds,
) -> Dict[str, Any]:
    estimated = load_trajectory(estimated_csv)
    reference = load_trajectory(reference_csv)
    metrics = _trajectory_metrics(estimated, reference, thresholds)
    runtime = _read_json(runtime_json)
    run_summary = _read_json(run_summary_json)

    processing_p95 = runtime.get("processing_p95_sec")
    scan_period = runtime.get("scan_period_sec")
    gates = {
        "trajectory_samples_present": metrics.get("matched_sample_count", 0) > 0,
        "coverage": metrics.get("coverage_ratio", 0.0) >= thresholds.min_coverage_ratio,
        "final_arrival": metrics.get("final_lag_sec", math.inf) <= thresholds.max_final_lag_sec,
        "pose_gap": (
            metrics.get("max_pose_gap_sec") is not None
            and metrics["max_pose_gap_sec"] <= thresholds.max_pose_gap_sec),
        "finite_poses": metrics.get("non_finite_row_count", 1) == 0,
        "monotonic_timestamps": metrics.get("timestamp_backwards_count", 1) == 0,
        "translation_ate": (
            metrics.get("translation_ate_rmse_m") is not None
            and metrics["translation_ate_rmse_m"] <= thresholds.max_translation_ate_rmse_m),
        "translation_end": (
            metrics.get("translation_end_error_m") is not None
            and metrics["translation_end_error_m"] <= thresholds.max_translation_end_error_m),
        "rpe_samples_present": metrics.get("rpe_sample_count", 0) > 0,
        "rpe_translation": (
            metrics.get("rpe_translation_median_m") is not None
            and metrics["rpe_translation_median_m"] <=
            thresholds.max_rpe_translation_median_m),
        "rpe_rotation": (
            metrics.get("rpe_rotation_median_deg") is not None
            and metrics["rpe_rotation_median_deg"] <=
            thresholds.max_rpe_rotation_median_deg),
        "bounded_pose_jump": (
            metrics.get("max_pose_jump_m") is not None
            and metrics["max_pose_jump_m"] <= thresholds.max_pose_jump_m
            and metrics.get("max_pose_jump_rotation_deg") is not None
            and metrics["max_pose_jump_rotation_deg"] <=
            thresholds.max_pose_jump_rotation_deg),
        "runtime_deadline": (
            isinstance(processing_p95, (int, float))
            and isinstance(scan_period, (int, float))
            and math.isfinite(float(processing_p95))
            and math.isfinite(float(scan_period))
            and 0.0 <= float(processing_p95) <= float(scan_period)),
        "bounded_queue": runtime.get("queue_growth_unbounded") is False,
        "no_tf_jump": runtime.get("tf_jump_count") == 0,
        "no_unauthorized_reset": runtime.get("unauthorized_reset_count") == 0,
        "process_survived": run_summary.get("target_process_died_during_run") is False,
        "bag_completed": (
            run_summary.get("return_codes", {}).get("bag_play") == 0
            or run_summary.get("bag_stopped_by_runner") is True),
    }
    failed = [name for name, passed in gates.items() if not passed]
    sequence_id = reference_csv.parent.name
    requested_duration = f"{thresholds.requested_duration_sec:g}"
    return {
        "ok": not failed,
        "goal": (
            f"Koide {sequence_id} full {requested_duration} s "
            "LiDAR/IMU odometry completion"),
        "estimated_csv": str(estimated_csv.resolve()),
        "reference_csv": str(reference_csv.resolve()),
        "runtime_json": str(runtime_json.resolve()),
        "run_summary_json": str(run_summary_json.resolve()),
        "thresholds": asdict(thresholds),
        "metrics": metrics,
        "runtime": runtime,
        "gates": gates,
        "failed_gates": failed,
    }


def _positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimated-csv", required=True)
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--runtime-json", required=True)
    parser.add_argument("--run-summary-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--requested-duration-sec", type=_positive, default=380.0)
    parser.add_argument("--min-coverage-ratio", type=float, default=0.99)
    parser.add_argument("--max-final-lag-sec", type=float, default=1.0)
    parser.add_argument("--max-pose-gap-sec", type=float, default=1.0)
    parser.add_argument("--max-translation-ate-rmse-m", type=float, default=2.0)
    parser.add_argument("--max-translation-end-error-m", type=float, default=5.0)
    parser.add_argument("--rpe-distance-m", type=_positive, default=10.0)
    parser.add_argument("--max-rpe-translation-median-m", type=float, default=0.20)
    parser.add_argument("--max-rpe-rotation-median-deg", type=float, default=1.0)
    parser.add_argument("--max-pose-jump-m", type=float, default=5.0)
    parser.add_argument("--max-pose-jump-rotation-deg", type=float, default=45.0)
    parser.add_argument("--max-time-diff-sec", type=float, default=0.05)
    args = parser.parse_args(argv)
    thresholds = CompletionThresholds(
        requested_duration_sec=args.requested_duration_sec,
        min_coverage_ratio=args.min_coverage_ratio,
        max_final_lag_sec=args.max_final_lag_sec,
        max_pose_gap_sec=args.max_pose_gap_sec,
        max_translation_ate_rmse_m=args.max_translation_ate_rmse_m,
        max_translation_end_error_m=args.max_translation_end_error_m,
        rpe_distance_m=args.rpe_distance_m,
        max_rpe_translation_median_m=args.max_rpe_translation_median_m,
        max_rpe_rotation_median_deg=args.max_rpe_rotation_median_deg,
        max_pose_jump_m=args.max_pose_jump_m,
        max_pose_jump_rotation_deg=args.max_pose_jump_rotation_deg,
        max_time_diff_sec=args.max_time_diff_sec,
    )
    result = evaluate_completion(
        Path(args.estimated_csv),
        Path(args.reference_csv),
        Path(args.runtime_json),
        Path(args.run_summary_json),
        thresholds,
    )
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(result, indent=2, sort_keys=True)
    output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
