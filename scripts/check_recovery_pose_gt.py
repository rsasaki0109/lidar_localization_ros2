#!/usr/bin/env python3
"""Ground-truth recovery verdict for Koide G3 replay pose traces.

Fitness-only recovery confirmation is insufficient on repetitive outdoor maps:
a low NDT fitness score can pass while the localizer locks to an along-route
alias tens of meters from the true pose (see docs/g3_live_closed_loop.md,
Koide 180 s boundary characterization). This checker compares recorded
/pcl_pose samples against TUM-format ground truth on the shared bag clock.

Supervisor event timestamps are wall-clock and cannot be aligned to bag-epoch
pose_trace.csv or GT without fragile mapping; pose trace and GT already share
the same stamp domain, so no clock translation is required.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class GroundTruthSample:
    stamp_sec: float
    x: float
    y: float


@dataclass(frozen=True)
class PoseSample:
    stamp_sec: float
    x: float
    y: float


@dataclass(frozen=True)
class PoseErrorSample:
    stamp_sec: float
    error_m: Optional[float]


@dataclass(frozen=True)
class RecoveredWindow:
    start_sec: float
    end_sec: float

    @property
    def duration_sec(self) -> float:
        return self.end_sec - self.start_sec


@dataclass(frozen=True)
class RecoveryPoseGtSummary:
    verdict: str
    first_loss_sec: Optional[float]
    first_loss_sec_bag_relative: Optional[float]
    qualifying_window_count: int
    longest_recovered_window_sec: float
    last_sample_error_m: Optional[float]
    max_error_after_loss_m: Optional[float]
    error_buckets: Dict[str, int]

    @property
    def ok(self) -> bool:
        return self.verdict in {"never_lost", "recovered_true"}


def _as_float(value: Optional[str]) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not (number == number):
        return None
    return number


def load_pose_trace(path: Path, min_stamp_sec: float = 1e9) -> List[PoseSample]:
    samples: List[PoseSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            stamp = _as_float(row.get("stamp_sec"))
            x = _as_float(row.get("position_x"))
            y = _as_float(row.get("position_y"))
            if stamp is None or x is None or y is None:
                continue
            if stamp < min_stamp_sec:
                continue
            samples.append(PoseSample(stamp_sec=stamp, x=x, y=y))
    samples.sort(key=lambda sample: sample.stamp_sec)
    return samples


def load_ground_truth(path: Path) -> List[GroundTruthSample]:
    samples: List[GroundTruthSample] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            stamp = _as_float(parts[0])
            x = _as_float(parts[1])
            y = _as_float(parts[2])
            if stamp is None or x is None or y is None:
                continue
            samples.append(GroundTruthSample(stamp_sec=stamp, x=x, y=y))
    samples.sort(key=lambda sample: sample.stamp_sec)
    return samples


def _nearest_gt_index(
    gt_stamps: Sequence[float],
    stamp_sec: float,
    max_stamp_delta_sec: float,
) -> Optional[int]:
    if not gt_stamps:
        return None
    index = bisect.bisect_left(gt_stamps, stamp_sec)
    candidates = []
    if index < len(gt_stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best_index = None
    best_delta = None
    for candidate in candidates:
        delta = abs(gt_stamps[candidate] - stamp_sec)
        if best_delta is None or delta < best_delta:
            best_delta = delta
            best_index = candidate
    if best_index is None or best_delta is None or best_delta > max_stamp_delta_sec:
        return None
    return best_index


def xy_error_m(pose: PoseSample, gt: GroundTruthSample) -> float:
    dx = pose.x - gt.x
    dy = pose.y - gt.y
    return math.hypot(dx, dy)


def compute_pose_errors(
    pose_samples: Sequence[PoseSample],
    ground_truth: Sequence[GroundTruthSample],
    max_gt_stamp_delta_sec: float = 0.5,
) -> List[PoseErrorSample]:
    gt_stamps = [sample.stamp_sec for sample in ground_truth]
    errors: List[PoseErrorSample] = []
    for pose in pose_samples:
        index = _nearest_gt_index(gt_stamps, pose.stamp_sec, max_gt_stamp_delta_sec)
        if index is None:
            errors.append(PoseErrorSample(stamp_sec=pose.stamp_sec, error_m=None))
            continue
        errors.append(
            PoseErrorSample(
                stamp_sec=pose.stamp_sec,
                error_m=xy_error_m(pose, ground_truth[index]),
            )
        )
    return errors


def find_first_loss_sec(
    errors: Sequence[PoseErrorSample],
    loss_threshold_m: float,
) -> Optional[float]:
    for sample in errors:
        if sample.error_m is not None and sample.error_m > loss_threshold_m:
            return sample.stamp_sec
    return None


def find_qualifying_recovered_windows(
    errors: Sequence[PoseErrorSample],
    first_loss_sec: float,
    recovered_threshold_m: float,
    min_recovered_window_sec: float,
    max_sample_gap_sec: float,
) -> List[RecoveredWindow]:
    after_loss = [sample for sample in errors if sample.stamp_sec >= first_loss_sec]
    windows: List[RecoveredWindow] = []
    window_start: Optional[float] = None
    prev_stamp: Optional[float] = None

    def close_window(end_stamp: Optional[float]) -> None:
        nonlocal window_start
        if window_start is None or end_stamp is None:
            window_start = None
            return
        duration = end_stamp - window_start
        if duration >= min_recovered_window_sec:
            windows.append(RecoveredWindow(start_sec=window_start, end_sec=end_stamp))
        window_start = None

    for sample in after_loss:
        gap_ok = prev_stamp is None or (sample.stamp_sec - prev_stamp) <= max_sample_gap_sec
        recovered_ok = sample.error_m is not None and sample.error_m <= recovered_threshold_m
        if recovered_ok and gap_ok:
            if window_start is None:
                window_start = sample.stamp_sec
        else:
            close_window(prev_stamp)
        prev_stamp = sample.stamp_sec

    close_window(prev_stamp)
    return windows


def bucket_errors(errors: Sequence[PoseErrorSample]) -> Dict[str, int]:
    buckets = {
        "lt_1m": 0,
        "1_to_3m": 0,
        "3_to_5m": 0,
        "5_to_10m": 0,
        "gt_10m": 0,
        "no_gt_match": 0,
    }
    for sample in errors:
        if sample.error_m is None:
            buckets["no_gt_match"] += 1
            continue
        value = sample.error_m
        if value < 1.0:
            buckets["lt_1m"] += 1
        elif value < 3.0:
            buckets["1_to_3m"] += 1
        elif value < 5.0:
            buckets["3_to_5m"] += 1
        elif value < 10.0:
            buckets["5_to_10m"] += 1
        else:
            buckets["gt_10m"] += 1
    return buckets


def summarize_recovery_pose_gt(
    pose_samples: Sequence[PoseSample],
    ground_truth: Sequence[GroundTruthSample],
    loss_threshold_m: float = 5.0,
    recovered_threshold_m: float = 3.0,
    min_recovered_window_sec: float = 15.0,
    max_sample_gap_sec: float = 5.0,
    max_gt_stamp_delta_sec: float = 0.5,
) -> RecoveryPoseGtSummary:
    if not pose_samples:
        raise ValueError("pose trace has no valid samples")
    if not ground_truth:
        raise ValueError("ground truth has no valid samples")

    errors = compute_pose_errors(pose_samples, ground_truth, max_gt_stamp_delta_sec)
    first_pose_stamp = pose_samples[0].stamp_sec
    first_loss = find_first_loss_sec(errors, loss_threshold_m)
    last_error = next(
        (sample.error_m for sample in reversed(errors) if sample.error_m is not None),
        None,
    )

    if first_loss is None:
        return RecoveryPoseGtSummary(
            verdict="never_lost",
            first_loss_sec=None,
            first_loss_sec_bag_relative=None,
            qualifying_window_count=0,
            longest_recovered_window_sec=0.0,
            last_sample_error_m=last_error,
            max_error_after_loss_m=None,
            error_buckets=bucket_errors(errors),
        )

    windows = find_qualifying_recovered_windows(
        errors,
        first_loss,
        recovered_threshold_m,
        min_recovered_window_sec,
        max_sample_gap_sec,
    )
    after_loss_errors = [
        sample.error_m
        for sample in errors
        if sample.stamp_sec >= first_loss and sample.error_m is not None
    ]
    max_after_loss = max(after_loss_errors) if after_loss_errors else None
    longest_window = max((window.duration_sec for window in windows), default=0.0)

    return RecoveryPoseGtSummary(
        verdict="recovered_true" if windows else "recovered_false",
        first_loss_sec=first_loss,
        first_loss_sec_bag_relative=first_loss - first_pose_stamp,
        qualifying_window_count=len(windows),
        longest_recovered_window_sec=longest_window,
        last_sample_error_m=last_error,
        max_error_after_loss_m=max_after_loss,
        error_buckets=bucket_errors(errors),
    )


def summary_to_payload(summary: RecoveryPoseGtSummary) -> Dict[str, object]:
    return {
        "verdict": summary.verdict,
        "first_loss_sec": summary.first_loss_sec,
        "first_loss_sec_bag_relative": summary.first_loss_sec_bag_relative,
        "qualifying_window_count": summary.qualifying_window_count,
        "longest_recovered_window_sec": summary.longest_recovered_window_sec,
        "last_sample_error_m": summary.last_sample_error_m,
        "max_error_after_loss_m": summary.max_error_after_loss_m,
        "error_buckets": summary.error_buckets,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pose-trace",
        required=True,
        help="pose_trace.csv from benchmark_pose_recorder.",
    )
    parser.add_argument(
        "--gt",
        required=True,
        help="TUM-format ground truth (stamp x y z qx qy qz qw).",
    )
    parser.add_argument(
        "--output-json",
        default="",
        help="Optional path to write the JSON summary.",
    )
    parser.add_argument(
        "--loss-threshold-m",
        type=float,
        default=5.0,
        help="XY error above which localization is considered lost.",
    )
    parser.add_argument(
        "--recovered-threshold-m",
        type=float,
        default=3.0,
        help="XY error at or below which a sample counts as recovered.",
    )
    parser.add_argument(
        "--min-recovered-window-sec",
        type=float,
        default=15.0,
        help="Minimum contiguous recovered window duration.",
    )
    parser.add_argument(
        "--max-sample-gap-sec",
        type=float,
        default=5.0,
        help="Maximum gap between consecutive samples within a recovered window.",
    )
    args = parser.parse_args(argv)

    pose_path = Path(args.pose_trace)
    gt_path = Path(args.gt)
    if not pose_path.is_file():
        print(f"pose trace not found: {pose_path}", file=sys.stderr)
        return 2
    if not gt_path.is_file():
        print(f"ground truth not found: {gt_path}", file=sys.stderr)
        return 2

    try:
        pose_samples = load_pose_trace(pose_path)
        ground_truth = load_ground_truth(gt_path)
        summary = summarize_recovery_pose_gt(
            pose_samples,
            ground_truth,
            loss_threshold_m=args.loss_threshold_m,
            recovered_threshold_m=args.recovered_threshold_m,
            min_recovered_window_sec=args.min_recovered_window_sec,
            max_sample_gap_sec=args.max_sample_gap_sec,
        )
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    payload = summary_to_payload(summary)
    text = json.dumps(payload, indent=2, sort_keys=True)
    print(text)
    if args.output_json:
        Path(args.output_json).write_text(text + "\n", encoding="utf-8")
    return 0 if summary.ok else 1


if __name__ == "__main__":
    sys.exit(main())
