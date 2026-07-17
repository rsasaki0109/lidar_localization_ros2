#!/usr/bin/env python3
"""Build strict runtime/safety evidence from an odometry pose trace.

The recorder clock must use bag simulated time when replaying a bag. Processing
latency is then receive_stamp_sec - source stamp_sec. Queue growth is flagged
when both the latency trend is positive and the final quartile has grown by more
than one scan period relative to the first quartile.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


def _percentile(values: Sequence[float], ratio: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(ratio * len(ordered)) - 1))
    return ordered[index]


def _distance(lhs: Tuple[float, float, float], rhs: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))


def _quat_angle_deg(lhs: Tuple[float, ...], rhs: Tuple[float, ...]) -> float:
    lhs_norm = math.sqrt(sum(value * value for value in lhs))
    rhs_norm = math.sqrt(sum(value * value for value in rhs))
    if lhs_norm <= 1e-12 or rhs_norm <= 1e-12:
        return math.inf
    dot = abs(sum(a * b for a, b in zip(lhs, rhs)) / (lhs_norm * rhs_norm))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _linear_slope(xs: Sequence[float], ys: Sequence[float]) -> float:
    if len(xs) < 2 or len(xs) != len(ys):
        return 0.0
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    if denominator <= 1e-12:
        return 0.0
    return sum(
        (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denominator


def build_evidence(
    trace_csv: Path,
    max_tf_jump_m: float = 5.0,
    max_tf_jump_rotation_deg: float = 45.0,
) -> Dict[str, object]:
    stamps: List[float] = []
    receive_stamps: List[float] = []
    positions: List[Tuple[float, float, float]] = []
    quaternions: List[Tuple[float, float, float, float]] = []
    invalid_rows = 0
    with trace_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                values = [
                    float(row[name]) for name in (
                        "stamp_sec", "receive_stamp_sec",
                        "position_x", "position_y", "position_z",
                        "orientation_x", "orientation_y", "orientation_z",
                        "orientation_w",
                    )
                ]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("non-finite")
            except (KeyError, TypeError, ValueError):
                invalid_rows += 1
                continue
            stamps.append(values[0])
            receive_stamps.append(values[1])
            positions.append((values[2], values[3], values[4]))
            quaternions.append((values[5], values[6], values[7], values[8]))

    positive_intervals = [
        current - previous for previous, current in zip(stamps, stamps[1:])
        if current > previous]
    scan_period = statistics.median(positive_intervals) if positive_intervals else None
    latencies = [received - source for source, received in zip(stamps, receive_stamps)]
    valid_latency_pairs = [
        (stamp, latency) for stamp, latency in zip(stamps, latencies)
        if latency >= 0.0 and math.isfinite(latency)]
    valid_latency_stamps = [pair[0] for pair in valid_latency_pairs]
    valid_latencies = [pair[1] for pair in valid_latency_pairs]

    quartile_count = max(1, len(valid_latencies) // 4)
    first_latency_median = (
        statistics.median(valid_latencies[:quartile_count]) if valid_latencies else None)
    last_latency_median = (
        statistics.median(valid_latencies[-quartile_count:]) if valid_latencies else None)
    latency_slope = _linear_slope(valid_latency_stamps, valid_latencies)
    queue_growth_unbounded = True
    if (
        scan_period is not None
        and first_latency_median is not None
        and last_latency_median is not None
    ):
        queue_growth_unbounded = (
            latency_slope > 0.0
            and last_latency_median - first_latency_median > scan_period)

    translation_jumps = [
        _distance(previous, current)
        for previous, current in zip(positions, positions[1:])]
    rotation_jumps = [
        _quat_angle_deg(previous, current)
        for previous, current in zip(quaternions, quaternions[1:])]
    tf_jump_count = sum(
        translation > max_tf_jump_m or rotation > max_tf_jump_rotation_deg
        for translation, rotation in zip(translation_jumps, rotation_jumps))

    unauthorized_reset_count = 0
    if positions:
        origin = positions[0]
        for previous, current in zip(positions, positions[1:]):
            if (
                _distance(previous, origin) > max_tf_jump_m
                and _distance(current, origin) <= 0.5
                and _distance(previous, current) > max_tf_jump_m
            ):
                unauthorized_reset_count += 1

    return {
        "processing_p95_sec": _percentile(valid_latencies, 0.95),
        "scan_period_sec": scan_period,
        "queue_growth_unbounded": queue_growth_unbounded,
        "tf_jump_count": tf_jump_count,
        "unauthorized_reset_count": unauthorized_reset_count,
        "evidence": {
            "trace_csv": str(trace_csv.resolve()),
            "valid_sample_count": len(stamps),
            "invalid_row_count": invalid_rows,
            "valid_latency_sample_count": len(valid_latencies),
            "negative_latency_count": len(latencies) - len(valid_latencies),
            "processing_latency_first_quartile_median_sec": first_latency_median,
            "processing_latency_last_quartile_median_sec": last_latency_median,
            "processing_latency_slope_sec_per_sec": latency_slope,
            "max_translation_jump_m": max(translation_jumps) if translation_jumps else None,
            "max_rotation_jump_deg": max(rotation_jumps) if rotation_jumps else None,
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--max-tf-jump-m", type=float, default=5.0)
    parser.add_argument("--max-tf-jump-rotation-deg", type=float, default=45.0)
    args = parser.parse_args(argv)
    result = build_evidence(
        Path(args.trace_csv), args.max_tf_jump_m, args.max_tf_jump_rotation_deg)
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(result, indent=2, sort_keys=True)
    output.write_text(text + "\n", encoding="utf-8")
    print(text)
    complete = (
        result["processing_p95_sec"] is not None
        and result["scan_period_sec"] is not None
        and result["evidence"]["invalid_row_count"] == 0  # type: ignore[index]
    )
    return 0 if complete else 1


if __name__ == "__main__":
    sys.exit(main())
