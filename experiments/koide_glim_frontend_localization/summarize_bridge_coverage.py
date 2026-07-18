#!/usr/bin/env python3
"""Separate scan-match coverage from odom-bridge pose-output coverage.

``trajectory_eval.json`` reports how many emitted poses matched reference
timestamps.  Once timer-driven ``publish_bridge_pose_when_lost`` is enabled,
pose timestamps are intentionally independent of scan timestamps.  This
evaluator therefore reports NDT acceptance from alignment rows, output coverage
from the temporal support of the pose stream, and the largest gap between any
two emitted poses.  Exact scan/pose joins remain as diagnostic counts only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _load_alignment_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for record in csv.DictReader(stream):
            stamp = _finite_float(record.get("stamp_sec"))
            if stamp is None:
                continue
            try:
                values = json.loads(record.get("values_json") or "{}")
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"invalid values_json at alignment row {len(rows)}: {exc}"
                ) from exc
            recovery_action = str(values.get("recovery_action", ""))
            accepted = recovery_action == "accept_measurement"
            rows.append(
                {
                    "stamp_sec": stamp,
                    "accepted": accepted,
                    "message": str(record.get("message", "")),
                    "seed_source": str(
                        values.get("registration_seed_source", "unknown")
                    ),
                }
            )
    rows.sort(key=lambda row: row["stamp_sec"])
    return rows


def _load_pose_stamps(path: Path) -> List[float]:
    stamps: List[float] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for record in csv.DictReader(stream):
            stamp = _finite_float(record.get("stamp_sec"))
            # With simulated time, lifecycle/configured initial-pose output can
            # legitimately be recorded before the first /clock sample as stamp
            # zero. It is not part of the bag timeline and must not create an
            # artificial multi-decade output gap.
            if stamp is not None and stamp > 0.0:
                stamps.append(stamp)
    return sorted(stamps)


def _match_stamps(
    alignment_rows: Sequence[Dict[str, Any]],
    pose_stamps: Sequence[float],
    tolerance_sec: float,
) -> Tuple[List[Optional[float]], int]:
    """Greedily match ordered scan stamps to distinct ordered pose stamps."""
    if tolerance_sec < 0.0:
        raise ValueError("tolerance_sec must be non-negative")

    matches: List[Optional[float]] = []
    pose_index = 0
    unmatched_pose_rows = 0
    for row in alignment_rows:
        scan_stamp = float(row["stamp_sec"])
        while (
            pose_index < len(pose_stamps)
            and pose_stamps[pose_index] < scan_stamp - tolerance_sec
        ):
            unmatched_pose_rows += 1
            pose_index += 1

        best_index: Optional[int] = None
        best_error = math.inf
        candidate_index = pose_index
        while (
            candidate_index < len(pose_stamps)
            and pose_stamps[candidate_index] <= scan_stamp + tolerance_sec
        ):
            error = abs(pose_stamps[candidate_index] - scan_stamp)
            if error < best_error:
                best_index = candidate_index
                best_error = error
            candidate_index += 1

        if best_index is None:
            matches.append(None)
            continue

        unmatched_pose_rows += best_index - pose_index
        matches.append(pose_stamps[best_index])
        pose_index = best_index + 1

    unmatched_pose_rows += len(pose_stamps) - pose_index
    return matches, unmatched_pose_rows


def summarize(
    alignment_csv: Path,
    pose_csv: Path,
    tolerance_sec: float = 0.02,
) -> Dict[str, Any]:
    alignment_rows = _load_alignment_rows(alignment_csv)
    pose_stamps = _load_pose_stamps(pose_csv)
    matches, unmatched_pose_rows = _match_stamps(
        alignment_rows, pose_stamps, tolerance_sec
    )

    total = len(alignment_rows)
    accepted = sum(1 for row in alignment_rows if row["accepted"])
    output = sum(stamp is not None for stamp in matches)
    accepted_output = sum(
        row["accepted"] and stamp is not None
        for row, stamp in zip(alignment_rows, matches)
    )
    bridge_output = sum(
        not row["accepted"] and stamp is not None
        for row, stamp in zip(alignment_rows, matches)
    )
    if pose_stamps:
        first_pose_stamp = pose_stamps[0]
        last_pose_stamp = pose_stamps[-1]
        temporally_covered = sum(
            first_pose_stamp - tolerance_sec <= float(row["stamp_sec"])
            <= last_pose_stamp + tolerance_sec
            for row in alignment_rows
        )
    else:
        temporally_covered = 0
    missing_output = total - temporally_covered
    gaps = [
        later - earlier
        for earlier, later in zip(pose_stamps, pose_stamps[1:])
    ]

    seed_counts = Counter(row["seed_source"] for row in alignment_rows)
    accepted_seed_counts = Counter(
        row["seed_source"] for row in alignment_rows if row["accepted"]
    )
    rejected_seed_counts = Counter(
        row["seed_source"] for row in alignment_rows if not row["accepted"]
    )

    def percent(numerator: int) -> Optional[float]:
        return None if total == 0 else 100.0 * numerator / total

    return {
        "alignment_csv": str(alignment_csv),
        "pose_csv": str(pose_csv),
        "stamp_tolerance_sec": tolerance_sec,
        "alignment_scan_count": total,
        "accepted_measurement_count": accepted,
        "rejected_measurement_count": total - accepted,
        "scan_output_count": output,
        "temporally_covered_scan_count": temporally_covered,
        "accepted_output_count": accepted_output,
        "bridge_output_count": bridge_output,
        "missing_output_count": missing_output,
        "unmatched_pose_count": unmatched_pose_rows,
        "output_coverage_percent": percent(temporally_covered),
        "matched_coverage_percent": percent(accepted),
        "max_output_gap_sec": max(gaps) if gaps else 0.0,
        "seed_source_counts": dict(sorted(seed_counts.items())),
        "accepted_seed_source_counts": dict(sorted(accepted_seed_counts.items())),
        "rejected_seed_source_counts": dict(sorted(rejected_seed_counts.items())),
    }


def _gate(
    summary: Dict[str, Any],
    min_output_coverage: Optional[float],
    min_matched_coverage: Optional[float],
    max_output_gap: Optional[float],
) -> List[str]:
    failures: List[str] = []
    output_coverage = summary["output_coverage_percent"]
    matched_coverage = summary["matched_coverage_percent"]
    if min_output_coverage is not None and (
        output_coverage is None or output_coverage < min_output_coverage
    ):
        failures.append(
            f"output_coverage_percent={output_coverage} < {min_output_coverage}"
        )
    if min_matched_coverage is not None and (
        matched_coverage is None or matched_coverage < min_matched_coverage
    ):
        failures.append(
            f"matched_coverage_percent={matched_coverage} < {min_matched_coverage}"
        )
    if max_output_gap is not None and summary["max_output_gap_sec"] > max_output_gap:
        failures.append(
            f"max_output_gap_sec={summary['max_output_gap_sec']} > {max_output_gap}"
        )
    return failures


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alignment-csv", required=True)
    parser.add_argument("--pose-csv", required=True)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--stamp-tolerance-sec", type=float, default=0.02)
    parser.add_argument("--min-output-coverage", type=float)
    parser.add_argument("--min-matched-coverage", type=float)
    parser.add_argument("--max-output-gap-sec", type=float)
    args = parser.parse_args(argv)

    summary = summarize(
        Path(args.alignment_csv),
        Path(args.pose_csv),
        args.stamp_tolerance_sec,
    )
    failures = _gate(
        summary,
        args.min_output_coverage,
        args.min_matched_coverage,
        args.max_output_gap_sec,
    )
    summary["gate_ok"] = not failures
    summary["gate_failures"] = failures
    text = json.dumps(summary, indent=2, sort_keys=True)
    print(text)
    if args.output_json:
        Path(args.output_json).write_text(text + "\n", encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
