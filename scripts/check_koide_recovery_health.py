#!/usr/bin/env python3
"""Summarize Koide G3 recovery health from recorded supervisor/localizer artifacts.

This is a machine-independent rubric checker for the stable-recovery gate described
in docs/g3_live_closed_loop.md: a single recovery_confirmed is not enough; the run
must also show at least one stable recovered request window.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class RecoveryHealthSummary:
    recovery_confirmed_count: int
    stable_recovered_request_windows: int
    false_recovery_confirmed: bool
    reinitialization_requested_rows: int
    longest_lost_window_sec: float
    ok: bool
    reasons: Tuple[str, ...]


def _read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _as_float(value: Optional[str]) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not (number == number):  # NaN
        return None
    return number


def _as_bool(value: Optional[str]) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _count_events(rows: Sequence[Dict[str, str]], key: str, expected: str) -> int:
    return sum(1 for row in rows if row.get(key, "") == expected)


def _longest_true_window(rows: Sequence[Dict[str, str]], key: str) -> float:
    longest = 0.0
    current_start = None
    prev_time = None
    for row in rows:
        stamp = _as_float(row.get("stamp_sec") or row.get("time_sec"))
        if stamp is None:
            continue
        active = _as_bool(row.get(key))
        if active:
            if current_start is None:
                current_start = stamp
        elif current_start is not None and prev_time is not None:
            longest = max(longest, prev_time - current_start)
            current_start = None
        prev_time = stamp
    if current_start is not None and prev_time is not None:
        longest = max(longest, prev_time - current_start)
    return longest


def _alignment_field(row: Dict[str, str], key: str) -> Optional[str]:
    if key in row and row[key] not in ("", None):
        return row[key]
    values_json = row.get("values_json")
    if not values_json:
        return None
    try:
        payload = json.loads(values_json)
    except (TypeError, json.JSONDecodeError):
        return None
    value = payload.get(key)
    if value is None:
        return None
    return str(value)


def _alignment_bool(row: Dict[str, str], key: str) -> bool:
    return _as_bool(_alignment_field(row, key))


def _alignment_float(row: Dict[str, str], key: str) -> Optional[float]:
    return _as_float(_alignment_field(row, key))


def summarize_recovery_health(
    supervisor_events_csv: Path,
    alignment_status_csv: Optional[Path] = None,
    min_stable_windows: int = 1,
    max_recovery_fitness: float = 1.5,
) -> RecoveryHealthSummary:
    events = _read_csv_rows(supervisor_events_csv)
    recovery_confirmed_count = _count_events(events, "event", "recovery_confirmed")
    stable_windows = _count_events(events, "event", "stable_recovered_request_window")
    false_recovery = recovery_confirmed_count > 0 and stable_windows < min_stable_windows
    reinit_rows = 0
    longest_lost = 0.0
    if alignment_status_csv is not None and alignment_status_csv.is_file():
        alignment = _read_csv_rows(alignment_status_csv)
        reinit_rows = sum(
            1 for row in alignment
            if _alignment_bool(row, "reinitialization_requested"))
        longest_lost = _longest_true_window(
            [
                {
                    "stamp_sec": row.get("stamp_sec") or row.get("time_sec"),
                    "reinitialization_requested": (
                        "true" if _alignment_bool(row, "reinitialization_requested") else "false"),
                }
                for row in alignment
            ],
            "reinitialization_requested",
        )
        low_fitness_after_recovery = [
            _alignment_float(row, "fitness_score")
            for row in alignment
            if _alignment_bool(row, "tracking_ok")
            or _alignment_field(row, "failure_category") == "healthy"
        ]
        low_fitness_after_recovery = [
            value for value in low_fitness_after_recovery if value is not None]
        if low_fitness_after_recovery:
            if min(low_fitness_after_recovery) > max_recovery_fitness:
                false_recovery = True

    reasons = []
    if recovery_confirmed_count == 0:
        reasons.append("recovery_confirmed_count=0")
    if stable_windows < min_stable_windows:
        reasons.append(
            "stable_recovered_request_windows=%d (need >= %d)"
            % (stable_windows, min_stable_windows))
    if false_recovery:
        reasons.append("recovery_confirmed without stable window evidence")

    ok = (
        recovery_confirmed_count > 0
        and stable_windows >= min_stable_windows
        and not false_recovery
    )
    return RecoveryHealthSummary(
        recovery_confirmed_count=recovery_confirmed_count,
        stable_recovered_request_windows=stable_windows,
        false_recovery_confirmed=false_recovery,
        reinitialization_requested_rows=reinit_rows,
        longest_lost_window_sec=longest_lost,
        ok=ok,
        reasons=tuple(reasons),
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--supervisor-events-csv",
        required=True,
        help="CSV exported from the G3 supervisor event log.")
    parser.add_argument(
        "--alignment-status-csv",
        default="",
        help="Optional alignment_status.csv from benchmark_diagnostic_recorder.")
    parser.add_argument(
        "--min-stable-windows",
        type=int,
        default=1,
        help="Required stable recovered request windows for pass.")
    parser.add_argument(
        "--output-json",
        default="",
        help="Optional path to write the JSON summary.")
    args = parser.parse_args(argv)

    summary = summarize_recovery_health(
        Path(args.supervisor_events_csv),
        alignment_status_csv=(
            Path(args.alignment_status_csv) if args.alignment_status_csv else None),
        min_stable_windows=args.min_stable_windows,
    )
    payload = {
        "ok": summary.ok,
        "recovery_confirmed_count": summary.recovery_confirmed_count,
        "stable_recovered_request_windows": summary.stable_recovered_request_windows,
        "false_recovery_confirmed": summary.false_recovery_confirmed,
        "reinitialization_requested_rows": summary.reinitialization_requested_rows,
        "longest_lost_window_sec": summary.longest_lost_window_sec,
        "reasons": list(summary.reasons),
    }
    text = json.dumps(payload, indent=2, sort_keys=True)
    print(text)
    if args.output_json:
        Path(args.output_json).write_text(text + "\n", encoding="utf-8")
    return 0 if summary.ok else 1


if __name__ == "__main__":
    sys.exit(main())
