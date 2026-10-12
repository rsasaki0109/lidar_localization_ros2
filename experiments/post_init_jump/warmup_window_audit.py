#!/usr/bin/env python3
"""Judge the corrections that only the seed correction guard's warmup lets through.

For replay runs with the quickstart warmup (5 settled accepts), find accepted
corrections above the guard limit that came after the first settled accept of a
track (after an initial pose or reset) and before the warmup ended. A warmup of 1
would have rejected exactly these. Each is classed by the ground truth: did it
bring the pose closer to the truth (a pull-in W1 would block) or further away (an
alias W1 would stop)?

usage: warmup_window_audit.py RUN_DIR... --gt-for CASE=GT.txt ...
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

GUARD_TRANSLATION_M = 0.3  # quickstart's seed_correction_guard_translation_m
WARMUP = 5


def truth_xy(truth: np.ndarray, stamp: float) -> np.ndarray:
    return np.array(
        [
            np.interp(stamp, truth[:, 0], truth[:, 1]),
            np.interp(stamp, truth[:, 0], truth[:, 2]),
        ]
    )


def audit(run: Path, truth: np.ndarray) -> list[dict]:
    rows = [json.loads(line) for line in (run / "alignment.jsonl").open()]
    est = np.loadtxt(run / "est.tum", ndmin=2)
    if not rows or est.size == 0:
        return []
    found = []
    settled = 0
    seen_settled = False
    previous_error = None
    for row in rows:
        stamp = row["stamp"]
        index = int(np.argmin(np.abs(est[:, 0] - stamp)))
        if abs(est[index, 0] - stamp) > 0.05:
            continue
        error = float(np.linalg.norm(est[index, 1:3] - truth_xy(truth, stamp)))
        accepted = row["message"] in ("ok",) or "rejected" not in row["message"]
        correction = float(row["correction_translation_m"])
        if (
            row.get("recovery_state") == "tracking"
            and float(row["accepted_gap_sec"]) > 1.0
        ):
            settled = 0
        if accepted:
            if correction > GUARD_TRANSLATION_M:
                if seen_settled and settled < WARMUP and previous_error is not None:
                    found.append(
                        {
                            "stamp": stamp,
                            "correction_m": correction,
                            "error_before_m": previous_error,
                            "error_after_m": error,
                        }
                    )
                settled = 0
            else:
                settled += 1
                seen_settled = True
        previous_error = error
    return found


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument(
        "--gt-for", action="append", default=[], help="CASE=ground_truth.txt"
    )
    args = parser.parse_args(argv)
    gt = dict(item.split("=", 1) for item in args.gt_for)
    toward = away = 0
    for run in args.runs:
        case = run.parent.name
        key = next((k for k in gt if case.startswith(k)), None)
        if key is None or not (run / "alignment.jsonl").exists():
            continue
        truth = np.loadtxt(gt[key], ndmin=2)
        for item in audit(run, truth):
            better = item["error_after_m"] < item["error_before_m"]
            toward += better
            away += not better
            print(
                f"{run}: correction {item['correction_m']:.2f} m, error "
                f"{item['error_before_m']:.2f} -> {item['error_after_m']:.2f} m "
                f"({'toward' if better else 'away from'} the truth)"
            )
    print(f"toward the truth: {toward}, away: {away}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
