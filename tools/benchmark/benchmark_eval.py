#!/usr/bin/env python3
"""Score a localization run against ground truth in the map frame.

The map must be in the ground-truth frame, so no alignment is applied: an
aligned comparison hides a wrongly initialized pose that is tracked
consistently (it looked like 0.2 m on a Unitree Go2 while it was 1.8 m off).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import numpy as np

# A first fix further than this from the truth is a wrong initialization.
WRONG_INIT_M = 1.0
# Ground-truth samples count as matching an estimate this close in time.
MATCH_TOLERANCE_SEC = 0.05
# The first fix is judged on the estimates of this window after it.
FIRST_FIX_WINDOW_SEC = 2.0


@dataclass(frozen=True)
class RunScore:
    initialized: bool
    time_to_first_pose_sec: float | None
    first_fix_error_m: float | None
    wrong_init: bool
    matched: int
    tracked_fraction: float
    median_xy_m: float | None
    p95_xy_m: float | None
    max_xy_m: float | None
    frac_over_0_5_m: float | None
    frac_over_1_m: float | None
    median_abs_z_m: float | None
    # Share of poses more than 1 m off that /alignment_status did not report OK,
    # and of poses within 0.3 m that it did not report OK (None without data).
    flagged_when_off: float | None = None
    flagged_when_right: float | None = None

    def as_dict(self) -> dict:
        return asdict(self)


def load_tum(path) -> np.ndarray:
    """(N, 8) stamp x y z qx qy qz qw rows, or an empty array."""
    try:
        rows = np.loadtxt(path, ndmin=2)
    except (OSError, ValueError):
        return np.empty((0, 8))
    if rows.size == 0:
        return np.empty((0, 8))
    return rows[:, :8]


def match_to_ground_truth(estimate: np.ndarray, truth: np.ndarray):
    """Pairs of (estimate row, nearest ground-truth row) within the tolerance."""
    if len(estimate) == 0 or len(truth) == 0:
        return np.empty((0, 8)), np.empty((0, 8))
    order = np.argsort(truth[:, 0])
    truth = truth[order]
    index = np.searchsorted(truth[:, 0], estimate[:, 0]).clip(1, len(truth) - 1)
    before = truth[index - 1, 0]
    after = truth[index, 0]
    index = np.where(
        np.abs(before - estimate[:, 0]) < np.abs(after - estimate[:, 0]),
        index - 1,
        index,
    )
    close = np.abs(truth[index, 0] - estimate[:, 0]) <= MATCH_TOLERANCE_SEC
    return estimate[close], truth[index[close]]


def load_alignment_levels(path) -> np.ndarray:
    """(N, 2) rows of /alignment_status stamp and level (0 = OK), sorted."""
    rows = []
    try:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                record = json.loads(line)
                rows.append((float(record["stamp"]), float(record["level"])))
    except (OSError, ValueError, KeyError):
        return np.empty((0, 2))
    rows.sort()
    return np.array(rows) if rows else np.empty((0, 2))


def health_flags(
    stamps: np.ndarray, levels: np.ndarray, max_age_sec: float = 1.0
) -> np.ndarray:
    """Whether the latest status at or before each stamp was not OK.

    A pose with no status in the last max_age_sec counts as flagged: the
    localizer has stopped reporting.
    """
    if len(levels) == 0:
        return np.ones(len(stamps), dtype=bool)
    index = np.searchsorted(levels[:, 0], stamps, side="right") - 1
    valid = index >= 0
    age = np.where(valid, stamps - levels[index.clip(0), 0], np.inf)
    level = np.where(valid, levels[index.clip(0), 1], 2.0)
    return (age > max_age_sec) | (level != 0.0)


def _coverage(matched_stamps: np.ndarray, truth_stamps: np.ndarray) -> float:
    """Share of the ground-truth time (in 0.1 s bins) that has a matched estimate."""
    wanted = set(np.round(truth_stamps / 0.1).astype(np.int64).tolist())
    if not wanted:
        return 0.0
    have = set(np.round(matched_stamps / 0.1).astype(np.int64).tolist())
    return len(have & wanted) / len(wanted)


def score_run(
    estimate: np.ndarray,
    truth: np.ndarray,
    start_sec: float,
    alignment_levels: np.ndarray | None = None,
) -> RunScore:
    """Score poses published after start_sec (the replay start) against truth."""
    estimate = estimate[estimate[:, 0] >= start_sec] if len(estimate) else estimate
    if len(estimate) == 0:
        return RunScore(False, None, None, False, 0, 0.0, *([None] * 6))
    first_stamp = float(estimate[0, 0])
    est, gt = match_to_ground_truth(estimate, truth)
    if len(est) == 0:
        return RunScore(
            True, first_stamp - start_sec, None, False, 0, 0.0, *([None] * 6)
        )
    xy = np.hypot(est[:, 1] - gt[:, 1], est[:, 2] - gt[:, 2])
    z = np.abs(est[:, 3] - gt[:, 3])
    first_window = est[:, 0] <= est[0, 0] + FIRST_FIX_WINDOW_SEC
    first_fix_error = float(np.median(xy[first_window]))
    after_init = truth[
        (truth[:, 0] >= first_stamp) & (truth[:, 0] <= estimate[-1, 0]), 0
    ]
    flagged_when_off = flagged_when_right = None
    if alignment_levels is not None and len(alignment_levels):
        flagged = health_flags(est[:, 0], alignment_levels)
        off = xy > 1.0
        right = xy < 0.3
        flagged_when_off = float(np.mean(flagged[off])) if off.any() else None
        flagged_when_right = float(np.mean(flagged[right])) if right.any() else None
    return RunScore(
        flagged_when_off=flagged_when_off,
        flagged_when_right=flagged_when_right,
        initialized=True,
        time_to_first_pose_sec=first_stamp - start_sec,
        first_fix_error_m=first_fix_error,
        wrong_init=first_fix_error > WRONG_INIT_M,
        matched=len(est),
        tracked_fraction=_coverage(gt[:, 0], after_init),
        median_xy_m=float(np.median(xy)),
        p95_xy_m=float(np.percentile(xy, 95)),
        max_xy_m=float(np.max(xy)),
        frac_over_0_5_m=float(np.mean(xy > 0.5)),
        frac_over_1_m=float(np.mean(xy > 1.0)),
        median_abs_z_m=float(np.median(z)),
    )


def _fmt(value, digits=2) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "-"
    return f"{value:.{digits}f}"


def health_table(results: dict[str, list[RunScore]]) -> str:
    """Markdown table of how well /alignment_status flags wrong poses."""
    lines = [
        "| case | runs with >1 m poses | flagged when >1 m off | flagged when <0.3 m |",
        "|---|---|---|---|",
    ]
    for case, scores in results.items():
        off = [s.flagged_when_off for s in scores if s.flagged_when_off is not None]
        right = [
            s.flagged_when_right for s in scores if s.flagged_when_right is not None
        ]
        lines.append(
            f"| {case} | {len(off)} | {_fmt(float(np.mean(off)) if off else None)} | "
            f"{_fmt(float(np.mean(right)) if right else None)} |"
        )
    return "\n".join(lines)


def summary_table(results: dict[str, list[RunScore]]) -> str:
    """Markdown table: one row per case, medians over its repeats."""
    lines = [
        "| case | runs | initialized | wrong init | time to first pose (s) | "
        "median xy (m) | p95 xy (m) | max xy (m) | >1 m |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for case, scores in results.items():
        good = [s for s in scores if s.initialized and s.median_xy_m is not None]

        def median_of(field, runs=good):
            values = [getattr(s, field) for s in runs if getattr(s, field) is not None]
            return float(np.median(values)) if values else None

        lines.append(
            f"| {case} | {len(scores)} | {sum(s.initialized for s in scores)} | "
            f"{sum(s.wrong_init for s in scores)} | "
            f"{_fmt(median_of('time_to_first_pose_sec', scores), 1)} | "
            f"{_fmt(median_of('median_xy_m'))} | {_fmt(median_of('p95_xy_m'))} | "
            f"{_fmt(max((s.max_xy_m for s in good), default=None))} | "
            f"{_fmt(median_of('frac_over_1_m'), 3)} |"
        )
    return "\n".join(lines)
