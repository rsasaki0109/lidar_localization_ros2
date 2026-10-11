#!/usr/bin/env python3
"""Score /alignment_status as a health signal against map-frame ground truth.

Reads benchmark run directories written by tools/benchmark/run_benchmark.py
(`est.tum`, `alignment.jsonl`). Each status row is joined with the GT error of the
pose published for the same scan. A row is "off" when that pose is more than 1 m
from GT and "good" when it is within 0.3 m (as in docs/benchmark.md). For every
rule, a row is flagged when its level is not OK or when the rule fires.

    python3 analyze_health.py --gt outdoor_kidnap_b=/data/gt/traj_lidar_outdoor_kidnap.txt \\
        /tmp/runs/outdoor_kidnap_b/run1 ...

The case name is the run directory's parent, as run_benchmark.py lays it out.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "benchmark"))

import benchmark_eval  # noqa: E402

OFF_M = 1.0
GOOD_M = 0.3
MATCH_SEC = 0.15


def value(row: dict, key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def poor_accept(fitness: float, correction_m: float):
    """An accepted scan that both registered poorly and moved the pose."""
    return lambda row: (
        value(row, "fitness_score") > fitness
        and value(row, "correction_translation_m") > correction_m
    )


# Rules chosen on outdoor_hard_02b and outdoor_kidnap_b before the held-out runs.
RULES = {
    "current": lambda row: False,
    "poor_accept_a (fitness > 0.5, correction > 0.3 m)": poor_accept(0.5, 0.3),
    "poor_accept_b (fitness > 1.0, correction > 0.3 m)": poor_accept(1.0, 0.3),
}


def load_rows(run: Path, truth: np.ndarray) -> list[dict]:
    estimate = benchmark_eval.load_tum(run / "est.tum")
    est, gt = benchmark_eval.match_to_ground_truth(estimate, truth)
    if not len(est):
        return []
    error = np.hypot(est[:, 1] - gt[:, 1], est[:, 2] - gt[:, 2])
    stamps = est[:, 0]
    rows = []
    with (run / "alignment.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            index = int(np.searchsorted(stamps, row["stamp"]))
            near = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
            best = min(near, key=lambda i: abs(stamps[i] - row["stamp"]))
            if abs(stamps[best] - row["stamp"]) <= MATCH_SEC:
                row["error_m"] = float(error[best])
                rows.append(row)
    return rows


def share(rows, predicate) -> str:
    return f"{sum(map(predicate, rows)) / len(rows):.2f}" if rows else "-"


def report(rows_by_case: dict[str, list[dict]]) -> str:
    lines = [
        "| case | status rows | off | off reported OK | good |"
        + "".join(f" {name}: flagged off / good |" for name in RULES),
        "|---|---:|---:|---:|---:|" + "---|" * len(RULES),
    ]
    for case, rows in rows_by_case.items():
        off = [r for r in rows if r["error_m"] > OFF_M]
        good = [r for r in rows if r["error_m"] < GOOD_M]
        cells = []
        for rule in RULES.values():

            def flagged(row, rule=rule):
                return row["level"] != 0 or rule(row)

            cells.append(f" {share(off, flagged)} / {share(good, flagged)} |")
        lines.append(
            f"| `{case}` | {len(rows)} | {len(off)} "
            f"| {sum(r['level'] == 0 for r in off)} | {len(good)} |" + "".join(cells)
        )
    alarms = [
        r
        for rows in rows_by_case.values()
        for r in rows
        if r["error_m"] < GOOD_M and r["level"] != 0
    ]
    if alarms:
        lines += ["", f"Not-OK rows on good poses: {len(alarms)}"]
        for key in ("level", "message", "recovery_state"):
            counts = collections.Counter(r.get(key) for r in alarms).most_common(4)
            lines.append(f"- {key}: " + ", ".join(f"{k} {n}" for k, n in counts))
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="benchmark run directories")
    parser.add_argument(
        "--gt", action="append", required=True, help="CASE=path to a TUM GT file"
    )
    args = parser.parse_args(argv)
    truth = {}
    for item in args.gt:
        case, path = item.split("=", 1)
        truth[case] = benchmark_eval.load_tum(path)
    rows_by_case: dict[str, list[dict]] = collections.defaultdict(list)
    for run in args.runs:
        case = run.parent.name
        if case in truth:
            rows_by_case[case] += load_rows(run, truth[case])
    print(report(dict(rows_by_case)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
