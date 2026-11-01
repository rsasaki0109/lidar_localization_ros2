#!/usr/bin/env python3
"""Summarize run_koide_public_bag.py runs: the gate metrics, which input clouds got
no /alignment_status (missed), and the alignment cost.

usage: summarize_runs.py BAG LABEL=RUN_DIR [LABEL=RUN_DIR ...] [--json OUT]

BAG is the prepared SI bag (koide-data/outdoor_hard_02b_si). A cloud is expected
from 4 s after the bag start, as the check counts it.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import koide_public_bag as data_tool

SEGMENT = (100.0, 132.0)  # seconds from the first expected cloud


def summarize(run_dir, expected):
    summary = json.loads((run_dir / "summary.json").read_text())
    diagnostics = {}
    for line in (run_dir / "alignment.jsonl").read_text().splitlines():
        record = json.loads(line)
        diagnostics[record["stamp_ns"]] = record["values"]
    first = expected[0]
    missed = [s for s in expected if s not in diagnostics]
    in_segment = [s for s in missed if SEGMENT[0] <= (s - first) * 1e-9 < SEGMENT[1]]
    seconds = np.array(
        [
            float(v["alignment_time_sec"])
            for v in diagnostics.values()
            if float(v.get("alignment_time_sec", 0.0)) > 0.0
        ]
    )
    iterations = np.array(
        [int(v.get("registration_iterations", -1)) for v in diagnostics.values()]
    )
    iterations = iterations[iterations >= 0]
    return {
        "passed": summary["passed"],
        "rmse_m": round(summary["rmse_m"], 3),
        "p95_error_m": round(summary["p95_error_m"], 3),
        "diagnostic_coverage": round(summary["diagnostic_coverage"], 4),
        "missed_clouds": len(missed),
        "missed_clouds_100_132_s": len(in_segment),
        "max_output_gap_sec": round(summary["max_output_gap_sec"], 2),
        "alignment_ms": {
            "median": round(1000 * float(np.median(seconds)), 1),
            "p95": round(1000 * float(np.percentile(seconds, 95)), 1),
            "max": round(1000 * float(seconds.max()), 1),
        },
        "iterations_median": float(np.median(iterations)) if len(iterations) else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("runs", nargs="+", metavar="LABEL=RUN_DIR")
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    start_ns, clouds = data_tool.bag_cloud_times(Path(args.bag))
    expected = sorted(s for s in clouds if s >= start_ns + 4 * 10**9)
    results = {}
    for item in args.runs:
        label, run_dir = item.split("=", 1)
        results.setdefault(label, {})[Path(run_dir).name] = summarize(
            Path(run_dir), expected
        )
    print("| variant | runs | failed | coverage % | RMSE m | max output gap s |")
    print("| --- | --- | --- | --- | --- | --- |")
    for label, runs in results.items():
        cells = []
        for key, scale, digits in (
            ("diagnostic_coverage", 100, 1),
            ("rmse_m", 1, 3),
            ("max_output_gap_sec", 1, 1),
        ):
            values = [scale * r[key] for r in runs.values()]
            cells.append(
                f"{np.median(values):.{digits}f} "
                f"({min(values):.{digits}f}-{max(values):.{digits}f})"
            )
        failed = sum(not r["passed"] for r in runs.values())
        print(f"| {label} | {len(runs)} | {failed} | " + " | ".join(cells) + " |")
    if args.json:
        args.json.write_text(json.dumps(results, indent=1) + "\n")


if __name__ == "__main__":
    main()
