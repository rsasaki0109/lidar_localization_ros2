#!/usr/bin/env python3
"""Summarize run_koide_public_bag.py runs of two registration backends: the gate
metrics, and the same split at the stretch the localizer accepted no scan.

usage: summarize_runs.py DATA_DIR LABEL=RUN_DIR [LABEL=RUN_DIR ...] [--json OUT]

DATA_DIR is the run_koide_public_bag.py data directory (koide-data). A cloud is
expected from 4 s after the bag start, as the check counts it. The poses are matched
to the GT pose with the nearest stamp within 60 ms, as in the check.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import koide_public_bag as data_tool

MATCH_SEC = 0.06


def errors(estimate, reference):
    idx = np.searchsorted(reference[:, 0], estimate[:, 0]).clip(1, len(reference) - 1)
    before = np.abs(reference[idx - 1, 0] - estimate[:, 0])
    after = np.abs(reference[idx, 0] - estimate[:, 0])
    near = np.where(after < before, idx, idx - 1)
    ok = np.abs(reference[near, 0] - estimate[:, 0]) < MATCH_SEC
    return estimate[ok, 0], estimate[ok, 1:4] - reference[near[ok], 1:4]


def rmse(values):
    return float(np.sqrt(np.mean(values**2)))


def summarize(run_dir, expected, reference):
    summary = json.loads((run_dir / "summary.json").read_text())
    records = [
        json.loads(line)
        for line in (run_dir / "alignment.jsonl").read_text().splitlines()
    ]
    stamps = np.array([r["stamp_ns"] for r in records]) * 1e-9
    seconds = np.array([float(r["values"]["alignment_time_sec"]) for r in records])
    accepted = np.array(
        [r["values"]["recovery_action"].startswith("accept") for r in records]
    )
    first = expected[0] * 1e-9

    # The longest stretch without an accepted scan: off the map in outdoor_hard_02b.
    accepted_stamps = stamps[accepted]
    gaps = np.diff(accepted_stamps)
    start, end = accepted_stamps[np.argmax(gaps)], accepted_stamps[np.argmax(gaps) + 1]
    inside = (stamps > start) & (stamps < end)
    timed = seconds > 0.0

    pose_stamps, diff = errors(np.loadtxt(run_dir / "estimate.tum"), reference)
    err = np.linalg.norm(diff, axis=1)
    in_stretch = (pose_stamps > start) & (pose_stamps < end)
    worst = np.flatnonzero(in_stretch)[np.argmax(err[in_stretch])]
    missed = sorted(set(expected) - {r["stamp_ns"] for r in records})
    return {
        "passed": summary["passed"],
        "rmse_m": round(summary["rmse_m"], 3),
        "diagnostic_coverage": round(summary["diagnostic_coverage"], 4),
        "missed_clouds": len(missed),
        "max_output_gap_sec": round(summary["max_output_gap_sec"], 2),
        "no_accept_stretch_sec": [round(start - first, 1), round(end - first, 1)],
        "missed_clouds_in_stretch": int(sum(start < s * 1e-9 < end for s in missed)),
        "rmse_outside_stretch_m": round(rmse(err[~in_stretch]), 3),
        "max_error_in_stretch_m": round(float(err[worst]), 2),
        "max_error_in_stretch_xy_z_m": [
            round(float(np.hypot(diff[worst, 0], diff[worst, 1])), 2),
            round(float(diff[worst, 2]), 2),
        ],
        "alignment_ms_outside_stretch": {
            "median": round(1000 * float(np.median(seconds[~inside & timed])), 1),
            "p95": round(1000 * float(np.percentile(seconds[~inside & timed], 95)), 1),
        },
        "alignment_ms_in_stretch": {
            "median": round(1000 * float(np.median(seconds[inside & timed])), 1),
            "p95": round(1000 * float(np.percentile(seconds[inside & timed], 95)), 1),
            "max": round(1000 * float(seconds[inside].max()), 1),
        },
        "alignment_sec_per_sec_in_stretch": round(
            float(seconds[inside].sum() / (end - start)), 2
        ),
    }


def cell(values, digits):
    return f"{np.median(values):.{digits}f} ({min(values):.{digits}f}-{max(values):.{digits}f})"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data_dir", type=Path)
    parser.add_argument("runs", nargs="+", metavar="LABEL=RUN_DIR")
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    start_ns, clouds = data_tool.bag_cloud_times(args.data_dir / "outdoor_hard_02b_si")
    expected = sorted(s for s in clouds if s >= start_ns + 4 * 10**9)
    reference = np.loadtxt(args.data_dir / "gt" / "traj_lidar_outdoor_hard_02.txt")
    results = {}
    for item in args.runs:
        label, run_dir = item.split("=", 1)
        results.setdefault(label, {})[Path(run_dir).name] = summarize(
            Path(run_dir), expected, reference
        )

    print(
        "| variant | runs | failed | coverage % | RMSE m | RMSE outside stretch m | max error in stretch m (horizontal, z) |"
    )
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for label, runs in results.items():
        r = list(runs.values())
        print(
            f"| {label} | {len(r)} | {sum(not x['passed'] for x in r)} | "
            f"{cell([100 * x['diagnostic_coverage'] for x in r], 1)} | "
            f"{cell([x['rmse_m'] for x in r], 3)} | "
            f"{cell([x['rmse_outside_stretch_m'] for x in r], 3)} | "
            f"{cell([x['max_error_in_stretch_m'] for x in r], 2)} "
            f"({cell([x['max_error_in_stretch_xy_z_m'][0] for x in r], 2)}, "
            f"{cell([x['max_error_in_stretch_xy_z_m'][1] for x in r], 2)}) |"
        )
    print()
    print(
        "| variant | missed clouds (in stretch) | alignment ms outside stretch, median / p95 | in stretch, median / p95 / max | alignment s per s in stretch |"
    )
    print("| --- | --- | --- | --- | --- |")
    for label, runs in results.items():
        r = list(runs.values())
        print(
            f"| {label} | {cell([x['missed_clouds'] for x in r], 0)} "
            f"({cell([x['missed_clouds_in_stretch'] for x in r], 0)}) | "
            f"{cell([x['alignment_ms_outside_stretch']['median'] for x in r], 0)} / "
            f"{cell([x['alignment_ms_outside_stretch']['p95'] for x in r], 0)} | "
            f"{cell([x['alignment_ms_in_stretch']['median'] for x in r], 0)} / "
            f"{cell([x['alignment_ms_in_stretch']['p95'] for x in r], 0)} / "
            f"{cell([x['alignment_ms_in_stretch']['max'] for x in r], 0)} | "
            f"{cell([x['alignment_sec_per_sec_in_stretch'] for x in r], 2)} |"
        )
    stretches = {
        tuple(x["no_accept_stretch_sec"])
        for runs in results.values()
        for x in runs.values()
    }
    print(
        f"\nstretches without an accepted scan (s from the first expected cloud): {sorted(stretches)}"
    )
    if args.json:
        args.json.write_text(json.dumps(results, indent=1) + "\n")


if __name__ == "__main__":
    main()
