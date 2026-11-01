#!/usr/bin/env python3
"""Run a baseline and a candidate workspace alternately over the Istanbul
windows with istanbul_replay.py, then print a summary table (median and range).

usage: istanbul_compare.py --base-ws WS --cand-ws WS --fixture DIR --bag BAG --map PCD
                           --out DIR [--repeats N] [--set KEY=VALUE ...]
                           [--cand-set KEY=VALUE ...] [--summary-only]
"""

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
# Seconds from bag start and window length: the old 60 s regression start, the
# stretch with sustained 6-10 % grades, and the fastest climb (4-5 % at up to
# 21 m/s). GNSS is accurate (sigma <= 0.52 m) in all three; it is not during
# 750-1030 s and 3020-3215 s, where the first window C (3160 s) started.
WINDOWS = {
    "A_start_0": (0, 60),
    "B_grade_2820": (2820, 150),
    "C_climb_2300": (2300, 120),
}
METRICS = (
    ("horizontal_m", "rmse"),
    ("z_m", "rmse"),
    ("yaw_deg", "rmse"),
    ("trans3d_m", "p95"),
)
# score.json "alignment" values for the throughput table (istanbul_replay.py).
COST_METRICS = (
    ("aligned_share", None),
    ("align_ms", "median"),
    ("align_ms", "p95"),
    ("iterations", "median"),
    ("iterations", "at_cap_share"),
)


def run(args):
    variants = {
        "base": (args.base_ws, args.set),
        "cand": (args.cand_ws, args.set + args.cand_set),
    }
    domain = 80
    for repeat in range(args.repeats):
        for window, (start, duration) in WINDOWS.items():
            for variant, (ws, overrides) in variants.items():
                out = Path(args.out) / window / f"{variant}_{repeat}"
                domain += 1
                if (out / "score.json").exists():
                    continue
                subprocess.run(
                    [
                        sys.executable,
                        str(HERE / "istanbul_replay.py"),
                        "--ws",
                        ws,
                        "--fixture",
                        args.fixture,
                        "--bag",
                        args.bag,
                        "--map",
                        args.map,
                        "--start",
                        str(start),
                        "--duration",
                        str(duration),
                        "--out",
                        str(out),
                        "--domain",
                        str(domain),
                        *(f"--set={item}" for item in overrides),
                    ],
                    check=True,
                    capture_output=True,
                    timeout=1800,
                )
                print(window, variant, repeat, "done", flush=True)


def _cell(values, digits=3):
    return (
        f"{statistics.median(values):.{digits}f} "
        f"({min(values):.{digits}f}-{max(values):.{digits}f})"
    )


def _scores(out):
    for window in WINDOWS:
        for variant in ("base", "cand"):
            scores = [
                json.loads(p.read_text())
                for p in sorted((Path(out) / window).glob(f"{variant}_*/score.json"))
            ]
            if scores:
                yield window, variant, scores


def summarize(out):
    print(
        "| window | variant | runs | horizontal RMSE m | z RMSE m | yaw RMSE deg | 3D p95 m |"
    )
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for window, variant, scores in _scores(out):
        cells = [_cell([s[group][key] for s in scores]) for group, key in METRICS]
        print(f"| {window} | {variant} | {len(scores)} | " + " | ".join(cells) + " |")

    # Runs recorded before istanbul_replay.py scored the alignment cost have no
    # "alignment"; the table lists only the runs that do.
    rows = []
    for window, variant, scores in _scores(out):
        costs = [s for s in scores if "alignment" in s]
        if not costs:
            continue
        cells = [_cell([s.get("poses_per_sec", 0.0) for s in costs], 1)]
        for group, key in COST_METRICS:
            values = [
                s["alignment"][group] if key is None else s["alignment"][group][key]
                for s in costs
                if group in s["alignment"]
            ]
            cells.append(
                _cell(values, 1 if group == "align_ms" else 2) if values else "-"
            )
        rows.append(
            f"| {window} | {variant} | {len(costs)} | " + " | ".join(cells) + " |"
        )
    if rows:
        print()
        print(
            "| window | variant | runs | pose rate Hz | aligned share | align ms median"
            " | align ms p95 | iterations median | at iteration cap |"
        )
        print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        print("\n".join(rows))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base-ws", required=True)
    parser.add_argument("--cand-ws", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="preset override for both variants (istanbul_replay.py --set); repeatable",
    )
    parser.add_argument(
        "--cand-set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="preset override for the candidate only; repeatable",
    )
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()
    if not args.summary_only:
        run(args)
    summarize(args.out)
