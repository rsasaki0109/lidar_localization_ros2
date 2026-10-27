#!/usr/bin/env python3
"""Run a baseline and a candidate workspace alternately over the Istanbul
windows with replay.py, then print a summary table (median and range).

usage: compare.py --base-ws WS --cand-ws WS --fixture DIR --bag BAG --map PCD --out DIR
                  [--repeats N] [--set KEY=VALUE ...] [--cand-set KEY=VALUE ...]
                  [--summary-only]
"""

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
# Seconds from bag start and window length: the old 60 s regression start, and
# the two stretches with sustained 7-10 % grades.
WINDOWS = {
    "A_start_0": (0, 60),
    "B_grade_2820": (2820, 150),
    "C_grade_3160": (3160, 120),
}
METRICS = (
    ("horizontal_m", "rmse"),
    ("z_m", "rmse"),
    ("yaw_deg", "rmse"),
    ("trans3d_m", "p95"),
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
                        str(HERE / "replay.py"),
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


def summarize(out):
    print(
        "| window | variant | runs | horizontal RMSE m | z RMSE m | yaw RMSE deg | 3D p95 m |"
    )
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for window in WINDOWS:
        for variant in ("base", "cand"):
            scores = [
                json.loads(p.read_text())
                for p in sorted((Path(out) / window).glob(f"{variant}_*/score.json"))
            ]
            if not scores:
                continue
            cells = []
            for group, key in METRICS:
                values = [s[group][key] for s in scores]
                cells.append(
                    f"{statistics.median(values):.3f} ({min(values):.3f}-{max(values):.3f})"
                )
            print(
                f"| {window} | {variant} | {len(scores)} | " + " | ".join(cells) + " |"
            )


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
        help="preset override for both variants (replay.py --set); repeatable",
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
