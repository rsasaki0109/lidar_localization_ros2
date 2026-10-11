#!/usr/bin/env python3
"""Run g2_offline_probe.py for every (case, variant) pair and tabulate the summaries.

Cases and variants come from a YAML file (see matrix.yaml). Paths in it may use
$JEPLO_PC2, $JEPLO_ODOM, $JEPLO_GT and $JEPLO_MAPS.

usage: run_matrix.py matrix.yaml --out DIR [--jobs 3] [--variant NAME ...] [--case NAME ...]
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent


def run_one(
    case: dict, variant: dict, out_dir: Path
) -> tuple[str, str, dict | None, str]:
    out = out_dir / f"{case['name']}.{variant['name']}.json"
    if not out.exists():
        occupancy = os.path.expandvars(variant.get("occupancy", case["occupancy"]))
        occupancy = occupancy.replace(
            "{map_dir}", str(Path(os.path.expandvars(case["map"])).parent)
        )
        command = [
            sys.executable,
            str(HERE / "g2_offline_probe.py"),
            "--bag",
            os.path.expandvars(case["bag"]),
            "--odometry",
            os.path.expandvars(case["odometry"]),
            "--ground-truth",
            os.path.expandvars(case["gt"]),
            "--occupancy",
            occupancy,
            "--map",
            os.path.expandvars(case["map"]),
            "--out",
            str(out),
            "--config",
            json.dumps(variant.get("config", {})),
            "--accumulate-sec",
            str(variant.get("accumulate_sec", 0.0)),
            "--window-sec",
            str(case.get("window_sec", 45.0)),
            "--topic",
            case.get("topic", "/livox/lidar"),
        ]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            return case["name"], variant["name"], None, result.stderr[-2000:]
    return case["name"], variant["name"], json.loads(out.read_text())["summary"], ""


def table(results: dict) -> str:
    lines = [
        "| case | variant | band pts | truth in list | top1 near | gated right/wrong | "
        "top1 err (m) | truth bbs | top bbs | startup | query (s) |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for (case, variant), summary in sorted(results.items()):
        if not summary.get("queries"):
            lines.append(f"| {case} | {variant} | no queries |" + " |" * 8)
            continue
        startup = summary["startup"]
        if startup.get("initialized"):
            verdict = (
                f"WRONG {startup['error_m']:.2f} m @{startup['elapsed_sec']:.0f}s"
                if startup["wrong"]
                else f"ok {startup['error_m']:.2f} m @{startup['elapsed_sec']:.0f}s"
            )
        else:
            verdict = "none"
        lines.append(
            f"| {case} | {variant} | {summary['band_points_median']:.0f} | "
            f"{summary['any_near']:.2f} | {summary['top1_near']:.2f} | "
            f"{summary['gated_right']}/{summary['gated_wrong']} | "
            f"{summary['top1_error_median_m']:.2f} | {summary['truth_bbs_median']:.2f} | "
            f"{summary['top_bbs_median']:.2f} | {verdict} | {summary['runtime_median_sec']:.2f} |"
        )
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("matrix", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--variant", action="append")
    parser.add_argument("--case", action="append")
    args = parser.parse_args(argv)
    matrix = yaml.safe_load(args.matrix.read_text())
    cases = [c for c in matrix["cases"] if not args.case or c["name"] in args.case]
    variants = [
        v for v in matrix["variants"] if not args.variant or v["name"] in args.variant
    ]
    args.out.mkdir(parents=True, exist_ok=True)
    results = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [
            pool.submit(run_one, case, variant, args.out)
            for case in cases
            for variant in variants
        ]
        for future in concurrent.futures.as_completed(futures):
            case, variant, summary, error = future.result()
            if summary is None:
                print(f"FAILED {case} {variant}: {error}", file=sys.stderr)
                continue
            results[(case, variant)] = summary
    # Tabulate every finished pair of the matrix, not only this run's selection.
    for case in matrix["cases"]:
        for variant in matrix["variants"]:
            out = args.out / f"{case['name']}.{variant['name']}.json"
            if (case["name"], variant["name"]) not in results and out.exists():
                results[(case["name"], variant["name"])] = json.loads(out.read_text())[
                    "summary"
                ]
    text = table(results)
    (args.out / "summary.md").write_text(text)
    (args.out / "summary.json").write_text(
        json.dumps({f"{c}|{v}": s for (c, v), s in results.items()}, indent=1) + "\n"
    )
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
