#!/usr/bin/env python3
"""Compare how often consecutive G2 answers confirm each other at the wrong place.

Reads g2_offline_probe.py outputs (<case>.<variant>.json) and recomputes, from
the stored queries, the consecutive top answers that agree after the odometry
motion (right / near / far from the truth, all and gated) and the simulated
startup, summed over cases.

usage: consensus_report.py RESULT_DIR [RESULT_DIR ...] --variant NAME [--variant NAME ...]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import g2_offline_probe as probe


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", type=Path, nargs="+")
    parser.add_argument("--variant", action="append", required=True)
    args = parser.parse_args(argv)
    print(
        "| variant | cases | queries | all pairs right / near / far | "
        "gated pairs right / near / far | startups ok / near / far / none |"
    )
    print("|---|---|---|---|---|---|")
    for variant in args.variant:
        totals = {"right": 0, "near": 0, "far": 0}
        gated = {"right": 0, "near": 0, "far": 0}
        startups = {"ok": 0, "near": 0, "far": 0, "none": 0}
        cases = queries = 0
        for directory in args.dirs:
            for path in sorted(directory.glob(f"*.{variant}.json")):
                data = json.loads(path.read_text())["queries"]
                if not data:
                    continue
                cases += 1
                queries += len(data)
                for key, value in probe.consensus_pairs(data, gated_only=False).items():
                    totals[key] += value
                for key, value in probe.consensus_pairs(data, gated_only=True).items():
                    gated[key] += value
                startup = probe.simulate_startup(data)
                if not startup.get("initialized"):
                    startups["none"] += 1
                elif startup["error_m"] <= probe.NEAR_TRANSLATION_M:
                    startups["ok"] += 1
                elif startup["error_m"] <= probe.FAR_ERROR_M:
                    startups["near"] += 1
                else:
                    startups["far"] += 1
        print(
            f"| `{variant}` | {cases} | {queries} | "
            f"{totals['right']} / {totals['near']} / {totals['far']} | "
            f"{gated['right']} / {gated['near']} / {gated['far']} | "
            f"{startups['ok']} / {startups['near']} / {startups['far']} / {startups['none']} |"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
