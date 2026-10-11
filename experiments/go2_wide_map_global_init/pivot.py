#!/usr/bin/env python3
"""Pivot run_matrix.py results: one row per variant, one column group per case.

Each cell is ``top1/gated_wrong/startup``: the share of queries whose top
candidate is within 1 m and 15 deg of the truth, how many queries passed the
single-query startup gates with a top candidate more than 1 m off, and the
simulated startup (error of the published fix and seconds after the first
query, or ``-`` when it never initialized).

usage: pivot.py RESULT_DIR | results.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def cell(summary: dict) -> str:
    startup = summary["startup"]
    verdict = (
        f"{startup['error_m']:.1f}m@{startup['elapsed_sec']:.0f}s"
        if startup.get("initialized")
        else "-"
    )
    return f"{summary['top1_near']:.2f} / {summary['gated_wrong']} / {verdict}"


def main(argv=None) -> int:
    path = Path((argv or sys.argv[1:])[0])
    if path.is_dir():
        path = path / "summary.json"
    results = json.loads(path.read_text())
    results = results.get("summaries", results)
    cases = sorted(
        {key.split("|")[0] for key in results}, key=lambda c: (c.startswith("EIL"), c)
    )
    variants = sorted({key.split("|")[1] for key in results})
    print("| variant | " + " | ".join(cases) + " |")
    print("|---" * (len(cases) + 1) + "|")
    for variant in variants:
        row = [
            cell(results[f"{c}|{variant}"]) if f"{c}|{variant}" in results else ""
            for c in cases
        ]
        print(f"| {variant} | " + " | ".join(row) + " |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
