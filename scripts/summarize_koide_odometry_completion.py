#!/usr/bin/env python3
"""Require repeatable passes of the Koide full-sequence odometry gate."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


def _read_result(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"completion result must be a JSON object: {path}")
    return payload


def summarize(paths: Sequence[Path], required_repeats: int = 3) -> Dict[str, Any]:
    results = [_read_result(path) for path in paths]
    reference_thresholds = results[0].get("thresholds") if results else None
    threshold_mismatch = any(
        result.get("thresholds") != reference_thresholds for result in results[1:])
    passing = [result for result in results if result.get("ok") is True]
    failing = [result for result in results if result.get("ok") is not True]
    estimated_paths = [result.get("estimated_csv") for result in results]
    duplicate_artifacts = len(set(estimated_paths)) != len(estimated_paths)
    gates = {
        "enough_repeats": len(results) >= required_repeats,
        "all_repeats_pass": bool(results) and not failing,
        "identical_thresholds": bool(results) and not threshold_mismatch,
        "distinct_run_artifacts": bool(results) and not duplicate_artifacts,
    }
    return {
        "ok": all(gates.values()),
        "goal": "Koide outdoor_hard_01a full 380 s completion repeat gate",
        "required_repeats": required_repeats,
        "run_count": len(results),
        "passing_run_count": len(passing),
        "failing_run_count": len(failing),
        "gates": gates,
        "failed_gates": [name for name, passed in gates.items() if not passed],
        "thresholds": reference_thresholds,
        "runs": [
            {
                "completion_json": str(path.resolve()),
                "estimated_csv": result.get("estimated_csv"),
                "ok": result.get("ok") is True,
                "failed_gates": result.get("failed_gates", []),
                "coverage_ratio": result.get("metrics", {}).get("coverage_ratio"),
                "final_lag_sec": result.get("metrics", {}).get("final_lag_sec"),
                "translation_ate_rmse_m": result.get("metrics", {}).get(
                    "translation_ate_rmse_m"),
                "translation_end_error_m": result.get("metrics", {}).get(
                    "translation_end_error_m"),
                "rpe_translation_median_m": result.get("metrics", {}).get(
                    "rpe_translation_median_m"),
                "rpe_rotation_median_deg": result.get("metrics", {}).get(
                    "rpe_rotation_median_deg"),
            }
            for path, result in zip(paths, results)
        ],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--completion-json", action="append", required=True,
        help="Single-run completion JSON; repeat once per independent run.")
    parser.add_argument("--required-repeats", type=int, default=3)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args(argv)
    if args.required_repeats <= 0:
        parser.error("--required-repeats must be positive")
    result = summarize(
        [Path(path) for path in args.completion_json], args.required_repeats)
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(result, indent=2, sort_keys=True)
    output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
