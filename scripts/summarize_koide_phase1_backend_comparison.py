#!/usr/bin/env python3
"""Aggregate Koide Phase 1 backend comparison runs into one JSON report."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


BACKENDS = (
    ("NDT_OMP", "ndt"),
    ("SMALL_GICP", "small_gicp_ds"),
    ("SMALL_VGICP", "small_vgicp_ds"),
)


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def audit_alignment_csv(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {
            "present": False,
            "row_count": 0,
            "failure_category_rows": 0,
            "pose_covariance_mode_rows": 0,
            "registration_method_values": [],
        }
    row_count = 0
    failure_category_rows = 0
    pose_covariance_mode_rows = 0
    registration_methods = set()
    with path.open(newline="", encoding="utf-8") as handle:
        for record in csv.DictReader(handle):
            row_count += 1
            try:
                values = json.loads(record["values_json"])
            except (KeyError, json.JSONDecodeError):
                continue
            if values.get("failure_category"):
                failure_category_rows += 1
            if values.get("pose_covariance_mode"):
                pose_covariance_mode_rows += 1
            method = values.get("registration_method")
            if method:
                registration_methods.add(str(method))
    return {
        "present": True,
        "row_count": row_count,
        "failure_category_rows": failure_category_rows,
        "pose_covariance_mode_rows": pose_covariance_mode_rows,
        "registration_method_values": sorted(registration_methods),
        "diagnostics_ok": row_count > 0 and failure_category_rows > 0,
    }


def summarize_run(output_dir: Path, backend: str) -> Dict[str, Any]:
    health = _read_json(output_dir / "health_summary.json")
    trajectory = _read_json(output_dir / "trajectory_eval.json")
    alignment_audit = audit_alignment_csv(output_dir / "alignment_status.csv")
    alignment = (health or {}).get("alignment", {})
    lost_windows = alignment.get("lost_windows") or []
    longest_lost = max(
        (float(window.get("duration_sec") or 0.0) for window in lost_windows),
        default=0.0,
    )
    return {
        "backend": backend,
        "output_dir": str(output_dir),
        "completed": health is not None and trajectory is not None,
        "alignment_diagnostics": alignment_audit,
        "poses": trajectory.get("matched_sample_count") if trajectory else None,
        "ok_rate_percent": alignment.get("ok_rate_percent"),
        "max_alignment_time_sec": alignment.get("max_alignment_time_sec"),
        "translation_rmse_m": (
            trajectory.get("translation_rmse_m") if trajectory else None),
        "rotation_rmse_deg": (
            trajectory.get("rotation_rmse_deg") if trajectory else None),
        "longest_lost_window_sec": longest_lost,
        "message_counts": alignment.get("message_counts"),
    }


def build_comparison(run_dirs: Dict[str, Path]) -> Dict[str, Any]:
    runs = [summarize_run(run_dirs[key], key) for key in ("ndt", "small_gicp_ds", "small_vgicp_ds")]
    completed = [run for run in runs if run["completed"]]
    diagnostics_ok = all(
        run["alignment_diagnostics"].get("diagnostics_ok") for run in completed)
    ndt = next((run for run in runs if run["backend"] == "ndt"), None)
    overall_pass = (
        len(completed) >= 1
        and ndt is not None
        and ndt["completed"]
        and ndt["alignment_diagnostics"].get("diagnostics_ok")
        and diagnostics_ok
    )
    if len(completed) == 3 and diagnostics_ok:
        # Phase 1 verdict echo: NDT should remain the robust default on full window;
        # smoke60 is necessary-but-not-sufficient — report numbers, do not auto-rank.
        overall_pass = True
    return {
        "overall_pass": overall_pass,
        "completed_backends": len(completed),
        "diagnostics_ok": diagnostics_ok,
        "recommended_default": "NDT_OMP",
        "runs": runs,
        "notes": [
            "Read ok_rate_percent and longest_lost_window_sec before translation_rmse_m.",
            "SMALL_GICP can win smoke60 RMSE yet fail the full 380 s window (see docs/phase1_koide_backend_comparison.md).",
        ],
    }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ndt-dir",
        default="/tmp/lidarloc_koide_outdoor_hard_01a_smoke60",
        help="NDT_OMP smoke60 output directory.")
    parser.add_argument(
        "--small-gicp-dir",
        default="/tmp/lidarloc_koide_01a_smoke60_small_gicp_ds",
        help="SMALL_GICP downsampled smoke60 output directory.")
    parser.add_argument(
        "--small-vgicp-dir",
        default="/tmp/lidarloc_koide_01a_smoke60_small_vgicp_ds",
        help="SMALL_VGICP downsampled smoke60 output directory.")
    parser.add_argument("--output-json", required=True, help="Comparison summary JSON path.")
    args = parser.parse_args(argv)

    comparison = build_comparison({
        "ndt": Path(args.ndt_dir),
        "small_gicp_ds": Path(args.small_gicp_dir),
        "small_vgicp_ds": Path(args.small_vgicp_dir),
    })
    out = Path(args.output_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(comparison, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    print(json.dumps(comparison, indent=2, sort_keys=False))
    return 0 if comparison["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
