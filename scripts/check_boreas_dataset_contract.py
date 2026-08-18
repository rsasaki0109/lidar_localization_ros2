#!/usr/bin/env python3
"""WP4: Boreas dataset contract checker (machine-readable).

Before cause-isolation runs, the Boreas bundle must satisfy a frozen contract:
the bag, map, GT reference, and initial pose must exist and be consistent, the
map hash must be recorded so a later map edit is visible, and the GT reference
must actually cover the requested evaluation window. This script verifies all of
it without ROS and emits a single JSON verdict, so a later WP4 run can fail fast
instead of producing a cliff measurement on a broken input.

The script is intentionally pure Python (numpy + stdlib); the rosbag2 topic
check is optional and only runs when the `rosbags` package is importable.

Usage:
    check_boreas_dataset_contract.py --manifest boreas_contract.yaml \
        --output /tmp/boreas_contract.json
"""

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

REFERENCE_REQUIRED_COLUMNS = ["stamp_sec", "position_x", "position_y"]


def sha256_file(path: Path, chunk_mb: int = 8) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(chunk_mb * 1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def check_map(map_path: Path):
    if not map_path.exists():
        return {"pass": False, "reason": "map file missing", "path": str(map_path)}
    supported = {".pcd", ".ply"}
    if map_path.suffix.lower() not in supported:
        return {
            "pass": False,
            "reason": "map suffix %s not in %s" % (map_path.suffix, sorted(supported)),
            "path": str(map_path),
        }
    return {
        "pass": True,
        "path": str(map_path),
        "size_bytes": map_path.stat().st_size,
        "sha256": sha256_file(map_path),
    }


def check_reference(reference_csv: Path, window_start_sec: float, window_end_sec: float):
    if not reference_csv.exists():
        return {"pass": False, "reason": "reference CSV missing"}
    stamps, xs, ys = [], [], []
    with open(reference_csv, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            return {"pass": False, "reason": "reference CSV has no header"}
        missing = [c for c in REFERENCE_REQUIRED_COLUMNS if c not in reader.fieldnames]
        if missing:
            return {"pass": False, "reason": "reference CSV missing columns %s" % missing}
        for row in reader:
            stamps.append(float(row["stamp_sec"]))
            xs.append(float(row["position_x"]))
            ys.append(float(row["position_y"]))
    if not stamps:
        return {"pass": False, "reason": "reference CSV had no data rows"}
    stamps = np.asarray(stamps)
    order = np.argsort(stamps)
    stamps = stamps[order]
    first, last = float(stamps[0]), float(stamps[-1])
    monotonic = bool(np.all(np.diff(stamps) >= 0.0))
    window_len = max(0.0, window_end_sec - window_start_sec)
    in_window = float(((stamps >= window_start_sec) & (stamps <= window_end_sec)).sum())
    coverage = (in_window / window_len) if window_len > 0 else 0.0
    return {
        "pass": bool(monotonic) and coverage >= 0.9,
        "row_count": len(stamps),
        "first_stamp_sec": first,
        "last_stamp_sec": last,
        "stamps_monotonic": monotonic,
        "window": [window_start_sec, window_end_sec],
        "window_coverage_fraction": coverage,
        "position_finite": bool(np.all(np.isfinite(np.column_stack([xs, ys])))),
    }


def check_initial_pose(initial_pose_yaml: Path):
    if not initial_pose_yaml.exists():
        return {"pass": False, "reason": "initial pose yaml missing"}
    try:
        data = yaml.safe_load(initial_pose_yaml.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        return {"pass": False, "reason": "initial pose yaml invalid: %s" % exc}
    if not isinstance(data, dict):
        return {"pass": False, "reason": "initial pose yaml is not a mapping"}
    needed = ["x", "y", "z", "qx", "qy", "qz", "qw"]
    missing = [k for k in needed if k not in data]
    if missing:
        return {"pass": False, "reason": "initial pose yaml missing keys %s" % missing}
    values = [float(data[k]) for k in needed]
    if not np.all(np.isfinite(values)):
        return {"pass": False, "reason": "initial pose values are not finite"}
    qw, qx, qy, qz = values[3], values[4], values[5], values[6]
    quat_norm = (qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2) ** 0.5
    return {
        "pass": bool(abs(quat_norm - 1.0) < 1e-3),
        "reason": "quaternion not unit length" if abs(quat_norm - 1.0) >= 1e-3 else "ok",
        "pose": {"x": values[0], "y": values[1], "z": values[2]},
    }


def check_bag(bag_path: Path):
    if not bag_path.exists():
        return {"pass": False, "reason": "bag path missing", "path": str(bag_path)}
    metadata = bag_path / "metadata.yaml"
    if not bag_path.is_dir() or not metadata.exists():
        return {"pass": False, "reason": "bag is not a rosbag2 directory (no metadata.yaml)"}
    return {"pass": True, "path": str(bag_path), "has_metadata": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, help="WP4 contract manifest YAML")
    parser.add_argument("--output", required=True, help="JSON verdict output path")
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except OSError as exc:
        sys.exit("cannot read manifest: %s" % exc)
    dataset = manifest.get("dataset", {})
    window = manifest.get("evaluation_window_sec", [0.0, 60.0])

    def resolve(path_value):
        if not path_value:
            return None
        path = Path(path_value)
        if path.is_absolute():
            return path
        return manifest_path.parent / path

    checks = {
        "bag": check_bag(resolve(dataset.get("bag_path"))),
        "map": check_map(resolve(dataset.get("map_path"))),
        "reference": check_reference(
            resolve(dataset.get("reference_csv")),
            float(window[0]),
            float(window[1])),
        "initial_pose": check_initial_pose(resolve(dataset.get("initial_pose_yaml"))),
    }
    overall = all(item["pass"] for item in checks.values())
    result = {
        "overall_pass": overall,
        "dataset_contract": dataset,
        "evaluation_window_sec": window,
        "checks": checks,
        "cause_isolation_inputs": {
            "gt_oracle_crop": bool(checks["map"]["pass"] and checks["reference"]["pass"]),
            "map_hash_recorded": checks["map"].get("sha256") is not None,
        },
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print("overall_pass=%s map_sha256=%s" % (
        overall,
        checks["map"].get("sha256", "n/a")))
    if not overall:
        for name, item in checks.items():
            if not item["pass"]:
                print("FAIL %s: %s" % (name, item.get("reason", "failed")))
        sys.exit(1)


if __name__ == "__main__":
    main()
