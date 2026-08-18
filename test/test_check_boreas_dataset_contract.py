#!/usr/bin/env python3
"""WP4 dataset-contract checker test (data-independent).

Creates a tiny fake Boreas bundle (rosbag2-shaped directory, small PLY map,
reference CSV, initial-pose YAML) and asserts the checker passes it and reports
the recorded map hash, then breaks the bundle and asserts a clean failure.
"""

import csv
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

MAP_PLY = (
    "ply\nformat binary_little_endian 1.0\n"
    "element vertex 4\n"
    "property float x\nproperty float y\nproperty float z\n"
    "end_header\n"
)


def make_fake_bundle(root: Path) -> dict:
    bag = root / "bag"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  version: 5\n")
    map_path = root / "map.ply"
    map_path.write_bytes(MAP_PLY.encode() + bytes.fromhex(
        "000080bf000080bf000080bf000080bf000080bf000080bf"
        "0000803f0000803f0000803f"))
    reference = root / "reference.csv"
    with open(reference, "w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["stamp_sec", "position_x", "position_y", "position_z"])
        writer.writeheader()
        for i in range(61):
            writer.writerow({
                "stamp_sec": float(i),
                "position_x": float(i * 0.5),
                "position_y": 0.0,
                "position_z": 0.0,
            })
    initial = root / "initial.yaml"
    initial.write_text(
        "x: 1.0\ny: 2.0\nz: 0.0\nqx: 0.0\nqy: 0.0\nqz: 0.0\nqw: 1.0\n")
    manifest = root / "manifest.yaml"
    manifest.write_text(
        "name: boreas_wp4_contract\n"
        "dataset:\n"
        f"  bag_path: {bag}\n"
        f"  map_path: {map_path}\n"
        f"  reference_csv: {reference}\n"
        f"  initial_pose_yaml: {initial}\n"
        "evaluation_window_sec: [0.0, 60.0]\n")
    return {
        "bag": bag,
        "map": map_path,
        "reference": reference,
        "initial": initial,
        "manifest": manifest,
    }


def run_checker(manifest: Path, output: Path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT / "scripts")
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "check_boreas_dataset_contract.py"),
         "--manifest", str(manifest), "--output", str(output)],
        env=env,
        capture_output=True,
        text=True,
    )


def test_valid_bundle_passes_and_records_map_hash():
    with tempfile.TemporaryDirectory() as tmp:
        bundle = make_fake_bundle(Path(tmp))
        output = Path(tmp) / "contract.json"
        result = run_checker(bundle["manifest"], output)
        assert result.returncode == 0, result.stderr
        data = json.loads(output.read_text())
        assert data["overall_pass"] is True
        assert data["checks"]["map"]["sha256"]
        assert data["cause_isolation_inputs"]["gt_oracle_crop"] is True


def test_broken_map_fails_cleanly():
    with tempfile.TemporaryDirectory() as tmp:
        bundle = make_fake_bundle(Path(tmp))
        (bundle["map"]).unlink()
        output = Path(tmp) / "contract.json"
        result = run_checker(bundle["manifest"], output)
        assert result.returncode == 1
        assert "map" in result.stdout


if __name__ == "__main__":
    test_valid_bundle_passes_and_records_map_hash()
    test_broken_map_fails_cleanly()
    print("test_check_boreas_dataset_contract: all tests passed")
