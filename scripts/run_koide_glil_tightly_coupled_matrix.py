#!/usr/bin/env python3
"""Run the pinned tightly coupled GLIL four-sequence acceptance matrix."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Sequence:
    duration_sec: float
    center: tuple[float, float, float]
    yaw_deg: float
    reference: str
    map_name: str


SEQUENCES = {
    "outdoor_hard_01a": Sequence(
        380.0, (-86.040205, -8.857126, -11.043077), -82.0063,
        "benchmark/outdoor_hard_01a/reference.csv", "map_outdoor_hard.ply"),
    "outdoor_hard_01b": Sequence(
        302.0, (103.703859, -3.360581, -11.276523), -3.0042,
        "benchmark/outdoor_hard_01b/reference.csv", "map_outdoor_hard.ply"),
    "outdoor_hard_02a": Sequence(
        363.0, (-104.343538, -10.324673, -11.620099), 163.3613,
        "benchmark/outdoor_hard_02a/reference.csv", "map_outdoor_hard.ply"),
    "outdoor_hard_02b": Sequence(
        298.0, (103.616685, -1.693832, -11.022091), 4.4813,
        "benchmark/outdoor_hard_02b/reference.csv", "map_outdoor_hard.ply"),
    "outdoor_kidnap_a": Sequence(
        203.0, (-91.067991, -8.690833, -11.279882), -78.8705,
        "generated/localization_gif_benchmarks/assets/outdoor_kidnap_a_reference.csv",
        "map_outdoor_kidnap.ply"),
    "outdoor_kidnap_b": Sequence(
        349.0, (59.897016, 66.271040, -11.501114), 104.1564,
        "generated/localization_gif_benchmarks/assets/outdoor_kidnap_b_reference.csv",
        "map_outdoor_kidnap.ply"),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument(
        "--sequences", nargs="+", choices=tuple(SEQUENCES),
        default=list(SEQUENCES),
    )
    parser.add_argument(
        "--image", default="lidarloc/glim-ros2:jazzy-v1.2.2-tightly-coupled")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--ros-domain-id", type=int, default=100)
    parser.add_argument(
        "--recovery-occupancy-yaml", type=Path,
        help="Enable the GT-free BBS recovery sidecars for every sequence.",
    )
    parser.add_argument(
        "--recovery-bbs-extension", type=Path,
        help="Optional bbs_cpp shared library passed to the benchmark runner.",
    )
    parser.add_argument("--fail-fast", action="store_true")
    args = parser.parse_args()

    if args.threads < 1:
        parser.error("--threads must be positive")
    if not 0 <= args.ros_domain_id <= 232 - len(args.sequences):
        parser.error("ROS domain range would exceed Fast DDS's safe domain limit")
    data_dir = args.data_dir.resolve()
    output_root = args.output_root.resolve()
    recovery_occupancy = (
        args.recovery_occupancy_yaml.resolve()
        if args.recovery_occupancy_yaml is not None else None
    )
    recovery_bbs_extension = (
        args.recovery_bbs_extension.resolve()
        if args.recovery_bbs_extension is not None else None
    )
    if recovery_occupancy is not None and not recovery_occupancy.is_file():
        parser.error(f"recovery occupancy YAML does not exist: {recovery_occupancy}")
    if recovery_bbs_extension is not None and not recovery_bbs_extension.is_file():
        parser.error(f"recovery BBS extension does not exist: {recovery_bbs_extension}")
    if recovery_bbs_extension is not None and recovery_occupancy is None:
        parser.error("--recovery-bbs-extension requires --recovery-occupancy-yaml")
    if output_root.exists() and any(output_root.iterdir()):
        parser.error(f"output root must be new or empty: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)

    repo = Path(__file__).resolve().parents[1]
    runner = repo / "scripts/run_koide_glim_odometry_benchmark.py"
    manifest = {
        "image": args.image,
        "threads": args.threads,
        "data_dir": str(data_dir),
        "output_root": str(output_root),
        "recovery_occupancy_yaml": (
            str(recovery_occupancy) if recovery_occupancy is not None else None),
        "recovery_bbs_extension": (
            str(recovery_bbs_extension) if recovery_bbs_extension is not None else None),
        "runs": [],
    }
    failed = False
    for offset, name in enumerate(args.sequences):
        sequence = SEQUENCES[name]
        output = output_root / name
        command = [
            sys.executable, str(runner),
            "--bag", str(data_dir / "sequences" / name),
            "--reference", str(data_dir / sequence.reference),
            "--output", str(output),
            "--requested-duration-sec", str(sequence.duration_sec),
            "--image", args.image,
            "--prior-map", str(data_dir / sequence.map_name),
            "--prior-map-bootstrap-center", *(str(value) for value in sequence.center),
            "--prior-map-bootstrap-yaw-deg", str(sequence.yaw_deg),
            "--prior-map-tightly-coupled",
            "--tightly-coupled-num-threads", str(args.threads),
            "--ros-domain-id", str(args.ros_domain_id + offset),
        ]
        if recovery_occupancy is not None:
            command.extend(["--recovery-occupancy-yaml", str(recovery_occupancy)])
        if recovery_bbs_extension is not None:
            command.extend(["--recovery-bbs-extension", str(recovery_bbs_extension)])
        if name.startswith("outdoor_kidnap_"):
            command.append("--evaluate-kidnap-recovery")
        print("+", " ".join(command), flush=True)
        started = time.time()
        result = subprocess.run(command)
        manifest["runs"].append({
            "sequence": name,
            "command": command,
            "return_code": result.returncode,
            "started_unix_sec": started,
            "elapsed_sec": time.time() - started,
            "output": str(output),
        })
        (output_root / "matrix_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        failed = failed or result.returncode != 0
        if failed and args.fail_fast:
            break

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
