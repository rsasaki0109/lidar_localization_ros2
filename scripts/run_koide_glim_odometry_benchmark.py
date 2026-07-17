#!/usr/bin/env python3
"""Run the fixed GLIM Koide outdoor_hard odometry benchmark and all acceptance checks."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_IMAGE = "lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded"
EXPECTED_IMAGE_ID = "sha256:1002a7bc67d26f5605173f33784e08690c0c45f2c332853db18013d3967e6f17"


def run_checked(command):
    print("+", " ".join(str(value) for value in command), flush=True)
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="outdoor_hard rosbag2 directory")
    parser.add_argument("--reference", required=True, help="benchmark reference.csv")
    parser.add_argument("--output", required=True, help="new or empty output directory")
    parser.add_argument(
        "--requested-duration-sec", type=float, default=380.0,
        help="Expected covered duration forwarded to the completion checker "
        "(reference start to bag end; default keeps the outdoor_hard_01a gate).")
    parser.add_argument("--image", default=os.environ.get("GLIM_IMAGE", DEFAULT_IMAGE))
    parser.add_argument("--ros-domain-id", type=int, default=91)
    parser.add_argument(
        "--allow-image-mismatch", action="store_true",
        help="Allow a rebuilt image whose local ID differs from the archived validation image.")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    bag = Path(args.bag).resolve()
    reference = Path(args.reference).resolve()
    output = Path(args.output).resolve()
    config = repo / "param/odometry/glim_koide_outdoor_gicp6500"
    if not (bag / "metadata.yaml").is_file():
        parser.error(f"not a rosbag2 directory: {bag}")
    if not reference.is_file():
        parser.error(f"reference CSV does not exist: {reference}")
    if output.exists() and any(output.iterdir()):
        parser.error(f"output directory must be empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    image_id = subprocess.check_output(
        ["docker", "image", "inspect", args.image, "--format", "{{.Id}}"], text=True
    ).strip()
    if image_id != EXPECTED_IMAGE_ID and not args.allow_image_mismatch:
        parser.error(
            f"image ID is {image_id}, expected {EXPECTED_IMAGE_ID}; "
            "rebuild from experiments/koide_odometry_glim_gicp6500 or pass "
            "--allow-image-mismatch and archive the new ID")

    container_bag = f"/data/{bag.name}"
    docker_command = [
        "docker", "run", "--rm", "--network=host", "--ipc=host",
        "-e", f"ROS_DOMAIN_ID={args.ros_domain_id}",
        "-v", f"{bag.parent}:/data:ro",
        "-v", f"{config}:/glim/config:ro",
        "-v", f"{output}:/tmp",
        args.image,
        "bash", "-lc",
        ". /opt/ros/jazzy/setup.bash && . /root/ros2_ws/install/setup.bash && "
        f"ros2 run glim_ros glim_rosbag {container_bag} --ros-args "
        "-p config_path:=/glim/config -p auto_quit:=true",
    ]
    started = time.monotonic()
    with (output / "glim.log").open("w", encoding="utf-8") as log:
        process = subprocess.run(
            docker_command, stdout=log, stderr=subprocess.STDOUT, text=True)
    wall_duration = time.monotonic() - started
    process_result = {"return_code": process.returncode, "wall_duration_sec": wall_duration}
    (output / "run_process.json").write_text(
        json.dumps(process_result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    summary = {
        "target_process_died_during_run": process.returncode != 0,
        "bag_stopped_by_runner": False,
        "return_codes": {"bag_play": process.returncode},
        "wall_duration_sec": wall_duration,
        "image": args.image,
        "image_id": image_id,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if process.returncode:
        print(f"GLIM failed; inspect {output / 'glim.log'}", file=sys.stderr)
        return process.returncode

    python = sys.executable
    run_checked([
        python, str(repo / "scripts/convert_glim_dump_to_pose_csv.py"),
        "--dump-dir", str(output / "dump"),
        "--output-csv", str(output / "pose_trace.csv"),
        "--summary-json", str(output / "conversion.json"),
        "--apply-global-correction", "--planarize-z",
    ])
    run_checked([
        python, str(repo / "scripts/build_glim_runtime_evidence.py"),
        "--glim-log", str(output / "glim.log"),
        "--pose-csv", str(output / "pose_trace.csv"),
        "--output-json", str(output / "runtime.json"),
    ])
    run_checked([
        python, str(repo / "scripts/check_koide_odometry_completion.py"),
        "--estimated-csv", str(output / "pose_trace.csv"),
        "--reference-csv", str(reference),
        "--runtime-json", str(output / "runtime.json"),
        "--run-summary-json", str(output / "summary.json"),
        "--output-json", str(output / "odometry_completion.json"),
        "--requested-duration-sec", str(args.requested_duration_sec),
    ])
    print(f"PASS: {output / 'odometry_completion.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
