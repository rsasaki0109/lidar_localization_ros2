#!/usr/bin/env python3
"""Run the fixed GLIM Koide outdoor_hard odometry benchmark and all acceptance checks."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_IMAGE = "lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded"
EXPECTED_IMAGE_IDS = {
    DEFAULT_IMAGE:
        "sha256:1002a7bc67d26f5605173f33784e08690c0c45f2c332853db18013d3967e6f17",
    "lidarloc/glim-ros2:jazzy-v1.2.2-live-map-odom":
        "sha256:70d056317a475e8ce5a029e5b6d83d0b5e8f4e1587acd069d708eb00cd1b134a",
}


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
    parser.add_argument(
        "--config",
        help="GLIM config directory (default: param/odometry/glim_koide_outdoor_gicp6500).",
    )
    parser.add_argument(
        "--prior-map",
        help="Enable libglim_prior_map_localizer.so with this PLY prior map.",
    )
    parser.add_argument(
        "--prior-map-bootstrap-center", type=float, nargs=3, metavar=("X", "Y", "Z"),
        help="Known approximate initial position in the prior-map frame; crop seed only.",
    )
    parser.add_argument(
        "--prior-map-bootstrap-yaw-deg", type=float,
        help="Known approximate initial yaw in the prior-map frame; requires bootstrap center.",
    )
    parser.add_argument("--ros-domain-id", type=int, default=91)
    parser.add_argument(
        "--allow-image-mismatch", action="store_true",
        help="Allow an unregistered image or a rebuilt image whose ID differs from its archive.")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    bag = Path(args.bag).resolve()
    reference = Path(args.reference).resolve()
    output = Path(args.output).resolve()
    config = Path(args.config).resolve() if args.config else (
        repo / "param/odometry/glim_koide_outdoor_gicp6500"
    )
    if not (bag / "metadata.yaml").is_file():
        parser.error(f"not a rosbag2 directory: {bag}")
    if not reference.is_file():
        parser.error(f"reference CSV does not exist: {reference}")
    if output.exists() and any(output.iterdir()):
        parser.error(f"output directory must be empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    prior_map = Path(args.prior_map).resolve() if args.prior_map else None
    if prior_map is not None:
        if prior_map.suffix.lower() != ".ply" or not prior_map.is_file():
            parser.error(f"prior map must be an existing PLY file: {prior_map}")
        generated_config = output / "glim_config"
        shutil.copytree(config, generated_config)
        config_ros_path = generated_config / "config_ros.json"
        config_ros = json.loads(config_ros_path.read_text(encoding="utf-8"))
        modules = config_ros["glim_ros"].setdefault("extension_modules", [])
        if "libglim_prior_map_localizer.so" not in modules:
            modules.append("libglim_prior_map_localizer.so")
        config_ros_path.write_text(
            json.dumps(config_ros, indent=2) + "\n", encoding="utf-8")
        config = generated_config
    elif (args.prior_map_bootstrap_center is not None or
          args.prior_map_bootstrap_yaw_deg is not None):
        parser.error("--prior-map-bootstrap-center requires --prior-map")
    if ((args.prior_map_bootstrap_center is None) !=
            (args.prior_map_bootstrap_yaw_deg is None)):
        parser.error("bootstrap center and bootstrap yaw must be provided together")

    image_id = subprocess.check_output(
        ["docker", "image", "inspect", args.image, "--format", "{{.Id}}"], text=True
    ).strip()
    expected_image_id = EXPECTED_IMAGE_IDS.get(args.image)
    if expected_image_id is None and not args.allow_image_mismatch:
        parser.error(
            f"image tag {args.image} has no archived validation ID; pass "
            "--allow-image-mismatch and archive the new tag and ID")
    if (expected_image_id is not None and image_id != expected_image_id and
            not args.allow_image_mismatch):
        parser.error(
            f"image ID is {image_id}, expected {expected_image_id} for {args.image}; "
            "rebuild the archived image or pass "
            "--allow-image-mismatch and archive the new ID")

    container_bag = f"/data/{bag.name}"
    docker_command = [
        "docker", "run", "--rm", "--network=host", "--ipc=host",
        "-e", f"ROS_DOMAIN_ID={args.ros_domain_id}",
        "-v", f"{bag.parent}:/data:ro",
        "-v", f"{config}:/glim/config:ro",
        "-v", f"{output}:/tmp",
    ]
    if prior_map is not None:
        docker_command.extend([
            "-e", "GLIM_PRIOR_MAP_PATH=/prior_map.ply",
            "-v", f"{prior_map}:/prior_map.ply:ro",
        ])
        if args.prior_map_bootstrap_center is not None:
            for axis, value in zip("XYZ", args.prior_map_bootstrap_center):
                docker_command.extend([
                    "-e", f"GLIM_PRIOR_MAP_BOOTSTRAP_CENTER_{axis}={value}",
                ])
            docker_command.extend([
                "-e", f"GLIM_PRIOR_MAP_BOOTSTRAP_YAW_DEG={args.prior_map_bootstrap_yaw_deg}",
            ])
    docker_command.extend([
        args.image,
        "bash", "-lc",
        ". /opt/ros/jazzy/setup.bash && . /root/ros2_ws/install/setup.bash && "
        f"ros2 run glim_ros glim_rosbag {container_bag} --ros-args "
        "-p config_path:=/glim/config -p auto_quit:=true",
    ])
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
        "prior_map": str(prior_map) if prior_map is not None else None,
        "config": str(config),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if process.returncode:
        print(f"GLIM failed; inspect {output / 'glim.log'}", file=sys.stderr)
        return process.returncode

    python = sys.executable
    conversion_command = [
        python, str(repo / "scripts/convert_glim_dump_to_pose_csv.py"),
        "--dump-dir", str(output / "dump"),
        "--output-csv", str(output / "pose_trace.csv"),
        "--summary-json", str(output / "conversion.json"),
        "--planarize-z",
    ]
    if prior_map is not None:
        live_map_odom = output / "dump" / "external_map_odom.txt"
        if not live_map_odom.is_file():
            print(
                f"GLIM did not save its live external correction history: {live_map_odom}",
                file=sys.stderr,
            )
            return 2
        conversion_command.extend(["--map-odom-tum", str(live_map_odom)])
    else:
        conversion_command.append("--apply-global-correction")
    run_checked(conversion_command)
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
