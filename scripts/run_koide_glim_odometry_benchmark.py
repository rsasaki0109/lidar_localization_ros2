#!/usr/bin/env python3
"""Run the fixed GLIM Koide outdoor_hard odometry benchmark and all acceptance checks."""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
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
    "lidarloc/glim-ros2:jazzy-v1.2.2-tightly-coupled":
        "sha256:b1c945ea0a1df5c7922e67d5648d35c24b605fe54559cff521e8d94a7d18b8c4",
}

SIZE_MULTIPLIERS = {
    "B": 1.0,
    "KB": 1000.0,
    "MB": 1000.0 ** 2,
    "GB": 1000.0 ** 3,
    "KIB": 1024.0,
    "MIB": 1024.0 ** 2,
    "GIB": 1024.0 ** 3,
}

INDOOR_AZURE_KINECT_T_LIDAR_IMU = [
    0.003463566434548747,
    0.0041740033449125195,
    -0.05071645628165228,
    -0.47551890747667463,
    0.4736570557695826,
    0.5236230425615598,
    0.5247377168171308,
]


def run_checked(command):
    print("+", " ".join(str(value) for value in command), flush=True)
    subprocess.run(command, check=True)


def parse_size_bytes(value: str) -> int:
    """Parse the used-memory half of a Docker stats MemUsage value."""
    used = value.split("/", 1)[0].strip()
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)\s*([A-Za-z]+)", used)
    if match is None:
        raise ValueError(f"unsupported Docker memory value: {value!r}")
    unit = match.group(2).upper()
    if unit not in SIZE_MULTIPLIERS:
        raise ValueError(f"unsupported Docker memory unit: {unit}")
    return int(float(match.group(1)) * SIZE_MULTIPLIERS[unit])


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def container_rss_bytes(container_name: str) -> int:
    """Sum VmRSS for every host PID currently belonging to the container."""
    output = subprocess.check_output(
        ["docker", "top", container_name, "-eo", "pid"],
        text=True,
        stderr=subprocess.DEVNULL,
        timeout=5.0,
    )
    total_kib = 0
    for value in output.splitlines()[1:]:
        value = value.strip()
        if not value.isdigit():
            continue
        try:
            status = Path(f"/proc/{value}/status").read_text(encoding="utf-8")
        except (FileNotFoundError, PermissionError):
            continue
        match = re.search(r"^VmRSS:\s+([0-9]+)\s+kB$", status, re.MULTILINE)
        if match is not None:
            total_kib += int(match.group(1))
    return total_kib * 1024


def sample_container_resources(container_name: str, started: float):
    try:
        output = subprocess.check_output(
            [
                "docker", "stats", "--no-stream", "--format", "{{json .}}",
                container_name,
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        ).strip()
        if not output:
            return None
        stats = json.loads(output.splitlines()[-1])
        return {
            "elapsed_sec": time.monotonic() - started,
            "cpu_percent": float(stats["CPUPerc"].rstrip("%")),
            "memory_bytes": parse_size_bytes(stats["MemUsage"]),
            "rss_bytes": container_rss_bytes(container_name),
        }
    except (subprocess.SubprocessError, KeyError, ValueError, json.JSONDecodeError):
        return None


def write_resource_evidence(output: Path, samples: list[dict]) -> dict:
    trace_path = output / "resource_trace.csv"
    with trace_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("elapsed_sec", "cpu_percent", "memory_bytes", "rss_bytes"),
        )
        writer.writeheader()
        writer.writerows(samples)

    cpu = [sample["cpu_percent"] for sample in samples]
    memory = [sample["memory_bytes"] for sample in samples]
    rss = [sample["rss_bytes"] for sample in samples]
    summary = {
        "sample_count": len(samples),
        "sampling_source": "docker stats --no-stream",
        "cpu_percent_mean": sum(cpu) / len(cpu) if cpu else 0.0,
        "cpu_percent_p95": percentile(cpu, 0.95),
        "cpu_percent_max": max(cpu, default=0.0),
        "memory_peak_bytes": max(memory, default=0),
        "rss_peak_bytes": max(rss, default=0),
        "trace_csv": str(trace_path),
    }
    (output / "resource_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def apply_sensor_profile(config_dir: Path, profile: str) -> None:
    if profile == "outdoor_livox":
        return
    if profile != "indoor_azure_kinect":
        raise ValueError(f"unsupported sensor profile: {profile}")

    ros_path = config_dir / "config_ros.json"
    ros_config = json.loads(ros_path.read_text(encoding="utf-8"))
    ros = ros_config["glim_ros"]
    ros.update({
        "imu_topic": "/imu",
        "points_topic": "/points2/decompressed",
        "acc_scale": 0.0,
        "base_frame_id": "depth_camera_link",
        "publish_imu2lidar": True,
    })
    ros_path.write_text(json.dumps(ros_config, indent=2) + "\n", encoding="utf-8")

    sensors_path = config_dir / "config_sensors.json"
    sensors_config = json.loads(sensors_path.read_text(encoding="utf-8"))
    sensors = sensors_config["sensors"]
    sensors.update({
        "global_shutter_lidar": True,
        "T_lidar_imu": INDOOR_AZURE_KINECT_T_LIDAR_IMU,
        "intensity_field": "",
        "ring_field": "",
        "autoconf_perpoint_times": False,
    })
    sensors_path.write_text(
        json.dumps(sensors_config, indent=2) + "\n", encoding="utf-8")

    preprocess_path = config_dir / "config_preprocess.json"
    preprocess_config = json.loads(preprocess_path.read_text(encoding="utf-8"))
    preprocess_config["preprocess"].update({
        "distance_near_thresh": 0.2,
        "distance_far_thresh": 30.0,
        "downsample_resolution": 0.25,
    })
    preprocess_path.write_text(
        json.dumps(preprocess_config, indent=2) + "\n", encoding="utf-8")

    odometry_path = config_dir / "config_odometry_cpu.json"
    odometry_config = json.loads(odometry_path.read_text(encoding="utf-8"))
    odometry_config["odometry_estimation"]["ivox_resolution"] = 0.5
    odometry_path.write_text(
        json.dumps(odometry_config, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="outdoor_hard rosbag2 directory")
    parser.add_argument("--reference", required=True, help="benchmark reference.csv")
    parser.add_argument("--output", required=True, help="new or empty output directory")
    parser.add_argument(
        "--requested-duration-sec", type=float, default=380.0,
        help="Expected covered duration forwarded to the completion checker "
        "(reference start to bag end; default keeps the outdoor_hard_01a gate).")
    parser.add_argument(
        "--playback-duration-sec", type=float,
        help="Stop rosbag playback after this many seconds (for focused smoke runs).",
    )
    parser.add_argument("--image", default=os.environ.get("GLIM_IMAGE", DEFAULT_IMAGE))
    parser.add_argument(
        "--config",
        help="GLIM config directory (default: param/odometry/glim_koide_outdoor_gicp6500).",
    )
    parser.add_argument(
        "--sensor-profile", choices=("outdoor_livox", "indoor_azure_kinect"),
        default="outdoor_livox",
        help="Dataset sensor topics, extrinsic, and preprocessing profile.",
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
    parser.add_argument(
        "--prior-map-vertical-gain", type=float,
        help="Optional map-to-odom Z correction gain in [0, 1].",
    )
    parser.add_argument(
        "--prior-map-tightly-coupled", action="store_true",
        help="Insert exact-coreset scan-to-prior-map factors into GLIM's fixed-lag "
        "graph instead of producing an external map-to-odom correction.",
    )
    parser.add_argument(
        "--tightly-coupled-num-threads", type=int, default=8,
        help="CPU threads for tightly coupled scan and prior-map factors (default: 8).",
    )
    parser.add_argument(
        "--inject-initial-pose", type=float, nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Publish one reliable /initialpose candidate after the extension subscribes; "
        "intended for repeatable recovery integration tests.",
    )
    parser.add_argument(
        "--initial-pose-delay-sec", type=float, default=0.0,
        help="Wall-clock delay before waiting for the /initialpose subscriber (default: 0).",
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
    if prior_map is not None or args.sensor_profile != "outdoor_livox":
        generated_config = output / "glim_config"
        shutil.copytree(config, generated_config)
        apply_sensor_profile(generated_config, args.sensor_profile)
        if prior_map is not None:
            config_ros_path = generated_config / "config_ros.json"
            config_ros = json.loads(config_ros_path.read_text(encoding="utf-8"))
            modules = config_ros["glim_ros"].setdefault("extension_modules", [])
            if "libglim_prior_map_localizer.so" not in modules:
                modules.append("libglim_prior_map_localizer.so")
            config_ros_path.write_text(
                json.dumps(config_ros, indent=2) + "\n", encoding="utf-8")
            if args.prior_map_tightly_coupled:
                config_odometry_path = generated_config / "config_odometry_cpu.json"
                config_odometry = json.loads(
                    config_odometry_path.read_text(encoding="utf-8"))
                odometry = config_odometry["odometry_estimation"]
                odometry["use_gicp_coreset"] = True
                odometry["use_tightly_coupled_coreset"] = True
                odometry["full_connection_window_size"] = 3
                odometry["num_threads"] = args.tightly_coupled_num_threads
                odometry["coreset_reuse_tolerance_trans"] = 0.25
                odometry["coreset_reuse_tolerance_rot"] = 0.035
                config_odometry_path.write_text(
                    json.dumps(config_odometry, indent=2) + "\n", encoding="utf-8")
        config = generated_config
    if prior_map is None and (args.prior_map_bootstrap_center is not None or
                              args.prior_map_bootstrap_yaw_deg is not None):
        parser.error("--prior-map-bootstrap-center requires --prior-map")
    if ((args.prior_map_bootstrap_center is None) !=
            (args.prior_map_bootstrap_yaw_deg is None)):
        parser.error("bootstrap center and bootstrap yaw must be provided together")
    if args.prior_map_tightly_coupled and prior_map is None:
        parser.error("--prior-map-tightly-coupled requires --prior-map")
    if args.prior_map_tightly_coupled and args.prior_map_bootstrap_center is None:
        parser.error(
            "--prior-map-tightly-coupled currently requires the bootstrap center and yaw")
    if args.prior_map_tightly_coupled and args.prior_map_vertical_gain is not None:
        parser.error(
            "--prior-map-vertical-gain applies only to external map-to-odom mode")
    if args.tightly_coupled_num_threads < 1:
        parser.error("--tightly-coupled-num-threads must be positive")
    if args.initial_pose_delay_sec < 0.0:
        parser.error("--initial-pose-delay-sec must be non-negative")
    if args.inject_initial_pose is not None and prior_map is None:
        parser.error("--inject-initial-pose requires --prior-map")
    if args.playback_duration_sec is not None and args.playback_duration_sec <= 0.0:
        parser.error("--playback-duration-sec must be positive")
    if (args.prior_map_vertical_gain is not None and not (
            0.0 <= args.prior_map_vertical_gain <= 1.0)):
        parser.error("--prior-map-vertical-gain must be in [0, 1]")

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
    container_name = f"glil-koide-{os.getpid()}-{args.ros_domain_id}"
    docker_command = [
        "docker", "run", "--rm", "--network=host", "--ipc=host",
        "--name", container_name,
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
        if args.prior_map_vertical_gain is not None:
            docker_command.extend([
                "-e", f"GLIM_PRIOR_MAP_VERTICAL_GAIN={args.prior_map_vertical_gain}",
            ])
        if args.prior_map_tightly_coupled:
            docker_command.extend([
                "-e", "GLIM_PRIOR_MAP_TIGHTLY_COUPLED=true",
                "-e", (
                    "GLIM_PRIOR_MAP_FACTOR_NUM_THREADS="
                    f"{args.tightly_coupled_num_threads}"
                ),
            ])
    playback_duration_parameter = (
        f" -p playback_duration:={args.playback_duration_sec}"
        if args.playback_duration_sec is not None else ""
    )
    run_command = (
        f"ros2 run glim_ros glim_rosbag {container_bag} --ros-args "
        f"-p config_path:=/glim/config -p auto_quit:=true{playback_duration_parameter}"
    )
    if args.inject_initial_pose is not None:
        x, y, z, qx, qy, qz, qw = args.inject_initial_pose
        pose_message = (
            "{header: {frame_id: map}, pose: {pose: {position: "
            f"{{x: {x:.17g}, y: {y:.17g}, z: {z:.17g}}}, orientation: "
            f"{{x: {qx:.17g}, y: {qy:.17g}, z: {qz:.17g}, w: {qw:.17g}}}}}}}}}"
        )
        publisher_command = (
            f"sleep {args.initial_pose_delay_sec:.17g}; "
            "ros2 topic pub --once --wait-matching-subscriptions 1 /initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped '{pose_message}'"
        )
        run_command = (
            f"({publisher_command}) > /tmp/injected_initial_pose.log 2>&1 & "
            "injection_pid=$!; "
            f"{run_command}; run_rc=$?; "
            "wait \"$injection_pid\" || true; exit \"$run_rc\""
        )
    docker_command.extend([
        args.image,
        "bash", "-lc",
        ". /opt/ros/jazzy/setup.bash && . /root/ros2_ws/install/setup.bash && "
        f"{run_command}",
    ])
    started = time.monotonic()
    resource_samples = []
    with (output / "glim.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            docker_command, stdout=log, stderr=subprocess.STDOUT, text=True)
        while process.poll() is None:
            sample = sample_container_resources(container_name, started)
            if sample is not None:
                resource_samples.append(sample)
            if process.poll() is None:
                time.sleep(1.0)
        return_code = process.wait()
    wall_duration = time.monotonic() - started
    resource_summary = write_resource_evidence(output, resource_samples)
    process_result = {"return_code": return_code, "wall_duration_sec": wall_duration}
    (output / "run_process.json").write_text(
        json.dumps(process_result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    summary = {
        "target_process_died_during_run": return_code != 0,
        "bag_stopped_by_runner": False,
        "return_codes": {"bag_play": return_code},
        "wall_duration_sec": wall_duration,
        "playback_duration_sec": args.playback_duration_sec,
        "image": args.image,
        "image_id": image_id,
        "prior_map": str(prior_map) if prior_map is not None else None,
        "prior_map_tightly_coupled": args.prior_map_tightly_coupled,
        "tightly_coupled_num_threads": args.tightly_coupled_num_threads,
        "injected_initial_pose": args.inject_initial_pose,
        "initial_pose_delay_sec": args.initial_pose_delay_sec,
        "sensor_profile": args.sensor_profile,
        "resource_summary": resource_summary,
        "config": str(config),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if return_code:
        print(f"GLIM failed; inspect {output / 'glim.log'}", file=sys.stderr)
        return return_code

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
