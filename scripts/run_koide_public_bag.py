#!/usr/bin/env python3
"""Prepare and evaluate a complete online Koide outdoor_hard_02b replay."""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import zipfile
from pathlib import Path

import koide_public_bag as data_tool


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--data-dir",
        type=Path,
        default=Path("koide-data"),
        help="official download and SI bag cache (default: ./koide-data)",
    )
    result.add_argument(
        "--output",
        type=Path,
        help="new directory for logs, estimate.tum and summary.json",
    )
    result.add_argument(
        "--download",
        action="store_true",
        help="download missing official assets (~1.3 GB); verify published checksums",
    )
    result.add_argument(
        "--ros-domain-id",
        type=int,
        default=83,
        help="isolated ROS domain for this replay (default: 83)",
    )
    result.add_argument(
        "--rko-executable",
        type=Path,
        help="RKO online_node path; otherwise use the sourced rko_lio package",
    )
    result.add_argument(
        "--dry-run",
        action="store_true",
        help="check installed executables and print the plan without downloading or starting nodes",
    )
    return result


def installed_paths(args):
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_prefix,
        get_package_share_directory,
    )

    try:
        prefix = Path(get_package_prefix("lidar_localization_ros2"))
    except PackageNotFoundError as exc:
        raise ValueError(
            "lidar_localization_ros2 is not sourced; build and source its ROS workspace"
        ) from exc
    localizer = prefix / "lib/lidar_localization_ros2/lidar_localization_node"
    if args.rko_executable:
        rko = args.rko_executable.expanduser().resolve()
    else:
        try:
            rko = Path(get_package_prefix("rko_lio")) / "lib/rko_lio/online_node"
        except PackageNotFoundError as exc:
            raise ValueError(
                "rko_lio is not sourced. Build/source lidar_slam_ros2 or pass --rko-executable /path/to/online_node."
            ) from exc
    for binary in (localizer, rko):
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError(
                f"executable not found: {binary}; build and source its ROS workspace"
            )
    profiles = (
        Path(get_package_share_directory("lidar_localization_ros2")) / "public_bag"
    )
    if not profiles.is_dir():
        raise ValueError(
            "public-bag presets are not installed; rebuild and source lidar_localization_ros2"
        )
    return localizer, rko, profiles


def stop_process(process):
    if process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def run_replay(args, output, bag, map_path, reference_path, paths):
    import numpy as np
    import rclpy
    import yaml
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from start_lifecycle_node import activate

    localizer, rko, profiles = paths
    start_ns, clouds = data_tool.bag_cloud_times(bag)
    initialization_ns = start_ns + 4 * 10**9
    expected = {stamp for stamp in clouds if stamp >= initialization_ns}
    reference = np.loadtxt(reference_path)
    if (
        reference.ndim != 2
        or reference.shape[1] != 8
        or not np.isfinite(reference).all()
    ):
        raise ValueError(f"invalid TUM reference: {reference_path}")
    row = reference[np.argmin(abs(reference[:, 0] - initialization_ns * 1e-9))]
    if abs(row[0] - initialization_ns * 1e-9) > 0.05:
        raise ValueError("ground truth has no initial pose within 0.05 s of bag + 4 s")
    config = yaml.safe_load(
        (profiles / "koide_outdoor_hard_02b_online.yaml").read_text()
    )
    config["/**"]["ros__parameters"]["map_path"] = str(map_path)
    config_path = output / "localizer.yaml"
    config_path.write_text(yaml.safe_dump(config))
    rko_config = output / "rko.yaml"
    qos_config = output / "points-qos.yaml"
    shutil.copyfile(profiles / "koide_outdoor_hard_02b_online_rko.yaml", rko_config)
    shutil.copyfile(profiles / "koide_reliable_points_qos.yaml", qos_config)
    os.environ["ROS_LOG_DIR"] = str(output / "ros-logs")
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(output / "ros-logs")
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [
            str(rko.parent.parent),
            str(localizer.parent.parent),
            environment.get("LD_LIBRARY_PATH", ""),
        ]
    )
    receipt = {
        "source": data_tool.SOURCE,
        "sequence": "outdoor_hard_02b",
        "full_sequence": True,
        "rate": 1.0,
        "initial_pose_offset_sec": 4.0,
        "bag": str(bag),
        "map": str(map_path),
        "ros_domain_id": args.ros_domain_id,
        "localizer_sha256": data_tool.checksum(localizer, "sha256"),
        "rko_sha256": data_tool.checksum(rko, "sha256"),
        "configuration_sha256": {
            path.name: data_tool.checksum(path, "sha256")
            for path in (config_path, rko_config, qos_config)
        },
        "input_preparation": json.loads((bag / "preparation.json").read_text()),
        "map_sha256": data_tool.checksum(map_path, "sha256"),
        "reference_sha256": data_tool.checksum(reference_path, "sha256"),
        "status": "running",
    }
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    poses, diagnostics, clock_ns = [], set(), [0]
    processes = []
    rclpy.init(args=[])
    node = rclpy.create_node(f"koide_public_bag_control_{os.getpid()}")
    try:
        # An occupied domain can inject poses/TF and invalidate this benchmark.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if set(node.get_node_names()) - {node.get_name()}:
            raise ValueError(
                "ROS domain is already in use; retry with another --ros-domain-id"
            )
        with contextlib.ExitStack() as stack:
            tum = stack.enter_context((output / "estimate.tum").open("w"))
            diagnostic_log = stack.enter_context((output / "alignment.jsonl").open("w"))

            def pose_received(message):
                stamp = data_tool.stamp_ns(message.header.stamp)
                if stamp < initialization_ns:
                    return
                p, q = message.pose.pose.position, message.pose.pose.orientation
                values = [stamp * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if not all(np.isfinite(values)):
                    raise ValueError("non-finite pose received")
                poses.append(values)
                tum.write(" ".join(f"{value:.9f}" for value in values) + "\n")

            def diagnostic_received(message):
                stamp = data_tool.stamp_ns(message.header.stamp)
                diagnostics.add(stamp)
                for entry in message.status:
                    diagnostic_log.write(
                        json.dumps(
                            {
                                "stamp_ns": stamp,
                                "message": entry.message,
                                "values": {v.key: v.value for v in entry.values},
                            }
                        )
                        + "\n"
                    )

            node.create_subscription(
                Clock,
                "/clock",
                lambda message: clock_ns.__setitem__(
                    0, data_tool.stamp_ns(message.clock)
                ),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            node.create_subscription(
                PoseWithCovarianceStamped,
                "/pcl_pose",
                pose_received,
                QoSProfile(
                    depth=100,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            node.create_subscription(
                DiagnosticArray,
                "/alignment_status",
                diagnostic_received,
                QoSProfile(
                    depth=1000,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            initialpose = node.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )

            def spawn(command, name):
                log = stack.enter_context((output / f"{name}.log").open("w"))
                process = subprocess.Popen(
                    command,
                    env=environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append(process)
                return process

            def check_nodes():
                for name, process in (
                    ("rko", rko_process),
                    ("localizer", localizer_process),
                ):
                    if process.poll() is not None:
                        raise RuntimeError(
                            f"{name} exited ({process.returncode}); see {output / (name + '.log')}"
                        )

            try:
                print("Starting online RKO and activating localization", flush=True)
                rko_process = spawn(
                    [
                        str(rko),
                        "--ros-args",
                        "--params-file",
                        str(rko_config),
                    ],
                    "rko",
                )
                localizer_process = spawn(
                    [
                        str(localizer),
                        "--ros-args",
                        "--params-file",
                        str(config_path),
                        "-r",
                        "cloud:=/livox/points",
                        "-r",
                        "imu:=/livox/imu",
                    ],
                    "localizer",
                )
                activate(node, "/lidar_localization", 120.0)
                deadline = time.monotonic() + 30
                while True:
                    check_nodes()
                    subscriptions = node.get_subscriptions_info_by_topic(
                        data_tool.POINTS
                    )
                    reliable = {
                        item.node_name
                        for item in subscriptions
                        if item.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                    }
                    if initialpose.get_subscription_count() and len(reliable) >= 2:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            "RKO/localizer RELIABLE point subscriptions did not become ready; see logs"
                        )
                    rclpy.spin_once(node, timeout_sec=0.1)
                # Allow discovery of the diagnostic and pose readers before playback.
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                print(
                    "Replaying the complete bag at 1x (~5 minutes); initial pose is supplied once at +4 s",
                    flush=True,
                )
                player = spawn(
                    [
                        "ros2",
                        "bag",
                        "play",
                        str(bag),
                        "--clock",
                        "--rate",
                        "1",
                        "--qos-profile-overrides-path",
                        str(qos_config),
                        "--topics",
                        data_tool.POINTS,
                        data_tool.IMU,
                    ],
                    "player",
                )
                sent = False
                duration = (max(clouds.values()) - start_ns) * 1e-9
                deadline = time.monotonic() + duration + 90
                next_progress = time.monotonic() + 30
                while player.poll() is None:
                    check_nodes()
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if not sent and clock_ns[0] >= initialization_ns:
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id = "map"
                        pose_ns = round(row[0] * 1e9)
                        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                            pose_ns, 10**9
                        )
                        p, q = message.pose.pose.position, message.pose.pose.orientation
                        p.x, p.y, p.z = map(float, row[1:4])
                        q.x, q.y, q.z, q.w = map(float, row[4:8])
                        message.pose.covariance[0] = message.pose.covariance[7] = 0.25
                        message.pose.covariance[35] = 0.07
                        initialpose.publish(message)
                        sent = True
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            "bag playback exceeded its duration + 90 s; see player.log"
                        )
                    if time.monotonic() >= next_progress:
                        print(
                            f"Replay {max(0, (clock_ns[0] - start_ns) * 1e-9):.0f}/{duration:.0f} s; {len(poses)} poses",
                            flush=True,
                        )
                        next_progress = time.monotonic() + 30
                if player.returncode != 0 or not sent:
                    raise RuntimeError(
                        f"playback failed (exit {player.returncode}, initial pose sent={sent}); see player.log"
                    )
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    check_nodes()
                    rclpy.spin_once(node, timeout_sec=0.05)
                result = data_tool.evaluate(poses, reference, expected, diagnostics)
                result.update(
                    {
                        "source": data_tool.SOURCE,
                        "sequence": "outdoor_hard_02b",
                        "full_sequence": True,
                        "rate": 1.0,
                        "thresholds": {
                            "rmse_m": 0.35,
                            "diagnostic_coverage": 0.97,
                            "matched_pose_fraction": 0.95,
                        },
                        "note": "Map-frame position error without spatial alignment. Poses include odometry bridge predictions; diagnostic coverage is not NDT acceptance. One public ~5-minute sequence, not real-robot or multi-hour validation.",
                    }
                )
                (output / "summary.json").write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n"
                )
                receipt["status"] = "passed" if result["passed"] else "failed"
                print(
                    f"{'PASS' if result['passed'] else 'FAIL'}: RMSE {result['rmse_m']:.3f} m; diagnostic coverage {result['diagnostic_coverage']:.1%}; results: {output}",
                    flush=True,
                )
                return 0 if result["passed"] else 1
            finally:
                for process in reversed(processes):
                    stop_process(process)
    except KeyboardInterrupt:
        receipt["status"] = "interrupted"
        raise
    finally:
        if receipt["status"] == "running":
            receipt["status"] = "failed"
        receipt["process_exit_codes"] = [p.returncode for p in processes]
        (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    args = parser().parse_args(argv)
    if args.ros_domain_id < 0 or args.ros_domain_id > 232:
        print("koide demo: --ros-domain-id must be between 0 and 232", file=sys.stderr)
        return 2
    output = (
        (args.output or Path(f"koide-run-{time.strftime('%Y%m%d-%H%M%S')}"))
        .expanduser()
        .resolve()
    )
    data = args.data_dir.expanduser().resolve()
    try:
        paths = installed_paths(args)
        # Check dependencies before downloading hundreds of megabytes.
        import numpy  # noqa: F401
        import rosbag2_py  # noqa: F401
        import yaml  # noqa: F401

        print(f"Dataset: {data_tool.SOURCE} (Koide, CC BY 4.0)")
        print(
            f"Data: {data}\nResults: {output}\nROS domain: {args.ros_domain_id}",
            flush=True,
        )
        if args.dry_run:
            print(
                "Plan: verify/download assets -> SI IMU conversion -> online RKO + localization -> 1x replay -> one GT initial pose at +4 s -> map-frame scoring."
            )
            print(f"Localizer: {paths[0]}\nRKO: {paths[1]}")
            return 0
        if output.exists() and any(output.iterdir()):
            raise ValueError(
                f"output directory is not empty: {output}; choose a new --output directory"
            )
        print("Verifying official assets and preparing the SI bag cache", flush=True)
        raw, map_path, reference = data_tool.prepare_assets(data, args.download)
        bag = data_tool.prepare_si_bag(raw, data / "outdoor_hard_02b_si")
        output.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
        return run_replay(args, output, bag, map_path, reference, paths)
    except ImportError as exc:
        print(
            f"koide demo: missing dependency ({exc}); source ROS 2 and the built localization workspace (requires numpy, PyYAML and rosbag2_py)",
            file=sys.stderr,
        )
        return 2
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile) as exc:
        print(f"koide demo: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(f"koide demo: interrupted; results/logs: {output}", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
