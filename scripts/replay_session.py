"""Plumbing the replay checks share.

A control node on a ROS domain no one else uses; child processes in their own
sessions, one log each, stopped on exit; and the localizer's output recorded in the
output directory: /pcl_pose to estimate.tum, /alignment_status to alignment.jsonl.
Used by scripts/run_koide_public_bag.py and tools/benchmark/boreas_replay.py.
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml


def stamp_ns(stamp) -> int:
    return stamp.sec * 10**9 + stamp.nanosec


def apply_overrides(parameters: dict, items) -> None:
    """Set each KEY=VALUE item (a YAML value) in a ros__parameters dict."""
    for item in items:
        key, separator, value = item.partition("=")
        if not separator or not key:
            raise ValueError(f"expected KEY=VALUE, got {item!r}")
        parameters[key] = yaml.safe_load(value)


def initial_pose_parameters(pose) -> dict:
    """The set_initial_pose parameters for an (x, y, z, qx, qy, qz, qw) pose."""
    names = ("x", "y", "z", "qx", "qy", "qz", "qw")
    values = [float(value) for value in pose]
    if len(values) != len(names):
        raise ValueError(f"expected x y z qx qy qz qw, got {len(values)} values")
    return {"set_initial_pose": True} | {
        f"initial_pose_{name}": value for name, value in zip(names, values, strict=True)
    }


def stop_process(process: subprocess.Popen) -> None:
    """SIGINT the process group, then SIGTERM, then SIGKILL."""
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


class ReplaySession:
    """A control node and the processes it starts. On exit it stops them, newest
    first, and closes their logs.

    Set ROS_DOMAIN_ID in os.environ before creating one: rclpy reads it there.
    """

    def __init__(self, output: Path, environment: dict, name: str):
        import rclpy

        self.output = Path(output)
        self.environment = environment
        self.processes: list[tuple[str, subprocess.Popen]] = []
        self.poses: list[list[float]] = []
        self.diagnostic_stamps: set[int] = set()
        self._files = contextlib.ExitStack()
        self._rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node(f"{name}_{os.getpid()}")

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        try:
            for _, process in reversed(self.processes):
                stop_process(process)
            self._files.close()
        finally:
            self.node.destroy_node()
            self._rclpy.try_shutdown()
        return False

    def spin_for(self, seconds: float, check=None) -> None:
        """Spin the control node for SECONDS, calling CHECK before each spin."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if check is not None:
                check()
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def require_free_domain(self) -> None:
        """Fail if another node is on this domain: it could inject poses or TF."""
        self.spin_for(1.5)
        if set(self.node.get_node_names()) - {self.node.get_name()}:
            raise RuntimeError(
                "ROS domain is already in use; pass another --ros-domain-id"
            )

    def record(self, accept_stamp=None, on_status=None) -> None:
        """Record /pcl_pose into self.poses and estimate.tum, and /alignment_status
        into self.diagnostic_stamps and alignment.jsonl.

        ACCEPT_STAMP(stamp_ns) drops a pose when it returns False; ON_STATUS(values)
        sees the key/value dict of each status entry. A non-finite pose raises.
        """
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        tum = self._files.enter_context((self.output / "estimate.tum").open("w"))
        status_log = self._files.enter_context(
            (self.output / "alignment.jsonl").open("w")
        )

        pose_wall = self._files.enter_context((self.output / "pose_wall.csv").open("w"))

        def pose_received(message):
            stamp = stamp_ns(message.header.stamp)
            pose_wall.write(f"{stamp},{time.monotonic():.6f}\n")
            if accept_stamp is not None and not accept_stamp(stamp):
                return
            p, q = message.pose.pose.position, message.pose.pose.orientation
            values = [stamp * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite pose received")
            self.poses.append(values)
            tum.write(" ".join(f"{value:.9f}" for value in values) + "\n")

        def status_received(message):
            stamp = stamp_ns(message.header.stamp)
            self.diagnostic_stamps.add(stamp)
            for entry in message.status:
                values = {value.key: value.value for value in entry.values}
                if on_status is not None:
                    on_status(values)
                status_log.write(
                    json.dumps(
                        {"stamp_ns": stamp, "message": entry.message, "values": values}
                    )
                    + "\n"
                )

        reliable_latched = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped, "/pcl_pose", pose_received, reliable_latched
        )
        self.node.create_subscription(
            DiagnosticArray, "/alignment_status", status_received, reliable_latched
        )

    def spawn(self, command: list[str], name: str) -> subprocess.Popen:
        """Start COMMAND in its own session, logging to NAME.log."""
        log = self._files.enter_context((self.output / f"{name}.log").open("w"))
        process = subprocess.Popen(
            command,
            env=self.environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.processes.append((name, process))
        return process

    def require_running(self, *names: str) -> None:
        """Fail if one of the processes started as NAMES has exited."""
        for name, process in self.processes:
            if name in names and process.poll() is not None:
                raise RuntimeError(
                    f"{name} exited ({process.returncode}); see {self.output / (name + '.log')}"
                )
