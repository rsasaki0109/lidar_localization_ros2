"""Plumbing the replay checks share.

A control node on a ROS domain no one else uses; child processes in their own
sessions, one log each, stopped on exit; and the localizer's output recorded in the
output directory: /pcl_pose to estimate.tum, /alignment_status to alignment.jsonl.
ResourceMonitor samples a process's memory and CPU time from /proc.
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
    return {
        "set_initial_pose": True,
        **{f"initial_pose_{name}": value for name, value in zip(names, values)},
    }


def find_process(group: int, name: str) -> int:
    """The process in process group GROUP whose name is NAME (as /proc shows it, cut
    to 15 characters), or GROUP itself when there is none: the node a --wrap command
    such as heaptrack started."""
    for stat in Path("/proc").glob("[0-9]*/stat"):
        try:
            head, tail = stat.read_text().rsplit(")", 1)
        except OSError:
            continue
        comm = head.split("(", 1)[1]
        if int(tail.split()[2]) == group and comm == name[:15]:
            return int(stat.parent.name)
    return group


def read_process(pid: int) -> tuple:
    """(rss_mb, peak_rss_mb, cpu_sec, anon_rss_mb) of PID from /proc: its resident
    memory now and at most (VmHWM), the CPU time it has used (utime + stime) and its
    anonymous resident memory (RssAnon: the heap and stacks, without the shared
    libraries' pages; what a memory cgroup charges it for beyond page cache)."""
    status = Path(f"/proc/{pid}/status").read_text()
    fields = dict(line.split(":", 1) for line in status.splitlines() if ":" in line)
    stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    ticks = int(stat[11]) + int(stat[12])
    anon = fields.get("RssAnon")
    return (
        int(fields["VmRSS"].split()[0]) / 1024.0,
        int(fields["VmHWM"].split()[0]) / 1024.0,
        ticks / os.sysconf("SC_CLK_TCK"),
        int(anon.split()[0]) / 1024.0 if anon else None,
    )


def resource_summary(samples, data_sec=None, scans=None, window=None) -> dict:
    """CPU and memory of (wall_sec, rss_mb, peak_rss_mb, cpu_sec[, anon_rss_mb])
    SAMPLES: the cores in use on average and at p95 of the sampling intervals, the CPU
    seconds per second of data (DATA_SEC) and per scan (SCANS), the peak resident
    memory (VmHWM, so including the map load before the first sample) and the steady
    one (the median over the second half of the samples), and with anon_rss_mb its
    peak and steady values. WINDOW, a (first, last) wall_sec pair, limits all but the
    peak resident memory to the samples inside it."""
    peak = max((row[2] for row in samples), default=None)
    rows = sorted(samples, key=lambda row: row[0])
    if window is not None:
        rows = [row for row in rows if window[0] <= row[0] <= window[1]]
    if len(rows) < 2:
        return {}
    wall = [row[0] for row in rows]
    cpu = [row[3] for row in rows]
    cores = sorted(
        (c1 - c0) / (w1 - w0)
        for w0, w1, c0, c1 in zip(wall, wall[1:], cpu, cpu[1:])
        if w1 > w0
    )
    used = cpu[-1] - cpu[0]
    steady = sorted(row[1] for row in rows[len(rows) // 2 :])
    anon = [row[4] for row in rows if len(row) > 4 and row[4] is not None]
    result = {
        "cpu_sec": round(used, 2),
        "cpu_cores_mean": round(used / (wall[-1] - wall[0]), 3),
        "cpu_cores_p95": round(cores[min(len(cores) - 1, int(0.95 * len(cores)))], 3),
        "rss_mb_peak": round(peak, 1),
        "rss_mb_steady": round(steady[len(steady) // 2], 1),
        "rss_mb_last": round(rows[-1][1], 1),
    }
    if anon:
        later = sorted(anon[len(anon) // 2 :])
        result["anon_rss_mb_peak"] = round(max(anon), 1)
        result["anon_rss_mb_steady"] = round(later[len(later) // 2], 1)
    if data_sec:
        result["cpu_sec_per_data_sec"] = round(used / data_sec, 3)
    if scans:
        result["cpu_ms_per_scan"] = round(1000.0 * used / scans, 2)
    return result


class ResourceMonitor:
    """Samples a process's memory and CPU time every INTERVAL_SEC into a CSV file
    (wall_sec, rss_mb, peak_rss_mb, cpu_sec, anon_rss_mb). Call poll() often; it reads
    /proc only when a sample is due. A process that has exited is no longer sampled."""

    def __init__(self, pid: int, path: Path, interval_sec: float = 1.0):
        self.pid = pid
        self.interval_sec = interval_sec
        self.samples: list[tuple] = []
        self._began = time.monotonic()
        self._next = self._began
        self.path = Path(path)
        self.path.write_text("wall_sec,rss_mb,peak_rss_mb,cpu_sec,anon_rss_mb\n")
        self._closed = False

    def poll(self) -> None:
        now = time.monotonic()
        if now < self._next or self._closed:
            return
        self._next = now + self.interval_sec
        try:
            rss, peak, cpu, anon = read_process(self.pid)
        except (OSError, KeyError, IndexError, ValueError):
            return
        row = (round(now - self._began, 3), rss, peak, cpu, anon)
        self.samples.append(row)
        with self.path.open("a") as csv:
            csv.write(
                f"{row[0]:.3f},{rss:.1f},{peak:.1f},{cpu:.2f},"
                f"{'' if anon is None else f'{anon:.1f}'}\n"
            )

    def elapsed(self) -> float:
        """Seconds since the monitor started, on the clock of the samples' wall_sec."""
        return time.monotonic() - self._began

    def close(self) -> None:
        """Stop sampling."""
        self._closed = True

    def summary(self, data_sec=None, scans=None, window=None) -> dict:
        return resource_summary(self.samples, data_sec, scans, window)


def stop_process(process: subprocess.Popen) -> None:
    """SIGCONT (in case it is stopped), SIGINT the process group, then SIGTERM, then
    SIGKILL."""
    if process.poll() is not None:
        return
    with contextlib.suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGCONT)
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

        def pose_received(message):
            stamp = stamp_ns(message.header.stamp)
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
