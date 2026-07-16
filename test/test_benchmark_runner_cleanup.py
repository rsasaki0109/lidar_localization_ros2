#!/usr/bin/env python3

from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import importlib.util
from importlib.machinery import SourceFileLoader


REPO = Path(__file__).resolve().parents[1]
RUNNER = REPO / "scripts" / "benchmark_runner"
LOADER = SourceFileLoader("benchmark_runner", str(RUNNER))
SPEC = importlib.util.spec_from_loader("benchmark_runner", LOADER)
RUNNER_MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER_MODULE)


def child_pids(pid: int) -> list[int]:
    path = Path(f"/proc/{pid}/task/{pid}/children")
    try:
        return [int(value) for value in path.read_text().split()]
    except FileNotFoundError:
        return []


def cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
    except FileNotFoundError:
        return ""


class BenchmarkRunnerCleanupTest(unittest.TestCase):
    def test_target_pid_matches_executable_not_argument_text(self):
        original = RUNNER_MODULE.scan_processes
        RUNNER_MODULE.scan_processes = lambda: [
            {"pid": 10, "argv0": "python3", "cmdline": "runner --target fastlio_mapping"},
            {"pid": 20, "argv0": "/opt/bin/fastlio_mapping", "cmdline": "/opt/bin/fastlio_mapping"},
        ]
        try:
            self.assertEqual(RUNNER_MODULE.find_latest_pid("fastlio_mapping"), 20)
        finally:
            RUNNER_MODULE.scan_processes = original

    def run_interrupted_benchmark(self, interrupt_signal: signal.Signals, expected_code: int):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "output"
            process = subprocess.Popen(
                [
                    str(RUNNER),
                    "--bag-path", str(Path(tmp) / "unused_bag"),
                    "--output-dir", str(output),
                    "--system-command", "exec -a benchmark_cleanup_system sleep 60",
                    "--bag-command", "exec -a benchmark_cleanup_bag sleep 60",
                    "--diagnostic-topic", "",
                    "--target-process-pattern", "benchmark-target-that-does-not-exist",
                    "--settle-seconds", "0.1",
                    "--post-roll-seconds", "0",
                    "--shutdown-timeout", "0.2",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                deadline = time.monotonic() + 5.0
                named_children = []
                while time.monotonic() < deadline:
                    named_children = [
                        pid for pid in child_pids(process.pid)
                        if "benchmark_cleanup_" in cmdline(pid)]
                    if len(named_children) == 2:
                        break
                    time.sleep(0.02)
                self.assertEqual(len(named_children), 2)
                tracked = child_pids(process.pid)

                process.send_signal(interrupt_signal)
                self.assertEqual(process.wait(timeout=5.0), expected_code)
                deadline = time.monotonic() + 2.0
                while (
                    any(Path(f"/proc/{pid}").exists() for pid in tracked)
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.02)
                survivors = [pid for pid in tracked if Path(f"/proc/{pid}").exists()]
                self.assertFalse(survivors, f"managed processes survived cleanup: {survivors}")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5.0)

    def test_sigint_stops_all_managed_process_groups(self):
        self.run_interrupted_benchmark(signal.SIGINT, 130)

    def test_sigterm_stops_all_managed_process_groups(self):
        self.run_interrupted_benchmark(signal.SIGTERM, 128 + signal.SIGTERM)


if __name__ == "__main__":
    unittest.main()
