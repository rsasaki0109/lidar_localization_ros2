"""Check the --stall option of the Koide replay script, without ROS nodes."""

import argparse
import importlib.util
import signal
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "run_koide_public_bag", ROOT / "scripts/run_koide_public_bag.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TestStallOption(unittest.TestCase):
    def test_parse(self):
        stall = MODULE.parse_stall("60:10.5")
        self.assertEqual(stall["start_sec"], 60.0)
        self.assertEqual(stall["duration_sec"], 10.5)
        self.assertEqual((stall["kind"], stall["lap"]), ("stall", 0))

    def test_parse_rejects_bad_values(self):
        for text in (
            "60",
            "a:b",
            "-1:5",
            "60:0",
            "60:-2",
            "nan:5",
            "5:inf",
            "1:2:3",
            "",
        ):
            with self.subTest(text=text), self.assertRaises(argparse.ArgumentTypeError):
                MODULE.parse_stall(text)

    def test_signals_stop_once_then_continue_once(self):
        stalls = [MODULE.parse_stall("10:2")]
        state = [{"stopped_at": None, "done": False}]
        due = MODULE.due_stall_signals
        self.assertEqual(due(stalls, state, 9.9, 100.0), [])
        self.assertEqual(due(stalls, state, 10.0, 100.0), [(signal.SIGSTOP, 0)])
        self.assertEqual(due(stalls, state, 10.5, 100.5), [])
        self.assertEqual(due(stalls, state, 12.0, 102.0), [(signal.SIGCONT, 0)])
        self.assertEqual(due(stalls, state, 30.0, 120.0), [])

    def test_two_stalls_in_order(self):
        stalls = [MODULE.parse_stall("10:1"), MODULE.parse_stall("60:2")]
        state = [{"stopped_at": None, "done": False} for _ in stalls]
        sent = []
        for elapsed in np.arange(0.0, 70.0, 0.5):
            sent += MODULE.due_stall_signals(stalls, state, elapsed, elapsed)
        self.assertEqual(
            sent,
            [
                (signal.SIGSTOP, 0),
                (signal.SIGCONT, 0),
                (signal.SIGSTOP, 1),
                (signal.SIGCONT, 1),
            ],
        )

    def test_scoring_measures_the_error_after_the_stall(self):
        t0 = 1000.0
        stamps = t0 + np.arange(0.0, 60.0, 0.1)
        reference = np.zeros((len(stamps), 8))
        reference[:, 0] = stamps
        reference[:, 1] = (stamps - t0) * 1.5
        poses = reference.copy()
        # No poses from 20 s to 30 s; 3 m off for 2 s afterwards, then back.
        keep = (stamps < t0 + 20) | (stamps >= t0 + 30)
        poses = poses[keep]
        off = (poses[:, 0] >= t0 + 30) & (poses[:, 0] < t0 + 32)
        poses[off, 2] += 3.0
        result = MODULE.score_stalls(
            poses, reference, t0, [MODULE.parse_stall("20:10")]
        )[0]
        self.assertEqual(result["poses_during"], 0)
        self.assertAlmostEqual(result["max_error_m"], 3.0, places=3)
        self.assertAlmostEqual(result["recovered_after_sec"], 2.0, places=1)


if __name__ == "__main__":
    unittest.main()
