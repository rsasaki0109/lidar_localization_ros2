#!/usr/bin/env python3

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "build_odometry_runtime_evidence.py"
SPEC = importlib.util.spec_from_file_location("runtime_evidence", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def write_trace(path: Path, latencies, positions=None):
    if positions is None:
        positions = [(0.1 * index, 0.0, 0.0) for index in range(len(latencies))]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "stamp_sec", "receive_stamp_sec", "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w"])
        for index, (latency, position) in enumerate(zip(latencies, positions)):
            stamp = 0.1 * index
            writer.writerow([stamp, stamp + latency, *position, 0.0, 0.0, 0.0, 1.0])


class RuntimeEvidenceTest(unittest.TestCase):
    def test_bounded_realtime_trace(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.csv"
            write_trace(trace, [0.02] * 100)
            result = MODULE.build_evidence(trace)
            self.assertAlmostEqual(result["processing_p95_sec"], 0.02)
            self.assertAlmostEqual(result["scan_period_sec"], 0.1)
            self.assertFalse(result["queue_growth_unbounded"])
            self.assertEqual(result["tf_jump_count"], 0)
            self.assertEqual(result["unauthorized_reset_count"], 0)

    def test_latency_growth_and_reset_are_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.csv"
            latencies = [0.01 + index * 0.005 for index in range(100)]
            positions = [(float(index), 0.0, 0.0) for index in range(99)] + [(0.0, 0.0, 0.0)]
            write_trace(trace, latencies, positions)
            result = MODULE.build_evidence(trace)
            self.assertTrue(result["queue_growth_unbounded"])
            self.assertEqual(result["tf_jump_count"], 1)
            self.assertEqual(result["unauthorized_reset_count"], 1)

    def test_missing_receive_stamp_is_not_valid_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.csv"
            trace.write_text("stamp_sec,position_x\n0,0\n", encoding="utf-8")
            result = MODULE.build_evidence(trace)
            self.assertIsNone(result["processing_p95_sec"])
            self.assertEqual(result["evidence"]["invalid_row_count"], 1)


if __name__ == "__main__":
    unittest.main()
