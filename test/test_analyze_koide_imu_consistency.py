#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_koide_imu_consistency.py"
SPEC = importlib.util.spec_from_file_location("imu_consistency", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ImuConsistencyMathTest(unittest.TestCase):
    def test_wahba_bias_fit_recovers_known_mapping(self):
        angle = math.radians(20.0)
        rotation = np.array([
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        bias = np.array([0.01, -0.02, 0.005])
        durations = np.linspace(0.08, 0.16, 20)
        target = np.array([
            [0.03 * math.sin(i), 0.02 * math.cos(0.7 * i), 0.04 * math.sin(0.3 * i)]
            for i in range(20)
        ])
        source = (rotation.T @ target.T).T + durations[:, None] * bias
        fitted_rotation, fitted_bias = MODULE.fit_rotation_and_bias(source, target, durations)
        self.assertTrue(np.allclose(fitted_rotation, rotation, atol=1e-9))
        self.assertTrue(np.allclose(fitted_bias, bias, atol=1e-9))

    def test_signed_axis_candidates_are_24_proper_rotations(self):
        candidates = list(MODULE.right_handed_axis_permutations())
        self.assertEqual(len(candidates), 24)
        for candidate in candidates:
            self.assertAlmostEqual(float(np.linalg.det(candidate)), 1.0)
            self.assertTrue(np.array_equal(candidate.T @ candidate, np.eye(3)))

    def test_quaternion_slerp_midpoint(self):
        start = np.array([0.0, 0.0, 0.0, 1.0])
        end = np.array([0.0, 0.0, 1.0, 0.0])
        midpoint = MODULE.quat_slerp(start, end, 0.5)
        self.assertTrue(np.allclose(midpoint, [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]))


if __name__ == "__main__":
    unittest.main()
