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

    def test_cumulative_integration_handles_vector_rate(self):
        times = np.linspace(0.0, 2.0, 201)
        rates = np.column_stack((times, np.ones_like(times) * 2.0, -times))
        cumulative = MODULE.cumulative_trapezoid(times, rates)
        integral, valid = MODULE.integrate_from_cumulative(
            times, cumulative, np.array([0.25]), np.array([1.75]))
        self.assertTrue(valid[0])
        self.assertTrue(np.allclose(integral[0], [1.5, 3.0, -1.5], atol=1e-9))

    def test_between_cloud_alignment_recovers_time_offset(self):
        imu_times = np.linspace(0.0, 12.0, 12001)
        true_offset = 0.08
        angular_rate = 0.2 + 0.08 * np.sin(1.7 * (imu_times - true_offset))
        gyro = np.column_stack((
            np.zeros_like(imu_times), np.zeros_like(imu_times), angular_rate))
        cloud_times = np.arange(1.0, 11.0, 0.1)
        yaw = 0.2 * cloud_times - (0.08 / 1.7) * np.cos(1.7 * cloud_times)
        quaternions = np.column_stack((
            np.zeros_like(yaw), np.zeros_like(yaw), np.sin(0.5 * yaw), np.cos(0.5 * yaw)))
        scans = [(stamp, None, "lidar") for stamp in cloud_times]
        result = MODULE.evaluate_between_cloud_stamps(
            scans, imu_times, gyro, cloud_times, quaternions,
            offset_min_sec=-0.15, offset_max_sec=0.15, offset_step_sec=0.01)
        self.assertAlmostEqual(
            result["imu_minus_reference_time_offset_sec"], true_offset, delta=0.011)
        self.assertLess(result["wahba"]["rate_rmse_rad_s"], 1e-3)

    def test_accelerometer_unit_scale_distinguishes_g_and_si(self):
        g_samples = np.tile([0.0, 0.0, 1.0], (20, 1))
        si_samples = np.tile([0.0, 0.0, -9.81], (20, 1))
        self.assertEqual(MODULE.accelerometer_unit_scale(g_samples)[0], "g")
        self.assertEqual(MODULE.accelerometer_unit_scale(si_samples)[0], "m_s2")


if __name__ == "__main__":
    unittest.main()
