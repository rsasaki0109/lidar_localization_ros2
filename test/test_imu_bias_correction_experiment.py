#!/usr/bin/env python3

import unittest

from experiments.imu_bias_correction.interface import BiasFactor
from experiments.imu_bias_correction.variants import LatestFactorOnly
from experiments.imu_bias_correction.variants import PerFactorBiasSnapshot
from experiments.imu_bias_correction.variants import UncorrectedFactors


def make_factor(true_rate, sensor_bias, integration_bias, current_bias, duration):
    return BiasFactor(
        measured_delta_rad=(true_rate + sensor_bias - integration_bias) * duration,
        integration_bias_rad_s=integration_bias,
        current_bias_rad_s=current_bias,
        duration_sec=duration,
        expected_delta_rad=true_rate * duration,
    )


class ImuBiasCorrectionExperimentTest(unittest.TestCase):
    def test_every_factor_requires_its_own_linearization_bias(self):
        factors = [
            make_factor(0.20, 0.012, 0.000, 0.012, 0.5),
            make_factor(-0.10, 0.012, 0.005, 0.012, 0.8),
            make_factor(0.05, 0.012, 0.010, 0.012, 1.2),
        ]
        expected = [factor.expected_delta_rad for factor in factors]

        baseline = UncorrectedFactors().correct(factors)
        latest_only = LatestFactorOnly().correct(factors)
        per_factor = PerFactorBiasSnapshot().correct(factors)

        self.assertGreater(max(abs(a - b) for a, b in zip(baseline, expected)), 1e-3)
        self.assertGreater(max(abs(a - b) for a, b in zip(latest_only, expected)), 1e-3)
        self.assertLess(max(abs(a - b) for a, b in zip(per_factor, expected)), 1e-12)

    def test_correction_is_noop_when_bias_did_not_change(self):
        factors = [make_factor(0.3, 0.01, 0.01, 0.01, 0.4)]
        for strategy in (UncorrectedFactors(), LatestFactorOnly(), PerFactorBiasSnapshot()):
            self.assertAlmostEqual(
                strategy.correct(factors)[0], factors[0].expected_delta_rad, places=12)


if __name__ == "__main__":
    unittest.main()
