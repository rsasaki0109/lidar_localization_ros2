from __future__ import annotations

from experiments.imu_bias_correction.interface import BiasFactor


def _correct_factor(factor: BiasFactor) -> float:
    # d(delta_theta)/d(bg) = -dt for the scalar constant-rate fixture.
    delta_bias = factor.current_bias_rad_s - factor.integration_bias_rad_s
    return factor.measured_delta_rad - factor.duration_sec * delta_bias


class UncorrectedFactors:
    name = "uncorrected_factors"
    design = "runtime baseline: reuse every stored delta at its original bias"

    def correct(self, factors: list[BiasFactor]) -> list[float]:
        return [factor.measured_delta_rad for factor in factors]


class LatestFactorOnly:
    name = "latest_factor_only"
    design = "correct only the newest factor after each bias update"

    def correct(self, factors: list[BiasFactor]) -> list[float]:
        if not factors:
            return []
        corrected = [factor.measured_delta_rad for factor in factors]
        corrected[-1] = _correct_factor(factors[-1])
        return corrected


class PerFactorBiasSnapshot:
    name = "per_factor_bias_snapshot"
    design = "correct every factor from its stored integration bias to the current bias"

    def correct(self, factors: list[BiasFactor]) -> list[float]:
        return [_correct_factor(factor) for factor in factors]
