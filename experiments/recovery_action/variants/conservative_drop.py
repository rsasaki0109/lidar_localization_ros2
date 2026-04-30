from __future__ import annotations

from experiments.recovery_action.interface import RecoveryDecision
from experiments.recovery_action.interface import RecoverySample


class ConservativeDropStrategy:
    name = "conservative_drop"
    design = "minimal baseline"

    def reset(self) -> None:
        return None

    def step(self, sample: RecoverySample) -> RecoveryDecision:
        return RecoveryDecision("advance_prediction_only", "no_recovery")
