from __future__ import annotations

from dataclasses import dataclass

from experiments.borderline_gate.interface import GateDecision
from experiments.borderline_gate.interface import GateSample


@dataclass(frozen=True)
class Config:
    strict_threshold: float = 5.5


class PostRejectStrictGate:
    name = "post_reject_strict"
    design = "state-escalation rule table"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: GateSample) -> GateDecision:
        if sample.fitness_score > sample.effective_score_threshold:
            return GateDecision(True, "over_effective_threshold")
        if (
            sample.consecutive_rejected_updates > 0 and
            sample.fitness_score > self.config.strict_threshold
        ):
            return GateDecision(True, "over_post_reject_strict_threshold")
        return GateDecision(False, "within_dynamic_thresholds")
