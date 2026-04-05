from __future__ import annotations

from experiments.reinit_trigger.interface import ReinitDecision
from experiments.reinit_trigger.interface import ReinitSample


class FailureKindEagerReinit:
    name = "failure_kind_eager_reinit"
    design = "event-driven rule table"

    def reset(self) -> None:
        return None

    def step(self, sample: ReinitSample) -> ReinitDecision:
        if sample.failure_kind == "target_unavailable":
            return ReinitDecision(True, "target_unavailable")
        if sample.fitness_score >= 1000.0:
            return ReinitDecision(True, "fitness_exploded")
        if sample.consecutive_rejected_updates >= 200:
            return ReinitDecision(True, "reject_streak_exploded")
        return ReinitDecision(False, "continue_local_recovery")
