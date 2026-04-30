from __future__ import annotations

from experiments.reinit_trigger.interface import ReinitDecision
from experiments.reinit_trigger.interface import ReinitSample


class NeverReinit:
    name = "never_reinit"
    design = "minimal baseline"

    def reset(self) -> None:
        return None

    def step(self, sample: ReinitSample) -> ReinitDecision:
        return ReinitDecision(False, "stay_in_local_recovery")
