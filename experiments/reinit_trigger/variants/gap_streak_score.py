from __future__ import annotations

from dataclasses import dataclass

from experiments.reinit_trigger.interface import ReinitDecision
from experiments.reinit_trigger.interface import ReinitSample


@dataclass(frozen=True)
class Config:
    threshold: float = 0.95
    gap_scale_sec: float = 30.0
    seed_scale_m: float = 100.0
    streak_scale: float = 200.0


class GapStreakScoreReinit:
    name = "gap_streak_score_reinit"
    design = "scorecard threshold"

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()

    def reset(self) -> None:
        return None

    def step(self, sample: ReinitSample) -> ReinitDecision:
        score = 0.0
        score += 0.45 * min(1.0, sample.accepted_gap_sec / self.config.gap_scale_sec)
        score += 0.30 * min(1.0, sample.seed_translation_since_accept_m / self.config.seed_scale_m)
        score += 0.20 * min(1.0, sample.consecutive_rejected_updates / self.config.streak_scale)
        if sample.failure_kind == "target_unavailable":
            score += 0.15
        if sample.fitness_score >= 1000.0:
            score += 0.15
        if score >= self.config.threshold:
            return ReinitDecision(True, "reinit_score_exceeded", score)
        return ReinitDecision(False, "reinit_score_safe", score)
