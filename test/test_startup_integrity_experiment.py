#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path

from experiments.startup_integrity.interface import StartupIntegritySample
from experiments.startup_integrity.variants import CumulativeTranslationMonitor
from experiments.startup_integrity.variants import FitnessOnlyMonitor
from experiments.startup_integrity.variants import NormalizedInnovationEnergyMonitor
from experiments.startup_integrity.variants import PeakInnovationMonitor


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "experiments" / "startup_integrity" / "fixtures"
VARIANTS = [
    FitnessOnlyMonitor,
    PeakInnovationMonitor,
    CumulativeTranslationMonitor,
    NormalizedInnovationEnergyMonitor,
]


def trigger_index(variant_cls: type, fixture: dict) -> int | None:
    variant = variant_cls()
    variant.reset()
    for raw in fixture["samples"]:
        decision = variant.step(StartupIntegritySample(**raw))
        if decision.request_reinitialization:
            return int(raw["index"])
    return None


def expected(fixture: dict, index: int | None) -> bool:
    expectation = fixture["expectation"]
    if expectation.get("must_not_trigger"):
        return index is None
    return index is not None and index <= int(expectation["should_trigger_by_index"])


def test_no_candidate_passes_all_repeated_closed_loop_fixtures() -> None:
    fixtures = [json.loads(path.read_text()) for path in sorted(FIXTURES.glob("*.json"))]
    scores = {
        variant.name: sum(expected(fixture, trigger_index(variant, fixture)) for fixture in fixtures)
        for variant in VARIANTS
    }
    assert len(fixtures) == 7
    assert max(scores.values()) == 5
    assert scores["fitness_only"] == 4


def test_repeated_runs_expose_budget_false_positive_and_false_negative() -> None:
    by_name = {
        path.stem: json.loads(path.read_text()) for path in FIXTURES.glob("*.json")
    }
    assert trigger_index(
        CumulativeTranslationMonitor, by_name["koide_indoor_easy_02_live_r02"]
    ) == 2
    assert trigger_index(
        CumulativeTranslationMonitor, by_name["koide_indoor_kidnap_01_live_r02"]
    ) is None
