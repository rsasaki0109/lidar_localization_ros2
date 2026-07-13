#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PACKAGE_NAME = "lidar_localization_ros2"


def resolve_repo_root() -> Path:
    script_path = Path(__file__).resolve()
    candidates = [script_path.parents[1], script_path.parents[3] / "share" / PACKAGE_NAME]
    for candidate in candidates:
        if (candidate / "experiments" / "startup_integrity").is_dir():
            return candidate
    raise RuntimeError(f"Could not resolve repo root from {script_path}")


repo_root = resolve_repo_root()
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from experiments.reporting import compute_static_metrics
from experiments.reporting import generate_combined_docs
from experiments.reporting import overall_score
from experiments.reporting import repo_relative_path
from experiments.startup_integrity.interface import StartupIntegritySample
from experiments.startup_integrity.variants import CumulativeTranslationMonitor
from experiments.startup_integrity.variants import FitnessOnlyMonitor
from experiments.startup_integrity.variants import NormalizedInnovationEnergyMonitor
from experiments.startup_integrity.variants import PeakInnovationMonitor


VARIANTS = [
    FitnessOnlyMonitor,
    PeakInnovationMonitor,
    CumulativeTranslationMonitor,
    NormalizedInnovationEnergyMonitor,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run offline startup-integrity experiments.")
    parser.add_argument(
        "--output-json",
        default=str(repo_root / "experiments" / "startup_integrity" / "results.json"),
    )
    return parser.parse_args()


def load_fixture(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["fixture_path"] = str(path)
    return data


def to_sample(raw: dict[str, Any]) -> StartupIntegritySample:
    return StartupIntegritySample(
        index=int(raw["index"]),
        fitness_score=float(raw["fitness_score"]),
        correction_translation_m=float(raw["correction_translation_m"]),
        correction_yaw_deg=float(raw["correction_yaw_deg"]),
    )


def evaluate_trigger(trigger_index: int | None, expectation: dict[str, Any]) -> tuple[bool, str]:
    if expectation.get("must_not_trigger"):
        return trigger_index is None, "must_not_trigger"
    latest = int(expectation["should_trigger_by_index"])
    if trigger_index is None:
        return False, "missing_trigger"
    return trigger_index <= latest, "within_expected_window" if trigger_index <= latest else "too_late"


def run_variant_on_fixture(variant_cls: type, fixture: dict[str, Any]) -> dict[str, Any]:
    variant = variant_cls()
    variant.reset()
    trigger_index = None
    trigger_reason = ""
    trigger_score = None
    for raw in fixture["samples"]:
        sample = to_sample(raw)
        decision = variant.step(sample)
        if trigger_index is None and decision.request_reinitialization:
            trigger_index = sample.index
            trigger_reason = decision.reason
            trigger_score = decision.score
    passed, outcome = evaluate_trigger(trigger_index, fixture["expectation"])
    return {
        "fixture": fixture["name"],
        "passed": passed,
        "outcome": outcome,
        "trigger_index": trigger_index,
        "trigger_reason": trigger_reason,
        "trigger_score": trigger_score,
    }


def main() -> int:
    args = parse_args()
    fixtures = [
        load_fixture(path)
        for path in sorted((repo_root / "experiments" / "startup_integrity" / "fixtures").glob("*.json"))
    ]
    variants = []
    for variant_cls in VARIANTS:
        fixture_results = [run_variant_on_fixture(variant_cls, fixture) for fixture in fixtures]
        benchmark_score = 100.0 * sum(result["passed"] for result in fixture_results) / len(fixtures)
        static_metrics = compute_static_metrics(variant_cls)
        result = {
            "name": variant_cls.name,
            "design": variant_cls.design,
            "fixture_results": fixture_results,
            "benchmark_score": benchmark_score,
            "readability_score": static_metrics["readability_score"],
            "extensibility_score": static_metrics["extensibility_score"],
            "static_metrics": static_metrics,
        }
        result["overall_score"] = overall_score(
            benchmark_score, result["readability_score"], result["extensibility_score"]
        )
        variants.append(result)
    variants.sort(key=lambda item: (-item["overall_score"], -item["benchmark_score"], item["name"]))
    results = {
        "problem": "startup_integrity",
        "title": "Startup False-Convergence Integrity Monitor",
        "problem_statement": (
            "Detect a self-consistent but wrong NDT basin during the first five accepted updates, "
            "where scalar fitness remains healthy, without rejecting bounded Koide indoor startups."
        ),
        "generated_from": repo_relative_path(__file__),
        "promotion_decision": "no_promotion",
        "promotion_reason": (
            "No candidate passes every repeated closed-loop fixture: correction-budget "
            "variants false-trigger on indoor_easy_02_live_r02 and miss "
            "indoor_kidnap_01_live_r02, while peak and fitness variants miss kidnaps."
        ),
        "research_context": [
            {
                "title": "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map",
                "url": "https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf",
                "relevance": (
                    "an independent global place descriptor can challenge a locally "
                    "self-consistent NDT basin"
                ),
            },
            {
                "title": "NDT-Transformer: Large-Scale 3D Point Cloud Localisation using the Normal Distribution Transform Representation",
                "url": "https://arxiv.org/abs/2103.12292",
                "relevance": (
                    "global retrieval over NDT-cell descriptors is a second candidate "
                    "verifier family, separate from local registration fitness"
                ),
            },
        ],
        "interface": {
            "methods": [
                "reset() -> None",
                "step(sample: StartupIntegritySample) -> StartupIntegrityDecision",
            ],
            "sample_type": "StartupIntegritySample",
            "sample_fields": [
                "index",
                "fitness_score",
                "correction_translation_m",
                "correction_yaw_deg",
            ],
            "decision_type": "StartupIntegrityDecision",
            "decision_fields": ["request_reinitialization", "reason", "score"],
        },
        "fixtures": [
            {
                "name": fixture["name"],
                "description": fixture["description"],
                "source": fixture["source"],
            }
            for fixture in fixtures
        ],
        "variants": variants,
        "benchmark_score_mean": sum(item["benchmark_score"] for item in variants) / len(variants),
    }
    output_path = Path(args.output_json).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    generate_combined_docs(repo_root)
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
