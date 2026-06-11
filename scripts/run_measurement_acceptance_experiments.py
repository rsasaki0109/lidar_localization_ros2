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
    candidates = [
        script_path.parents[1],
        script_path.parents[3] / "share" / PACKAGE_NAME,
    ]
    for candidate in candidates:
        if (candidate / "experiments" / "measurement_acceptance").is_dir() and (candidate / "docs").is_dir():
            return candidate
    raise RuntimeError(f"Could not resolve repo root from {script_path}")


repo_root = resolve_repo_root()
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from experiments.measurement_acceptance.interface import AcceptanceSample
from experiments.measurement_acceptance.variants import BoundedDegradedAcceptance
from experiments.measurement_acceptance.variants import CorrectionConditionedAcceptance
from experiments.measurement_acceptance.variants import FixedThresholdAcceptance
from experiments.measurement_acceptance.variants import ScoreRatioBudgetAcceptance
from experiments.reporting import compute_static_metrics
from experiments.reporting import overall_score
from experiments.reporting import repo_relative_path


VARIANTS = [
    FixedThresholdAcceptance,
    CorrectionConditionedAcceptance,
    ScoreRatioBudgetAcceptance,
    BoundedDegradedAcceptance,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run offline measurement acceptance experiments.")
    parser.add_argument(
        "--output-json",
        default=str(repo_root / "experiments" / "measurement_acceptance" / "results.json"),
        help="Where to write the structured experiment result JSON.",
    )
    return parser.parse_args()


def load_fixture(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["fixture_path"] = str(path)
    return data


def to_sample(raw: dict[str, Any]) -> AcceptanceSample:
    return AcceptanceSample(
        index=int(raw["index"]),
        fitness_score=float(raw["fitness_score"]),
        effective_score_threshold=float(raw["effective_score_threshold"]),
        correction_translation_m=float(raw["correction_translation_m"]),
        correction_yaw_deg=float(raw["correction_yaw_deg"]),
        accepted_gap_sec=float(raw["accepted_gap_sec"]),
        consecutive_rejected_updates=int(raw["consecutive_rejected_updates"]),
    )


def run_variant_on_fixture(variant_cls: type, fixture: dict[str, Any]) -> dict[str, Any]:
    variant = variant_cls()
    variant.reset()
    sample = to_sample(fixture["sample"])
    decision = variant.step(sample)
    expected_reject = bool(fixture["expectation"]["expected_reject"])
    passed = decision.reject_measurement == expected_reject
    return {
        "fixture": fixture["name"],
        "passed": passed,
        "outcome": "matched_expected_reject" if passed else "mismatched_expected_reject",
        "decision": "reject" if decision.reject_measurement else "accept",
        "decision_reason": decision.reason,
        "decision_score": decision.score,
        "expected_reject": expected_reject,
    }


def run_variant_on_sequence(variant_cls: type, fixture: dict[str, Any]) -> dict[str, Any]:
    """Closed-loop-style harness: gap and rejection streak follow the policy's
    own decisions, so self-confirming unbounded acceptance becomes visible."""
    variant = variant_cls()
    variant.reset()
    threshold = float(fixture["effective_score_threshold"])
    consecutive_rejected = 0
    degraded_accepts = 0
    max_consecutive_degraded = 0
    current_run = 0
    last_decision_reject = True
    rejected_indices = []
    for index, step in enumerate(fixture["steps"]):
        gap = 0.4 + 0.5 * consecutive_rejected
        sample = AcceptanceSample(
            index=index,
            fitness_score=float(step["fitness_score"]),
            effective_score_threshold=threshold,
            correction_translation_m=float(step["correction_translation_m"]),
            correction_yaw_deg=float(step["correction_yaw_deg"]),
            accepted_gap_sec=gap,
            consecutive_rejected_updates=consecutive_rejected,
        )
        decision = variant.step(sample)
        last_decision_reject = decision.reject_measurement
        if decision.reject_measurement:
            rejected_indices.append(index)
            consecutive_rejected += 1
            current_run = 0
        else:
            consecutive_rejected = 0
            if sample.fitness_score > threshold:
                degraded_accepts += 1
                current_run += 1
                max_consecutive_degraded = max(max_consecutive_degraded, current_run)
            else:
                current_run = 0
    expectations = fixture["expectations"]
    checks = {
        "bounded_degraded_run": max_consecutive_degraded
        <= int(expectations["max_consecutive_degraded_accepts"]),
        "bridges_onset": degraded_accepts >= int(expectations["min_degraded_accepts"]),
        "rejects_when_exhausted": (not expectations["must_reject_final_step"]) or last_decision_reject,
        "rejects_required_steps": all(
            index in rejected_indices
            for index in expectations.get("must_reject_indices", [])
        ),
    }
    return {
        "fixture": fixture["name"],
        "passed": all(checks.values()),
        "checks": checks,
        "degraded_accepts": degraded_accepts,
        "max_consecutive_degraded_accepts": max_consecutive_degraded,
        "rejected_indices": rejected_indices,
    }


def main() -> int:
    args = parse_args()
    fixtures_dir = repo_root / "experiments" / "measurement_acceptance" / "fixtures"
    fixtures = [load_fixture(path) for path in sorted(fixtures_dir.glob("*.json"))]
    sequences_dir = repo_root / "experiments" / "measurement_acceptance" / "fixtures_sequences"
    sequence_fixtures = [load_fixture(path) for path in sorted(sequences_dir.glob("*.json"))]
    variants = []
    for variant_cls in VARIANTS:
        fixture_results = [run_variant_on_fixture(variant_cls, fixture) for fixture in fixtures]
        sequence_results = [
            run_variant_on_sequence(variant_cls, fixture) for fixture in sequence_fixtures
        ]
        passed_units = sum(1 for result in fixture_results if result["passed"]) + sum(
            1 for result in sequence_results if result["passed"]
        )
        total_units = len(fixture_results) + len(sequence_results)
        benchmark_score = 100.0 * passed_units / max(1, total_units)
        static_metrics = compute_static_metrics(variant_cls)
        variant_result = {
            "name": variant_cls.name,
            "design": variant_cls.design,
            "fixture_results": fixture_results,
            "sequence_results": sequence_results,
            "benchmark_score": benchmark_score,
            "readability_score": static_metrics["readability_score"],
            "extensibility_score": static_metrics["extensibility_score"],
            "static_metrics": static_metrics,
        }
        variant_result["overall_score"] = overall_score(
            benchmark_score,
            variant_result["readability_score"],
            variant_result["extensibility_score"],
        )
        variants.append(variant_result)

    variants.sort(key=lambda item: (-item["overall_score"], -item["benchmark_score"], item["name"]))
    results = {
        "problem": "measurement_acceptance",
        "title": "Multi-Criteria Measurement Acceptance",
        "problem_statement": (
            "Accept degraded-but-consistent registration results that the scalar fitness gate "
            "rejects, without accepting stale or contradicted measurements. Motivated by the "
            "Koide outdoor_hard_01a window where the ground-truth pose scores ~9.6 against the "
            "6.0 gate and the scalar gate starts a 300-row reject streak."
        ),
        "generated_from": repo_relative_path(__file__),
        "interface": {
            "methods": [
                "reset() -> None",
                "step(sample: AcceptanceSample) -> AcceptanceDecision",
            ],
            "sample_type": "AcceptanceSample",
            "sample_fields": [
                "index",
                "fitness_score",
                "effective_score_threshold",
                "correction_translation_m",
                "correction_yaw_deg",
                "accepted_gap_sec",
                "consecutive_rejected_updates",
            ],
            "decision_type": "AcceptanceDecision",
            "decision_fields": [
                "reject_measurement",
                "reason",
                "score",
            ],
        },
        "fixtures": [
            {
                "name": fixture["name"],
                "description": fixture["description"],
                "source": fixture["source"],
                "expected_reject": bool(fixture["expectation"]["expected_reject"]),
            }
            for fixture in fixtures
        ],
        "variants": variants,
        "benchmark_score_mean": (
            sum(variant["benchmark_score"] for variant in variants) / max(1, len(variants))
        ),
    }
    output_path = Path(args.output_json)
    output_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
