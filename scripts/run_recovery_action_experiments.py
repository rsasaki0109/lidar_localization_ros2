#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from statistics import mean
from typing import Any

PACKAGE_NAME = "lidar_localization_ros2"


def resolve_repo_root() -> Path:
    script_path = Path(__file__).resolve()
    candidates = [
        script_path.parents[1],
        script_path.parents[3] / "share" / PACKAGE_NAME,
    ]
    for candidate in candidates:
        if (candidate / "experiments" / "recovery_action").is_dir() and (candidate / "docs").is_dir():
            return candidate
    raise RuntimeError(f"Could not resolve repo root from {script_path}")


repo_root = resolve_repo_root()
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from experiments.recovery_action.interface import RecoverySample
from experiments.recovery_action.variants import ConservativeDropStrategy
from experiments.recovery_action.variants import GuardedLastPoseRetryStrategy
from experiments.recovery_action.variants import RejectedSeedReuseStrategy
from experiments.reporting import compute_static_metrics
from experiments.reporting import generate_combined_docs
from experiments.reporting import overall_score
from experiments.reporting import repo_relative_path


VARIANTS = [
    ConservativeDropStrategy,
    RejectedSeedReuseStrategy,
    GuardedLastPoseRetryStrategy,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run offline recovery-action experiments.")
    parser.add_argument(
        "--output-json",
        default=str(repo_root / "experiments" / "recovery_action" / "results.json"),
        help="Where to write the structured experiment result JSON.",
    )
    return parser.parse_args()


def load_fixture(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["fixture_path"] = str(path)
    return data


def to_sample(raw: dict[str, Any]) -> RecoverySample:
    return RecoverySample(
        index=int(raw["index"]),
        failure_kind=str(raw["failure_kind"]),
        fitness_score=float(raw["fitness_score"]),
        correction_translation_m=float(raw["correction_translation_m"]),
        correction_yaw_deg=float(raw["correction_yaw_deg"]),
        seed_translation_since_accept_m=float(raw["seed_translation_since_accept_m"]),
        accepted_gap_sec=float(raw["accepted_gap_sec"]),
        consecutive_rejected_updates=int(raw["consecutive_rejected_updates"]),
    )


def run_variant_on_fixture(variant_cls: type, fixture: dict[str, Any]) -> dict[str, Any]:
    variant = variant_cls()
    variant.reset()
    sample = to_sample(fixture["sample"])
    decision = variant.step(sample)
    expected_action = str(fixture["expectation"]["expected_action"])
    passed = decision.action == expected_action
    return {
        "fixture": fixture["name"],
        "passed": passed,
        "outcome": "matched_expected_action" if passed else "mismatched_expected_action",
        "decision": decision.action,
        "decision_reason": decision.reason,
        "decision_score": decision.score,
        "expected_action": expected_action
    }


def main() -> int:
    args = parse_args()
    fixtures_dir = repo_root / "experiments" / "recovery_action" / "fixtures"
    fixtures = [load_fixture(path) for path in sorted(fixtures_dir.glob("*.json"))]
    variants = []
    for variant_cls in VARIANTS:
        fixture_results = [run_variant_on_fixture(variant_cls, fixture) for fixture in fixtures]
        benchmark_score = 100.0 * (
            sum(1 for result in fixture_results if result["passed"]) / max(1, len(fixture_results))
        )
        static_metrics = compute_static_metrics(variant_cls)
        variant_result = {
            "name": variant_cls.name,
            "design": variant_cls.design,
            "fixture_results": fixture_results,
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
        "problem": "recovery_action",
        "title": "Recovery Action Selection",
        "problem_statement": "Choose whether to keep open-loop prediction, reuse a rejected seed, or retry from the last accepted pose after a failed measurement.",
        "generated_from": repo_relative_path(__file__),
        "interface": {
            "methods": [
                "reset() -> None",
                "step(sample: RecoverySample) -> RecoveryDecision",
            ],
            "sample_type": "RecoverySample",
            "sample_fields": [
                "index",
                "failure_kind",
                "fitness_score",
                "correction_translation_m",
                "correction_yaw_deg",
                "seed_translation_since_accept_m",
                "accepted_gap_sec",
                "consecutive_rejected_updates",
            ],
            "decision_type": "RecoveryDecision",
            "decision_fields": [
                "action",
                "reason",
                "score",
            ],
        },
        "fixtures": [
            {
                "name": fixture["name"],
                "description": fixture["description"],
            }
            for fixture in fixtures
        ],
        "variants": variants,
        "benchmark_score_mean": mean(item["benchmark_score"] for item in variants),
    }

    output_path = Path(args.output_json).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(results, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    generate_combined_docs(repo_root)
    print(json.dumps(results, indent=2, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
