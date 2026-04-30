#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ast
import inspect
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
        if (candidate / "experiments" / "imu_guard").is_dir() and (candidate / "docs").is_dir():
            return candidate
    raise RuntimeError(f"Could not resolve repo root from {script_path}")


repo_root = resolve_repo_root()
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from experiments.imu_guard.interface import GuardSample
from experiments.reporting import generate_combined_docs
from experiments.reporting import repo_relative_path
from experiments.imu_guard.variants import AbsoluteThresholdGuard
from experiments.imu_guard.variants import ScoreBudgetGuard
from experiments.imu_guard.variants import StreakGuard


VARIANTS = [
    AbsoluteThresholdGuard,
    StreakGuard,
    ScoreBudgetGuard,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run offline IMU guard experiments.")
    parser.add_argument(
        "--output-json",
        default=str(repo_root / "experiments" / "imu_guard" / "results.json"),
        help="Where to write the structured experiment result JSON.",
    )
    return parser.parse_args()


def clamp_score(value: float) -> float:
    return max(0.0, min(100.0, value))


def load_fixture(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["fixture_path"] = str(path)
    return data


def to_sample(raw: dict[str, Any]) -> GuardSample:
    return GuardSample(
        index=int(raw["index"]),
        status=str(raw["status"]),
        fitness_score=float(raw["fitness_score"]),
        alignment_time_sec=float(raw["alignment_time_sec"]),
        imu_prediction_active=bool(raw["imu_prediction_active"]),
        consecutive_rejected_updates=int(raw["consecutive_rejected_updates"]),
        seed_translation_since_accept_m=float(raw["seed_translation_since_accept_m"]),
        seed_yaw_since_accept_deg=float(raw["seed_yaw_since_accept_deg"]),
        accepted_gap_sec=float(raw["accepted_gap_sec"]),
        correction_translation_m=float(raw["correction_translation_m"]),
        correction_yaw_deg=float(raw["correction_yaw_deg"]),
    )


def evaluate_expectation(trigger_index: int | None, expectation: dict[str, Any]) -> tuple[bool, str]:
    if expectation.get("must_not_trigger"):
        passed = trigger_index is None
        return passed, "must_not_trigger"

    min_index = expectation.get("must_not_trigger_before_index")
    max_index = expectation.get("should_trigger_by_index")
    if trigger_index is None:
        return False, "missing_trigger"
    if min_index is not None and trigger_index < int(min_index):
        return False, "triggered_too_early"
    if max_index is not None and trigger_index > int(max_index):
        return False, "triggered_too_late"
    return True, "within_expected_window"


def run_variant_on_fixture(variant_cls: type, fixture: dict[str, Any]) -> dict[str, Any]:
    variant = variant_cls()
    variant.reset()
    trigger_index = None
    trigger_reason = ""
    decision_log = []
    for raw in fixture["samples"]:
        sample = to_sample(raw)
        decision = variant.step(sample)
        decision_log.append(
            {
                "sample_index": sample.index,
                "status": sample.status,
                "disable_imu_preintegration": decision.disable_imu_preintegration,
                "reason": decision.reason,
                "score": decision.score,
            }
        )
        if trigger_index is None and decision.disable_imu_preintegration:
            trigger_index = sample.index
            trigger_reason = decision.reason
    passed, outcome = evaluate_expectation(trigger_index, fixture["expectation"])
    return {
        "fixture": fixture["name"],
        "passed": passed,
        "outcome": outcome,
        "trigger_index": trigger_index,
        "trigger_reason": trigger_reason,
        "decision_log": decision_log,
    }


def max_nesting_depth(node: ast.AST, depth: int = 0) -> int:
    child_depths = [depth]
    for child in ast.iter_child_nodes(node):
        next_depth = depth + 1 if isinstance(child, (ast.If, ast.For, ast.While, ast.Try, ast.With)) else depth
        child_depths.append(max_nesting_depth(child, next_depth))
    return max(child_depths)


def compute_static_metrics(variant_cls: type) -> dict[str, Any]:
    source = inspect.getsource(variant_cls)
    tree = ast.parse(source)
    class_node = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    loc = sum(1 for line in source.splitlines() if line.strip() and not line.strip().startswith("#"))
    branch_count = sum(
        1
        for node in ast.walk(class_node)
        if isinstance(node, (ast.If, ast.For, ast.While, ast.BoolOp, ast.Try))
    )
    state_field_count = 0
    public_methods = 0
    for node in class_node.body:
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            public_methods += 1
        if isinstance(node, ast.FunctionDef):
            for subnode in ast.walk(node):
                if isinstance(subnode, ast.Assign):
                    for target in subnode.targets:
                        if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == "self":
                            state_field_count += 1
    top_tree = ast.parse(Path(inspect.getsourcefile(variant_cls)).read_text(encoding="utf-8"))
    import_count = sum(1 for node in top_tree.body if isinstance(node, (ast.Import, ast.ImportFrom)))
    has_config_dataclass = "class Config" in Path(inspect.getsourcefile(variant_cls)).read_text(encoding="utf-8")
    readability = clamp_score(100.0 - (1.2 * loc) - (5.0 * branch_count) - (4.0 * max_nesting_depth(class_node)))
    extensibility = clamp_score(
        55.0
        + (15.0 if has_config_dataclass else 0.0)
        + max(0.0, 15.0 - 5.0 * state_field_count)
        + max(0.0, 10.0 - 2.0 * max(0, public_methods - 2))
        + max(0.0, 5.0 - 1.0 * max(0, import_count - 3))
    )
    return {
        "implementation_file": repo_relative_path(inspect.getsourcefile(variant_cls)),
        "loc": loc,
        "branch_count": branch_count,
        "max_nesting_depth": max_nesting_depth(class_node),
        "state_field_count": state_field_count,
        "public_method_count": public_methods,
        "import_count": import_count,
        "has_config_dataclass": has_config_dataclass,
        "readability_score": readability,
        "extensibility_score": extensibility,
    }


def overall_scores(benchmark_score: float, readability_score: float, extensibility_score: float) -> float:
    return round(0.60 * benchmark_score + 0.20 * readability_score + 0.20 * extensibility_score, 2)


def write_interfaces_doc(fixtures: list[dict[str, Any]], variants: list[dict[str, Any]]) -> None:
    path = repo_root / "docs" / "interfaces.md"
    lines = [
        "# Interfaces",
        "",
        "## Stable Boundary",
        "",
        "- `repo/src/` and `repo/include/` remain the runtime core.",
        "- `repo/experiments/` is discardable space for competing implementations.",
        "- promotion rule: only the winning behavior graduates into core, never the whole experiment framework.",
        "",
        "## Minimal Experiment Interface",
        "",
        "Every candidate implementation must expose exactly this surface:",
        "",
        "- `reset() -> None`",
        "- `step(sample: GuardSample) -> GuardDecision`",
        "",
        "### `GuardSample` fields",
        "",
        "- `index`",
        "- `status`",
        "- `fitness_score`",
        "- `alignment_time_sec`",
        "- `imu_prediction_active`",
        "- `consecutive_rejected_updates`",
        "- `seed_translation_since_accept_m`",
        "- `seed_yaw_since_accept_deg`",
        "- `accepted_gap_sec`",
        "- `correction_translation_m`",
        "- `correction_yaw_deg`",
        "",
        "### `GuardDecision` fields",
        "",
        "- `disable_imu_preintegration`",
        "- `reason`",
        "- `score`",
        "",
        "## Shared Fixtures",
        "",
    ]
    for fixture in fixtures:
        lines.append(f"- `{fixture['name']}`: {fixture['description']}")
    lines += [
        "",
        "## Candidate Families",
        "",
    ]
    for variant in variants:
        lines.append(f"- `{variant['name']}`: {variant['design']}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_experiments_doc(results: dict[str, Any]) -> None:
    path = repo_root / "docs" / "experiments.md"
    lines = [
        "# Experiments",
        "",
        "## Problem",
        "",
        "Disable the IMU-preintegration path early enough to avoid poisoning localization, without false positives on no-IMU traces.",
        "",
        "## Comparison Rules",
        "",
        "- same interface",
        "- same fixture set",
        "- same benchmark pass/fail rubric",
        "- same readability and extensibility heuristics",
        "",
        "## Results",
        "",
        "| Variant | Design | Benchmark | Readability | Extensibility | Overall |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for variant in results["variants"]:
        lines.append(
            f"| `{variant['name']}` | {variant['design']} | "
            f"{variant['benchmark_score']:.1f} | {variant['readability_score']:.1f} | "
            f"{variant['extensibility_score']:.1f} | {variant['overall_score']:.2f} |"
        )
    lines += [
        "",
        "## Fixture Outcomes",
        "",
    ]
    for variant in results["variants"]:
        lines.append(f"### `{variant['name']}`")
        for outcome in variant["fixture_results"]:
            lines.append(
                f"- `{outcome['fixture']}`: pass=`{outcome['passed']}` "
                f"trigger_index=`{outcome['trigger_index']}` reason=`{outcome['trigger_reason'] or outcome['outcome']}`"
            )
        lines.append("")
    path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def write_decisions_doc(results: dict[str, Any]) -> None:
    best = results["variants"][0]
    runner_up = results["variants"][1] if len(results["variants"]) > 1 else None
    path = repo_root / "docs" / "decisions.md"
    lines = [
        "# Decisions",
        "",
        "## Current Problem Decision",
        "",
        f"- adopted variant for the current IMU correction-guard problem: `{best['name']}`",
        f"- design family: {best['design']}",
        f"- rationale: highest combined score (`{best['overall_score']}`) under the shared fixture/evaluation contract",
        "",
        "## Why This Variant",
        "",
        f"- benchmark score: `{best['benchmark_score']:.1f}`",
        f"- readability score: `{best['readability_score']:.1f}`",
        f"- extensibility score: `{best['extensibility_score']:.1f}`",
    ]
    if runner_up is not None:
        lines.append(
            f"- nearest alternative: `{runner_up['name']}` at `{runner_up['overall_score']}`"
        )
    lines += [
        "",
        "## Rejection Rule",
        "",
        "- variants are not deleted because they are wrong; they stay in `repo/experiments/` until a stronger comparison says otherwise",
        "- only the chosen behavior is allowed to influence runtime core code",
        "",
        "## Process Rule",
        "",
        "- new guard work must land as multiple comparable variants first",
        "- abstraction is only allowed after two or more variants share the same irreducible boundary",
        "- docs/experiments.md, docs/decisions.md, and docs/interfaces.md are updated by rerunning the experiment script",
        "",
        f"- generated_from: `{results['generated_from']}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    fixtures_dir = repo_root / "experiments" / "imu_guard" / "fixtures"
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
        variant_result["overall_score"] = overall_scores(
            benchmark_score,
            variant_result["readability_score"],
            variant_result["extensibility_score"],
        )
        variants.append(variant_result)

    variants.sort(key=lambda item: (-item["overall_score"], -item["benchmark_score"], item["name"]))
    results = {
        "problem": "imu_guard",
        "title": "IMU Correction Guard",
        "problem_statement": "Disable the IMU-preintegration path early enough to avoid poisoning localization, without false positives on no-IMU traces.",
        "generated_from": repo_relative_path(__file__),
        "interface": {
            "methods": [
                "reset() -> None",
                "step(sample: GuardSample) -> GuardDecision",
            ],
            "sample_type": "GuardSample",
            "sample_fields": [
                "index",
                "status",
                "fitness_score",
                "alignment_time_sec",
                "imu_prediction_active",
                "consecutive_rejected_updates",
                "seed_translation_since_accept_m",
                "seed_yaw_since_accept_deg",
                "accepted_gap_sec",
                "correction_translation_m",
                "correction_yaw_deg",
            ],
            "decision_type": "GuardDecision",
            "decision_fields": [
                "disable_imu_preintegration",
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
        "fixture_names": [fixture["name"] for fixture in fixtures],
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
