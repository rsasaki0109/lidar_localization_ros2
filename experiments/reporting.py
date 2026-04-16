from __future__ import annotations

import ast
import inspect
import json
from pathlib import Path
from typing import Any


def clamp_score(value: float) -> float:
    return max(0.0, min(100.0, value))


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
    source_file = Path(inspect.getsourcefile(variant_cls))
    top_tree = ast.parse(source_file.read_text(encoding="utf-8"))
    import_count = sum(1 for node in top_tree.body if isinstance(node, (ast.Import, ast.ImportFrom)))
    has_config_dataclass = "class Config" in source_file.read_text(encoding="utf-8")
    readability = clamp_score(100.0 - (1.2 * loc) - (5.0 * branch_count) - (4.0 * max_nesting_depth(class_node)))
    extensibility = clamp_score(
        55.0
        + (15.0 if has_config_dataclass else 0.0)
        + max(0.0, 15.0 - 5.0 * state_field_count)
        + max(0.0, 10.0 - 2.0 * max(0, public_methods - 2))
        + max(0.0, 5.0 - 1.0 * max(0, import_count - 3))
    )
    return {
        "implementation_file": str(source_file.resolve()),
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


def overall_score(benchmark_score: float, readability_score: float, extensibility_score: float) -> float:
    return round(0.60 * benchmark_score + 0.20 * readability_score + 0.20 * extensibility_score, 2)


def load_problem_results(repo_root: Path) -> list[dict[str, Any]]:
    results = []
    for path in sorted(repo_root.glob("experiments/*/results.json")):
        result = json.loads(path.read_text(encoding="utf-8"))
        if result.get("variants"):
            results.append(result)
    return results


def write_interfaces_doc(repo_root: Path, results: list[dict[str, Any]]) -> None:
    path = repo_root / "docs" / "interfaces.md"
    lines = [
        "# Interfaces",
        "",
        "## Stable Boundary",
        "",
        "- `repo/src/` and `repo/include/` remain the runtime core.",
        "- `repo/experiments/` is discardable space for competing implementations.",
        "- promotion rule: only the winning behavior graduates into core, never the whole experiment scaffold.",
        "",
    ]
    for result in results:
        interface = result["interface"]
        lines += [
            f"## {result['title']}",
            "",
            result["problem_statement"],
            "",
            "### Minimal Interface",
            "",
        ]
        for method in interface["methods"]:
            lines.append(f"- `{method}`")
        lines += [
            "",
            f"### `{interface['sample_type']}` fields",
            "",
        ]
        for field in interface["sample_fields"]:
            lines.append(f"- `{field}`")
        lines += [
            "",
            f"### `{interface['decision_type']}` fields",
            "",
        ]
        for field in interface["decision_fields"]:
            lines.append(f"- `{field}`")
        lines += [
            "",
            "### Shared Fixtures",
            "",
        ]
        for fixture in result["fixtures"]:
            lines.append(f"- `{fixture['name']}`: {fixture['description']}")
        lines += [
            "",
            "### Candidate Families",
            "",
        ]
        for variant in result["variants"]:
            lines.append(f"- `{variant['name']}`: {variant['design']}")
        lines += ["", ""]
    path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def write_experiments_doc(repo_root: Path, results: list[dict[str, Any]]) -> None:
    path = repo_root / "docs" / "experiments.md"
    lines = [
        "# Experiments",
        "",
        "## Operating Rules",
        "",
        "- same interface inside each problem",
        "- same fixture set inside each problem",
        "- same benchmark pass/fail rubric inside each problem",
        "- same readability and extensibility heuristics across problems",
        "",
    ]
    for result in results:
        lines += [
            f"## {result['title']}",
            "",
            result["problem_statement"],
            "",
            "| Variant | Design | Benchmark | Readability | Extensibility | Overall |",
            "|---|---|---:|---:|---:|---:|",
        ]
        for variant in result["variants"]:
            lines.append(
                f"| `{variant['name']}` | {variant['design']} | "
                f"{variant['benchmark_score']:.1f} | {variant['readability_score']:.1f} | "
                f"{variant['extensibility_score']:.1f} | {variant['overall_score']:.2f} |"
            )
        lines += [
            "",
            "### Fixture Outcomes",
            "",
        ]
        for variant in result["variants"]:
            lines.append(f"#### `{variant['name']}`")
            for outcome in variant["fixture_results"]:
                decision = outcome.get("decision")
                if decision is None:
                    trigger_index = outcome.get("trigger_index")
                    decision = f"trigger_at_{trigger_index}" if trigger_index is not None else "no_trigger"
                decision_reason = outcome.get("decision_reason")
                if decision_reason is None:
                    decision_reason = outcome.get("trigger_reason") or outcome.get("outcome")
                lines.append(
                    f"- `{outcome['fixture']}`: pass=`{outcome['passed']}` "
                    f"decision=`{decision}` reason=`{decision_reason}`"
                )
            lines.append("")
    path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def write_decisions_doc(repo_root: Path, results: list[dict[str, Any]]) -> None:
    path = repo_root / "docs" / "decisions.md"
    lines = [
        "# Decisions",
        "",
        "## Process Rule",
        "",
        "- new behavior work lands as multiple comparable variants first",
        "- abstraction is only allowed after two or more variants share the same irreducible boundary",
        "- `docs/interfaces.md`, `docs/experiments.md`, and `docs/decisions.md` are regenerated from experiment results, not handwritten",
        "",
    ]
    for result in results:
        best = result["variants"][0]
        runner_up = result["variants"][1] if len(result["variants"]) > 1 else None
        lines += [
            f"## {result['title']}",
            "",
            f"- adopted variant for this problem: `{best['name']}`",
            f"- design family: {best['design']}",
            f"- rationale: highest combined score (`{best['overall_score']}`) under the shared fixture/evaluation contract",
            f"- benchmark score: `{best['benchmark_score']:.1f}`",
            f"- readability score: `{best['readability_score']:.1f}`",
            f"- extensibility score: `{best['extensibility_score']:.1f}`",
        ]
        if runner_up is not None:
            lines.append(f"- nearest alternative: `{runner_up['name']}` at `{runner_up['overall_score']}`")
        lines += [
            f"- generated_from: `{result['generated_from']}`",
            "",
        ]
    path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def generate_combined_docs(repo_root: Path) -> list[dict[str, Any]]:
    results = load_problem_results(repo_root)
    write_interfaces_doc(repo_root, results)
    write_experiments_doc(repo_root, results)
    write_decisions_doc(repo_root, results)
    return results
