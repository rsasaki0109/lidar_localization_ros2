#!/usr/bin/env python3
"""Keep the odometry-dropout confirmation experiment reproducible.

The experiment lives under experiments/ and changes no runtime behavior. These
tests pin that its committed results regenerate exactly, and that its copy of the
runtime policy still shows the recorded failure and the recovery it was built for.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "experiments" / "odometry_dropout_confirmation"
sys.path.insert(0, str(EXPERIMENT))

import dropout_confirmation as dc


def test_committed_results_regenerate():
    committed = json.loads((EXPERIMENT / "results.json").read_text(encoding="utf-8"))
    assert dc.run() == committed, (
        "results.json is stale; run dropout_confirmation.py --json results.json"
    )


def test_runtime_baseline_resets_far_after_a_short_dropout():
    fixtures = {f.name: f for f in dc.build_fixtures()}
    params = dc.Params()

    after = dc.simulate(
        dc.window_waiver, fixtures["hard02b_wrong_answer_after_short_dropout"], params
    )
    assert after["outcome"] == "wrong_reset"
    assert after["trace"] == ["106:reset:dropout_waiver"]

    # The waiver is what lets a carried sensor recover on the first answer.
    carry = dc.simulate(dc.window_waiver, fixtures["kidnap_covered_carry"], params)
    assert carry["outcome"] == "correct_reset"
    assert carry["time_to_recover_sec"] == 5.0


def test_no_variant_changes_behavior_without_external_odometry():
    fixture = next(f for f in dc.build_fixtures() if f.name == "no_external_odometry")
    for _, function in dc.VARIANTS.values():
        result = dc.simulate(function, fixture, dc.Params())
        assert result["trace"] == ["30:reset:no_bridged_pose"]
