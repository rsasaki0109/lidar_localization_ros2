#!/usr/bin/env python3
"""Demonstrate consuming the calibrated `/pcl_pose` covariance for arbitration.

This is the runnable artifact behind the competitive-roadmap "Next" gate
*"covariance is documented enough to be consumed by fusion or arbitration
logic"*. It replays the `experiments/pose_arbitration` fixtures through two
consumers -- a naive covariance-only one and the reference covariance + category
one -- and prints, per fixture, what each decided and whether that decision was
correct against ground truth.

The headline it makes concrete: on the calibrated `error_floor` model a
covariance-only consumer is *actively* unsafe, because a low-fitness
degenerate-geometry pose has a tight covariance, so the naive consumer trusts the
poses carrying multi-metre error. The reference consumer, which also watches
`/alignment_status` failure_category / reject streaks / reinit requests exactly
as docs/pose_covariance.md prescribes, rejects them.

Usage::

    python3 scripts/demo_pose_arbitration.py

Exit code is non-zero if the reference consumer makes any incorrect (dangerous)
trust decision, so this doubles as a smoke check.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.pose_arbitration.interface import PoseSample  # noqa: E402
from experiments.pose_arbitration.policy import (  # noqa: E402
    CovarianceOnlyConsumer,
    CovariancePlusCategoryConsumer,
    verdict_is_correct,
)

TRUST_RADIUS_M = 1.0
FIXTURE_DIR = ROOT / "experiments" / "pose_arbitration" / "fixtures"


def load_fixtures():
    fixtures = []
    for path in sorted(FIXTURE_DIR.glob("*.json")):
        data = json.loads(path.read_text())
        s = data["sample"]
        fixtures.append((data["name"], PoseSample(**s)))
    return fixtures


def main() -> int:
    fixtures = load_fixtures()
    consumers = [CovarianceOnlyConsumer(), CovariancePlusCategoryConsumer()]

    header = f"{'fixture':<48} {'true_err':>8} {'cov2sig':>8}  " + "  ".join(
        f"{c.name:>26}" for c in consumers)
    print(header)
    print("-" * len(header))

    wrong = {c.name: 0 for c in consumers}
    for name, sample in fixtures:
        cells = []
        for c in consumers:
            v = c.decide(sample)
            ok = verdict_is_correct(sample, v, TRUST_RADIUS_M)
            if not ok:
                wrong[c.name] += 1
            mark = " " if ok else "!"
            cells.append(f"{v.trust:>10} w={v.weight:0.2f}{mark}")
        two_sigma = 2.0 * sample.cov_xy_std_m
        print(f"{name:<48} {sample.abs_xy_error_m:>7.2f}m {two_sigma:>7.2f}m  "
              + "  ".join(f"{cell:>26}" for cell in cells))

    print()
    print(f"trust radius = {TRUST_RADIUS_M:.1f} m; '!' marks a DANGEROUS trust "
          f"(trusted a pose whose true error exceeds the radius)")
    for c in consumers:
        n = wrong[c.name]
        verdict = "UNSAFE" if n else "safe"
        print(f"  {c.name:<26}: {n} dangerous trust(s)  [{verdict}]")

    ref = CovariancePlusCategoryConsumer().name
    if wrong[ref]:
        print(f"\nFAIL: reference consumer '{ref}' made {wrong[ref]} dangerous "
              f"trust decision(s)")
        return 1
    print(f"\nOK: reference consumer '{ref}' made no dangerous trust decisions; "
          f"covariance-only made {wrong['covariance_only']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
