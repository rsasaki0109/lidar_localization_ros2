#!/usr/bin/env python3

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import g2_candidate_registration_rank_policy as policy  # noqa: E402


def _candidate(x, y, bbs_score, fitness):
    return policy.RankedG2Candidate(
        x_m=x,
        y_m=y,
        z_m=0.0,
        yaw_rad=0.0,
        bbs_score=bbs_score,
        registration_fitness=fitness,
        score=0.0,
        hit_count=100,
        point_count=512,
        registration_converged=fitness is not None and math.isfinite(fitness),
    )


def test_registration_fitness_to_supervisor_score():
    assert policy.registration_fitness_to_supervisor_score(0.0) == 1.0
    assert policy.registration_fitness_to_supervisor_score(3.0) == 0.5
    assert policy.registration_fitness_to_supervisor_score(6.0) == 0.0
    assert policy.registration_fitness_to_supervisor_score(27.0) == 0.0
    assert policy.registration_fitness_to_supervisor_score(None) == 0.0


def test_ranking_prefers_low_fitness_over_high_bbs_score():
    aliased = _candidate(10.0, 20.0, bbs_score=0.99, fitness=27.0)
    correct = _candidate(11.0, 21.0, bbs_score=0.85, fitness=0.23)
    ranked = policy.apply_registration_ranking([aliased, correct])
    assert ranked[0].x_m == 11.0
    assert ranked[0].score > ranked[1].score
    assert ranked[1].score == 0.0


def test_apply_registration_ranking_sets_supervisor_scores():
    ranked = policy.apply_registration_ranking(
        [_candidate(0.0, 0.0, 0.9, 1.5)], score_gate=6.0)
    assert ranked[0].score == 0.75


def test_bbs_only_candidates_preserve_score():
    wrapped = policy.bbs_only_candidates([(1.0, 2.0, 3.0, 0.1, 0.88, 10, 20)])
    assert wrapped[0].score == 0.88
    assert wrapped[0].bbs_score == 0.88
    assert wrapped[0].registration_fitness is None


def test_candidate_pose_from_registration_uses_refined_pose_when_converged():
    raw = policy.RankedG2Candidate(
        x_m=-13.5,
        y_m=29.7,
        z_m=0.0,
        yaw_rad=math.radians(10.0),
        bbs_score=0.99,
        registration_fitness=0.042,
        score=0.99,
    )
    pose = policy.candidate_pose_from_registration(
        raw,
        converged=True,
        fitness=0.042,
        refined_x=-10.5,
        refined_y=27.1,
        refined_z=1.2,
        refined_yaw=math.radians(12.0),
    )
    assert pose == (-10.5, 27.1, 1.2, math.radians(12.0))


def test_candidate_pose_from_registration_keeps_raw_pose_when_not_converged():
    raw = policy.RankedG2Candidate(
        x_m=-13.5,
        y_m=29.7,
        z_m=0.0,
        yaw_rad=math.radians(10.0),
        bbs_score=0.99,
        registration_fitness=float("inf"),
        score=0.99,
        registration_converged=False,
    )
    pose = policy.candidate_pose_from_registration(
        raw,
        converged=False,
        fitness=None,
        refined_x=-10.5,
        refined_y=27.1,
        refined_z=1.2,
        refined_yaw=math.radians(12.0),
    )
    assert pose == (-13.5, 29.7, 0.0, math.radians(10.0))


if __name__ == "__main__":
    test_registration_fitness_to_supervisor_score()
    test_ranking_prefers_low_fitness_over_high_bbs_score()
    test_apply_registration_ranking_sets_supervisor_scores()
    test_bbs_only_candidates_preserve_score()
    test_candidate_pose_from_registration_uses_refined_pose_when_converged()
    test_candidate_pose_from_registration_keeps_raw_pose_when_not_converged()
    print("test_g2_candidate_registration_rank_policy: all tests passed")
