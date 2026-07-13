#!/usr/bin/env python3

import math

from experiments.kiss_matcher_verifier.interface import Candidate
from experiments.kiss_matcher_verifier.interface import GlobalEstimate
from experiments.kiss_matcher_verifier.variants import BbsFirstSelector
from experiments.kiss_matcher_verifier.variants import KissNearestSelector


def candidates():
    return [
        Candidate("a", 0, 10.0, 0.0, 0.0, False),
        Candidate("a", 1, 1.0, 2.0, 0.1, True),
    ]


def test_bbs_selector_preserves_candidate_index_order():
    selected = BbsFirstSelector().select(reversed(candidates()))
    assert selected.accepted
    assert selected.candidate.candidate_index == 0


def test_kiss_selector_recovers_nearest_top_k_candidate():
    estimate = GlobalEstimate(True, 1.1, 2.1, 0.12, 12, 0.2)
    selected = KissNearestSelector().select(candidates(), estimate)
    assert selected.accepted
    assert selected.candidate.candidate_index == 1


def test_kiss_selector_abstains_on_weak_or_distant_estimate():
    weak = GlobalEstimate(True, 1.0, 2.0, 0.1, 4, 0.2)
    assert KissNearestSelector().select(candidates(), weak).reason == "insufficient_final_inliers"
    distant = GlobalEstimate(True, 50.0, 50.0, math.pi, 20, 0.2)
    result = KissNearestSelector().select(candidates(), distant)
    assert not result.accepted
    assert result.reason == "translation_delta_above_threshold"


if __name__ == "__main__":
    test_bbs_selector_preserves_candidate_index_order()
    test_kiss_selector_recovers_nearest_top_k_candidate()
    test_kiss_selector_abstains_on_weak_or_distant_estimate()
