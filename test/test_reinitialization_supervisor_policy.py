#!/usr/bin/env python3
"""Regression tests for the G3 reinitialization guard policy.

These encode the Decision Gates from docs/global_localization_roadmap.md as
adversarial sequences. They are the "regression test that fails on unsafe
publication or false acceptance" the roadmap requires, and they exist because
the Phase 3 degraded-acceptance work proved that a closed recovery loop can
diverge if its bounds can be reset by the very acts they bound.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import reinitialization_supervisor_policy as rsp  # noqa: E402


def run(
    params,
    ticks,
    *,
    requested,
    candidate_score=None,
    query_latency=1,
    fitness=None,
    dt=1.0,
):
    """Drive the policy like the node would and return the event stream.

    ``requested``/``candidate_score`` may be constants or callables of ``now``.
    A query reply (``candidate_score``) is delivered exactly once, ``query_latency``
    ticks after each ACTION_QUERY, mimicking a single service response.
    ``fitness`` is a callable(now, last_reset_sec) or constant (lower is better).
    """
    state = rsp.initial_state()
    events = []
    deliver_at = None
    last_reset_sec = None

    def value(spec, *args):
        return spec(*args) if callable(spec) else spec

    for i in range(ticks):
        now = i * dt
        req = bool(value(requested, now))
        score = None
        if deliver_at is not None and i == deliver_at:
            score = value(candidate_score, now)
            deliver_at = None
        fit = None
        if fitness is not None:
            fit = fitness(now, last_reset_sec) if callable(fitness) else fitness
        obs = rsp.SupervisorObservation(now, req, best_candidate_score=score, best_fitness=fit)
        dec = rsp.decide(params, state, obs)
        state = dec.state
        events.append((now, dec.action, dec.reason, state.name))
        if dec.action == rsp.ACTION_QUERY:
            deliver_at = i + query_latency
        if dec.action == rsp.ACTION_PUBLISH_RESET:
            last_reset_sec = now
    return events


def actions(events):
    # Tolerates both the 4-tuple events from run() and the 5-tuple events from
    # run_ranked() (which also carries the published candidate_index).
    return [e[1] for e in events]


def reset_times(events):
    return [e[0] for e in events if e[1] == rsp.ACTION_PUBLISH_RESET]


# --- Gate: nothing happens without a sustained request -----------------------

def test_transient_request_blip_never_queries_or_publishes():
    params = rsp.SupervisorParams(request_debounce_sec=2.0)
    # Request asserted for one second only (< debounce), then gone.
    events = run(params, 30, requested=lambda now: now < 1.0, candidate_score=0.99)
    assert rsp.ACTION_QUERY not in actions(events)
    assert rsp.ACTION_PUBLISH_RESET not in actions(events)


def test_no_request_is_pure_tracking():
    events = run(rsp.SupervisorParams(), 20, requested=False, candidate_score=0.99)
    assert set(actions(events)) == {rsp.ACTION_NONE}


# --- Gate: happy path publishes once and stands down on recovery -------------

def test_sustained_request_strong_candidate_publishes_once_then_recovers():
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, recovery_fitness_threshold=1.5)

    # Fitness is bad until a reset lands, then recovers a couple ticks later.
    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        return 0.4 if now - last_reset_sec >= 2.0 else 9.0

    events = run(params, 40, requested=True, candidate_score=0.95, fitness=fitness)
    assert len(reset_times(events)) == 1
    reasons = [r for (_, _, r, _) in events]
    assert "recovery_confirmed" in reasons
    # Even though the request line stays high after recovery, the supervisor
    # stands down and does not re-fire (edge-triggered on the next problem).
    recovery_i = reasons.index("recovery_confirmed")
    after = actions(events)[recovery_i + 1:]
    assert rsp.ACTION_QUERY not in after
    assert rsp.ACTION_PUBLISH_RESET not in after
    assert events[-1][3] == rsp.STATE_STANDDOWN


# --- Gate: never publish an unsafe (low-score) candidate ---------------------

def test_weak_candidates_are_never_published():
    params = rsp.SupervisorParams(min_candidate_score=0.6, max_attempts=3)
    events = run(params, 80, requested=True, candidate_score=0.2, fitness=9.0)
    assert rsp.ACTION_PUBLISH_RESET not in actions(events)
    # And it must not query forever: it gives up after max_attempts.
    assert rsp.ACTION_GIVE_UP in actions(events)
    assert events[-1][3] == rsp.STATE_EXHAUSTED


def test_query_count_is_bounded_by_max_attempts_when_candidates_weak():
    params = rsp.SupervisorParams(min_candidate_score=0.6, max_attempts=3)
    events = run(params, 200, requested=True, candidate_score=0.2, fitness=9.0)
    assert actions(events).count(rsp.ACTION_QUERY) <= params.max_attempts


# --- Gate: a confidently-wrong candidate cannot loop forever -----------------

def test_false_acceptance_does_not_loop_forever():
    # Strong score every time, but fitness never recovers: the classic
    # closed-loop blowup. Must be bounded by max_attempts, then give up.
    params = rsp.SupervisorParams(min_candidate_score=0.6, max_attempts=3, settle_timeout_sec=4.0)
    events = run(params, 300, requested=True, candidate_score=0.99, fitness=9.0)
    assert len(reset_times(events)) <= params.max_attempts
    assert rsp.ACTION_GIVE_UP in actions(events)
    assert events[-1][3] == rsp.STATE_EXHAUSTED


# --- Gate: reset publications respect the minimum spacing --------------------

def test_resets_respect_minimum_spacing():
    params = rsp.SupervisorParams(
        min_seconds_between_attempts=5.0, settle_timeout_sec=3.0, max_attempts=5)
    events = run(params, 300, requested=True, candidate_score=0.99, fitness=9.0, dt=1.0)
    times = reset_times(events)
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(g >= 5.0 for g in gaps), gaps


# --- Gate: recovery clears the ceiling so a later problem gets a fresh budget -

def test_recovery_resets_attempt_budget_for_a_later_problem():
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, max_attempts=3, min_seconds_between_attempts=3.0,
        settle_timeout_sec=3.0, recovery_fitness_threshold=1.5)

    # Two separate problem episodes separated by a healthy stretch.
    def requested(now):
        return now < 20.0 or 40.0 <= now < 60.0

    # Recovers ~2 s after each reset.
    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        return 0.4 if now - last_reset_sec >= 2.0 else 9.0

    events = run(params, 80, requested=requested, candidate_score=0.95, fitness=fitness)
    # One reset per episode, never exhausted (budget reset by the recovery between).
    assert len(reset_times(events)) == 2
    assert rsp.ACTION_GIVE_UP not in actions(events)


# --- Gate: exhausted latches until the problem clears, then re-arms -----------

def test_exhausted_latches_until_request_clears():
    params = rsp.SupervisorParams(min_candidate_score=0.6, max_attempts=2)

    # Unsolvable while requested for the first 120 s, then the request clears.
    def requested(now):
        return now < 120.0

    events = run(params, 160, requested=requested, candidate_score=0.2, fitness=9.0)
    states = [s for (_, _, _, s) in events]
    assert rsp.STATE_EXHAUSTED in states
    # Once the request clears it must fall back to idle (re-armable), not stay stuck.
    assert events[-1][3] == rsp.STATE_IDLE


def test_decide_is_pure_same_inputs_same_outputs():
    params = rsp.SupervisorParams()
    state = rsp.initial_state()
    obs = rsp.SupervisorObservation(3.0, True, best_candidate_score=0.9, best_fitness=2.0)
    d1 = rsp.decide(params, state, obs)
    d2 = rsp.decide(params, state, obs)
    assert d1 == d2
    # Input state object is not mutated.
    assert state == rsp.initial_state()


# --- Gate: ranked-candidate walk (the 2026-06-15 live-run lesson) -------------
#
# A query's top candidate can be confidently wrong (BBS occupancy score does not
# separate a 190 m error from a 5 m hit). The supervisor must walk the ranked list
# from one query, using the localizer's fitness as the registration oracle, before
# spending another attempt -- and the bounds must still hold.

def run_ranked(
    params,
    ticks,
    *,
    requested,
    candidate_scores,
    recover_on_index=None,
    query_latency=1,
    dt=1.0,
):
    """Drive the policy delivering a ranked candidate list once per query.

    ``recover_on_index`` is the rank index whose published reset the localizer
    accepts (fitness drops); None means no candidate ever recovers. Fitness is high
    while any other (or no) candidate is the most recently published one.
    """
    state = rsp.initial_state()
    events = []
    deliver_at = None
    published_index = None

    for i in range(ticks):
        now = i * dt
        req = bool(requested(now)) if callable(requested) else bool(requested)
        scores = None
        if deliver_at is not None and i == deliver_at:
            scores = tuple(candidate_scores)
            deliver_at = None
        fit = None
        if published_index is not None:
            fit = 0.2 if published_index == recover_on_index else 9.0
        obs = rsp.SupervisorObservation(
            now, req, candidate_scores=scores, best_fitness=fit)
        dec = rsp.decide(params, state, obs)
        state = dec.state
        events.append((now, dec.action, dec.reason, state.name, dec.candidate_index))
        if dec.action == rsp.ACTION_QUERY:
            deliver_at = i + query_latency
            published_index = None
        if dec.action == rsp.ACTION_PUBLISH_RESET:
            published_index = dec.candidate_index
    return events


def published_indices(events):
    return [ci for (_, a, _, _, ci) in events if a == rsp.ACTION_PUBLISH_RESET]


def test_walk_recovers_on_lower_ranked_candidate_within_one_query():
    # Top two candidates are aliased-wrong; rank 2 is the true pose. A single query
    # (max_attempts=1) must walk 0 -> 1 -> 2 and recover, without giving up: walking
    # does not spend attempts, so the ceiling is never hit.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, settle_timeout_sec=3.0,
        max_attempts=1, recovery_fitness_threshold=1.5)
    events = run_ranked(
        params, 60, requested=True,
        candidate_scores=[0.99, 0.98, 0.97], recover_on_index=2)
    assert published_indices(events) == [0, 1, 2]
    assert actions(events).count(rsp.ACTION_QUERY) == 1
    assert rsp.ACTION_GIVE_UP not in actions(events)
    reasons = [r for (_, _, r, _, _) in events]
    assert "next_candidate" in reasons
    assert "recovery_confirmed" in reasons
    assert events[-1][3] == rsp.STATE_STANDDOWN


def test_walk_exhausts_list_then_respects_max_attempts():
    # No candidate ever recovers. Each query walks its whole list (2 candidates),
    # then a fresh query costs one attempt. The reset count must stay bounded by
    # max_attempts * list_length, and it must give up -- never loop forever.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, settle_timeout_sec=3.0,
        min_seconds_between_attempts=2.0, max_attempts=2)
    events = run_ranked(
        params, 300, requested=True,
        candidate_scores=[0.99, 0.98], recover_on_index=None)
    assert actions(events).count(rsp.ACTION_QUERY) <= params.max_attempts
    # Two candidates walked per query, at most max_attempts queries.
    assert len(reset_times(events)) <= params.max_attempts * 2
    assert rsp.ACTION_GIVE_UP in actions(events)
    assert events[-1][3] == rsp.STATE_EXHAUSTED


def test_walk_stops_at_min_candidate_score_floor():
    # Rank 0 is strong, rank 1 is below the score floor: the walk must not publish
    # the sub-floor candidate even though the list has more entries.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, settle_timeout_sec=3.0,
        max_attempts=1)
    events = run_ranked(
        params, 60, requested=True,
        candidate_scores=[0.99, 0.4], recover_on_index=None)
    # Only the top candidate is ever published; index 1 (score 0.4) is never tried.
    assert published_indices(events) == [0]
    assert rsp.ACTION_GIVE_UP in actions(events)


def test_walk_is_bounded_by_max_walk_candidates():
    # A stale query returns a long ranked list none of whose poses lock. The walk
    # must stop after max_walk_candidates instead of burning settle_timeout_sec on
    # all 16 -- past the top few occupancy maxima a fresh query is the better spend.
    # With max_attempts=1 (a single query) exactly max_walk_candidates resets fire.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, settle_timeout_sec=3.0,
        max_attempts=1, max_walk_candidates=4)
    events = run_ranked(
        params, 120, requested=True,
        candidate_scores=[0.99] * 16, recover_on_index=None)
    assert published_indices(events) == [0, 1, 2, 3]
    assert actions(events).count(rsp.ACTION_QUERY) == 1
    assert rsp.ACTION_GIVE_UP in actions(events)
    assert events[-1][3] == rsp.STATE_EXHAUSTED


def test_high_max_walk_candidates_restores_full_list_walk():
    # Raising the cap restores walking the entire ranked list: the true pose at
    # rank 5 (beyond the default cap of 4) is reached and recovers only because the
    # cap is lifted -- proving the cap is what gates the deeper walk.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6, settle_timeout_sec=3.0,
        max_attempts=1, max_walk_candidates=999)
    events = run_ranked(
        params, 120, requested=True,
        candidate_scores=[0.99] * 6, recover_on_index=5)
    assert published_indices(events) == [0, 1, 2, 3, 4, 5]
    assert events[-1][3] == rsp.STATE_STANDDOWN


# --- seed motion compensation --------------------------------------------------
#
# The query->publish latency is the decisive G3 live blocker: a correct rank-1 fix
# goes stale because the vehicle drives on while the BBS query runs. These tests
# pin the forward-compensation helpers that push a seed ahead by the measured
# latency, and -- crucially -- that a bad/implausible estimate is always a no-op so
# compensation can never *worsen* a publish.


def test_seed_velocity_from_consecutive_fixes():
    # Two fixes 10 m apart in x over 5 s -> 2 m/s east.
    v = rsp.estimate_seed_velocity((0.0, 0.0), 100.0, (10.0, 0.0), 105.0,
                                   max_speed_mps=30.0)
    assert v.valid
    assert abs(v.vx - 2.0) < 1e-9
    assert abs(v.vy - 0.0) < 1e-9


def test_seed_velocity_rejects_within_query_walk():
    # Same query (issue times within min_dt) -> not a motion sample, invalid.
    v = rsp.estimate_seed_velocity((0.0, 0.0), 100.0, (50.0, 0.0), 100.1,
                                   max_speed_mps=30.0)
    assert not v.valid


def test_seed_velocity_rejects_implausible_speed():
    # A perceptual-aliasing wrong candidate jumps 200 m in 2 s = 100 m/s -> rejected,
    # so a wrong/right fix pair never drives compensation.
    v = rsp.estimate_seed_velocity((0.0, 0.0), 100.0, (200.0, 0.0), 102.0,
                                   max_speed_mps=30.0)
    assert not v.valid


def test_forward_compensate_pushes_seed_ahead_by_latency():
    v = rsp.SeedVelocity(vx=2.0, vy=1.0, valid=True)
    # 8 s of query latency at (2, 1) m/s -> +16 m east, +8 m north.
    x, y = rsp.forward_compensate_xy((100.0, 50.0), v, latency_sec=8.0,
                                     max_latency_sec=30.0)
    assert abs(x - 116.0) < 1e-9
    assert abs(y - 58.0) < 1e-9


def test_forward_compensate_clamps_latency():
    v = rsp.SeedVelocity(vx=2.0, vy=0.0, valid=True)
    # A 100 s query is clamped to max_latency_sec (30 s) -> +60 m, not +200 m.
    x, _ = rsp.forward_compensate_xy((0.0, 0.0), v, latency_sec=100.0,
                                     max_latency_sec=30.0)
    assert abs(x - 60.0) < 1e-9


def test_forward_compensate_invalid_velocity_is_noop():
    # An invalid estimate must never move the seed -- compensation cannot worsen a
    # publish, it can only help when the estimate is trustworthy.
    x, y = rsp.forward_compensate_xy((7.0, 3.0), rsp.SeedVelocity(),
                                     latency_sec=10.0, max_latency_sec=30.0)
    assert (x, y) == (7.0, 3.0)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
