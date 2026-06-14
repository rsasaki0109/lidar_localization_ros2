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
    return [a for (_, a, _, _) in events]


def reset_times(events):
    return [t for (t, a, _, _) in events if a == rsp.ACTION_PUBLISH_RESET]


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


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
