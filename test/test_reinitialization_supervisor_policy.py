#!/usr/bin/env python3
"""Regression tests for the G3 reinitialization guard policy.

These encode the Decision Gates from docs/global_localization_roadmap.md as
adversarial sequences. They are the "regression test that fails on unsafe
publication or false acceptance" the roadmap requires, and they exist because
the Phase 3 degraded-acceptance work proved that a closed recovery loop can
diverge if its bounds can be reset by the very acts they bound.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import reinitialization_supervisor_policy as rsp  # noqa: E402


def test_shadow_relative_motion_ignores_constant_global_transform():
    # Candidate fixes and odometry have different absolute origins/yaws, but
    # describe the same 5 m forward move and 20 degree turn.
    first_candidate = (100.0, -40.0, math.radians(70.0))
    second_candidate = (
        100.0 + 5.0 * math.cos(math.radians(70.0)),
        -40.0 + 5.0 * math.sin(math.radians(70.0)),
        math.radians(90.0),
    )
    first_odom = (-3.0, 8.0, math.radians(-30.0))
    second_odom = (
        -3.0 + 5.0 * math.cos(math.radians(-30.0)),
        8.0 + 5.0 * math.sin(math.radians(-30.0)),
        math.radians(-10.0),
    )
    mismatch = rsp.compare_shadow_relative_motion(
        first_candidate, second_candidate, first_odom, second_odom)
    assert mismatch.translation_m < 1.0e-9
    assert mismatch.yaw_deg < 1.0e-9


def test_shadow_relative_motion_rejects_alias_jump_and_wrong_turn():
    mismatch = rsp.compare_shadow_relative_motion(
        (0.0, 0.0, 0.0),
        (80.0, 30.0, math.radians(100.0)),
        (10.0, 20.0, math.radians(40.0)),
        (13.0, 20.0, math.radians(45.0)),
    )
    assert mismatch.translation_m > 80.0
    assert mismatch.yaw_deg > 90.0


def test_shadow_motion_gate_withholds_until_required_consistent_sequence():
    gate = rsp.BbsShadowMotionGate(
        required_samples=3,
        max_translation_mismatch_m=1.0,
        max_yaw_mismatch_deg=5.0)
    assert not gate.observe((0.0, 0.0, 0.0), (10.0, 5.0, 0.4)).publish_allowed
    second = gate.observe((2.0, 0.0, 0.0), (
        10.0 + 2.0 * math.cos(0.4),
        5.0 + 2.0 * math.sin(0.4), 0.4))
    assert second.consistent_samples == 2
    assert not second.publish_allowed
    third = gate.observe((4.0, 0.0, 0.0), (
        10.0 + 4.0 * math.cos(0.4),
        5.0 + 4.0 * math.sin(0.4), 0.4))
    assert third.publish_allowed
    assert third.consistent_samples == 3


def test_shadow_motion_gate_alias_resets_streak_and_reset_clears_history():
    gate = rsp.BbsShadowMotionGate(2, 2.0, 10.0)
    gate.observe((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    rejected = gate.observe((100.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert not rejected.publish_allowed
    assert rejected.consistent_samples == 1
    gate.reset()
    primed = gate.observe((101.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert not primed.publish_allowed
    assert primed.mismatch is None


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
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False)

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


def test_recovery_confirmation_requires_consecutive_low_fitness():
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0,
        min_candidate_score=0.6,
        recovery_fitness_threshold=1.5,
        recovery_confirmation_samples=3,
        settle_timeout_sec=8.0,
        max_attempts=1,
        enable_confirm_cross_check=False,
    )

    # One transient low-fitness sample is followed by a bad sample, then the pose
    # becomes genuinely stable. Only the stable run should confirm recovery.
    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        age = now - last_reset_sec
        if 2.0 <= age < 3.0:
            return 0.4
        if age >= 5.0:
            return 0.4
        return 9.0

    events = run(params, 40, requested=True, candidate_score=0.95, fitness=fitness)
    reasons = [r for (_, _, r, _) in events]
    assert "recovery_confirmed" in reasons
    assert events[reasons.index("recovery_confirmed")][0] >= reset_times(events)[0] + 7.0


def test_recovery_confirmation_does_not_double_count_same_fitness_sample():
    params = rsp.SupervisorParams(
        recovery_fitness_threshold=1.5,
        recovery_confirmation_samples=2,
        settle_timeout_sec=10.0,
        enable_confirm_cross_check=False,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_SETTLING,
        attempts=1,
        last_reset_sec=100.0,
        candidate_scores=(0.9,),
    )
    obs = rsp.SupervisorObservation(
        now_sec=101.0,
        reinitialization_requested=True,
        best_fitness=0.4,
        fitness_observed_sec=101.0,
    )
    first = rsp.decide(params, state, obs)
    assert first.state.name == rsp.STATE_SETTLING
    assert first.state.recovery_evidence_count == 1

    duplicate = rsp.decide(params, first.state, rsp.SupervisorObservation(
        now_sec=101.5,
        reinitialization_requested=True,
        best_fitness=0.4,
        fitness_observed_sec=101.0,
    ))
    assert duplicate.state.name == rsp.STATE_SETTLING
    assert duplicate.state.recovery_evidence_count == 1

    second = rsp.decide(params, duplicate.state, rsp.SupervisorObservation(
        now_sec=102.0,
        reinitialization_requested=True,
        best_fitness=0.4,
        fitness_observed_sec=102.0,
    ))
    assert second.reason == "recovery_confirmed"
    assert second.state.name == rsp.STATE_STANDDOWN


def test_recovery_confirmation_requires_stable_tracking_when_available():
    params = rsp.SupervisorParams(
        recovery_fitness_threshold=1.5,
        recovery_confirmation_samples=2,
        settle_timeout_sec=10.0,
        enable_confirm_cross_check=False,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_SETTLING,
        attempts=1,
        last_reset_sec=100.0,
        candidate_scores=(0.9,),
    )

    degraded = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=101.0,
        reinitialization_requested=False,
        best_fitness=0.4,
        fitness_observed_sec=101.0,
        stable_tracking=False,
    ))
    assert degraded.state.name == rsp.STATE_SETTLING
    assert degraded.state.recovery_evidence_count == 0

    first_ok = rsp.decide(params, degraded.state, rsp.SupervisorObservation(
        now_sec=102.0,
        reinitialization_requested=False,
        best_fitness=0.4,
        fitness_observed_sec=102.0,
        stable_tracking=True,
    ))
    assert first_ok.state.name == rsp.STATE_SETTLING
    assert first_ok.state.recovery_evidence_count == 1

    second_ok = rsp.decide(params, first_ok.state, rsp.SupervisorObservation(
        now_sec=103.0,
        reinitialization_requested=False,
        best_fitness=0.4,
        fitness_observed_sec=103.0,
        stable_tracking=True,
    ))
    assert second_ok.reason == "recovery_confirmed"
    assert second_ok.state.name == rsp.STATE_STANDDOWN


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


def test_scan_not_ready_reply_retries_without_spending_attempt_budget():
    params = rsp.SupervisorParams(max_attempts=1)
    waiting = rsp.SupervisorState(
        name=rsp.STATE_AWAIT_CANDIDATES, attempts=0, query_issued_sec=10.0)
    decision = rsp.decide(params, waiting, rsp.SupervisorObservation(
        now_sec=10.5,
        reinitialization_requested=True,
        candidate_scores=(),
        retryable_empty_reply=True,
    ))
    assert decision.reason == "scan_not_ready"
    assert decision.state.name == rsp.STATE_AWAIT_QUERY
    assert decision.state.attempts == 0


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
        settle_timeout_sec=5.0, recovery_fitness_threshold=1.5,
        enable_confirm_cross_check=False)

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


# --- Gate: odom bridge (map -> odom(last accepted) x live external odom) -----
#
# An external LIO front end (e.g. GLIM) with low relative odom drift makes
# map -> odom(last accepted) x odom -> base_link(now) a far tighter reseed than
# a BBS candidate during a short dropout, and it costs zero query latency. It is
# opt-in (SupervisorParams.use_odom_bridge_candidate, default False) and must
# never change BBS-only behavior when off, must be tried before every BBS query
# when on and available, must never spend a BBS attempt, and must fall back to
# BBS after odom_bridge_max_attempts unconfirmed tries.

def run_with_bridge(
    params,
    ticks,
    *,
    requested,
    odom_bridge_available,
    candidate_score=None,
    query_latency=1,
    fitness=None,
    dt=1.0,
):
    """Like run(), but also feeds SupervisorObservation.odom_bridge_available.

    Returns 5-tuples (now, action, reason, state_name, candidate_source).
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
        bridge_available = bool(value(odom_bridge_available, now))
        score = None
        if deliver_at is not None and i == deliver_at:
            score = value(candidate_score, now)
            deliver_at = None
        fit = None
        if fitness is not None:
            fit = fitness(now, last_reset_sec) if callable(fitness) else fitness
        obs = rsp.SupervisorObservation(
            now, req, best_candidate_score=score, best_fitness=fit,
            odom_bridge_available=bridge_available)
        dec = rsp.decide(params, state, obs)
        state = dec.state
        events.append((now, dec.action, dec.reason, state.name, dec.candidate_source))
        if dec.action == rsp.ACTION_QUERY:
            deliver_at = i + query_latency
        if dec.action == rsp.ACTION_PUBLISH_RESET:
            last_reset_sec = now
    return events


def test_odom_bridge_off_by_default_preserves_bbs_only_behavior():
    # use_odom_bridge_candidate defaults to False: even with a bridge candidate
    # always available, behavior must be identical to plain BBS-only.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False)
    assert params.use_odom_bridge_candidate is False

    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        return 0.4 if now - last_reset_sec >= 2.0 else 9.0

    events = run_with_bridge(
        params, 40, requested=True, odom_bridge_available=True,
        candidate_score=0.95, fitness=fitness)
    reasons = [r for (_, _, r, _, _) in events]
    assert "odom_bridge_reset_published" not in reasons
    assert "recovery_confirmed" in reasons
    assert all(cs == "bbs" for (_, a, _, _, cs) in events
               if a == rsp.ACTION_PUBLISH_RESET)


def test_odom_bridge_tried_before_bbs_query_and_never_spends_bbs_attempt():
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False,
        use_odom_bridge_candidate=True, odom_bridge_max_attempts=2,
        max_attempts=1)

    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        return 0.4 if now - last_reset_sec >= 2.0 else 9.0

    events = run_with_bridge(
        params, 40, requested=True, odom_bridge_available=True, fitness=fitness)
    reasons = [r for (_, _, r, _, _) in events]
    assert "odom_bridge_reset_published" in reasons
    assert rsp.ACTION_QUERY not in [e[1] for e in events]
    assert "recovery_confirmed" in reasons
    publish_events = [e for e in events if e[1] == rsp.ACTION_PUBLISH_RESET]
    assert len(publish_events) == 1
    assert publish_events[0][4] == "odom_bridge"
    # A BBS attempt (max_attempts=1) was never spent -- the recovery is the
    # bridge's, and max_attempts is the BBS query ceiling.
    assert events[-1][3] == rsp.STATE_STANDDOWN


def test_odom_bridge_falls_back_to_bbs_after_unconfirmed_tries():
    # Bridge is available but never recovers (fitness stays high); after
    # odom_bridge_max_attempts unconfirmed tries it must stop retrying the
    # bridge and fall back to a BBS query, which does recover.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False,
        use_odom_bridge_candidate=True, odom_bridge_max_attempts=1,
        settle_timeout_sec=3.0, min_seconds_between_attempts=1.0,
        max_attempts=2)

    def fitness(now, last_reset_sec):
        return 9.0

    events = run_with_bridge(
        params, 60, requested=True, odom_bridge_available=True,
        candidate_score=0.95, fitness=fitness, query_latency=1)
    reasons = [r for (_, _, r, _, _) in events]
    assert "odom_bridge_unconfirmed" in reasons
    # After exactly one unconfirmed bridge try (odom_bridge_max_attempts=1) it
    # must fall back to the BBS query path.
    assert rsp.ACTION_QUERY in [e[1] for e in events]
    publish_sources = [cs for (_, a, _, _, cs) in events
                        if a == rsp.ACTION_PUBLISH_RESET]
    assert publish_sources[0] == "odom_bridge"
    assert "bbs" in publish_sources


def test_odom_bridge_unavailable_falls_back_to_bbs_immediately():
    # use_odom_bridge_candidate is on, but the node never has a fresh TF fix
    # (e.g. the external front end never came up): must behave like BBS-only.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False,
        use_odom_bridge_candidate=True)

    def fitness(now, last_reset_sec):
        if last_reset_sec is None:
            return 9.0
        return 0.4 if now - last_reset_sec >= 2.0 else 9.0

    events = run_with_bridge(
        params, 40, requested=True, odom_bridge_available=False,
        candidate_score=0.95, fitness=fitness)
    reasons = [r for (_, _, r, _, _) in events]
    assert "odom_bridge_reset_published" not in reasons
    assert rsp.ACTION_QUERY in [e[1] for e in events]
    assert "recovery_confirmed" in reasons


def test_odom_bridge_respects_reset_spacing():
    # The bridge must not republish faster than min_seconds_between_attempts,
    # exactly like the BBS path.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False,
        use_odom_bridge_candidate=True, odom_bridge_max_attempts=5,
        settle_timeout_sec=1.0, min_seconds_between_attempts=6.0)

    def fitness(now, last_reset_sec):
        return 9.0  # never recovers, forcing repeated bridge tries

    events = run_with_bridge(
        params, 30, requested=True, odom_bridge_available=True, fitness=fitness)
    reset_times_local = [e[0] for e in events if e[1] == rsp.ACTION_PUBLISH_RESET]
    for a, b in zip(reset_times_local, reset_times_local[1:]):
        assert b - a >= params.min_seconds_between_attempts


def test_odom_bridge_budget_and_exhausted_flag_reset_on_new_episode():
    # Exhausting the bridge budget in one episode must not carry over to a later,
    # unrelated episode after the request clears.
    params = rsp.SupervisorParams(
        request_debounce_sec=1.0, min_candidate_score=0.6,
        recovery_fitness_threshold=1.5, enable_confirm_cross_check=False,
        use_odom_bridge_candidate=True, odom_bridge_max_attempts=1,
        settle_timeout_sec=2.0, min_seconds_between_attempts=1.0,
        max_attempts=1)

    state = rsp.initial_state()
    now = 0.0
    seen_bridge_exhausted_mid_episode = False
    # First episode: bridge never recovers, exhausts, falls back to BBS which is
    # never delivered a candidate (times out) -> exhausted.
    for _ in range(40):
        obs = rsp.SupervisorObservation(
            now, True, best_fitness=9.0, odom_bridge_available=True)
        dec = rsp.decide(params, state, obs)
        state = dec.state
        now += 1.0
        if state.odom_bridge_exhausted:
            seen_bridge_exhausted_mid_episode = True
        if state.name == rsp.STATE_EXHAUSTED:
            break
    assert state.name == rsp.STATE_EXHAUSTED
    # The bridge budget was spent and set the flag while the episode was still
    # live (falling back to BBS); reaching the terminal EXHAUSTED state clears
    # it again along with every other per-episode field (it is meaningless once
    # latched -- nothing reads it there).
    assert seen_bridge_exhausted_mid_episode is True
    assert state.odom_bridge_exhausted is False

    # Request clears -> back to idle with the bridge budget reset.
    dec = rsp.decide(
        params, state, rsp.SupervisorObservation(now, False, odom_bridge_available=True))
    state = dec.state
    assert state.name == rsp.STATE_IDLE
    assert state.odom_bridge_exhausted is False
    assert state.odom_bridge_attempts == 0


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
        max_attempts=1, recovery_fitness_threshold=1.5,
        enable_confirm_cross_check=False)
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
        max_attempts=1, max_walk_candidates=999, enable_confirm_cross_check=False)
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


def test_trusted_velocity_tracker_from_two_trusted_samples():
    tracker = rsp.TrustedSeedVelocityTracker(max_speed_mps=30.0)
    tracker.observe(0.0, 0.0, 100.0, trusted=True)
    tracker.observe(10.0, 0.0, 105.0, trusted=True)
    v = tracker.velocity_at(105.0, max_age_sec=60.0)
    assert v.valid
    assert abs(v.vx - 2.0) < 1e-9
    assert abs(v.vy - 0.0) < 1e-9


def test_trusted_velocity_tracker_untrusted_gap_prevents_pairing():
    tracker = rsp.TrustedSeedVelocityTracker(max_speed_mps=30.0)
    tracker.observe(0.0, 0.0, 100.0, trusted=True)
    tracker.observe(10.0, 0.0, 105.0, trusted=True)
    v_before = tracker.velocity_at(105.0, max_age_sec=60.0)
    assert v_before.valid
    assert abs(v_before.vx - 2.0) < 1e-9
    tracker.observe(20.0, 0.0, 110.0, trusted=False)
    tracker.observe(100.0, 0.0, 120.0, trusted=True)
    v_mid = tracker.velocity_at(120.0, max_age_sec=60.0)
    assert v_mid.valid
    assert abs(v_mid.vx - v_before.vx) < 1e-9
    tracker.observe(200.0, 0.0, 121.0, trusted=True)
    v_after = tracker.velocity_at(121.0, max_age_sec=60.0)
    assert v_after.valid
    assert abs(v_after.vx - v_before.vx) < 1e-9


def test_trusted_velocity_tracker_rejects_stale_velocity():
    tracker = rsp.TrustedSeedVelocityTracker(max_speed_mps=30.0)
    tracker.observe(0.0, 0.0, 100.0, trusted=True)
    tracker.observe(10.0, 0.0, 105.0, trusted=True)
    assert tracker.velocity_at(160.0, max_age_sec=60.0).valid
    assert not tracker.velocity_at(166.0, max_age_sec=60.0).valid
    reason = tracker.velocity_rejection_reason(166.0, max_age_sec=60.0)
    assert reason is not None
    assert "stale trusted velocity" in reason


def test_trusted_velocity_tracker_rejects_implausible_speed_update():
    tracker = rsp.TrustedSeedVelocityTracker(max_speed_mps=5.0)
    tracker.observe(0.0, 0.0, 100.0, trusted=True)
    tracker.observe(10.0, 0.0, 105.0, trusted=True)
    v_good = tracker.velocity_at(105.0, max_age_sec=60.0)
    assert v_good.valid
    assert abs(v_good.vx - 2.0) < 1e-9
    tracker.observe(200.0, 0.0, 107.0, trusted=True)
    v_after = tracker.velocity_at(107.0, max_age_sec=60.0)
    assert v_after.valid
    assert abs(v_after.vx - v_good.vx) < 1e-9
    assert abs(v_after.vy - v_good.vy) < 1e-9


# --- Gate: post-confirm G2 cross-check (alias rejection) ---------------------


def test_settling_confirm_with_cross_check_issues_verify_query():
    params = rsp.SupervisorParams(
        recovery_confirmation_samples=2,
        enable_confirm_cross_check=True,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_SETTLING,
        attempts=1,
        last_reset_sec=100.0,
        recovery_evidence_count=1,
        candidate_scores=(0.9,),
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=101.0,
        reinitialization_requested=True,
        best_fitness=0.4,
        fitness_observed_sec=101.0,
    ))
    assert dec.action == rsp.ACTION_QUERY
    assert dec.reason == "confirm_cross_check_query"
    assert dec.state.name == rsp.STATE_VERIFYING
    assert dec.state.attempts == 1
    assert dec.state.query_issued_sec == 101.0


def test_settling_confirm_without_cross_check_stands_down():
    params = rsp.SupervisorParams(
        recovery_confirmation_samples=2,
        enable_confirm_cross_check=False,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_SETTLING,
        attempts=1,
        last_reset_sec=100.0,
        recovery_evidence_count=1,
        candidate_scores=(0.9,),
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=101.0,
        reinitialization_requested=True,
        best_fitness=0.4,
        fitness_observed_sec=101.0,
    ))
    assert dec.action == rsp.ACTION_NONE
    assert dec.reason == "recovery_confirmed"
    assert dec.state.name == rsp.STATE_STANDDOWN
    assert dec.state.attempts == 0


def _verifying_state(**kwargs):
    defaults = dict(
        name=rsp.STATE_VERIFYING,
        attempts=1,
        query_issued_sec=100.0,
    )
    defaults.update(kwargs)
    return rsp.SupervisorState(**defaults)


def test_verifying_mismatch_none_fail_open_confirms():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0)
    dec = rsp.decide(
        params,
        _verifying_state(),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=None,
        ),
    )
    assert dec.reason == "recovery_confirmed"
    assert dec.state.name == rsp.STATE_STANDDOWN
    assert dec.state.attempts == 0


def test_verifying_mismatch_within_threshold_confirms():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0)
    dec = rsp.decide(
        params,
        _verifying_state(),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=2.0,
        ),
    )
    assert dec.reason == "recovery_confirmed"
    assert dec.state.name == rsp.STATE_STANDDOWN


def test_verifying_mismatch_reseeds_from_verify_fix():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0, max_attempts=3)
    dec = rsp.decide(
        params,
        _verifying_state(attempts=1),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=12.0,
        ),
    )
    # The verify reply is the freshest trustworthy fix: publish it immediately
    # instead of burning a cooldown + re-query on more staleness.
    assert dec.action == rsp.ACTION_PUBLISH_RESET
    assert dec.reason == "cross_check_reseed"
    assert dec.state.name == rsp.STATE_SETTLING
    assert dec.state.candidate_scores == (0.9,)
    assert dec.state.attempts == 2  # the reseed publish pays a fresh attempt


def test_verifying_mismatch_weak_verify_fix_cooldown():
    params = rsp.SupervisorParams(
        cross_check_mismatch_m=5.0, max_attempts=3, min_candidate_score=0.6)
    dec = rsp.decide(
        params,
        _verifying_state(attempts=1),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.3,),
            cross_check_mismatch_m=12.0,
        ),
    )
    assert dec.action == rsp.ACTION_NONE
    assert dec.reason == "cross_check_mismatch"
    assert dec.state.name == rsp.STATE_COOLDOWN
    # The reset under verification already paid its attempt at publish time;
    # the mismatch detection must not charge a second one.
    assert dec.state.attempts == 1


def test_verifying_mismatch_at_max_attempts_exhausted():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0, max_attempts=3)
    dec = rsp.decide(
        params,
        _verifying_state(attempts=3),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=12.0,
        ),
    )
    assert dec.action == rsp.ACTION_GIVE_UP
    assert dec.reason == "recovery_failed_exhausted"
    assert dec.state.name == rsp.STATE_EXHAUSTED
    assert dec.state.attempts == 3


def test_verifying_timeout_fail_open_confirms():
    params = rsp.SupervisorParams(query_timeout_sec=10.0)
    dec = rsp.decide(
        params,
        _verifying_state(query_issued_sec=100.0, attempts=2),
        rsp.SupervisorObservation(
            now_sec=110.0,
            reinitialization_requested=True,
        ),
    )
    assert dec.reason == "cross_check_timeout"
    assert dec.state.name == rsp.STATE_STANDDOWN
    assert dec.state.attempts == 0


def test_verifying_ignores_reinitialization_cleared():
    params = rsp.SupervisorParams()
    dec = rsp.decide(
        params,
        _verifying_state(query_issued_sec=100.0),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=False,
        ),
    )
    assert dec.reason == "verifying"
    assert dec.state.name == rsp.STATE_VERIFYING


# --- Gate: self-clear cross-check (localizer de-assert without confirm) ------


def test_cooldown_self_clear_cross_check_on_request_deassert():
    params = rsp.SupervisorParams(
        enable_confirm_cross_check=True,
        min_seconds_between_attempts=5.0,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_COOLDOWN,
        attempts=1,
        last_reset_sec=90.0,
        cooldown_since_sec=100.0,
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=106.0,
        reinitialization_requested=False,
    ))
    assert dec.action == rsp.ACTION_QUERY
    assert dec.reason == "self_clear_cross_check_query"
    assert dec.state.name == rsp.STATE_VERIFYING
    assert dec.state.query_issued_sec == 106.0
    assert dec.state.alias_confirmed is False


def test_cooldown_request_deassert_without_reset_goes_idle():
    params = rsp.SupervisorParams(enable_confirm_cross_check=True)
    state = rsp.SupervisorState(
        name=rsp.STATE_COOLDOWN,
        attempts=1,
        last_reset_sec=None,
        cooldown_since_sec=100.0,
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=106.0,
        reinitialization_requested=False,
    ))
    assert dec.action == rsp.ACTION_NONE
    assert dec.reason == "request_cleared"
    assert dec.state.name == rsp.STATE_IDLE
    assert dec.state.attempts == 0
    assert dec.state.alias_confirmed is False


def test_cooldown_request_deassert_alias_confirmed_stays_in_episode():
    params = rsp.SupervisorParams(
        enable_confirm_cross_check=True,
        min_seconds_between_attempts=5.0,
    )
    state = rsp.SupervisorState(
        name=rsp.STATE_COOLDOWN,
        attempts=2,
        last_reset_sec=90.0,
        cooldown_since_sec=100.0,
        alias_confirmed=True,
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=106.0,
        reinitialization_requested=False,
    ))
    assert dec.action == rsp.ACTION_NONE
    assert dec.reason == "cooldown_elapsed"
    assert dec.state.name == rsp.STATE_AWAIT_QUERY
    assert dec.state.alias_confirmed is True


def test_await_query_alias_confirmed_still_issues_query():
    params = rsp.SupervisorParams()
    state = rsp.SupervisorState(
        name=rsp.STATE_AWAIT_QUERY,
        attempts=1,
        alias_confirmed=True,
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=10.0,
        reinitialization_requested=False,
    ))
    assert dec.action == rsp.ACTION_QUERY
    assert dec.reason == "query_issued"
    assert dec.state.name == rsp.STATE_AWAIT_CANDIDATES


def test_verifying_mismatch_above_threshold_sets_alias_confirmed():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0, max_attempts=3)
    dec = rsp.decide(
        params,
        _verifying_state(attempts=1),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=12.0,
        ),
    )
    assert dec.reason == "cross_check_reseed"
    assert dec.state.name == rsp.STATE_SETTLING
    assert dec.state.alias_confirmed is True


def test_verifying_mismatch_within_threshold_clears_alias_confirmed():
    params = rsp.SupervisorParams(cross_check_mismatch_m=5.0)
    dec = rsp.decide(
        params,
        _verifying_state(alias_confirmed=True),
        rsp.SupervisorObservation(
            now_sec=101.0,
            reinitialization_requested=True,
            candidate_scores=(0.9,),
            cross_check_mismatch_m=2.0,
        ),
    )
    assert dec.reason == "recovery_confirmed"
    assert dec.state.name == rsp.STATE_STANDDOWN
    assert dec.state.alias_confirmed is False


def test_cooldown_request_deassert_cross_check_disabled_goes_idle():
    params = rsp.SupervisorParams(enable_confirm_cross_check=False)
    state = rsp.SupervisorState(
        name=rsp.STATE_COOLDOWN,
        attempts=1,
        last_reset_sec=90.0,
        cooldown_since_sec=100.0,
    )
    dec = rsp.decide(params, state, rsp.SupervisorObservation(
        now_sec=106.0,
        reinitialization_requested=False,
    ))
    assert dec.action == rsp.ACTION_NONE
    assert dec.reason == "request_cleared"
    assert dec.state.name == rsp.STATE_IDLE


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))

def test_sim_fix_velocity_from_consecutive_fixes():
    # Two fixes 10 m apart in x over 5 s on the bag clock -> 2 m/s east.
    v = rsp.estimate_sim_fix_velocity(
        (0.0, 0.0), 100.0, (10.0, 0.0), 105.0, max_speed_mps=30.0)
    assert v.valid
    assert abs(v.vx - 2.0) < 1e-9
    assert abs(v.vy - 0.0) < 1e-9


def test_sim_fix_velocity_rejects_short_or_long_dt():
    short = rsp.estimate_sim_fix_velocity(
        (0.0, 0.0), 100.0, (10.0, 0.0), 101.0, max_speed_mps=30.0)
    assert not short.valid
    long = rsp.estimate_sim_fix_velocity(
        (0.0, 0.0), 100.0, (10.0, 0.0), 161.0, max_speed_mps=30.0)
    assert not long.valid


def test_sim_fix_velocity_rejects_implausible_speed():
    v = rsp.estimate_sim_fix_velocity(
        (0.0, 0.0), 100.0, (200.0, 0.0), 105.0, max_speed_mps=30.0)
    assert not v.valid


def test_sim_fix_pose_delta_clamps_staleness():
    v = rsp.SeedVelocity(vx=2.0, vy=1.0, valid=True)
    delta = rsp.estimate_pose_delta_from_sim_fix(
        v, fix_scan_stamp_sec=100.0, last_sim_stamp_sec=138.0, max_latency_sec=30.0)
    assert delta is not None
    dx, dy, speed, staleness = delta
    assert abs(staleness - 30.0) < 1e-9
    assert abs(speed - 2.236067977) < 1e-6
    assert abs(dx - 60.0) < 1e-9
    assert abs(dy - 30.0) < 1e-9


def test_sim_fix_pose_delta_none_when_staleness_nonpositive():
    v = rsp.SeedVelocity(vx=2.0, vy=0.0, valid=True)
    assert rsp.estimate_pose_delta_from_sim_fix(
        v, fix_scan_stamp_sec=100.0, last_sim_stamp_sec=100.0,
        max_latency_sec=30.0) is None
    assert rsp.estimate_pose_delta_from_sim_fix(
        v, fix_scan_stamp_sec=100.0, last_sim_stamp_sec=None,
        max_latency_sec=30.0) is None


def test_trusted_velocity_tracker_high_rate_stream_still_estimates():
    # Poses arrive faster than the 0.5 s pair floor (e.g. 4 Hz wall in a replay).
    # The anchor must be held across sub-floor samples -- advancing it every
    # message would make every pair sub-floor and the tracker permanently invalid.
    tracker = rsp.TrustedSeedVelocityTracker(max_speed_mps=30.0)
    for i in range(9):  # 0.25 s apart, 1.0 m/s in +x
        tracker.observe(0.25 * i, 0.0, 100.0 + 0.25 * i, trusted=True)
    v = tracker.velocity_at(102.0, max_age_sec=60.0)
    assert v.valid
    assert abs(v.vx - 1.0) < 1e-9
    assert abs(v.vy - 0.0) < 1e-9
