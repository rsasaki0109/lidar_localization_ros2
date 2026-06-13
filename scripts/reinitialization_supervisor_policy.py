#!/usr/bin/env python3
"""ROS-free guard policy for G3 automatic reinitialization.

The localization component already decides *when* recovery is needed and
publishes `/reinitialization_requested` (std_msgs/Bool) from its C++ recovery
supervisor. G3 closes the loop: on a sustained request it queries the G2
global-localization service, and -- only if the result clears explicit safety
gates -- republishes `/initialpose` to reseed tracking.

This module is the safety brain of that loop, deliberately separated from ROS so
it can be exhaustively regression-tested. It is a deterministic reducer:

    decide(params, state, observation) -> SupervisorDecision

The node threads the returned ``state`` back in on the next tick and performs the
returned ``action`` (query the service, publish a reset, or surface a give-up).

Why a guard at all -- the Phase 3 lesson
----------------------------------------
The degraded-acceptance work (development_plan.md, 2026-06-12) showed that a
closed loop which feeds its own corrections back into an acceptance decision can
diverge catastrophically (0.58 m -> 62.7 m) even when every single-shot fixture
looked healthy, because the budget that was supposed to bound bad acceptances was
reset by the very acceptances it was meant to limit. A reinitialization loop has
the same failure shape: a confidently-scored-but-wrong candidate can be published
forever if nothing bounds the attempts. So the guard here is built around hard,
non-self-resetting bounds:

* never publish a candidate whose score is below ``min_candidate_score``
  (no *unsafe publication*);
* never publish two resets closer than ``min_seconds_between_attempts``;
* after a reset, require *post-reset recovery evidence* (fitness back under
  ``recovery_fitness_threshold`` within ``settle_timeout_sec``) before standing
  down -- and if it never recovers, give up after ``max_attempts`` instead of
  looping (no *false-acceptance loop*);
* the attempt counter only resets on a confirmed recovery or when the request
  clears, never as a side effect of attempting -- so it is a true ceiling.

These map directly onto the G3 Decision Gates in
``docs/global_localization_roadmap.md``. ``test_reinitialization_supervisor_policy.py``
fails if any of them regress.

Conventions
-----------
* Candidate ``score`` is "higher is better" (BBS hit ratio, ~0.99 on a good lock),
  matching ``GlobalLocalizationCandidate.score``.
* ``fitness`` is "lower is better" (NDT/GICP alignment fitness), matching the
  localization component's ``/alignment_status`` fitness.
* All times are monotonic seconds supplied by the caller; the module never reads
  a clock, so replays are exact.
"""

from dataclasses import dataclass, replace
from typing import Optional

# Supervisor states. The names mirror the C++ RecoverySupervisorState vocabulary
# where they overlap so the two halves of the loop read consistently in logs.
STATE_IDLE = "idle"
STATE_AWAIT_QUERY = "await_query"
STATE_AWAIT_CANDIDATES = "await_candidates"
STATE_SETTLING = "settling"
STATE_COOLDOWN = "cooldown"
# Terminal-but-recoverable: the loop reached an outcome (recovered, or gave up)
# and now waits for the request to de-assert before it will act again. This makes
# the supervisor edge-triggered on the *next* problem rather than re-firing on a
# request line that is still (or stuck) high -- the Phase 3 lesson that a bound
# must not be cleared by the act it bounds. The C++ recovery supervisor is
# expected to drop /reinitialization_requested once tracking recovers, providing
# that clean edge; this state also protects us if it does not.
STATE_STANDDOWN = "standdown"
STATE_EXHAUSTED = "exhausted"

# Actions the node must carry out for a decision.
ACTION_NONE = "none"
ACTION_QUERY = "query"
ACTION_PUBLISH_RESET = "publish_reset"
ACTION_GIVE_UP = "give_up"


@dataclass(frozen=True)
class SupervisorParams:
    # A request must stay asserted this long before the first query, so a single
    # transient blip does not trigger a reset.
    request_debounce_sec: float = 1.0
    # Minimum spacing between two reset publications (and between attempts).
    min_seconds_between_attempts: float = 5.0
    # Hard ceiling on attempts for one continuous problem before giving up.
    max_attempts: int = 3
    # Candidates scoring below this are never published.
    min_candidate_score: float = 0.6
    # If the service does not return candidates within this long after a query,
    # the attempt is abandoned (counts against max_attempts).
    query_timeout_sec: float = 10.0
    # After a reset, how long to wait for recovery evidence before declaring the
    # attempt failed.
    settle_timeout_sec: float = 8.0
    # Post-reset alignment fitness at or below this counts as recovered.
    recovery_fitness_threshold: float = 1.5


@dataclass(frozen=True)
class SupervisorState:
    name: str = STATE_IDLE
    attempts: int = 0
    # When the current sustained request was first seen (for debounce).
    request_since_sec: Optional[float] = None
    # When the in-flight query was issued (for query_timeout).
    query_issued_sec: Optional[float] = None
    # When the most recent reset was published (for settle/cooldown spacing).
    last_reset_sec: Optional[float] = None
    # When the current cooldown began.
    cooldown_since_sec: Optional[float] = None


@dataclass(frozen=True)
class SupervisorObservation:
    now_sec: float
    reinitialization_requested: bool
    # Best candidate score from the most recent service reply, if one is ready
    # this tick (None while no fresh reply is available).
    best_candidate_score: Optional[float] = None
    # Current alignment fitness, if known (lower is better).
    best_fitness: Optional[float] = None


@dataclass(frozen=True)
class SupervisorDecision:
    action: str
    reason: str
    state: SupervisorState


def initial_state() -> SupervisorState:
    return SupervisorState()


def _cooldown(state: SupervisorState, now_sec: float, reason: str) -> SupervisorDecision:
    return SupervisorDecision(
        ACTION_NONE,
        reason,
        replace(state, name=STATE_COOLDOWN, cooldown_since_sec=now_sec,
                query_issued_sec=None),
    )


def decide(
    params: SupervisorParams,
    state: SupervisorState,
    obs: SupervisorObservation,
) -> SupervisorDecision:
    """Advance the supervisor one tick and return the action to take.

    Pure and deterministic: identical (params, state, obs) always yield the same
    decision and next state. The returned ``state`` must be threaded back in.
    """
    now = obs.now_sec
    requested = obs.reinitialization_requested

    # Track how long the request has been continuously asserted, for debounce.
    if requested:
        request_since = state.request_since_sec if state.request_since_sec is not None else now
    else:
        request_since = None
    state = replace(state, request_since_sec=request_since)

    name = state.name

    if name == STATE_IDLE:
        if not requested:
            return SupervisorDecision(ACTION_NONE, "tracking", state)
        if now - request_since < params.request_debounce_sec:
            return SupervisorDecision(ACTION_NONE, "request_debouncing", state)
        # Sustained request: move toward a query (cooldown handled in AWAIT_QUERY).
        return SupervisorDecision(
            ACTION_NONE, "request_armed", replace(state, name=STATE_AWAIT_QUERY))

    if name == STATE_AWAIT_QUERY:
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "request_cleared", replace(state, name=STATE_IDLE, attempts=0))
        if state.attempts >= params.max_attempts:
            return SupervisorDecision(
                ACTION_GIVE_UP, "attempts_exhausted", replace(state, name=STATE_EXHAUSTED))
        # Respect spacing relative to the previous reset, if any.
        if (state.last_reset_sec is not None
                and now - state.last_reset_sec < params.min_seconds_between_attempts):
            return SupervisorDecision(ACTION_NONE, "spacing_hold", state)
        return SupervisorDecision(
            ACTION_QUERY, "query_issued",
            replace(state, name=STATE_AWAIT_CANDIDATES, query_issued_sec=now))

    if name == STATE_AWAIT_CANDIDATES:
        if obs.best_candidate_score is not None:
            if obs.best_candidate_score >= params.min_candidate_score:
                return SupervisorDecision(
                    ACTION_PUBLISH_RESET, "reset_published",
                    replace(state, name=STATE_SETTLING, last_reset_sec=now,
                            query_issued_sec=None, attempts=state.attempts + 1))
            # Confidently-bad candidate: never publish it; count the attempt.
            return _cooldown(
                replace(state, attempts=state.attempts + 1), now, "weak_candidate_rejected")
        # No candidates yet -- wait, or abandon on timeout.
        if (state.query_issued_sec is not None
                and now - state.query_issued_sec >= params.query_timeout_sec):
            return _cooldown(
                replace(state, attempts=state.attempts + 1), now, "query_timeout")
        return SupervisorDecision(ACTION_NONE, "awaiting_candidates", state)

    if name == STATE_SETTLING:
        recovered = (obs.best_fitness is not None
                     and obs.best_fitness <= params.recovery_fitness_threshold)
        if recovered:
            # Confirmed recovery: stand down and clear the ceiling, but wait for
            # the request to de-assert before re-arming (edge-triggered next time).
            return SupervisorDecision(
                ACTION_NONE, "recovery_confirmed",
                replace(state, name=STATE_STANDDOWN, attempts=0, last_reset_sec=None,
                        cooldown_since_sec=None))
        if state.last_reset_sec is not None and now - state.last_reset_sec >= params.settle_timeout_sec:
            # Reset did not take. Give up if exhausted, else cool down and retry.
            if state.attempts >= params.max_attempts:
                return SupervisorDecision(
                    ACTION_GIVE_UP, "recovery_failed_exhausted",
                    replace(state, name=STATE_EXHAUSTED))
            return _cooldown(state, now, "recovery_unconfirmed")
        return SupervisorDecision(ACTION_NONE, "settling", state)

    if name == STATE_COOLDOWN:
        if state.cooldown_since_sec is not None and now - state.cooldown_since_sec < params.min_seconds_between_attempts:
            return SupervisorDecision(ACTION_NONE, "cooldown", state)
        if state.attempts >= params.max_attempts:
            return SupervisorDecision(
                ACTION_GIVE_UP, "attempts_exhausted", replace(state, name=STATE_EXHAUSTED))
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "request_cleared", replace(state, name=STATE_IDLE, attempts=0))
        return SupervisorDecision(
            ACTION_NONE, "cooldown_elapsed",
            replace(state, name=STATE_AWAIT_QUERY, cooldown_since_sec=None))

    if name == STATE_STANDDOWN:
        # Wait for the request to clear, then re-arm for the next problem.
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "standdown_cleared",
                replace(state, name=STATE_IDLE, attempts=0, request_since_sec=None))
        return SupervisorDecision(ACTION_NONE, "standdown", state)

    if name == STATE_EXHAUSTED:
        # Latched until the underlying problem clears (request drops) -- the node
        # should surface this to an operator rather than retry silently.
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "exhausted_cleared",
                replace(state, name=STATE_IDLE, attempts=0, last_reset_sec=None,
                        cooldown_since_sec=None))
        return SupervisorDecision(ACTION_NONE, "exhausted", state)

    raise ValueError(f"unknown supervisor state: {name!r}")
