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
  ``recovery_fitness_threshold`` for ``recovery_confirmation_samples`` consecutive
  observations, and stable-tracking diagnostics when the node provides them,
  within ``settle_timeout_sec``) before standing down -- optionally preceded by
  a second G2 query in ``verifying`` that compares the fresh global fix to the
  localizer pose and rejects along-corridor alias locks -- and if it never recovers,
  give up after ``max_attempts`` instead of looping (no *false-acceptance loop*);
* the attempt counter only resets on a confirmed recovery or when the request
  clears, never as a side effect of attempting -- so it is a true ceiling.

Ranked-candidate walk -- the live-run lesson
--------------------------------------------
The 2026-06-15 live closed-loop run (docs/g3_live_closed_loop.md) showed the
*candidate score is not a registration score*: G2's BBS occupancy score labelled a
190 m-wrong candidate 0.99 and the 5.5 m-correct one 0.998, so the score floor alone
cannot separate them. The localizer's own NDT fitness *can* (that is what its
`score_threshold` does). So when a query returns a ranked list, the supervisor walks
it: it publishes the top candidate, waits ``settle_timeout_sec`` for the localizer to
show recovery (fitness under ``recovery_fitness_threshold``), and if it does not,
publishes the *next* candidate from the same query -- using the localizer's fitness as
the registration oracle -- before spending another ``max_attempts`` slot on a fresh
query. Walking within one query does not consume an attempt; only re-querying does, so
``max_attempts`` stays a true ceiling on queries while letting one query try all the
poses it actually found. A wrong reset is rejected by the localizer (high fitness, no
accepted pose), so walking cannot cause the Phase 3 *acceptance* blowup -- it only
changes which seed is offered next.

The walk is bounded by ``max_walk_candidates``: the same live run showed a *stale*
query can return a full ranked list none of whose poses lock, so walking all 16 burned
~95 s at ``settle_timeout_sec`` each before the next query -- on a newer, more
distinctive scan -- produced the candidate that recovered. Past the top few occupancy
maxima a fresh query beats a deeper walk, so the supervisor abandons the query after
``max_walk_candidates`` and re-queries (spending one attempt) instead.

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

import math
from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

# Supervisor states. The names mirror the C++ RecoverySupervisorState vocabulary
# where they overlap so the two halves of the loop read consistently in logs.
# Flow: idle -> await_query -> await_candidates -> settling -> verifying (optional
# post-confirm cross-check) -> standdown, with cooldown/exhausted branches.
STATE_IDLE = "idle"
STATE_AWAIT_QUERY = "await_query"
STATE_AWAIT_CANDIDATES = "await_candidates"
STATE_SETTLING = "settling"
STATE_VERIFYING = "verifying"
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
    # Require this many consecutive post-reset low-fitness observations before
    # confirming recovery. A single transient low score can still be a wrong pose
    # passing briefly through a local basin.
    recovery_confirmation_samples: int = 3
    # Walk at most this many candidates from one query (counting the top one)
    # before abandoning the whole query and re-querying with a fresh scan. The
    # 2026-06-15 live run (docs/g3_live_closed_loop.md) showed a stale query can
    # return a full ranked list none of whose poses lock, so walking all 16 just
    # burned ~95 s at settle_timeout_sec each before the *next* query -- on a
    # newer, more distinctive scan -- produced the candidate that recovered. Past
    # the top few BBS occupancy maxima a fresh query beats a deeper walk, so cap
    # the walk and spend the time on a new scan instead. Set very high to restore
    # walking the entire list.
    max_walk_candidates: int = 4
    # After post-reset recovery evidence is met, issue one more G2 query and compare
    # the fresh global fix to the localizer pose (catches along-corridor alias locks
    # that pass the fitness gate alone).
    enable_confirm_cross_check: bool = True
    # Euclidean xy distance (m) above which the verify query is treated as an alias.
    cross_check_mismatch_m: float = 5.0


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
    # Ranked candidate scores (high-to-low) from the query currently being walked,
    # and the index of the candidate published most recently. Set when a query
    # reply is consumed; used to walk to the next-best candidate on a failed settle
    # without spending another attempt.
    candidate_scores: Tuple[float, ...] = field(default_factory=tuple)
    candidate_index: int = 0
    recovery_evidence_count: int = 0
    last_recovery_fitness_observed_sec: Optional[float] = None
    # Set when a cross-check objectively detected the localizer is off; while True
    # the episode proceeds even if reinitialization_requested is de-asserted.
    alias_confirmed: bool = False


@dataclass(frozen=True)
class SupervisorObservation:
    now_sec: float
    reinitialization_requested: bool
    # Best candidate score from the most recent service reply, if one is ready
    # this tick (None while no fresh reply is available). Back-compat shorthand for
    # a single-candidate reply; prefer ``candidate_scores`` for the full ranked list.
    best_candidate_score: Optional[float] = None
    # Full ranked candidate scores (high-to-low) from the most recent reply, if one
    # is ready this tick. When present it supersedes ``best_candidate_score`` and
    # enables the ranked-candidate walk. None while no fresh reply is available.
    candidate_scores: Optional[Tuple[float, ...]] = None
    # Current alignment fitness, if known (lower is better).
    best_fitness: Optional[float] = None
    # Optional unique timestamp for the fitness observation. When supplied, the
    # reducer counts a recovery sample only once even if the node ticks faster
    # than /alignment_status arrives.
    fitness_observed_sec: Optional[float] = None
    # Optional health classification from /alignment_status. When supplied,
    # recovery evidence must be both low-fitness and stable tracking, not just a
    # transient low score from a degraded/recovering state.
    stable_tracking: Optional[bool] = None
    # Distance (m) between the verify query's top raw fix and the localizer pose at
    # the fix scan stamp; None when the node could not evaluate it.
    cross_check_mismatch_m: Optional[float] = None


@dataclass(frozen=True)
class SupervisorDecision:
    action: str
    reason: str
    state: SupervisorState
    # On ACTION_PUBLISH_RESET, which candidate (rank index, 0 = best) the node must
    # publish. The node holds the ranked candidate poses from the last query reply.
    candidate_index: int = 0


def initial_state() -> SupervisorState:
    return SupervisorState()


def _standdown_confirmed(state: SupervisorState) -> SupervisorState:
    """Next state after a confirmed recovery (direct or post verify cross-check)."""
    return replace(
        state,
        name=STATE_STANDDOWN,
        attempts=0,
        last_reset_sec=None,
        cooldown_since_sec=None,
        query_issued_sec=None,
        candidate_scores=(),
        candidate_index=0,
        recovery_evidence_count=0,
        last_recovery_fitness_observed_sec=None,
        alias_confirmed=False,
    )


def _cooldown(state: SupervisorState, now_sec: float, reason: str) -> SupervisorDecision:
    return SupervisorDecision(
        ACTION_NONE,
        reason,
        replace(state, name=STATE_COOLDOWN, cooldown_since_sec=now_sec,
                query_issued_sec=None, recovery_evidence_count=0,
                last_recovery_fitness_observed_sec=None),
    )


def _resolve_candidate_scores(
    obs: SupervisorObservation,
) -> Optional[Tuple[float, ...]]:
    """Return the ranked candidate scores ready this tick, or None if no reply.

    Prefers the full ranked ``candidate_scores`` list; falls back to the
    single-candidate ``best_candidate_score`` shorthand for back-compat. An empty
    tuple means a reply arrived but carried no usable candidate (distinct from None,
    which means no reply yet).
    """
    if obs.candidate_scores is not None:
        return tuple(obs.candidate_scores)
    if obs.best_candidate_score is not None:
        return (obs.best_candidate_score,)
    return None


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
        if not requested and not state.alias_confirmed:
            return SupervisorDecision(
                ACTION_NONE, "request_cleared",
                replace(
                    state, name=STATE_IDLE, attempts=0, recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None, alias_confirmed=False))
        if state.attempts >= params.max_attempts:
            return SupervisorDecision(
                ACTION_GIVE_UP, "attempts_exhausted",
                replace(
                    state, name=STATE_EXHAUSTED, recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None, alias_confirmed=False))
        # Respect spacing relative to the previous reset, if any.
        if (state.last_reset_sec is not None
                and now - state.last_reset_sec < params.min_seconds_between_attempts):
            return SupervisorDecision(ACTION_NONE, "spacing_hold", state)
        return SupervisorDecision(
            ACTION_QUERY, "query_issued",
            replace(state, name=STATE_AWAIT_CANDIDATES, query_issued_sec=now))

    if name == STATE_AWAIT_CANDIDATES:
        reply_scores = _resolve_candidate_scores(obs)
        if reply_scores is not None:
            if reply_scores and reply_scores[0] >= params.min_candidate_score:
                # Publish the best candidate and start walking the ranked list.
                return SupervisorDecision(
                    ACTION_PUBLISH_RESET, "reset_published",
                    replace(state, name=STATE_SETTLING, last_reset_sec=now,
                            query_issued_sec=None, attempts=state.attempts + 1,
                            candidate_scores=tuple(reply_scores), candidate_index=0,
                            recovery_evidence_count=0,
                            last_recovery_fitness_observed_sec=None),
                    candidate_index=0)
            # Empty list or confidently-bad best candidate: never publish; count attempt.
            return _cooldown(
                replace(state, attempts=state.attempts + 1,
                        candidate_scores=(), candidate_index=0),
                now, "weak_candidate_rejected")
        # No candidates yet -- wait, or abandon on timeout.
        if (state.query_issued_sec is not None
                and now - state.query_issued_sec >= params.query_timeout_sec):
            return _cooldown(
                replace(state, attempts=state.attempts + 1), now, "query_timeout")
        return SupervisorDecision(ACTION_NONE, "awaiting_candidates", state)

    if name == STATE_SETTLING:
        recovery_confirmation_samples = max(1, int(params.recovery_confirmation_samples))
        fitness_observed_sec = obs.fitness_observed_sec
        has_new_fitness = obs.best_fitness is not None
        if (has_new_fitness and fitness_observed_sec is not None
                and fitness_observed_sec == state.last_recovery_fitness_observed_sec):
            has_new_fitness = False
        if has_new_fitness:
            has_recovery_evidence = (
                obs.best_fitness is not None
                and obs.best_fitness <= params.recovery_fitness_threshold
                and obs.stable_tracking is not False)
            recovery_evidence_count = (
                state.recovery_evidence_count + 1 if has_recovery_evidence else 0)
            last_recovery_fitness_observed_sec = fitness_observed_sec
        else:
            recovery_evidence_count = state.recovery_evidence_count
            last_recovery_fitness_observed_sec = state.last_recovery_fitness_observed_sec
        state_with_evidence = replace(
            state,
            recovery_evidence_count=recovery_evidence_count,
            last_recovery_fitness_observed_sec=last_recovery_fitness_observed_sec)
        recovered = recovery_evidence_count >= recovery_confirmation_samples
        if recovered:
            if params.enable_confirm_cross_check:
                # One more G2 query: a fresh global fix that disagrees with the
                # localizer exposes along-corridor alias locks that pass fitness alone.
                return SupervisorDecision(
                    ACTION_QUERY, "confirm_cross_check_query",
                    replace(
                        state_with_evidence,
                        name=STATE_VERIFYING,
                        query_issued_sec=now,
                        recovery_evidence_count=0,
                        last_recovery_fitness_observed_sec=None,
                        candidate_scores=(),
                        candidate_index=0))
            # Confirmed recovery: stand down and clear the ceiling, but wait for
            # the request to de-assert before re-arming (edge-triggered next time).
            return SupervisorDecision(
                ACTION_NONE, "recovery_confirmed",
                _standdown_confirmed(state_with_evidence))
        if (state.last_reset_sec is not None
                and now - state.last_reset_sec >= params.settle_timeout_sec):
            # This candidate did not take. Walk to the next-best candidate from the
            # same query (the localizer's fitness is the registration oracle) before
            # spending another attempt -- only re-querying consumes max_attempts.
            next_index = state.candidate_index + 1
            if (next_index < len(state.candidate_scores)
                    and next_index < params.max_walk_candidates
                    and state.candidate_scores[next_index] >= params.min_candidate_score):
                return SupervisorDecision(
                    ACTION_PUBLISH_RESET, "next_candidate",
                    replace(
                        state_with_evidence,
                        last_reset_sec=now,
                        candidate_index=next_index,
                        recovery_evidence_count=0,
                        last_recovery_fitness_observed_sec=None),
                    candidate_index=next_index)
            # Ranked list exhausted: the whole query failed. Give up if the attempt
            # ceiling is reached, else cool down and re-query.
            if state.attempts >= params.max_attempts:
                return SupervisorDecision(
                    ACTION_GIVE_UP, "recovery_failed_exhausted",
                    replace(state_with_evidence, name=STATE_EXHAUSTED,
                            recovery_evidence_count=0,
                            last_recovery_fitness_observed_sec=None,
                            alias_confirmed=False))
            return _cooldown(
                replace(
                    state_with_evidence,
                    candidate_scores=(),
                    candidate_index=0,
                    recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None),
                now, "recovery_unconfirmed")
        return SupervisorDecision(ACTION_NONE, "settling", state_with_evidence)

    if name == STATE_VERIFYING:
        # Complete the cross-check even if the localizer reports tracking again
        # after a reset -- a false alias can look healthy until the verify reply.
        reply_scores = _resolve_candidate_scores(obs)
        if reply_scores is not None:
            mismatch = obs.cross_check_mismatch_m
            if (mismatch is None
                    or mismatch <= params.cross_check_mismatch_m):
                return SupervisorDecision(
                    ACTION_NONE, "recovery_confirmed",
                    _standdown_confirmed(state))
            # The reset this verify query is checking already paid an attempt at
            # publish time; the mismatch is only the honest detection of its
            # failure. Charging it again would halve the effective ceiling
            # (every alias round would cost 2 of max_attempts).
            cleared = replace(
                state,
                query_issued_sec=None,
                candidate_scores=(),
                candidate_index=0,
                recovery_evidence_count=0,
                last_recovery_fitness_observed_sec=None,
            )
            if state.attempts >= params.max_attempts:
                return SupervisorDecision(
                    ACTION_GIVE_UP, "recovery_failed_exhausted",
                    replace(
                        cleared,
                        name=STATE_EXHAUSTED,
                        recovery_evidence_count=0,
                        last_recovery_fitness_observed_sec=None,
                        alias_confirmed=False))
            if reply_scores and reply_scores[0] >= params.min_candidate_score:
                # The verify reply is itself the freshest trustworthy fix (and it
                # just updated the fix-to-fix velocity pair), so reseed from it
                # immediately: a cooldown + re-query would add a full query
                # runtime of staleness to the next seed for no information gain.
                return SupervisorDecision(
                    ACTION_PUBLISH_RESET, "cross_check_reseed",
                    replace(
                        cleared,
                        name=STATE_SETTLING,
                        last_reset_sec=now,
                        attempts=state.attempts + 1,
                        candidate_scores=tuple(reply_scores),
                        candidate_index=0,
                        alias_confirmed=True),
                    candidate_index=0)
            return _cooldown(
                replace(cleared, alias_confirmed=True), now, "cross_check_mismatch")
        if (state.query_issued_sec is not None
                and now - state.query_issued_sec >= params.query_timeout_sec):
            return SupervisorDecision(
                ACTION_NONE, "cross_check_timeout",
                _standdown_confirmed(state))
        return SupervisorDecision(ACTION_NONE, "verifying", state)

    if name == STATE_COOLDOWN:
        if state.cooldown_since_sec is not None and now - state.cooldown_since_sec < params.min_seconds_between_attempts:
            return SupervisorDecision(ACTION_NONE, "cooldown", state)
        if state.attempts >= params.max_attempts:
            return SupervisorDecision(
                ACTION_GIVE_UP, "attempts_exhausted",
                replace(
                    state, name=STATE_EXHAUSTED, recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None, alias_confirmed=False))
        if not requested and not state.alias_confirmed:
            if (params.enable_confirm_cross_check
                    and state.last_reset_sec is not None):
                return SupervisorDecision(
                    ACTION_QUERY, "self_clear_cross_check_query",
                    replace(
                        state,
                        name=STATE_VERIFYING,
                        query_issued_sec=now,
                        candidate_scores=(),
                        candidate_index=0))
            return SupervisorDecision(
                ACTION_NONE, "request_cleared",
                replace(
                    state, name=STATE_IDLE, attempts=0, recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None, alias_confirmed=False))
        return SupervisorDecision(
            ACTION_NONE, "cooldown_elapsed",
            replace(state, name=STATE_AWAIT_QUERY, cooldown_since_sec=None))

    if name == STATE_STANDDOWN:
        # Wait for the request to clear, then re-arm for the next problem.
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "standdown_cleared",
                replace(
                    state, name=STATE_IDLE, attempts=0, request_since_sec=None,
                    recovery_evidence_count=0,
                    last_recovery_fitness_observed_sec=None,
                    alias_confirmed=False))
        return SupervisorDecision(ACTION_NONE, "standdown", state)

    if name == STATE_EXHAUSTED:
        # Latched until the underlying problem clears (request drops) -- the node
        # should surface this to an operator rather than retry silently.
        if not requested:
            return SupervisorDecision(
                ACTION_NONE, "exhausted_cleared",
                replace(state, name=STATE_IDLE, attempts=0, last_reset_sec=None,
                        cooldown_since_sec=None, recovery_evidence_count=0,
                        last_recovery_fitness_observed_sec=None,
                        alias_confirmed=False))
        return SupervisorDecision(ACTION_NONE, "exhausted", state)

    raise ValueError(f"unknown supervisor state: {name!r}")


# --- seed motion compensation --------------------------------------------------
#
# The decisive G3 live-run blocker (docs/g3_live_closed_loop.md, 5th clean run)
# was *not* the supervisor, the walk, candidate quality, or yaw: it was latency.
# A G2 BBS query takes ~5-23 s, and the candidate it returns is a fix of the scan
# taken when the query was *issued*. By the time the reset is published the vehicle
# has driven on, so the seed lands metres behind the vehicle -- outside the
# registration basin -- and the lock never takes, even when the candidate was a
# correct rank-1 fix at query time.
#
# We cannot make the query instant here (that is the G2 C++ port), but we can
# forward-compensate: estimate the vehicle's map-frame velocity and push the seed
# ahead by the measured query->publish latency. The velocity source must work on
# the Koide bag, which carries no twist/odom topic (only /livox lidar+imu) -- so we
# estimate it from the data we already have: *successive query fixes*. Each query
# returns an absolute map-frame position at its issue time, so the displacement
# between two consecutive published fixes, divided by the wall time between their
# issues, is a direct map-frame velocity. Using monotonic wall time consistently
# for both the velocity dt and the latency keeps this correct under use_sim_time
# and any constant bag-replay rate (the rate cancels).
#
# Safety: a perceptual-aliasing wrong candidate (the live run saw a 190 m-off
# candidate scored 0.99) would yield an absurd inferred speed. We reject any
# estimate above ``max_speed_mps`` and fall back to publishing the raw candidate --
# so compensation only ever fires when two consecutive fixes are mutually
# consistent, which is exactly when the velocity is trustworthy. This is opt-in
# (off by default) and never weakens the publish: a bad estimate is a no-op.


@dataclass(frozen=True)
class SeedVelocity:
    """Map-frame velocity (m/s) inferred from two successive query fixes."""
    vx: float = 0.0
    vy: float = 0.0
    valid: bool = False


def estimate_seed_velocity(
    prev_xy: Tuple[float, float],
    prev_issue_sec: float,
    curr_xy: Tuple[float, float],
    curr_issue_sec: float,
    max_speed_mps: float,
    min_dt_sec: float = 0.5,
) -> SeedVelocity:
    """Map-frame velocity from two consecutive query fixes, or invalid.

    ``prev_xy`` / ``curr_xy`` are the published candidate positions of two
    *different* queries; ``*_issue_sec`` are the monotonic times those queries
    were issued (a good proxy for each fix's scan time). Returns ``valid=False``
    when the two fixes are too close in time to differentiate (same query / a
    within-query walk, where ``dt`` is ~0) or when the implied speed exceeds
    ``max_speed_mps`` -- the latter rejects an inconsistent pair (e.g. one fix was
    a perceptual-aliasing wrong candidate), so a bad pair never compensates.
    """
    dt = curr_issue_sec - prev_issue_sec
    if dt < min_dt_sec:
        return SeedVelocity()
    vx = (curr_xy[0] - prev_xy[0]) / dt
    vy = (curr_xy[1] - prev_xy[1]) / dt
    if math.hypot(vx, vy) > max_speed_mps:
        return SeedVelocity()
    return SeedVelocity(vx=vx, vy=vy, valid=True)


class TrustedSeedVelocityTracker:
    """Map-frame velocity from consecutive trusted localizer poses only.

    The Koide 180 s boundary characterization (docs/g3_live_closed_loop.md) showed
    that extrapolating reset seeds with velocity from the kidnapped localizer's pose
    history yields wrong direction and magnitude -- turning 1 m-accurate G2 fixes into
    14-28 m seed errors. Pose-derived velocity is trustworthy only while tracking was
    stable and no recovery episode was active; during the episode itself the pose
    stream is exactly what we must not trust.

    Pair history is cleared across untrusted gaps so a later trusted pose never
    pairs with one from before the gap. The last computed velocity and its timestamp
    are retained across gaps so a recent pre-episode estimate can still be used
    within an age bound. Constant-velocity extrapolation decays with time (e.g.
    cornering), so ``velocity_at`` rejects estimates older than ``max_age_sec``.
    """

    def __init__(self, max_speed_mps: float, min_pair_dt_sec: float = 0.5) -> None:
        self._max_speed_mps = max_speed_mps
        # Poses can arrive faster than min_pair_dt_sec (the estimate_seed_velocity
        # floor below which a pair is indistinguishable from noise). The anchor is
        # held until the baseline is long enough; advancing it on every message
        # would make every pair sub-floor and the tracker permanently invalid.
        self._min_pair_dt_sec = min_pair_dt_sec
        self._prev_x: Optional[float] = None
        self._prev_y: Optional[float] = None
        self._prev_sec: Optional[float] = None
        self._prev_trusted = False
        self._velocity = SeedVelocity()
        self._velocity_sec: Optional[float] = None

    def observe(self, x: float, y: float, observed_sec: float, trusted: bool) -> None:
        if trusted:
            if (self._prev_trusted and self._prev_x is not None
                    and self._prev_y is not None and self._prev_sec is not None):
                if observed_sec - self._prev_sec < self._min_pair_dt_sec:
                    return  # hold the anchor until the baseline is long enough
                new_velocity = estimate_seed_velocity(
                    (self._prev_x, self._prev_y), self._prev_sec,
                    (x, y), observed_sec, self._max_speed_mps,
                    min_dt_sec=self._min_pair_dt_sec)
                if new_velocity.valid:
                    self._velocity = new_velocity
                    self._velocity_sec = observed_sec
            self._prev_x = x
            self._prev_y = y
            self._prev_sec = observed_sec
            self._prev_trusted = True
            return
        self._prev_x = None
        self._prev_y = None
        self._prev_sec = None
        self._prev_trusted = False

    def velocity_at(self, now_sec: float, max_age_sec: float) -> SeedVelocity:
        if not self._velocity.valid or self._velocity_sec is None:
            return SeedVelocity()
        if now_sec - self._velocity_sec > max_age_sec:
            return SeedVelocity()
        return self._velocity

    def velocity_rejection_reason(
        self, now_sec: float, max_age_sec: float,
    ) -> Optional[str]:
        """Human-readable reason ``velocity_at`` would return invalid, or None."""
        if not self._velocity.valid or self._velocity_sec is None:
            return "no trusted velocity"
        age = now_sec - self._velocity_sec
        if age > max_age_sec:
            return "stale trusted velocity (age %.1fs > %.1fs)" % (age, max_age_sec)
        return None


def estimate_sim_fix_velocity(
    prev_xy: Tuple[float, float],
    prev_scan_stamp_sec: float,
    curr_xy: Tuple[float, float],
    curr_scan_stamp_sec: float,
    max_speed_mps: float,
    min_dt_sec: float = 2.0,
    max_dt_sec: float = 60.0,
) -> SeedVelocity:
    """Map-frame velocity from two consecutive query fixes on the bag clock.

    Real-kidnap Koide replays showed G2 raw fixes are accurate at their scan
    stamps, but seeds are published many bag-seconds later and land stale.
    Wall-clock compensation cannot measure post-kidnap velocity (the trusted
    pose tracker only has pre-kidnap motion), yet two consecutive query fixes on
    ``scan_stamp_sec`` give the true map-frame velocity on the simulation clock.
    Returns ``valid=False`` when the pair is too close or too far apart in bag
    time, or when the implied speed exceeds ``max_speed_mps`` (an inconsistent
    pair, e.g. one wrong-basin fix).
    """
    dt = curr_scan_stamp_sec - prev_scan_stamp_sec
    if dt < min_dt_sec or dt > max_dt_sec:
        return SeedVelocity()
    vx = (curr_xy[0] - prev_xy[0]) / dt
    vy = (curr_xy[1] - prev_xy[1]) / dt
    if math.hypot(vx, vy) > max_speed_mps:
        return SeedVelocity()
    return SeedVelocity(vx=vx, vy=vy, valid=True)


def estimate_pose_delta_from_sim_fix(
    velocity: SeedVelocity,
    fix_scan_stamp_sec: float,
    last_sim_stamp_sec: Optional[float],
    max_latency_sec: float,
) -> Optional[Tuple[float, float, float, float]]:
    """Forward-extrapolate a seed by bag-clock staleness since the latest fix.

    ``last_sim_stamp_sec`` is the most recent bag-clock stamp seen on
    ``/alignment_status`` or ``/pcl_pose``; ``fix_scan_stamp_sec`` is the scan
    time of the velocity-bearing query fix. Staleness is clamped to
    ``[0, max_latency_sec]`` so a pathologically delayed publish cannot
    extrapolate arbitrarily far. Returns ``(dx, dy, speed, staleness)`` or
    ``None`` when the velocity or timestamps are unusable.
    """
    if not velocity.valid or last_sim_stamp_sec is None:
        return None
    staleness = max(0.0, min(last_sim_stamp_sec - fix_scan_stamp_sec, max_latency_sec))
    if staleness <= 0.0:
        return None
    speed = math.hypot(velocity.vx, velocity.vy)
    return (
        velocity.vx * staleness,
        velocity.vy * staleness,
        speed,
        staleness,
    )


def forward_compensate_xy(
    xy: Tuple[float, float],
    velocity: SeedVelocity,
    latency_sec: float,
    max_latency_sec: float,
) -> Tuple[float, float]:
    """Push ``xy`` ahead by ``velocity * latency`` (clamped), or return it unchanged.

    ``latency_sec`` is the query->publish delay; it is clamped to
    ``max_latency_sec`` so a pathologically slow query cannot extrapolate the seed
    arbitrarily far on a first-order model. A non-positive latency or an invalid
    velocity is a no-op, so this can never move a seed backwards or sideways from a
    bad estimate.
    """
    if not velocity.valid or latency_sec <= 0.0:
        return xy
    dt = min(latency_sec, max_latency_sec)
    return (xy[0] + velocity.vx * dt, xy[1] + velocity.vy * dt)
