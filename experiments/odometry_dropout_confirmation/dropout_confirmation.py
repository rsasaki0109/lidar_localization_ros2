#!/usr/bin/env python3
"""Compare G3 answer-confirmation policies around odometry dropouts.

`reinitialization_supervisor_node._withhold_unconfirmed_answer` lets a G2 answer
reset the pose only when a second answer agrees after the bridged odometry motion.
It waives that confirmation for the whole 120 s window once odom TF had a gap over
2 s. On one Koide `outdoor_hard_02b` quickstart run, RKO-LIO dropped frames
with too few keypoints, and a single unconfirmed answer reset the pose 128 m away.
On `outdoor_kidnap_b`, the waiver lets a carried sensor recover.

Every variant here answers the same question for each G2 answer: release it
(it may reset the pose when it passed the gates) or withhold it. A shared
simulator applies the supervisor's attempt budget (`max_attempts`). The
fixtures are synthetic timelines. They do not replace the Koide replay that
`docs/v1_status.md` requires before any runtime default changes.

Run `python3 dropout_confirmation.py --json results.json` to regenerate the results.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import quickstart_model  # noqa: E402

Pose = tuple[float, float, float]  # map x, y, yaw (rad)


# --- shared parameters (supervisor and startup-consensus defaults) ------------


@dataclass(frozen=True)
class Params:
    odometry_confirmation_window_sec: float = 120.0
    odometry_max_gap_sec: float = 2.0
    bridge_lookup_tolerance_sec: float = 0.5
    anchor_history: int = 5
    max_attempts: int = 3
    consensus: quickstart_model.StartupParams = field(
        default_factory=quickstart_model.StartupParams
    )
    # Only for the speed-bound variant: an upper bound on platform speed (a person
    # carrying a handheld MID-360). It is platform-specific, not a general default.
    max_platform_speed_mps: float = 1.5


# --- fixtures ----------------------------------------------------------------


@dataclass(frozen=True)
class Answer:
    stamp: float
    pose: Pose
    gated: bool  # passed min score and the distinctiveness gates
    correct: bool  # evaluation only; no variant reads it


@dataclass(frozen=True)
class Fixture:
    name: str
    description: str
    odometry_stamps: tuple[float, ...]
    bridge: tuple[tuple[float, float, float, float], ...]  # (stamp, x, y, yaw)
    answers: tuple[Answer, ...]
    reference_stamp: float  # when tracking was lost; time to recover counts from here
    recovery_required: bool


def _stamps(start, end, gaps=(), rate_hz=10.0):
    count = round((end - start) * rate_hz)
    stamps = (round(start + i / rate_hz, 6) for i in range(count + 1))
    return tuple(t for t in stamps if not any(g0 < t < g1 for g0, g1 in gaps))


def _bridge(stamps, pose_at):
    return tuple((t, *pose_at(t)) for t in stamps)


def _offset(pose: Pose, dx: float, dy: float, dyaw: float = 0.0) -> Pose:
    return (pose[0] + dx, pose[1] + dy, pose[2] + dyaw)


def _restarted_odometry(frozen: Pose, truth, restart_stamp):
    """Bridge after the LIO restarts at identity: frozen pose x motion since restart."""

    def pose_at(t):
        motion = quickstart_model.planar_motion_between(truth(restart_stamp), truth(t))
        return quickstart_model.apply_planar_motion(frozen, motion)

    return pose_at


def _straight(speed, origin=(0.0, 0.0, 0.0)):
    def truth(t):
        return (
            origin[0] + speed * t * math.cos(origin[2]),
            origin[1] + speed * t * math.sin(origin[2]),
            origin[2],
        )

    return truth


def _carry_truth(t):
    """Stationary, carried 68 m and turned 90 deg while covered (60-90 s), then walks."""
    start, end = (10.0, 5.0, 0.0), (70.0, 40.0, math.pi / 2.0)
    if t <= 60.0:
        return start
    if t <= 90.0:
        a = (t - 60.0) / 30.0
        return tuple(s + a * (e - s) for s, e in zip(start, end, strict=True))
    return (end[0], end[1] + 0.5 * (t - 90.0), end[2])


def _short_dropout_fixture(name, description, gap, wrong_stamp, correct_stamps):
    """Walking at 1 m/s; RKO drops frames for a few seconds; tracking already lost."""
    truth = _straight(1.0)
    odometry = _stamps(0.0, 140.0, gaps=[gap])
    # The frozen map -> odom is 2.2 m off, so the bridged pose is off by as much.
    bridge = _bridge(odometry, lambda t: _offset(truth(t), 2.0, 1.0))
    wrong = Answer(
        wrong_stamp, _offset(truth(wrong_stamp), -90.5, 90.5, 0.6), True, False
    )
    noise = ((0.4, -0.3, 0.02), (-0.3, 0.2, -0.01))
    correct = tuple(
        Answer(t, _offset(truth(t), *n), True, True)
        for t, n in zip(correct_stamps, noise, strict=True)
    )
    return Fixture(name, description, odometry, bridge, (wrong, *correct), 100.0, True)


def _carry_odometry_and_bridge():
    odometry = _stamps(0.0, 140.0, gaps=[(60.0, 90.0)])
    before = _bridge((t for t in odometry if t <= 60.0), _carry_truth)
    after = _bridge(
        (t for t in odometry if t >= 90.0),
        _restarted_odometry(_carry_truth(60.0), _carry_truth, 90.0),
    )
    return odometry, before + after


def _correct_carry_answers(stamps):
    noise = ((0.3, 0.4, 0.02), (-0.2, 0.3, -0.02), (0.1, -0.4, 0.01))
    return tuple(
        Answer(t, _offset(_carry_truth(t), *n), True, True)
        for t, n in zip(stamps, noise[: len(stamps)], strict=True)
    )


def build_fixtures() -> tuple[Fixture, ...]:
    fixtures = []

    fixtures.append(
        _short_dropout_fixture(
            "hard02b_wrong_answer_after_short_dropout",
            "RKO drops frames for 3.5 s; the next answer is distinctive but 128 m off, "
            "later answers are correct (one reading of the outdoor_hard_02b failure)",
            gap=(100.0, 103.5),
            wrong_stamp=106.0,
            correct_stamps=(112.0, 118.0),
        )
    )
    fixtures.append(
        _short_dropout_fixture(
            "hard02b_wrong_answer_inside_dropout",
            "the wrong answer comes from a scan inside the odometry gap, so no bridged "
            "pose covers it; later answers are correct (the other reading)",
            gap=(100.0, 104.0),
            wrong_stamp=102.0,
            correct_stamps=(108.0, 114.0),
        )
    )

    odometry, bridge = _carry_odometry_and_bridge()
    fixtures.append(
        Fixture(
            "kidnap_covered_carry",
            "sensor covered and carried 68 m for 30 s (odom stops, then restarts at "
            "identity); every answer after the cover is correct",
            odometry,
            bridge,
            _correct_carry_answers((95.0, 101.0, 107.0)),
            90.0,
            True,
        )
    )
    fixtures.append(
        Fixture(
            "kidnap_stationary_alias_across_gap",
            "an answer before the cover, then a wrong answer where the robot would be "
            "had it not been carried; correct answers follow",
            odometry,
            bridge,
            (
                Answer(57.0, _offset(_carry_truth(57.0), 0.2, 0.1, 0.01), True, True),
                Answer(95.0, (12.3, 5.2, 0.03), True, False),
                *_correct_carry_answers((101.0, 107.0)),
            ),
            90.0,
            False,
        )
    )
    fixtures.append(
        Fixture(
            "kidnap_answers_during_cover",
            "G2 answers on two sparse covered scans (weak, not gated) before the "
            "odometry resumes; correct answers follow",
            odometry,
            bridge,
            (
                Answer(70.0, (10.0, 5.0, 0.0), False, False),
                Answer(80.0, (10.0, 5.0, 0.0), False, False),
                *_correct_carry_answers((95.0, 101.0)),
            ),
            90.0,
            True,
        )
    )

    # After the carry, RKO keeps losing keypoints: it drops out between every two
    # answers and restarts at identity each time.
    resume_gaps = [(60.0, 90.0), (97.0, 99.5), (103.0, 105.5), (109.0, 111.5)]
    intermittent = _stamps(0.0, 140.0, gaps=resume_gaps)
    segment_starts = [0.0, 90.0, 99.5, 105.5, 111.5]
    frozen = _carry_truth(60.0)

    def intermittent_bridge(t):
        start = max(s for s in segment_starts if s <= t)
        if start == 0.0:
            return _carry_truth(t)
        return _restarted_odometry(frozen, _carry_truth, start)(t)

    fixtures.append(
        Fixture(
            "kidnap_intermittent_odometry",
            "after the carry the odometry drops out (and restarts) between every "
            "two answers; all answers are correct",
            intermittent,
            _bridge(intermittent, intermittent_bridge),
            _correct_carry_answers((96.0, 102.0, 108.0)),
            90.0,
            True,
        )
    )

    truth = _straight(0.8)
    odometry = _stamps(0.0, 100.0)
    fixtures.append(
        Fixture(
            "continuous_alias_then_correct",
            "no dropout: an aliased answer 65 m off, then correct answers (#178)",
            odometry,
            _bridge(odometry, lambda t: _offset(truth(t), 1.5, -1.0)),
            (
                Answer(50.0, _offset(truth(50.0), -46.0, 46.0, 0.4), True, False),
                Answer(56.0, _offset(truth(56.0), 0.3, 0.2, 0.01), True, True),
                Answer(62.0, _offset(truth(62.0), -0.2, -0.3, -0.02), True, True),
            ),
            40.0,
            True,
        )
    )
    fixtures.append(
        Fixture(
            "no_external_odometry",
            "no odom TF and no bridge at all: one gated answer resets, as before #178",
            (),
            (),
            (Answer(30.0, (5.2, 1.1, 0.02), True, True),),
            25.0,
            True,
        )
    )

    truth = _straight(0.3)
    odometry = _stamps(0.0, 140.0, gaps=[(100.0, 103.5)])
    alias = (60.0, 80.0, 1.0)
    fixtures.append(
        Fixture(
            "repeated_alias_across_short_dropout",
            "walking at 0.3 m/s; the same wrong place is answered before and after a "
            "3.5 s odometry gap, then a correct answer",
            odometry,
            _bridge(odometry, lambda t: _offset(truth(t), 2.0, 1.0)),
            (
                Answer(98.0, alias, True, False),
                Answer(104.5, _offset(alias, 0.2, -0.1, 0.01), True, False),
                Answer(110.0, _offset(truth(110.0), 0.3, 0.2, 0.01), True, True),
            ),
            98.0,
            False,
        )
    )

    truth = _straight(0.5)
    odometry = _stamps(0.0, 60.0)
    fixtures.append(
        Fixture(
            "odometry_lost_permanently",
            "the LIO stops at 60 s and never returns; correct answers every 6 s",
            odometry,
            _bridge(odometry, truth),
            tuple(
                Answer(t, _offset(truth(t), 0.2, -0.2, 0.01), True, True)
                for t in (66.0 + 6.0 * i for i in range(23))
            ),
            60.0,
            True,
        )
    )
    return tuple(fixtures)


# --- timeline queries shared by the variants ---------------------------------


class Timeline:
    def __init__(self, fixture: Fixture, params: Params):
        self.odometry = fixture.odometry_stamps
        self.bridge = fixture.bridge
        self.params = params

    def bridge_at(self, stamp: float) -> Pose | None:
        best = min(self.bridge, key=lambda e: abs(e[0] - stamp), default=None)
        if (
            best is None
            or abs(best[0] - stamp) > self.params.bridge_lookup_tolerance_sec
        ):
            return None
        return best[1:4]

    def bridge_seen_within_window(self, stamp: float) -> bool:
        start = stamp - self.params.odometry_confirmation_window_sec
        return any(start <= e[0] <= stamp for e in self.bridge)

    def _gapless(self, stamps) -> bool:
        gap = self.params.odometry_max_gap_sec
        return all(b - a <= gap for a, b in itertools.pairwise(stamps))

    def window_continuous(self, stamp: float) -> bool:
        """`_odometry_continuous`: no gap over the window before the answer."""
        start = stamp - self.params.odometry_confirmation_window_sec
        stamps = [s for s in self.odometry if start <= s <= stamp]
        if not stamps or stamp - stamps[-1] > self.params.odometry_max_gap_sec:
            return False
        return self._gapless(stamps)

    def continuous_between(self, start: float, end: float) -> bool:
        """No odometry gap anywhere between two answers, including at either end."""
        gap = self.params.odometry_max_gap_sec
        stamps = [s for s in self.odometry if start - gap <= s <= end]
        inside = [s for s in stamps if s >= start]
        if not inside or end - inside[-1] > gap:
            return False
        return self._gapless(stamps) and stamps[0] <= start


# --- variants ----------------------------------------------------------------


@dataclass(frozen=True)
class Anchor:
    stamp: float
    correction: Pose  # answer x bridged^-1: the map -> bridge-frame correction
    bridged: Pose
    gated: bool


RELEASE, WITHHOLD, DEFER = "release", "withhold", "defer"


@dataclass
class Context:
    params: Params
    timeline: Timeline
    anchors: deque


def _correction(pose: Pose, bridged: Pose) -> Pose:
    return quickstart_model.apply_planar_motion(
        pose, quickstart_model.planar_motion_between(bridged, (0.0, 0.0, 0.0))
    )


def _agrees_by_odometry(ctx: Context, anchor: Anchor, answer: Answer, bridged: Pose):
    """The supervisor's test: the earlier answer moved by the bridged motion."""
    consensus = ctx.params.consensus
    predicted = quickstart_model.apply_planar_motion(anchor.correction, bridged)
    travelled = math.hypot(
        bridged[0] - anchor.bridged[0], bridged[1] - anchor.bridged[1]
    )
    tolerance = (
        consensus.global_consensus_translation_m
        + consensus.global_consensus_translation_per_odom_m * travelled
    )
    yaw = answer.pose[2] - predicted[2]
    yaw_error = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
    return (
        math.hypot(answer.pose[0] - predicted[0], answer.pose[1] - predicted[1])
        <= tolerance
        and math.degrees(yaw_error) <= consensus.global_consensus_yaw_deg
    )


def _agrees_by_speed_bound(ctx: Context, anchor: Anchor, answer: Answer):
    """Across a gap: the second answer is reachable from the first at bounded speed."""
    previous = quickstart_model.apply_planar_motion(anchor.correction, anchor.bridged)
    reach = ctx.params.consensus.global_consensus_translation_m + (
        ctx.params.max_platform_speed_mps * (answer.stamp - anchor.stamp)
    )
    return (
        math.hypot(answer.pose[0] - previous[0], answer.pose[1] - previous[1]) <= reach
    )


def _pair_confirmation(ctx: Context, answer: Answer, bridged: Pose, pair_test):
    """Release an answer that agrees with an earlier one; remember it either way."""
    window = ctx.params.odometry_confirmation_window_sec
    agrees = any(
        anchor.stamp < answer.stamp - 1.0e-9
        and answer.stamp - anchor.stamp <= window
        and (answer.gated or anchor.gated)
        and pair_test(anchor)
        for anchor in ctx.anchors
    )
    if not any(a.stamp >= answer.stamp - 1.0e-9 for a in ctx.anchors):
        ctx.anchors.append(
            Anchor(
                answer.stamp, _correction(answer.pose, bridged), bridged, answer.gated
            )
        )
    if agrees and answer.gated:
        ctx.anchors.clear()
        return RELEASE, "confirmed"
    return WITHHOLD, "unconfirmed"


def window_waiver(ctx: Context, answer: Answer):
    """Runtime today: no confirmation when the odom TF had a gap within 120 s."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        return RELEASE, "no_bridged_pose"
    if not ctx.timeline.window_continuous(answer.stamp):
        ctx.anchors.clear()
        return RELEASE, "dropout_waiver"
    return _pair_confirmation(
        ctx, answer, bridged, lambda a: _agrees_by_odometry(ctx, a, answer, bridged)
    )


def no_waiver(ctx: Context, answer: Answer):
    """The waiver removed: pairs are compared even across an odometry gap."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        return RELEASE, "no_bridged_pose"
    return _pair_confirmation(
        ctx, answer, bridged, lambda a: _agrees_by_odometry(ctx, a, answer, bridged)
    )


def _segment_test(ctx: Context, answer: Answer, bridged: Pose):
    return lambda a: (
        ctx.timeline.continuous_between(a.stamp, answer.stamp)
        and _agrees_by_odometry(ctx, a, answer, bridged)
    )


def segment(ctx: Context, answer: Answer):
    """Confirm only by a pair with gapless odometry between the two answers."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        return RELEASE, "no_bridged_pose"
    return _pair_confirmation(ctx, answer, bridged, _segment_test(ctx, answer, bridged))


def segment_bridge_guard(ctx: Context, answer: Answer):
    """As segment; an answer inside a dropout (bridge seen lately, none now) waits."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        if ctx.timeline.bridge_seen_within_window(answer.stamp):
            return WITHHOLD, "inside_dropout"
        return RELEASE, "no_bridged_pose"
    return _pair_confirmation(ctx, answer, bridged, _segment_test(ctx, answer, bridged))


def segment_defer(ctx: Context, answer: Answer):
    """As the guard, but no query is spent while the odometry is out."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        if ctx.timeline.bridge_seen_within_window(answer.stamp):
            return DEFER, "deferred_inside_dropout"
        return RELEASE, "no_bridged_pose"
    return _pair_confirmation(ctx, answer, bridged, _segment_test(ctx, answer, bridged))


def segment_defer_speed_bound(ctx: Context, answer: Answer):
    """As segment_defer; a pair across a gap agrees if reachable at bounded speed."""
    bridged = ctx.timeline.bridge_at(answer.stamp)
    if bridged is None:
        if ctx.timeline.bridge_seen_within_window(answer.stamp):
            return DEFER, "deferred_inside_dropout"
        return RELEASE, "no_bridged_pose"

    def pair_test(anchor):
        if ctx.timeline.continuous_between(anchor.stamp, answer.stamp):
            return _agrees_by_odometry(ctx, anchor, answer, bridged)
        return _agrees_by_speed_bound(ctx, anchor, answer)

    return _pair_confirmation(ctx, answer, bridged, pair_test)


VARIANTS: dict[str, tuple[str, Callable]] = {
    "window_waiver": ("runtime baseline", window_waiver),
    "no_waiver": ("waiver removed", no_waiver),
    "segment": ("gapless pair", segment),
    "segment_bridge_guard": (
        "gapless pair + withhold inside dropout",
        segment_bridge_guard,
    ),
    "segment_defer": ("gapless pair + defer query inside dropout", segment_defer),
    "segment_defer_speed_bound": (
        "segment_defer + speed-bound pair across gaps",
        segment_defer_speed_bound,
    ),
}


# --- simulation and rubric ---------------------------------------------------


def simulate(variant: Callable, fixture: Fixture, params: Params) -> dict:
    ctx = Context(
        params, Timeline(fixture, params), deque(maxlen=params.anchor_history)
    )
    attempts = 0
    trace = []
    reset = None
    for answer in fixture.answers:
        if attempts >= params.max_attempts:
            trace.append((answer.stamp, "exhausted"))
            break
        decision, reason = variant(ctx, answer)
        if decision == DEFER:
            trace.append((answer.stamp, reason))
            continue
        attempts += 1
        if decision == RELEASE and answer.gated:
            reset = answer
            trace.append((answer.stamp, f"reset:{reason}"))
            break
        trace.append((answer.stamp, reason if decision == WITHHOLD else "weak"))

    if reset is None:
        outcome = "no_reset"
    else:
        outcome = "correct_reset" if reset.correct else "wrong_reset"
    passed = outcome != "wrong_reset" and (
        outcome == "correct_reset" or not fixture.recovery_required
    )
    return {
        "pass": passed,
        "outcome": outcome,
        "time_to_recover_sec": (
            round(reset.stamp - fixture.reference_stamp, 1)
            if outcome == "correct_reset"
            else None
        ),
        "attempts": attempts,
        "trace": [f"{stamp:g}:{event}" for stamp, event in trace],
    }


def run(params: Params | None = None) -> dict:
    params = params or Params()
    fixtures = build_fixtures()
    variants = {}
    for name, (design, function) in VARIANTS.items():
        outcomes = {f.name: simulate(function, f, params) for f in fixtures}
        passed = sum(o["pass"] for o in outcomes.values())
        variants[name] = {
            "design": design,
            "passed": passed,
            "benchmark_score": round(100.0 * passed / len(fixtures), 1),
            "wrong_resets": sum(
                o["outcome"] == "wrong_reset" for o in outcomes.values()
            ),
            "fixtures": outcomes,
        }
    return {
        "schema_version": 1,
        "problem": "odometry_dropout_confirmation",
        "evidence": "synthetic fixtures only; no public replay",
        "fixtures": {
            f.name: {
                "description": f.description,
                "recovery_required": f.recovery_required,
            }
            for f in fixtures
        },
        "variants": variants,
    }


def markdown(results: dict) -> str:
    variants = results["variants"]
    fixtures = list(results["fixtures"])
    lines = [
        "| Variant | Design | Passed | Wrong resets | Score |",
        "|---|---|---:|---:|---:|",
    ]
    for name, v in variants.items():
        lines.append(
            f"| `{name}` | {v['design']} | {v['passed']}/{len(fixtures)} "
            f"| {v['wrong_resets']} | {v['benchmark_score']} |"
        )
    lines += ["", "| Fixture | " + " | ".join(f"`{n}`" for n in variants) + " |"]
    lines.append("|---" * (len(variants) + 1) + "|")
    for fixture in fixtures:
        cells = []
        for v in variants.values():
            o = v["fixtures"][fixture]
            mark = "pass" if o["pass"] else "**FAIL**"
            if o["outcome"] == "correct_reset":
                detail = f"{o['time_to_recover_sec']:g} s"
            else:
                detail = o["outcome"].replace("_", " ")
            cells.append(f"{mark}: {detail}")
        lines.append(f"| `{fixture}` | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", type=Path, help="write the results to this file")
    args = parser.parse_args()
    results = run()
    if args.json:
        args.json.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(markdown(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
