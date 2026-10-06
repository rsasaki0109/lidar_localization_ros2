#!/usr/bin/env python3
"""ROS-free policy and persistence helpers for the quickstart workflow."""

from __future__ import annotations

import contextlib
import hashlib
import json
import math
import os
import tempfile
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass, replace
from pathlib import Path

POSE_STATE_SCHEMA = 1

STATE_WAITING_FOR_SCAN = "waiting_for_scan"
STATE_QUERYING_GLOBAL = "querying_global"
STATE_VERIFYING = "verifying"
STATE_ACTIVE = "active"
STATE_NEEDS_OPERATOR = "needs_operator"

ACTION_WAIT = "wait"
ACTION_PUBLISH_SAVED = "publish_saved"
ACTION_QUERY_GLOBAL = "query_global"
ACTION_PUBLISH_GLOBAL = "publish_global"
ACTION_ACTIVE = "active"
ACTION_NEEDS_OPERATOR = "needs_operator"


@dataclass(frozen=True)
class MapIdentity:
    size_bytes: int
    sha256: str


@dataclass(frozen=True)
class StoredPose:
    schema_version: int
    map_identity: MapIdentity
    frame_id: str
    stamp_sec: float
    saved_at_sec: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    covariance: tuple[float, ...]


@dataclass(frozen=True)
class PoseLoadResult:
    pose: StoredPose | None
    reason: str


def compute_map_identity(path: Path, chunk_size: int = 4 * 1024 * 1024) -> MapIdentity:
    """Return a content identity; paths and mtimes are deliberately not trusted."""
    resolved = path.expanduser().resolve(strict=True)
    digest = hashlib.sha256()
    size = 0
    with resolved.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            size += len(chunk)
            digest.update(chunk)
    return MapIdentity(size_bytes=size, sha256=digest.hexdigest())


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate_stored_pose(pose: StoredPose) -> str | None:
    if pose.schema_version != POSE_STATE_SCHEMA:
        return "unsupported_schema"
    if not pose.frame_id:
        return "missing_frame"
    if not _finite(
        (*pose.position, *pose.orientation, pose.stamp_sec, pose.saved_at_sec)
    ):
        return "nonfinite_pose"
    norm = math.sqrt(sum(value * value for value in pose.orientation))
    if norm < 0.5 or norm > 1.5:
        return "invalid_quaternion"
    if len(pose.covariance) != 36 or not _finite(pose.covariance):
        return "invalid_covariance"
    if len(pose.map_identity.sha256) != 64 or pose.map_identity.size_bytes < 0:
        return "invalid_map_identity"
    return None


def saved_pose_score_acceptable(
    converged: bool, fitness: float, threshold: float
) -> bool:
    return (
        bool(converged)
        and math.isfinite(float(fitness))
        and math.isfinite(float(threshold))
        and float(fitness) <= float(threshold)
    )


def global_registration_scoring_acceptable(
    registration_scoring_enabled: object, required: bool
) -> bool:
    """Fail closed when automatic global initialization requires 3D scoring."""
    return not required or registration_scoring_enabled is True


def save_stored_pose(path: Path, pose: StoredPose) -> None:
    error = validate_stored_pose(pose)
    if error is not None:
        raise ValueError(error)
    target = path.expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(pose)
    fd, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    except BaseException:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(temporary)
        raise


def _stored_pose_from_dict(raw: dict) -> StoredPose:
    identity = raw["map_identity"]
    return StoredPose(
        schema_version=int(raw["schema_version"]),
        map_identity=MapIdentity(
            size_bytes=int(identity["size_bytes"]), sha256=str(identity["sha256"])
        ),
        frame_id=str(raw["frame_id"]),
        stamp_sec=float(raw["stamp_sec"]),
        saved_at_sec=float(raw["saved_at_sec"]),
        position=tuple(float(value) for value in raw["position"]),
        orientation=tuple(float(value) for value in raw["orientation"]),
        covariance=tuple(float(value) for value in raw["covariance"]),
    )


def load_stored_pose(
    path: Path,
    expected_map: MapIdentity,
    expected_frame: str,
    now_sec: float,
    max_age_sec: float = 0.0,
) -> PoseLoadResult:
    source = path.expanduser()
    if not source.exists():
        return PoseLoadResult(None, "not_found")
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
        pose = _stored_pose_from_dict(raw)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return PoseLoadResult(None, "invalid_file")
    error = validate_stored_pose(pose)
    if error is not None:
        return PoseLoadResult(None, error)
    if pose.map_identity != expected_map:
        return PoseLoadResult(None, "map_mismatch")
    if pose.frame_id != expected_frame:
        return PoseLoadResult(None, "frame_mismatch")
    if max_age_sec > 0.0 and now_sec - pose.saved_at_sec > max_age_sec:
        return PoseLoadResult(None, "expired")
    return PoseLoadResult(pose, "ok")


def select_discovered_topic(
    typed_topics: Sequence[tuple[str, str]], message_type: str, preferred: str
) -> tuple[str, str]:
    """Choose an unambiguous live topic, otherwise retain the profile default."""
    candidates = sorted(
        {name for name, type_name in typed_topics if type_name == message_type}
    )
    if preferred in candidates:
        return preferred, "preferred_live"
    if len(candidates) == 1:
        return candidates[0], "single_detected"
    if not candidates:
        return preferred, "not_detected"
    return preferred, "ambiguous"


def select_odometry_frame(
    tf_edges: Iterable[tuple[str, str]], odom_frame: str, base_frame: str | None
) -> tuple[str, bool]:
    """Choose the base frame and whether a live odom -> base TF can seed matching.

    tf_edges are (parent, child) pairs seen on /tf and /tf_static. Without an
    explicit base frame, a single child of odom (e.g. RKO-LIO's odom -> livox_frame)
    becomes the base; otherwise the default base_link is kept.
    """
    children: dict[str, set[str]] = {}
    for parent, child in tf_edges:
        children.setdefault(parent, set()).add(child)
    below_odom: set[str] = set()
    stack = [odom_frame]
    while stack:
        for child in children.get(stack.pop(), ()):
            if child not in below_odom:
                below_odom.add(child)
                stack.append(child)
    candidate = base_frame or "base_link"
    if candidate in below_odom:
        return candidate, True
    direct = children.get(odom_frame, set())
    if base_frame is None and len(direct) == 1:
        return next(iter(direct)), True
    return candidate, False


def lidar_tf_needed(
    tf_edges: Iterable[tuple[str, str]], base_frame: str, lidar_frame: str
) -> bool:
    """Publish a static base -> LiDAR TF only when nothing links the frames yet.

    A second publisher for a TF the robot already provides would make it flip
    between two transforms.
    """
    if lidar_frame == base_frame:
        return False
    neighbours: dict[str, set[str]] = {}
    for parent, child in tf_edges:
        neighbours.setdefault(parent, set()).add(child)
        neighbours.setdefault(child, set()).add(parent)
    seen = {base_frame}
    stack = [base_frame]
    while stack:
        for frame in neighbours.get(stack.pop(), ()):
            if frame == lidar_frame:
                return False
            if frame not in seen:
                seen.add(frame)
                stack.append(frame)
    return True


def detect_sim_time(typed_topics: Sequence[tuple[str, str]]) -> bool:
    """A live /clock publisher means a bag replay or simulator drives ROS time."""
    return ("/clock", "rosgraph_msgs/msg/Clock") in typed_topics


@dataclass(frozen=True)
class StartupParams:
    min_candidate_score: float = 0.6
    min_score_margin: float = 0.05
    max_candidate_age_sec: float = 30.0
    query_timeout_sec: float = 30.0
    verification_timeout_sec: float = 8.0
    verification_fitness_threshold: float = 1.5
    verification_samples: int = 3
    max_global_attempts: int = 6
    global_consensus_samples: int = 2
    global_consensus_translation_m: float = 2.0
    global_consensus_yaw_deg: float = 20.0
    # With odometry, a fix is confirmed by any of this many earlier fixes moved to
    # the new scan time, so one wrong fix does not discard an earlier good one.
    global_consensus_history: int = 5
    # A fix's heading error (G2 searches in 5 degree steps) grows into a position
    # error along the distance travelled, so widen the match by this per metre.
    global_consensus_translation_per_odom_m: float = 0.05
    # An answer from a place at least this far (by odometry) from the previous
    # answer is new evidence, so it gives its attempt back; a stationary robot
    # still stops after max_global_attempts. max_global_queries bounds the total.
    global_attempt_refund_travel_m: float = 5.0
    max_global_queries: int = 30
    # With odometry, a retry waits for a view the last answer did not have: this
    # much travel or turn, or this long. Without it a robot that is still (or
    # standing up, like a Unitree Go2) spends every attempt on the same view in
    # about a second.
    global_retry_new_view_travel_m: float = 0.5
    global_retry_new_view_turn_deg: float = 15.0
    global_retry_max_wait_sec: float = 2.0
    # When the top G2 candidate's NDT fitness is at or below this threshold, trust
    # registration over occupancy score margin / distinct-scan consensus. Disabled
    # by default (inf). Route-crop quickstart sets ~0.5 for mapping-run seeds.
    registration_fitness_high_confidence_threshold: float = 1.0e9
    # Reject a top G2 candidate whose NDT fitness is not clearly better than the
    # best candidate at another place (an aliased area scores many similar,
    # mediocre poses). Candidates within the separation radius may converge onto
    # the same pose, so they are not alternatives. A ratio of 0 disables the gate.
    max_registration_fitness_ratio: float = 0.5
    registration_alternative_min_separation_m: float = 5.0


def validate_startup_params(params: StartupParams) -> str | None:
    finite_values = (
        params.min_candidate_score,
        params.min_score_margin,
        params.max_candidate_age_sec,
        params.query_timeout_sec,
        params.verification_timeout_sec,
        params.verification_fitness_threshold,
        params.global_consensus_translation_m,
        params.global_consensus_yaw_deg,
        params.global_consensus_translation_per_odom_m,
        params.global_attempt_refund_travel_m,
        params.global_retry_new_view_travel_m,
        params.global_retry_new_view_turn_deg,
        params.global_retry_max_wait_sec,
        params.registration_fitness_high_confidence_threshold,
        params.max_registration_fitness_ratio,
        params.registration_alternative_min_separation_m,
    )
    if not _finite(finite_values):
        return "startup thresholds must be finite"
    if params.registration_fitness_high_confidence_threshold < 0.0:
        return "registration_fitness_high_confidence_threshold must be non-negative"
    if params.max_registration_fitness_ratio < 0.0:
        return "max_registration_fitness_ratio must be non-negative"
    if params.registration_alternative_min_separation_m < 0.0:
        return "registration_alternative_min_separation_m must be non-negative"
    if not 0.0 <= params.min_candidate_score <= 1.0:
        return "min_candidate_score must be between 0 and 1"
    if params.min_score_margin < 0.0:
        return "min_score_margin must be non-negative"
    if params.max_candidate_age_sec <= 0.0:
        return "max_candidate_age_sec must be positive"
    if params.query_timeout_sec <= 0.0 or params.verification_timeout_sec <= 0.0:
        return "startup timeouts must be positive"
    if params.verification_fitness_threshold < 0.0:
        return "verification_fitness_threshold must be non-negative"
    if params.verification_samples < 1 or params.max_global_attempts < 1:
        return "verification samples and global attempts must be positive"
    if params.global_consensus_samples < 1:
        return "global_consensus_samples must be positive"
    if params.global_consensus_translation_per_odom_m < 0.0:
        return "global_consensus_translation_per_odom_m must be non-negative"
    if params.global_attempt_refund_travel_m < 0.0:
        return "global_attempt_refund_travel_m must be non-negative"
    if (
        params.global_retry_new_view_travel_m < 0.0
        or params.global_retry_new_view_turn_deg < 0.0
        or params.global_retry_max_wait_sec < 0.0
    ):
        return "global retry new-view limits must be non-negative"
    if params.max_global_queries < params.max_global_attempts:
        return "max_global_queries must be at least max_global_attempts"
    if params.global_consensus_history < 1:
        return "global_consensus_history must be positive"
    if params.global_consensus_translation_m < 0.0:
        return "global_consensus_translation_m must be non-negative"
    if not 0.0 <= params.global_consensus_yaw_deg <= 180.0:
        return "global_consensus_yaw_deg must be between 0 and 180"
    return None


@dataclass(frozen=True)
class OdomAnchor:
    """map -> odom implied by one G2 fix, so it can be moved to later scan times."""

    map_from_odom: tuple[float, float, float]
    odom_pose: tuple[float, float, float]
    scan_stamp_sec: float
    # Whether the fix passed the score, margin, and distinctiveness gates itself.
    gated: bool


@dataclass(frozen=True)
class StartupState:
    name: str = STATE_WAITING_FOR_SCAN
    source: str = ""
    deadline_sec: float | None = None
    global_attempts: int = 0
    global_queries: int = 0
    last_answer_odom_pose: tuple[float, float, float] | None = None
    confirmation_samples: int = 0
    saved_attempted: bool = False
    consensus_samples: int = 0
    consensus_pose: tuple[float, float, float] | None = None
    consensus_scan_stamp_sec: float | None = None
    # Earlier fixes that had odometry, newest last.
    consensus_odom_anchors: tuple[OdomAnchor, ...] = ()


@dataclass(frozen=True)
class StartupObservation:
    now_sec: float
    scan_ready: bool
    saved_pose_available: bool
    global_available: bool
    preconfigured_pose_available: bool = False
    query_in_flight: bool = False
    query_candidate_scores: tuple[float, ...] | None = None
    query_candidate_age_sec: float | None = None
    query_top_pose: tuple[float, float, float] | None = None
    query_scan_stamp_sec: float | None = None
    query_top_registration_fitness: float | None = None
    query_alternative_registration_fitness: float | None = None
    # odom -> base at this query's scan; None when odometry is unavailable.
    query_odom_pose: tuple[float, float, float] | None = None
    diagnostic_fresh: bool = False
    tracking_good: bool = False
    fitness: float | None = None


@dataclass(frozen=True)
class StartupDecision:
    action: str
    reason: str
    state: StartupState
    candidate_index: int = 0


def _operator(state: StartupState, reason: str) -> StartupDecision:
    return StartupDecision(
        ACTION_NEEDS_OPERATOR,
        reason,
        replace(state, name=STATE_NEEDS_OPERATOR, deadline_sec=None),
    )


def _query(params: StartupParams, state: StartupState, now_sec: float, reason: str):
    if state.global_attempts >= params.max_global_attempts:
        return _operator(state, "global_attempts_exhausted")
    if state.global_queries >= params.max_global_queries:
        return _operator(state, "global_queries_exhausted")
    next_state = replace(
        state,
        name=STATE_QUERYING_GLOBAL,
        source="global",
        deadline_sec=now_sec + params.query_timeout_sec,
        global_attempts=state.global_attempts + 1,
        global_queries=state.global_queries + 1,
        confirmation_samples=0,
    )
    return StartupDecision(ACTION_QUERY_GLOBAL, reason, next_state)


def _angle_error_rad(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def _wrap_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def retry_sees_new_view(
    params: StartupParams,
    waited_sec: float,
    motion: tuple[float, float, float] | None,
) -> bool:
    """Whether another global query would see something the last one did not.

    motion is the planar odometry motion since the last answered scan, or None
    without odometry (then every retry goes ahead, as before).
    """
    if motion is None or waited_sec >= params.global_retry_max_wait_sec:
        return True
    return (
        math.hypot(motion[0], motion[1]) >= params.global_retry_new_view_travel_m
        or abs(math.degrees(motion[2])) >= params.global_retry_new_view_turn_deg
    )


def planar_motion_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Motion from planar pose ``start`` to ``end``, expressed in the start frame."""
    cos_yaw = math.cos(start[2])
    sin_yaw = math.sin(start[2])
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        _wrap_angle_rad(end[2] - start[2]),
    )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Quaternion (x, y, z, w) of Rz(yaw) Ry(pitch) Rx(roll)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def apply_planar_motion(
    pose: tuple[float, float, float], motion: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Move planar ``pose`` by ``motion`` expressed in the pose's own frame."""
    cos_yaw = math.cos(pose[2])
    sin_yaw = math.sin(pose[2])
    return (
        pose[0] + cos_yaw * motion[0] - sin_yaw * motion[1],
        pose[1] + sin_yaw * motion[0] + cos_yaw * motion[1],
        _wrap_angle_rad(pose[2] + motion[2]),
    )


def registration_high_confidence(
    params: StartupParams,
    fitness: float | None,
) -> bool:
    """True when NDT fitness alone is strong enough to skip BBS-style ambiguity gates."""
    threshold = params.registration_fitness_high_confidence_threshold
    if not math.isfinite(threshold) or threshold > 1.0e6:
        return False
    if fitness is None or not math.isfinite(fitness):
        return False
    return float(fitness) <= threshold


def alternative_registration_fitness(
    candidates: Sequence[dict], min_separation_m: float, index: int = 0
) -> float | None:
    """Best finite NDT fitness among candidates away from candidate ``index``."""
    if index >= len(candidates):
        return None
    reference = candidates[index]
    best = None
    for other_index, candidate in enumerate(candidates):
        if other_index == index:
            continue
        fitness = candidate.get("registration_fitness")
        if fitness is None or not math.isfinite(float(fitness)):
            continue
        separation = math.hypot(
            float(candidate["x"]) - float(reference["x"]),
            float(candidate["y"]) - float(reference["y"]),
        )
        if separation < min_separation_m:
            continue
        if best is None or float(fitness) < best:
            best = float(fitness)
    return best


def registration_distinctiveness_judged(
    params: StartupParams,
    top_fitness: float | None,
    alternative_fitness: float | None,
) -> bool:
    """Whether NDT fitness can tell the top candidate from the best one elsewhere.

    The 2D score margin is then not needed: in a small room the BBS scores of
    many candidates saturate near 1, and the top two are often the same place
    in two headings, so the margin rejected fixes whose NDT fitness was 10-20
    times better than anywhere else (Unitree Go2, JEPLO capture room).
    """
    return (
        params.max_registration_fitness_ratio > 0.0
        and top_fitness is not None
        and alternative_fitness is not None
        and math.isfinite(top_fitness)
        and math.isfinite(alternative_fitness)
    )


def registration_ambiguous(
    params: StartupParams,
    top_fitness: float | None,
    alternative_fitness: float | None,
) -> bool:
    """True when the top candidate does not register clearly better than elsewhere."""
    ratio = params.max_registration_fitness_ratio
    if ratio <= 0.0 or top_fitness is None or alternative_fitness is None:
        return False
    if not math.isfinite(top_fitness) or not math.isfinite(alternative_fitness):
        return False
    return top_fitness > ratio * alternative_fitness


def _consensus_consistent(
    params: StartupParams,
    pose: tuple[float, float, float],
    reference: tuple[float, float, float],
    travelled_m: float = 0.0,
) -> bool:
    return (
        math.hypot(pose[0] - reference[0], pose[1] - reference[1])
        <= params.global_consensus_translation_m
        + params.global_consensus_translation_per_odom_m * travelled_m
        and math.degrees(_angle_error_rad(pose[2], reference[2]))
        <= params.global_consensus_yaw_deg
    )


def _odom_fix_usable(params: StartupParams, obs: StartupObservation) -> bool:
    return (
        bool(obs.query_candidate_scores)
        and params.global_consensus_samples > 1
        and obs.query_odom_pose is not None
        and obs.query_top_pose is not None
        and obs.query_scan_stamp_sec is not None
        and obs.query_candidate_age_sec is not None
        and math.isfinite(obs.query_candidate_age_sec)
        and obs.query_candidate_age_sec <= params.max_candidate_age_sec
    )


def _odom_agreements(
    params: StartupParams,
    state: StartupState,
    obs: StartupObservation,
    gated_only: bool,
) -> int:
    """Earlier fixes (from other scans) that, moved by odometry, match this fix."""
    return sum(
        anchor.scan_stamp_sec < obs.query_scan_stamp_sec - 1.0e-9
        and (anchor.gated or not gated_only)
        and _consensus_consistent(
            params,
            obs.query_top_pose,
            apply_planar_motion(anchor.map_from_odom, obs.query_odom_pose),
            math.hypot(
                obs.query_odom_pose[0] - anchor.odom_pose[0],
                obs.query_odom_pose[1] - anchor.odom_pose[1],
            ),
        )
        for anchor in state.consensus_odom_anchors
    )


def _remember_odom_fix(
    params: StartupParams, state: StartupState, obs: StartupObservation, gated: bool
) -> StartupState:
    if any(
        anchor.scan_stamp_sec >= obs.query_scan_stamp_sec - 1.0e-9
        for anchor in state.consensus_odom_anchors
    ):
        return state
    anchor = OdomAnchor(
        map_from_odom=apply_planar_motion(
            obs.query_top_pose,
            planar_motion_between(obs.query_odom_pose, (0.0, 0.0, 0.0)),
        ),
        odom_pose=obs.query_odom_pose,
        scan_stamp_sec=obs.query_scan_stamp_sec,
        gated=gated,
    )
    return replace(
        state,
        consensus_odom_anchors=(*state.consensus_odom_anchors, anchor)[
            -params.global_consensus_history :
        ],
    )


def _refund_attempt_after_travel(
    params: StartupParams, state: StartupState, obs: StartupObservation
) -> StartupState:
    odom = obs.query_odom_pose
    if odom is None:
        return state
    previous = state.last_answer_odom_pose
    refund = (
        previous is not None
        and math.hypot(odom[0] - previous[0], odom[1] - previous[1])
        >= params.global_attempt_refund_travel_m
    )
    return replace(
        state,
        global_attempts=max(0, state.global_attempts - 1)
        if refund
        else state.global_attempts,
        last_answer_odom_pose=odom,
    )


def _publish_confirmed_fix(
    params: StartupParams, state: StartupState, obs: StartupObservation
) -> StartupDecision:
    verifying = replace(
        state,
        name=STATE_VERIFYING,
        source="global",
        deadline_sec=obs.now_sec + params.verification_timeout_sec,
        confirmation_samples=0,
    )
    return StartupDecision(
        ACTION_PUBLISH_GLOBAL, "global_candidate_accepted", verifying
    )


def decide_startup(
    params: StartupParams, state: StartupState, obs: StartupObservation
) -> StartupDecision:
    if state.name == STATE_ACTIVE:
        return StartupDecision(ACTION_ACTIVE, "tracking", state)
    if state.name == STATE_NEEDS_OPERATOR:
        confirmations = state.confirmation_samples
        if obs.diagnostic_fresh:
            good = (
                obs.tracking_good
                and obs.fitness is not None
                and math.isfinite(obs.fitness)
                and obs.fitness <= params.verification_fitness_threshold
            )
            confirmations = confirmations + 1 if good else 0
            state = replace(state, confirmation_samples=confirmations)
            if confirmations >= params.verification_samples:
                active = replace(
                    state, name=STATE_ACTIVE, source="manual", deadline_sec=None
                )
                return StartupDecision(ACTION_ACTIVE, "manual_pose_verified", active)
        return StartupDecision(ACTION_NEEDS_OPERATOR, "manual_pose_required", state)

    if state.name == STATE_WAITING_FOR_SCAN:
        if not obs.scan_ready:
            return StartupDecision(ACTION_WAIT, "waiting_for_scan", state)
        if obs.preconfigured_pose_available:
            next_state = replace(
                state,
                name=STATE_VERIFYING,
                source="explicit",
                deadline_sec=obs.now_sec + params.verification_timeout_sec,
                confirmation_samples=0,
            )
            return StartupDecision(ACTION_WAIT, "verifying_explicit_pose", next_state)
        if obs.saved_pose_available and not state.saved_attempted:
            next_state = replace(
                state,
                name=STATE_VERIFYING,
                source="saved",
                deadline_sec=obs.now_sec + params.verification_timeout_sec,
                saved_attempted=True,
                confirmation_samples=0,
            )
            return StartupDecision(
                ACTION_PUBLISH_SAVED, "saved_pose_available", next_state
            )
        if obs.global_available:
            return _query(params, state, obs.now_sec, "query_global_startup")
        return _operator(state, "no_safe_automatic_source")

    if state.name == STATE_QUERYING_GLOBAL:
        if obs.query_candidate_scores is None:
            if state.deadline_sec is not None and obs.now_sec > state.deadline_sec:
                if obs.query_in_flight:
                    return _operator(state, "global_query_timeout")
                return _query(params, state, obs.now_sec, "query_timeout_retry")
            return StartupDecision(ACTION_WAIT, "awaiting_global_query", state)
        scores = obs.query_candidate_scores
        state = _refund_attempt_after_travel(params, state, obs)
        odom_fix = _odom_fix_usable(params, obs)
        if (
            odom_fix
            and _odom_agreements(params, state, obs, gated_only=True) + 1
            >= params.global_consensus_samples
        ):
            # Matching where an earlier gated fix has moved to is stronger evidence
            # than this scan's own score or distinctiveness.
            return _publish_confirmed_fix(params, state, obs)
        # A fix that fails the gates below may still confirm a later gated one.
        retry_state = (
            _remember_odom_fix(params, state, obs, gated=False) if odom_fix else state
        )
        if (
            not scores
            or not math.isfinite(scores[0])
            or scores[0] < params.min_candidate_score
        ):
            return _query(params, retry_state, obs.now_sec, "weak_candidate_retry")
        high_confidence = registration_high_confidence(
            params, obs.query_top_registration_fitness
        )
        if (
            not high_confidence
            and not registration_distinctiveness_judged(
                params,
                obs.query_top_registration_fitness,
                obs.query_alternative_registration_fitness,
            )
            and len(scores) > 1
            and math.isfinite(scores[1])
            and scores[0] - scores[1] < params.min_score_margin
        ):
            return _query(params, retry_state, obs.now_sec, "ambiguous_candidate_retry")
        if not high_confidence and registration_ambiguous(
            params,
            obs.query_top_registration_fitness,
            obs.query_alternative_registration_fitness,
        ):
            return _query(
                params, retry_state, obs.now_sec, "ambiguous_registration_retry"
            )
        if (
            obs.query_candidate_age_sec is None
            or not math.isfinite(obs.query_candidate_age_sec)
            or obs.query_candidate_age_sec > params.max_candidate_age_sec
        ):
            return _query(params, state, obs.now_sec, "stale_candidate_retry")
        if obs.query_top_pose is None or obs.query_scan_stamp_sec is None:
            return _query(params, state, obs.now_sec, "candidate_pose_or_stamp_missing")
        if high_confidence:
            next_state = replace(
                state,
                name=STATE_VERIFYING,
                source="global",
                deadline_sec=obs.now_sec + params.verification_timeout_sec,
                confirmation_samples=0,
            )
            return StartupDecision(
                ACTION_PUBLISH_GLOBAL,
                "global_registration_high_confidence",
                next_state,
            )
        if odom_fix:
            agreeing = _odom_agreements(params, state, obs, gated_only=False)
            remembered = _remember_odom_fix(params, state, obs, gated=True)
            if agreeing + 1 >= params.global_consensus_samples:
                return _publish_confirmed_fix(params, remembered, obs)
            reason = (
                "global_consensus_mismatch_retry"
                if state.consensus_odom_anchors
                else "global_consensus_primed"
            )
            return _query(params, remembered, obs.now_sec, reason)
        if state.consensus_samples == 0 or state.consensus_pose is None:
            primed = replace(
                state,
                consensus_samples=1,
                consensus_pose=obs.query_top_pose,
                consensus_scan_stamp_sec=obs.query_scan_stamp_sec,
            )
            if params.global_consensus_samples > 1:
                return _query(params, primed, obs.now_sec, "global_consensus_primed")
        else:
            if (
                state.consensus_scan_stamp_sec is None
                or obs.query_scan_stamp_sec <= state.consensus_scan_stamp_sec + 1.0e-9
            ):
                return StartupDecision(ACTION_WAIT, "awaiting_fresh_global_scan", state)
            if not _consensus_consistent(
                params, obs.query_top_pose, state.consensus_pose
            ):
                restarted = replace(
                    state,
                    consensus_samples=1,
                    consensus_pose=obs.query_top_pose,
                    consensus_scan_stamp_sec=obs.query_scan_stamp_sec,
                )
                return _query(
                    params, restarted, obs.now_sec, "global_consensus_mismatch_retry"
                )
            agreed = replace(
                state,
                consensus_samples=state.consensus_samples + 1,
                consensus_pose=obs.query_top_pose,
                consensus_scan_stamp_sec=obs.query_scan_stamp_sec,
            )
            if agreed.consensus_samples < params.global_consensus_samples:
                return _query(params, agreed, obs.now_sec, "global_consensus_pending")
            state = agreed
        next_state = replace(
            state,
            name=STATE_VERIFYING,
            source="global",
            deadline_sec=obs.now_sec + params.verification_timeout_sec,
            confirmation_samples=0,
        )
        return StartupDecision(
            ACTION_PUBLISH_GLOBAL, "global_candidate_accepted", next_state
        )

    if state.name == STATE_VERIFYING:
        confirmations = state.confirmation_samples
        if obs.diagnostic_fresh:
            good = (
                obs.tracking_good
                and obs.fitness is not None
                and math.isfinite(obs.fitness)
                and obs.fitness <= params.verification_fitness_threshold
            )
            confirmations = confirmations + 1 if good else 0
            state = replace(state, confirmation_samples=confirmations)
            if confirmations >= params.verification_samples:
                active = replace(state, name=STATE_ACTIVE, deadline_sec=None)
                return StartupDecision(
                    ACTION_ACTIVE, f"{state.source}_pose_verified", active
                )
        if state.deadline_sec is not None and obs.now_sec > state.deadline_sec:
            if obs.global_available:
                return _query(
                    params, state, obs.now_sec, f"{state.source}_verification_failed"
                )
            return _operator(state, f"{state.source}_verification_failed")
        return StartupDecision(ACTION_WAIT, "verifying_pose", state)

    return _operator(state, "invalid_state")
