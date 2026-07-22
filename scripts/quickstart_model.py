#!/usr/bin/env python3
"""ROS-free policy and persistence helpers for the quickstart workflow."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple


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
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    covariance: Tuple[float, ...]


@dataclass(frozen=True)
class PoseLoadResult:
    pose: Optional[StoredPose]
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


def validate_stored_pose(pose: StoredPose) -> Optional[str]:
    if pose.schema_version != POSE_STATE_SCHEMA:
        return "unsupported_schema"
    if not pose.frame_id:
        return "missing_frame"
    if not _finite((*pose.position, *pose.orientation, pose.stamp_sec, pose.saved_at_sec)):
        return "nonfinite_pose"
    norm = math.sqrt(sum(value * value for value in pose.orientation))
    if norm < 0.5 or norm > 1.5:
        return "invalid_quaternion"
    if len(pose.covariance) != 36 or not _finite(pose.covariance):
        return "invalid_covariance"
    if len(pose.map_identity.sha256) != 64 or pose.map_identity.size_bytes < 0:
        return "invalid_map_identity"
    return None


def saved_pose_score_acceptable(converged: bool, fitness: float, threshold: float) -> bool:
    return (
        bool(converged)
        and math.isfinite(float(fitness))
        and math.isfinite(float(threshold))
        and float(fitness) <= float(threshold)
    )


def global_registration_scoring_acceptable(
        registration_scoring_enabled: object, required: bool) -> bool:
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
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _stored_pose_from_dict(raw: dict) -> StoredPose:
    identity = raw["map_identity"]
    return StoredPose(
        schema_version=int(raw["schema_version"]),
        map_identity=MapIdentity(
            size_bytes=int(identity["size_bytes"]), sha256=str(identity["sha256"])),
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
    typed_topics: Sequence[Tuple[str, str]], message_type: str, preferred: str
) -> Tuple[str, str]:
    """Choose an unambiguous live topic, otherwise retain the profile default."""
    candidates = sorted({name for name, type_name in typed_topics if type_name == message_type})
    if preferred in candidates:
        return preferred, "preferred_live"
    if len(candidates) == 1:
        return candidates[0], "single_detected"
    if not candidates:
        return preferred, "not_detected"
    return preferred, "ambiguous"


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


def validate_startup_params(params: StartupParams) -> Optional[str]:
    finite_values = (
        params.min_candidate_score,
        params.min_score_margin,
        params.max_candidate_age_sec,
        params.query_timeout_sec,
        params.verification_timeout_sec,
        params.verification_fitness_threshold,
        params.global_consensus_translation_m,
        params.global_consensus_yaw_deg,
    )
    if not _finite(finite_values):
        return "startup thresholds must be finite"
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
    if params.global_consensus_translation_m < 0.0:
        return "global_consensus_translation_m must be non-negative"
    if not 0.0 <= params.global_consensus_yaw_deg <= 180.0:
        return "global_consensus_yaw_deg must be between 0 and 180"
    return None


@dataclass(frozen=True)
class StartupState:
    name: str = STATE_WAITING_FOR_SCAN
    source: str = ""
    deadline_sec: Optional[float] = None
    global_attempts: int = 0
    confirmation_samples: int = 0
    saved_attempted: bool = False
    consensus_samples: int = 0
    consensus_pose: Optional[Tuple[float, float, float]] = None
    consensus_scan_stamp_sec: Optional[float] = None


@dataclass(frozen=True)
class StartupObservation:
    now_sec: float
    scan_ready: bool
    saved_pose_available: bool
    global_available: bool
    preconfigured_pose_available: bool = False
    query_in_flight: bool = False
    query_candidate_scores: Optional[Tuple[float, ...]] = None
    query_candidate_age_sec: Optional[float] = None
    query_top_pose: Optional[Tuple[float, float, float]] = None
    query_scan_stamp_sec: Optional[float] = None
    diagnostic_fresh: bool = False
    tracking_good: bool = False
    fitness: Optional[float] = None


@dataclass(frozen=True)
class StartupDecision:
    action: str
    reason: str
    state: StartupState
    candidate_index: int = 0


def _operator(state: StartupState, reason: str) -> StartupDecision:
    return StartupDecision(
        ACTION_NEEDS_OPERATOR, reason,
        replace(state, name=STATE_NEEDS_OPERATOR, deadline_sec=None))


def _query(params: StartupParams, state: StartupState, now_sec: float, reason: str):
    if state.global_attempts >= params.max_global_attempts:
        return _operator(state, "global_attempts_exhausted")
    next_state = replace(
        state,
        name=STATE_QUERYING_GLOBAL,
        source="global",
        deadline_sec=now_sec + params.query_timeout_sec,
        global_attempts=state.global_attempts + 1,
        confirmation_samples=0,
    )
    return StartupDecision(ACTION_QUERY_GLOBAL, reason, next_state)


def _angle_error_rad(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


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
                    state, name=STATE_ACTIVE, source="manual", deadline_sec=None)
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
            return StartupDecision(ACTION_PUBLISH_SAVED, "saved_pose_available", next_state)
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
        if not scores or not math.isfinite(scores[0]) or scores[0] < params.min_candidate_score:
            return _query(params, state, obs.now_sec, "weak_candidate_retry")
        if (
            len(scores) > 1
            and math.isfinite(scores[1])
            and scores[0] - scores[1] < params.min_score_margin
        ):
            return _query(params, state, obs.now_sec, "ambiguous_candidate_retry")
        if (
            obs.query_candidate_age_sec is None
            or not math.isfinite(obs.query_candidate_age_sec)
            or obs.query_candidate_age_sec > params.max_candidate_age_sec
        ):
            return _query(params, state, obs.now_sec, "stale_candidate_retry")
        if obs.query_top_pose is None or obs.query_scan_stamp_sec is None:
            return _query(params, state, obs.now_sec, "candidate_pose_or_stamp_missing")
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
            dx = obs.query_top_pose[0] - state.consensus_pose[0]
            dy = obs.query_top_pose[1] - state.consensus_pose[1]
            yaw_error = _angle_error_rad(obs.query_top_pose[2], state.consensus_pose[2])
            consistent = (
                math.hypot(dx, dy) <= params.global_consensus_translation_m
                and math.degrees(yaw_error) <= params.global_consensus_yaw_deg
            )
            if not consistent:
                restarted = replace(
                    state,
                    consensus_samples=1,
                    consensus_pose=obs.query_top_pose,
                    consensus_scan_stamp_sec=obs.query_scan_stamp_sec,
                )
                return _query(
                    params, restarted, obs.now_sec, "global_consensus_mismatch_retry")
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
        return StartupDecision(ACTION_PUBLISH_GLOBAL, "global_candidate_accepted", next_state)

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
                return StartupDecision(ACTION_ACTIVE, f"{state.source}_pose_verified", active)
        if state.deadline_sec is not None and obs.now_sec > state.deadline_sec:
            if obs.global_available:
                return _query(params, state, obs.now_sec, f"{state.source}_verification_failed")
            return _operator(state, f"{state.source}_verification_failed")
        return StartupDecision(ACTION_WAIT, "verifying_pose", state)

    return _operator(state, "invalid_state")
