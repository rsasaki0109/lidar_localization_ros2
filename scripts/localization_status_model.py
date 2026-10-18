#!/usr/bin/env python3
"""Plain-language localization status from the startup and alignment topics.

Pure Python (no rclpy) so the wording and the debounce are unit tested without a
ROS graph. localization_status_node.py feeds it /startup_initialization/status
and /alignment_status and prints the lines it returns; it never changes
acceptance, startup or recovery behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PHASE_STARTING = "starting"
PHASE_NEEDS_POSE = "needs_pose"
PHASE_WAITING_FOR_MAP = "waiting_for_map"
PHASE_TRACKING = "tracking"
PHASE_DEGRADED = "degraded"
PHASE_LOST = "lost"
PHASE_NO_SCANS = "no_scans"

SEVERITY_INFO = "info"
SEVERITY_WARNING = "warning"
SEVERITY_ERROR = "error"

# RGBA for the RViz text marker.
PHASE_COLORS = {
    PHASE_STARTING: (0.35, 0.75, 1.0, 1.0),
    PHASE_WAITING_FOR_MAP: (0.35, 0.75, 1.0, 1.0),
    PHASE_NEEDS_POSE: (1.0, 0.35, 0.3, 1.0),
    PHASE_TRACKING: (0.3, 0.9, 0.4, 1.0),
    PHASE_DEGRADED: (1.0, 0.8, 0.2, 1.0),
    PHASE_LOST: (1.0, 0.35, 0.3, 1.0),
    PHASE_NO_SCANS: (1.0, 0.35, 0.3, 1.0),
}

TROUBLESHOOTING = "docs/troubleshooting.md"

# Categories of a scan that was matched and accepted.
_ACCEPTED_CATEGORIES = frozenset({"", "healthy", "overload"})
# Messages for a scan with nothing left to register (see troubleshooting.md).
_EMPTY_SCAN_MESSAGES = frozenset({"filtered_scan_empty", "filtered_scan_too_sparse"})

_STARTUP_LABELS = {
    "waiting_for_scan": "Waiting for the first scan",
    "querying_global": "Searching the map",
    "verifying": "Checking the pose",
}


def _number(values: dict, key: str) -> float:
    try:
        return float(values.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan


def _flag(values: dict, key: str) -> bool:
    return str(values.get(key, "")).strip().lower() == "true"


@dataclass(frozen=True)
class Status:
    """What localization is doing, for the terminal and for RViz."""

    phase: str
    category: str = ""
    line: str = ""
    label: str = ""
    severity: str = SEVERITY_INFO


def alignment_status(
    values: dict,
    manual_pose_step: str,
    automatic_recovery: bool = False,
    lost_after_sec: float = 5.0,
    since_good_sec: float = math.nan,
) -> Status:
    """Classify one /alignment_status sample into a user-facing status.

    A poor match outranks stale prediction in failure_category, so a kidnapped
    robot reads as bad_match for as long as scans keep being rejected; no
    accepted scan for ``lost_after_sec`` counts as lost whatever the category.
    Scans that are empty after filtering (a covered sensor) carry no
    accepted_gap_sec, so ``since_good_sec``, the caller's own count since the
    last good sample, stands in for it.
    """
    category = str(values.get("failure_category", "") or "").strip()
    message = str(values.get("message", "") or "").strip()
    reinit = _flag(values, "reinitialization_requested")
    gap = _number(values, "accepted_gap_sec")
    if not math.isfinite(gap) and category not in _ACCEPTED_CATEGORIES:
        gap = since_good_sec
    unmatched = math.isfinite(gap) and gap >= lost_after_sec
    if category == "missing_map":
        return Status(
            PHASE_WAITING_FOR_MAP,
            category,
            "Waiting for the map. Check that map_path points to a readable .pcd/.ply "
            f"file (see {TROUBLESHOOTING}, Map Not Visible in RViz).",
            "Waiting for the map",
            SEVERITY_WARNING,
        )
    if category == "missing_initial_pose":
        return Status(
            PHASE_NEEDS_POSE,
            category,
            f"No initial pose yet: {manual_pose_step}.",
            "Set pose: 2D Pose Estimate (P)",
            SEVERITY_WARNING,
        )
    if category == "stale_prediction" or reinit or unmatched:
        since = f" for {gap:.0f} s" if math.isfinite(gap) and gap >= 1.0 else ""
        recovery = (
            "Automatic recovery is searching the map; if it does not recover, "
            if automatic_recovery
            else "If it does not recover, "
        )
        return Status(
            PHASE_LOST,
            "stale_prediction",
            f"Tracking lost: no scan has matched the map{since}. "
            f"{recovery}{manual_pose_step}.",
            "Lost: set 2D Pose Estimate",
            SEVERITY_ERROR,
        )
    if message in _EMPTY_SCAN_MESSAGES:
        return Status(
            PHASE_DEGRADED,
            "empty_scan",
            "No usable scan points: the scan is empty after filtering. The sensor "
            "may be covered or blocked, or the scan range filters remove every point.",
            "Weak: no scan points",
            SEVERITY_WARNING,
        )
    if category == "weak_overlap":
        points = _number(values, "filtered_point_count")
        detail = (
            f"only {points:.0f} scan points left after filtering"
            if math.isfinite(points)
            else "too few scan points overlap the map"
        )
        return Status(
            PHASE_DEGRADED,
            category,
            f"Tracking is weak: {detail}. Check the sensor view, the scan range "
            "filters, and that the robot is inside the mapped area.",
            "Weak: few scan points",
            SEVERITY_WARNING,
        )
    if category == "bad_match":
        fitness = _number(values, "fitness_score")
        threshold = _number(values, "effective_score_threshold")
        detail = (
            f" (fitness {fitness:.2f} > {threshold:.2f})"
            if math.isfinite(fitness) and math.isfinite(threshold)
            else ""
        )
        return Status(
            PHASE_DEGRADED,
            category,
            f"Scans do not match the map well{detail}. If this lasts, "
            f"{manual_pose_step}; if it happens from the start, check the LiDAR "
            "transform and the map frame.",
            "Weak: poor match",
            SEVERITY_WARNING,
        )
    if category == "overload":
        seconds = _number(values, "alignment_time_sec")
        detail = f" ({seconds:.2f} s per scan)" if math.isfinite(seconds) else ""
        return Status(
            PHASE_DEGRADED,
            category,
            f"Alignment is falling behind the scans{detail}. Raise voxel_leaf_size, "
            "set ndt_num_threads to the free CPU cores, or stop other heavy processes.",
            "Weak: CPU overload",
            SEVERITY_WARNING,
        )
    return Status(PHASE_TRACKING, category or "healthy", "Tracking.", "Tracking")


def startup_status(payload: dict) -> Status | None:
    """The status while guarded startup is still running, else None."""
    state = str(payload.get("state", "") or "")
    if not state or state == "active":
        return None
    if state == "needs_operator":
        return Status(
            PHASE_NEEDS_POSE,
            state,
            label="Set pose: 2D Pose Estimate (P)",
            severity=SEVERITY_WARNING,
        )
    label = _STARTUP_LABELS.get(state, "Starting")
    attempts = payload.get("global_attempts")
    if state == "querying_global" and isinstance(attempts, int) and attempts > 0:
        label += f" (try {attempts})"
    return Status(PHASE_STARTING, state, label=label)


class StatusTracker:
    """Debounced status: report a change only once it has held for a while.

    Single rejected scans are normal, so a new phase is reported after it has
    held for ``debounce_sec``. Startup lines are printed by the startup node
    itself; this tracker only reports from the moment localization runs.
    """

    def __init__(
        self,
        manual_pose_step: str = "set 2D Pose Estimate in RViz",
        debounce_sec: float = 2.0,
        no_scan_timeout_sec: float = 5.0,
        automatic_recovery: bool = False,
        lost_after_sec: float = 5.0,
    ) -> None:
        self.manual_pose_step = manual_pose_step
        self.automatic_recovery = automatic_recovery
        self.lost_after_sec = lost_after_sec
        self.debounce_sec = debounce_sec
        self.no_scan_timeout_sec = no_scan_timeout_sec
        self.startup: Status | None = None
        self.startup_seen = False
        self.current: Status | None = None
        self.current_since: float | None = None
        self.pending: Status | None = None
        self.pending_since = 0.0
        self.last_alignment_time: float | None = None
        self.last_good_time: float | None = None
        self.running_since: float | None = None

    def update_startup(self, payload: dict, now: float) -> None:
        self.startup_seen = True
        self.startup = startup_status(payload)
        if self.startup is None and self.running_since is None:
            self.running_since = now

    def update_alignment(self, values: dict, now: float) -> list[Status]:
        self.last_alignment_time = now
        if self.running_since is None and not self.startup_seen:
            self.running_since = now
        if self.startup is not None:
            return []
        good_since = (
            self.last_good_time
            if self.last_good_time is not None
            else self.running_since
        )
        status = alignment_status(
            values,
            self.manual_pose_step,
            self.automatic_recovery,
            self.lost_after_sec,
            now - good_since if good_since is not None else math.nan,
        )
        if status.category in _ACCEPTED_CATEGORIES:
            self.last_good_time = now
        return self._propose(status, now)

    def tick(self, now: float) -> list[Status]:
        """Time-driven checks: no scans arriving while localization runs."""
        if self.startup is not None or self.running_since is None:
            return []
        last = (
            self.last_alignment_time
            if self.last_alignment_time is not None
            else self.running_since
        )
        if now - last < self.no_scan_timeout_sec:
            return self._propose(self.pending, now) if self.pending else []
        quiet = now - last
        status = Status(
            PHASE_NO_SCANS,
            "no_scans",
            f"No scan processed for {quiet:.0f} s. Check that the LiDAR driver or "
            "bag is running and publishing the cloud topic, and that /clock runs "
            "when use_sim_time is on.",
            "No scans arriving",
            SEVERITY_ERROR,
        )
        # Waiting is the cause; report it at once instead of debouncing.
        if self.current is None or self.current.phase != PHASE_NO_SCANS:
            return self._commit(status, now)
        return []

    def display(self) -> Status:
        """What RViz shows now: startup first, then the reported phase."""
        if self.startup is not None:
            return self.startup
        if self.current is not None:
            return self.current
        return Status(PHASE_STARTING, label="Starting")

    def _propose(self, status: Status | None, now: float) -> list[Status]:
        if status is None:
            return []
        if self.current is not None and _same(self.current, status):
            self.pending = None
            return []
        if self.pending is None or not _same(self.pending, status):
            self.pending = status
            self.pending_since = now
        else:
            # Keep the newest numbers in the line that will be printed.
            self.pending = status
        if self.current is None or now - self.pending_since >= self.debounce_sec:
            return self._commit(self.pending, now)
        return []

    def _commit(self, status: Status, now: float) -> list[Status]:
        previous = self.current
        previous_since = self.current_since
        self.current = status
        self.current_since = now
        self.pending = None
        if (
            status.phase == PHASE_TRACKING
            and previous is not None
            and previous.phase != PHASE_TRACKING
            and previous_since is not None
        ):
            status = Status(
                status.phase,
                status.category,
                f"Tracking again after {now - previous_since:.0f} s.",
                status.label,
                status.severity,
            )
            self.current = status
        return [status]


def _same(a: Status, b: Status) -> bool:
    return a.phase == b.phase and a.category == b.category
