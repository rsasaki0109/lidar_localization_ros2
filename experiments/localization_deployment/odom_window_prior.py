"""Track the search prior through a discontinuity in relative odometry."""

import math
from dataclasses import dataclass

ODOM_SPEED_LIMIT_MPS = 30.0
ODOM_JUMP_TOLERANCE_M = 10.0


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


class OdometryWindowPrior:
    def __init__(self, initial_prior: PlanarPose):
        self.map_anchor = initial_prior
        self.odom_anchor = PlanarPose(0.0, 0.0, 0.0)
        self.previous_odometry = None
        self.last_match = None
        self.restart_stamp = None
        self.recovering = False

    def observe_odometry(self, stamp_ns: int, pose: PlanarPose) -> bool:
        previous = self.previous_odometry
        if previous is not None and stamp_ns <= previous[0]:
            return False
        self.previous_odometry = (stamp_ns, pose)
        if previous is None:
            return False
        seconds = (stamp_ns - previous[0]) * 1e-9
        distance = math.hypot(pose.x - previous[1].x, pose.y - previous[1].y)
        if distance <= ODOM_JUMP_TOLERANCE_M + ODOM_SPEED_LIMIT_MPS * seconds:
            return False
        self.restart_stamp = stamp_ns
        self.recovering = True
        self.odom_anchor = pose
        self.map_anchor = self.last_match[1] if self.last_match is not None else None
        return True

    def accept_map_match(
        self, stamp_ns: int, pose: PlanarPose, odometry: PlanarPose
    ) -> None:
        if self.last_match is not None and stamp_ns <= self.last_match[0]:
            return
        self.last_match = (stamp_ns, pose)
        if self.restart_stamp is not None and stamp_ns >= self.restart_stamp:
            self.map_anchor = pose
            self.odom_anchor = odometry
            self.recovering = False

    def center(self, odometry: PlanarPose):
        if self.map_anchor is None:
            return None
        yaw = self.map_anchor.yaw - self.odom_anchor.yaw
        c, s = math.cos(yaw), math.sin(yaw)
        dx = odometry.x - self.odom_anchor.x
        dy = odometry.y - self.odom_anchor.y
        return (
            self.map_anchor.x + c * dx - s * dy,
            self.map_anchor.y + s * dx + c * dy,
        )
