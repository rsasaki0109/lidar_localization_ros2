from dataclasses import dataclass
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence


OK = "OK"
WARN = "WARN"
FAIL = "FAIL"


@dataclass
class TopicStats:
    count: int = 0
    first_wall_time: Optional[float] = None
    last_wall_time: Optional[float] = None
    last_frame_id: str = ""
    last_stamp_sec: Optional[float] = None
    last_point_count: Optional[int] = None
    has_xyz_fields: Optional[bool] = None
    last_status_level: Optional[int] = None
    last_status_message: str = ""

    def mark(self, frame_id: str, stamp_sec: float, wall_time: float) -> None:
        if self.first_wall_time is None:
            self.first_wall_time = wall_time
        self.last_wall_time = wall_time
        self.last_frame_id = frame_id
        self.last_stamp_sec = stamp_sec
        self.count += 1

    def hz(self) -> Optional[float]:
        if self.count < 2 or self.first_wall_time is None or self.last_wall_time is None:
            return None
        elapsed = self.last_wall_time - self.first_wall_time
        if elapsed <= 0.0:
            return None
        return float(self.count - 1) / elapsed


@dataclass(frozen=True)
class BringupCheckConfig:
    cloud_topic: str = "/livox/points"
    imu_topic: str = "/livox/imu"
    pose_topic: str = "/localization/pose_with_covariance"
    alignment_status_topic: str = "/alignment_status"
    global_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    lidar_frame: str = "livox_frame"
    require_imu: bool = False
    require_map_odom_tf: bool = False
    require_localization_output: bool = False


@dataclass(frozen=True)
class CheckResult:
    level: str
    message: str


@dataclass(frozen=True)
class TfCheck:
    name: str
    available: bool


@dataclass(frozen=True)
class BringupSnapshot:
    cloud: TopicStats
    imu: TopicStats
    pose: TopicStats
    status: TopicStats
    tf_checks: Sequence[TfCheck]


def build_tf_checks(config: BringupCheckConfig, availability: Dict[str, bool]) -> List[TfCheck]:
    return [
        TfCheck(
            f"{config.base_frame} <- {config.lidar_frame}",
            availability.get(f"{config.base_frame} <- {config.lidar_frame}", False),
        ),
        TfCheck(
            f"{config.odom_frame} <- {config.base_frame}",
            availability.get(f"{config.odom_frame} <- {config.base_frame}", False),
        ),
        TfCheck(
            f"{config.global_frame} <- {config.odom_frame}",
            availability.get(f"{config.global_frame} <- {config.odom_frame}", False),
        ),
    ]


def evaluate_snapshot(
    config: BringupCheckConfig,
    snapshot: BringupSnapshot,
) -> List[CheckResult]:
    results: List[CheckResult] = []
    _evaluate_cloud(config, snapshot.cloud, results)
    _evaluate_imu(config, snapshot.imu, results)
    _evaluate_tf(config, snapshot.tf_checks, results)
    _evaluate_pose(config, snapshot.pose, results)
    _evaluate_status(config, snapshot.status, results)
    return results


def exit_code(results: Sequence[CheckResult]) -> int:
    return 1 if any(result.level == FAIL for result in results) else 0


def topic_summary(name: str, stats: TopicStats) -> str:
    hz = stats.hz()
    hz_text = "n/a" if hz is None else f"{hz:.1f} Hz"
    detail = f"{name}: count={stats.count} rate={hz_text} frame={stats.last_frame_id or 'n/a'}"
    if stats.last_point_count is not None:
        detail += f" points={stats.last_point_count}"
    if stats.has_xyz_fields is not None:
        detail += f" xyz_fields={str(stats.has_xyz_fields).lower()}"
    if stats.last_status_level is not None:
        detail += (
            f" status_level={stats.last_status_level}"
            f" message={stats.last_status_message or 'n/a'}"
        )
    return detail


def report_lines(
    config: BringupCheckConfig,
    snapshot: BringupSnapshot,
    results: Sequence[CheckResult],
) -> List[str]:
    lines = [
        topic_summary(config.cloud_topic, snapshot.cloud),
        topic_summary(config.imu_topic, snapshot.imu),
        topic_summary(config.pose_topic, snapshot.pose),
        topic_summary(config.alignment_status_topic, snapshot.status),
        "",
    ]
    lines.extend(f"[{result.level}] {result.message}" for result in results)
    return lines


def _evaluate_cloud(
    config: BringupCheckConfig,
    cloud: TopicStats,
    results: List[CheckResult],
) -> None:
    if cloud.count == 0:
        results.append(CheckResult(FAIL, f"no pointcloud received on {config.cloud_topic}"))
    elif cloud.has_xyz_fields is False:
        results.append(CheckResult(FAIL, f"{config.cloud_topic} is missing x/y/z fields"))
    else:
        results.append(CheckResult(OK, f"pointcloud received on {config.cloud_topic}"))


def _evaluate_imu(
    config: BringupCheckConfig,
    imu: TopicStats,
    results: List[CheckResult],
) -> None:
    if imu.count == 0:
        level = FAIL if config.require_imu else WARN
        results.append(CheckResult(level, f"no IMU received on {config.imu_topic}"))
    else:
        results.append(CheckResult(OK, f"IMU received on {config.imu_topic}"))


def _evaluate_tf(
    config: BringupCheckConfig,
    tf_checks: Sequence[TfCheck],
    results: List[CheckResult],
) -> None:
    map_odom_name = f"{config.global_frame} <- {config.odom_frame}"
    for tf_check in tf_checks:
        if tf_check.available:
            results.append(CheckResult(OK, f"TF available: {tf_check.name}"))
            continue
        level = FAIL if tf_check.name != map_odom_name or config.require_map_odom_tf else WARN
        results.append(CheckResult(level, f"TF missing: {tf_check.name}"))


def _evaluate_pose(
    config: BringupCheckConfig,
    pose: TopicStats,
    results: List[CheckResult],
) -> None:
    if pose.count == 0:
        level = FAIL if config.require_localization_output else WARN
        results.append(CheckResult(level, f"no localization pose received on {config.pose_topic}"))
    else:
        results.append(CheckResult(OK, f"localization pose received on {config.pose_topic}"))


def _evaluate_status(
    config: BringupCheckConfig,
    status: TopicStats,
    results: List[CheckResult],
) -> None:
    if status.count == 0:
        level = FAIL if config.require_localization_output else WARN
        results.append(
            CheckResult(level, f"no alignment status received on {config.alignment_status_topic}")
        )
        return
    if status.last_status_level is not None and status.last_status_level >= 2:
        results.append(
            CheckResult(FAIL, f"alignment status error: {status.last_status_message or 'n/a'}")
        )
    else:
        results.append(
            CheckResult(OK, f"alignment status observed: {status.last_status_message or 'n/a'}")
        )
