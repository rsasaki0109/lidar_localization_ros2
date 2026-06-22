from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence


OK = "OK"
WARN = "WARN"
FAIL = "FAIL"
POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"
CLOUD_TIME_FIELDS = ("time", "t", "timestamp", "offset_time")


@dataclass
class TopicStats:
    count: int = 0
    first_wall_time: Optional[float] = None
    last_wall_time: Optional[float] = None
    last_frame_id: str = ""
    last_stamp_sec: Optional[float] = None
    last_point_count: Optional[int] = None
    has_xyz_fields: Optional[bool] = None
    field_names: Sequence[str] = ()
    last_status_level: Optional[int] = None
    last_status_message: str = ""
    status_values: Dict[str, str] = field(default_factory=dict)

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
    imu_frame: str = "livox_imu_frame"
    require_imu: bool = False
    require_cloud_time_field: bool = False
    require_imu_base_tf: bool = False
    require_odom_base_tf: bool = True
    require_map_odom_tf: bool = False
    require_localization_output: bool = False


@dataclass(frozen=True)
class CheckResult:
    level: str
    message: str
    hint: str = ""


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
    topic_types: Dict[str, Sequence[str]] = field(default_factory=dict)


def build_tf_checks(config: BringupCheckConfig, availability: Dict[str, bool]) -> List[TfCheck]:
    checks = [
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
    if config.require_imu_base_tf:
        checks.append(
            TfCheck(
                f"{config.base_frame} <- {config.imu_frame}",
                availability.get(f"{config.base_frame} <- {config.imu_frame}", False),
            )
        )
    return checks


def evaluate_snapshot(
    config: BringupCheckConfig,
    snapshot: BringupSnapshot,
) -> List[CheckResult]:
    results: List[CheckResult] = []
    _evaluate_cloud(config, snapshot.cloud, snapshot.topic_types, results)
    _evaluate_imu(config, snapshot.imu, snapshot.topic_types, results)
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
    if stats.field_names:
        detail += f" fields={','.join(stats.field_names)}"
    if stats.last_status_level is not None:
        detail += (
            f" status_level={stats.last_status_level}"
            f" message={stats.last_status_message or 'n/a'}"
        )
    imu_status = stats.status_values.get("imu_preintegration_status", "")
    if imu_status:
        detail += f" imu_preintegration_status={imu_status}"
    seed_source = stats.status_values.get("registration_seed_source", "")
    if seed_source:
        detail += f" registration_seed_source={seed_source}"
    scan_time_status = stats.status_values.get("scan_time_status", "")
    if scan_time_status:
        detail += f" scan_time_status={scan_time_status}"
    deskew_status = stats.status_values.get("deskew_readiness_status", "")
    if deskew_status:
        detail += f" deskew_readiness_status={deskew_status}"
    continuous_time_status = stats.status_values.get("continuous_time_deskew_status", "")
    if continuous_time_status:
        detail += f" continuous_time_deskew_status={continuous_time_status}"
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
    for result in results:
        lines.append(f"[{result.level}] {result.message}")
        if result.hint:
            lines.append(f"  hint: {result.hint}")
    return lines


def _evaluate_cloud(
    config: BringupCheckConfig,
    cloud: TopicStats,
    topic_types: Dict[str, Sequence[str]],
    results: List[CheckResult],
) -> None:
    if cloud.count == 0:
        results.append(
            CheckResult(
                FAIL,
                f"no pointcloud received on {config.cloud_topic}",
                _missing_cloud_hint(config, topic_types),
            )
        )
    elif cloud.has_xyz_fields is False:
        results.append(
            CheckResult(
                FAIL,
                f"{config.cloud_topic} is missing x/y/z fields",
                "Use a sensor_msgs/PointCloud2 stream with x, y, z fields or convert the driver output.",
            )
        )
    else:
        results.append(CheckResult(OK, f"pointcloud received on {config.cloud_topic}"))
        if config.require_cloud_time_field and not _cloud_has_time_field(cloud):
            results.append(
                CheckResult(
                    FAIL,
                    f"{config.cloud_topic} has no per-point time field",
                    (
                        "Continuous-time deskew needs one of these PointCloud2 fields: "
                        f"{', '.join(CLOUD_TIME_FIELDS)}. Configure the LiDAR driver or "
                        "bridge to preserve per-point timing, or rerun without "
                        "--require-cloud-time-field for LiDAR-only registration."
                    ),
                )
            )
        if cloud.last_frame_id and cloud.last_frame_id != config.lidar_frame:
            results.append(
                CheckResult(
                    WARN,
                    (
                        f"pointcloud frame is {cloud.last_frame_id}, "
                        f"but lidar_frame is {config.lidar_frame}"
                    ),
                    (
                        f"Pass lidar_frame_id:={cloud.last_frame_id} to launch and "
                        f"--lidar-frame {cloud.last_frame_id} to this doctor, or remap/republish "
                        "the pointcloud frame to match your configured LiDAR frame."
                    ),
                )
            )


def _cloud_has_time_field(cloud: TopicStats) -> bool:
    return any(field_name in CLOUD_TIME_FIELDS for field_name in cloud.field_names)


def _evaluate_imu(
    config: BringupCheckConfig,
    imu: TopicStats,
    topic_types: Dict[str, Sequence[str]],
    results: List[CheckResult],
) -> None:
    if imu.count == 0:
        level = FAIL if config.require_imu else WARN
        hint = _missing_imu_hint(config, topic_types)
        results.append(CheckResult(level, f"no IMU received on {config.imu_topic}", hint))
    else:
        results.append(CheckResult(OK, f"IMU received on {config.imu_topic}"))


def _topics_with_type(
    topic_types: Dict[str, Sequence[str]],
    message_type: str,
    expected_topic: str,
) -> List[str]:
    return sorted(
        topic for topic, types in topic_types.items()
        if topic != expected_topic and message_type in types
    )


def _topic_types_text(types: Sequence[str]) -> str:
    return ", ".join(types) if types else "none"


def _format_topics(topics: Sequence[str]) -> str:
    if len(topics) <= 3:
        return ", ".join(topics)
    return ", ".join(topics[:3]) + f", and {len(topics) - 3} more"


def _missing_cloud_hint(
    config: BringupCheckConfig,
    topic_types: Dict[str, Sequence[str]],
) -> str:
    expected_types = topic_types.get(config.cloud_topic, [])
    if POINTCLOUD2_TYPE in expected_types:
        return (
            f"{config.cloud_topic} is advertised as PointCloud2, but no messages arrived "
            "during this check. Make sure the driver or bag is actively publishing, try "
            f"--duration-sec 10, or inspect `ros2 topic hz {config.cloud_topic}`."
        )
    if expected_types:
        return (
            f"{config.cloud_topic} exists, but its type is {_topic_types_text(expected_types)}. "
            "Use a sensor_msgs/msg/PointCloud2 topic for cloud_topic."
        )
    alternatives = _topics_with_type(topic_types, POINTCLOUD2_TYPE, config.cloud_topic)
    if alternatives:
        first = alternatives[0]
        return (
            f"Observed PointCloud2 topic(s): {_format_topics(alternatives)}. "
            f"Pass cloud_topic:={first} to launch and --cloud-topic {first} here, "
            f"or remap the driver to {config.cloud_topic}."
        )
    return (
        "Start the LiDAR driver or bag, then pass the active PointCloud2 topic as "
        f"cloud_topic:={config.cloud_topic} in launch and --cloud-topic here."
    )


def _missing_imu_hint(
    config: BringupCheckConfig,
    topic_types: Dict[str, Sequence[str]],
) -> str:
    expected_types = topic_types.get(config.imu_topic, [])
    if IMU_TYPE in expected_types:
        prefix = "IMU is optional for this profile." if not config.require_imu else "IMU is required."
        return (
            f"{prefix} {config.imu_topic} is advertised as Imu, but no messages arrived "
            "during this check. Make sure the driver or bag is actively publishing, try "
            f"--duration-sec 10, or inspect `ros2 topic hz {config.imu_topic}`."
        )
    if expected_types:
        prefix = "IMU is optional for this profile." if not config.require_imu else "IMU is required."
        return (
            f"{prefix} {config.imu_topic} exists, but its type is "
            f"{_topic_types_text(expected_types)}. Use a sensor_msgs/msg/Imu topic for imu_topic."
        )
    alternatives = _topics_with_type(topic_types, IMU_TYPE, config.imu_topic)
    if alternatives:
        first = alternatives[0]
        prefix = "IMU is optional for this profile." if not config.require_imu else "IMU is required."
        return (
            f"{prefix} Observed IMU topic(s): {_format_topics(alternatives)}. "
            f"Pass imu_topic:={first} to launch and --imu-topic {first} here."
        )
    if config.require_imu:
        return f"Set imu_topic:={config.imu_topic} and verify the driver publishes sensor_msgs/Imu."
    return "This is fine when use_imu is false; pass --require-imu for strict IMU bringup."


def _evaluate_tf(
    config: BringupCheckConfig,
    tf_checks: Sequence[TfCheck],
    results: List[CheckResult],
) -> None:
    odom_base_name = f"{config.odom_frame} <- {config.base_frame}"
    map_odom_name = f"{config.global_frame} <- {config.odom_frame}"
    for tf_check in tf_checks:
        if tf_check.available:
            results.append(CheckResult(OK, f"TF available: {tf_check.name}"))
            continue
        level = FAIL
        if tf_check.name == odom_base_name and not config.require_odom_base_tf:
            level = WARN
        elif tf_check.name == map_odom_name and not config.require_map_odom_tf:
            level = WARN
        results.append(
            CheckResult(level, f"TF missing: {tf_check.name}", _tf_hint(config, tf_check.name))
        )


def _tf_hint(config: BringupCheckConfig, tf_name: str) -> str:
    base_lidar_name = f"{config.base_frame} <- {config.lidar_frame}"
    base_imu_name = f"{config.base_frame} <- {config.imu_frame}"
    odom_base_name = f"{config.odom_frame} <- {config.base_frame}"
    map_odom_name = f"{config.global_frame} <- {config.odom_frame}"
    if tf_name == base_lidar_name:
        return (
            f"Pass lidar_frame_id:={config.lidar_frame} plus lidar_tf_x/y/z/roll/pitch/yaw "
            "when launch should publish the static LiDAR TF. If publish_lidar_tf:=false, "
            "verify robot_state_publisher or another static TF publisher provides it."
        )
    if tf_name == base_imu_name:
        return (
            f"Pass publish_imu_tf:=true imu_frame_id:={config.imu_frame} plus "
            "imu_tf_x/y/z/roll/pitch/yaw, or verify robot_state_publisher/static TF "
            "already provides it. If preintegration is disabled, rerun with "
            "--no-require-imu-base-tf."
        )
    if tf_name == odom_base_name:
        if not config.require_odom_base_tf:
            return (
                f"{odom_base_name} is optional for standalone map->base_link localization; "
                "use --profile nav2 when you expect odometry TF."
            )
        return (
            f"{odom_base_name} is required in this profile; start your odometry source or "
            "use --profile standalone / --no-require-odom-base-tf for map->base_link mode."
        )
    if tf_name == map_odom_name:
        if not config.require_map_odom_tf:
            return (
                f"{map_odom_name} is optional unless enable_map_odom_tf/Nav2 mode is enabled; "
                "standalone mode can publish map->base_link directly."
            )
        return (
            f"{map_odom_name} is required in this run; set enable_map_odom_tf:=true on the "
            "localizer or use standalone mode when publishing map->base_link directly."
        )
    return "Check the frame names passed to the launch file and this doctor."


def _evaluate_pose(
    config: BringupCheckConfig,
    pose: TopicStats,
    results: List[CheckResult],
) -> None:
    if pose.count == 0:
        level = FAIL if config.require_localization_output else WARN
        results.append(
            CheckResult(
                level,
                f"no localization pose received on {config.pose_topic}",
                (
                    "Check that the map is loaded, /initialpose was provided or set_initial_pose is true, "
                    "and /alignment_status is not reporting an error."
                ),
            )
        )
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
            CheckResult(
                level,
                f"no alignment status received on {config.alignment_status_topic}",
                "Make sure lidar_localization_node is active and receiving pointclouds after a map and initial pose.",
            )
        )
        return
    if status.last_status_level is not None and status.last_status_level >= 2:
        results.append(
            CheckResult(
                FAIL,
                f"alignment status error: {status.last_status_message or 'n/a'}",
                "Use the status message to choose the next fix: map path, initial pose, TF, crop size, or score threshold.",
            )
        )
    else:
        results.append(
            CheckResult(OK, f"alignment status observed: {status.last_status_message or 'n/a'}")
        )
    _evaluate_imu_preintegration_status(config, status, results)
    _evaluate_scan_time_status(config, status, results)
    _evaluate_deskew_readiness_status(config, status, results)
    _evaluate_continuous_time_deskew_status(config, status, results)


def _evaluate_imu_preintegration_status(
    config: BringupCheckConfig,
    status: TopicStats,
    results: List[CheckResult],
) -> None:
    imu_status = status.status_values.get("imu_preintegration_status", "")
    if not imu_status:
        return

    strict_imu = config.require_imu or config.require_imu_base_tf
    received = _status_int(status, "imu_received_sample_count")
    integrated = _status_int(status, "imu_integrated_sample_count")
    skipped = _status_int(status, "imu_skipped_sample_count")
    transform_failures = _status_int(status, "imu_transform_failure_count")
    invalid_dt = _status_int(status, "imu_invalid_dt_count")
    last_dt = status.status_values.get("imu_last_dt_sec", "n/a")
    integration_window = status.status_values.get("imu_integration_window_sec", "n/a")
    seed_source = status.status_values.get("registration_seed_source", "n/a")
    counts = (
        f"received={received} integrated={integrated} skipped={skipped} "
        f"tf_failures={transform_failures} invalid_dt={invalid_dt} "
        f"seed_source={seed_source}"
    )

    if imu_status == "imu_preintegration_disabled":
        if strict_imu:
            results.append(
                CheckResult(
                    FAIL,
                    "IMU preintegration is disabled",
                    "Set use_imu_preintegration:=true or rerun the doctor without strict IMU checks.",
                )
            )
        return

    if imu_status == "imu_preintegration_prediction_active":
        results.append(CheckResult(OK, f"IMU preintegration active ({counts})"))
        return

    if imu_status == "imu_preintegration_prediction_available":
        results.append(
            CheckResult(
                OK,
                f"IMU preintegration prediction available but not selected ({counts})",
            )
        )
        return

    if imu_status == "imu_preintegration_transform_unavailable":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration cannot transform samples ({counts})",
                (
                    f"Provide {config.base_frame} <- {config.imu_frame} TF. "
                    f"Use publish_imu_tf:=true imu_frame_id:={config.imu_frame} "
                    "with imu_tf_x/y/z/roll/pitch/yaw, or fix robot_state_publisher/static TF."
                ),
            )
        )
        return

    if imu_status == "imu_preintegration_invalid_delta_time":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration skipped samples because dt is invalid ({counts})",
                (
                    f"Last IMU dt was {last_dt} sec. Check IMU timestamps, rosbag clock, "
                    "use_sim_time, duplicate stamps, and driver time synchronization."
                ),
            )
        )
        return

    if imu_status == "imu_preintegration_non_finite_sample":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration received non-finite samples ({counts})",
                "Inspect the IMU driver output; angular_velocity and linear_acceleration must be finite.",
            )
        )
        return

    if imu_status == "imu_preintegration_stale_imu":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration is using stale IMU data ({counts})",
                "Check IMU rate, topic remapping, sensor timestamps, rosbag clock, and use_sim_time.",
            )
        )
        return

    if imu_status == "imu_preintegration_integration_window_too_large":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration buffered too large a time window ({counts})",
                (
                    f"The latest IMU integration window was {integration_window} sec. "
                    "Check rosbag playback rate, executor throughput, IMU queue depth, "
                    "and timestamp synchronization."
                ),
            )
        )
        return

    if imu_status == "imu_preintegration_waiting_for_imu":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                "IMU preintegration is waiting for IMU samples",
                _missing_imu_hint(config, {}),
            )
        )
        return

    if imu_status == "imu_preintegration_waiting_for_smoother_initialization":
        results.append(
            CheckResult(
                WARN,
                "IMU preintegration is waiting for smoother initialization",
                "Provide a valid initial pose and wait for the first accepted LiDAR alignment.",
            )
        )
        return

    if imu_status == "imu_preintegration_waiting_for_new_imu":
        results.append(
            CheckResult(
                WARN if strict_imu else OK,
                f"IMU preintegration has no new sample for the latest scan ({counts})",
                "Check IMU rate if this persists; it can be transient immediately after a scan update.",
            )
        )
        return

    if imu_status == "imu_preintegration_prediction_non_finite":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                f"IMU preintegration prediction is non-finite ({counts})",
                "Inspect IMU noise parameters, input units, gravity convention, and recent fallback warnings.",
            )
        )
        return

    if imu_status == "imu_preintegration_fallback_mode":
        results.append(
            CheckResult(
                FAIL if strict_imu else WARN,
                "IMU preintegration is in fallback mode",
                (
                    "The correction guard disabled IMU preintegration after a large disagreement. "
                    "Check initial pose, TF extrinsics, timestamp sync, and alignment score around the transition."
                ),
            )
        )


def _evaluate_scan_time_status(
    config: BringupCheckConfig,
    status: TopicStats,
    results: List[CheckResult],
) -> None:
    scan_time_status = status.status_values.get("scan_time_status", "")
    if not scan_time_status:
        return

    field = status.status_values.get("scan_time_field", "none")
    duration = status.status_values.get("scan_time_duration_sec", "n/a")
    valid = _status_int(status, "scan_time_valid_point_count")
    invalid = _status_int(status, "scan_time_invalid_point_count")
    counts = (
        f"field={field} duration={duration}s valid_points={valid} invalid_points={invalid}"
    )
    not_ready_level = FAIL if config.require_cloud_time_field else WARN

    if scan_time_status == "scan_time_range_ready":
        results.append(CheckResult(OK, f"per-point scan time ready ({counts})"))
        return

    if scan_time_status == "scan_time_field_missing":
        results.append(
            CheckResult(
                not_ready_level,
                "per-point scan time field is missing",
                (
                    "Continuous-time deskew needs one PointCloud2 field named "
                    f"{', '.join(CLOUD_TIME_FIELDS)}. Preserve the driver timing field "
                    "or rerun without --require-cloud-time-field for LiDAR-only registration."
                ),
            )
        )
        return

    if scan_time_status == "scan_time_field_invalid":
        results.append(
            CheckResult(
                not_ready_level,
                f"per-point scan time field is invalid ({counts})",
                (
                    "Check the PointCloud2 field datatype, offset, point_step, and values. "
                    "The localizer must be able to read finite per-point times from the cloud."
                ),
            )
        )
        return

    if scan_time_status == "scan_time_range_too_large":
        results.append(
            CheckResult(
                not_ready_level,
                f"per-point scan time range is too large ({counts})",
                (
                    "Check the LiDAR driver time units and timestamp synchronization. "
                    "Livox offset_time/t fields are expected in nanoseconds; time fields "
                    "are expected in seconds."
                ),
            )
        )
        return

    results.append(
        CheckResult(
            WARN,
            f"unknown per-point scan time status: {scan_time_status}",
            "Update the bringup doctor if the localizer reports a newer scan_time_status value.",
        )
    )


def _evaluate_deskew_readiness_status(
    config: BringupCheckConfig,
    status: TopicStats,
    results: List[CheckResult],
) -> None:
    deskew_status = status.status_values.get("deskew_readiness_status", "")
    if not deskew_status:
        return

    if deskew_status == "deskew_ready":
        results.append(CheckResult(OK, "continuous-time deskew inputs are ready"))
        return

    strict_deskew = config.require_cloud_time_field and (
        config.require_imu or config.require_imu_base_tf
    )
    level = FAIL if strict_deskew else WARN
    results.append(
        CheckResult(
            level,
            f"continuous-time deskew inputs are not ready: {deskew_status}",
            _deskew_readiness_hint(config, deskew_status),
        )
    )


def _deskew_readiness_hint(config: BringupCheckConfig, deskew_status: str) -> str:
    if deskew_status.startswith("deskew_scan_time_"):
        return (
            "Fix the PointCloud2 per-point timing first. The cloud must expose one "
            f"of {', '.join(CLOUD_TIME_FIELDS)} with readable values; use "
            "--require-cloud-time-field when validating deskew readiness."
        )
    if deskew_status == "deskew_imu_preintegration_disabled":
        return (
            "Set use_imu_preintegration:=true and pass --require-imu plus "
            "--require-imu-base-tf when validating continuous-time deskew inputs."
        )
    if deskew_status == "deskew_imu_transform_unavailable":
        return (
            f"Provide {config.base_frame} <- {config.imu_frame} TF. Use "
            f"publish_imu_tf:=true imu_frame_id:={config.imu_frame}, or fix the "
            "static transform from robot_state_publisher."
        )
    if deskew_status in {
        "deskew_waiting_for_imu",
        "deskew_waiting_for_new_imu",
        "deskew_imu_stale",
    }:
        return "Check IMU topic remapping, rate, timestamps, rosbag clock, and use_sim_time."
    if deskew_status in {
        "deskew_imu_invalid_delta_time",
        "deskew_imu_integration_window_too_large",
        "deskew_imu_non_finite_sample",
        "deskew_imu_prediction_non_finite",
    }:
        return "Inspect IMU message values, timestamp order, time synchronization, units, and noise parameters."
    if deskew_status == "deskew_waiting_for_smoother_initialization":
        return "Provide a valid initial pose and wait for the first accepted LiDAR alignment."
    if deskew_status == "deskew_imu_preintegration_fallback_mode":
        return "Check initial pose, TF extrinsics, timestamp sync, and recent LiDAR alignment quality."
    return "Use scan_time_status and imu_preintegration_status in /alignment_status to identify the blocker."


def _evaluate_continuous_time_deskew_status(
    config: BringupCheckConfig,
    status: TopicStats,
    results: List[CheckResult],
) -> None:
    continuous_time_status = status.status_values.get("continuous_time_deskew_status", "")
    if not continuous_time_status or continuous_time_status == "continuous_time_deskew_disabled":
        return

    applied = status.status_values.get("continuous_time_deskew_applied", "false") == "true"
    point_count = _status_int(status, "continuous_time_deskew_point_count")
    skipped = _status_int(status, "continuous_time_deskew_skipped_invalid_time_count")
    clamped = _status_int(status, "continuous_time_deskew_clamped_time_count")
    counts = f"points={point_count} skipped_invalid_time={skipped} clamped_time={clamped}"

    if applied or continuous_time_status == "continuous_time_deskew_applied":
        results.append(CheckResult(OK, f"continuous-time deskew applied ({counts})"))
        return

    strict_deskew = config.require_cloud_time_field and (
        config.require_imu or config.require_imu_base_tf
    )
    results.append(
        CheckResult(
            FAIL if strict_deskew else WARN,
            f"continuous-time deskew was not applied: {continuous_time_status}",
            _continuous_time_deskew_hint(config, continuous_time_status),
        )
    )


def _continuous_time_deskew_hint(
    config: BringupCheckConfig,
    continuous_time_status: str,
) -> str:
    if continuous_time_status in {
        "continuous_time_deskew_scan_time_not_ready",
        "continuous_time_deskew_times_not_aligned",
    }:
        return (
            "Fix PointCloud2 per-point timing first. Run with --require-cloud-time-field "
            "and inspect scan_time_status plus scan_time_valid/invalid_point_count."
        )
    if continuous_time_status == "continuous_time_deskew_imu_preintegration_disabled":
        return "Set use_imu_preintegration:=true before enabling use_continuous_time_deskew."
    if continuous_time_status in {
        "continuous_time_deskew_waiting_for_smoother",
        "continuous_time_deskew_waiting_for_new_imu",
        "continuous_time_deskew_imu_fallback_mode",
    }:
        return (
            f"Check IMU topic {config.imu_topic}, base/IMU TF, timestamps, initial pose, "
            "and imu_preintegration_status."
        )
    if continuous_time_status == "continuous_time_deskew_prediction_non_finite":
        return "Inspect IMU values, timestamp order, units, noise parameters, and recent fallback warnings."
    if continuous_time_status == "continuous_time_deskew_scan_not_prepared":
        return "Fix scan preparation first: x/y/z fields, TF, range filters, and empty filtered scans."
    return "Use scan_time_status, deskew_readiness_status, and imu_preintegration_status for the blocker."


def _status_int(status: TopicStats, key: str) -> int:
    try:
        return int(status.status_values.get(key, "0"))
    except ValueError:
        return 0
