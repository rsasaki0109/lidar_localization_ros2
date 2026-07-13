import csv
import json
from collections import Counter
from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
import statistics


IMU_ACTIVE_STATUS = "imu_preintegration_prediction_active"
IMU_AVAILABLE_STATUS = "imu_preintegration_prediction_available"
IMU_FALLBACK_STATUS = "imu_preintegration_fallback_mode"
DESKEW_APPLIED_STATUS = "continuous_time_deskew_applied"


@dataclass(frozen=True)
class AlignmentDiagnosticSample:
    stamp_sec: float
    level: int
    message: str
    values: Dict[str, str]


@dataclass(frozen=True)
class RuntimeValidationConfig:
    min_samples: int = 3
    min_imu_active_ratio: float = 0.5
    min_imu_integrated_samples: int = 1
    max_imu_fallback_count: int = 0
    require_imu_seed_source: bool = False
    min_imu_seed_source_ratio: float = 0.5
    require_deskew_applied: bool = False
    min_deskew_applied_ratio: float = 0.1


@dataclass(frozen=True)
class ValidationCheck:
    level: str
    message: str
    hint: str = ""


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() == "true"


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        number = int(float(str(value)))
    except (TypeError, ValueError):
        return None
    return number


def _as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(str(value))
    except (TypeError, ValueError):
        return None


def _percentile(values: Sequence[float], quantile: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _float_values(samples: Sequence[AlignmentDiagnosticSample], key: str) -> List[float]:
    return [
        value for value in (_as_float(sample.values.get(key)) for sample in samples)
        if value is not None
    ]


def _ratio(count: int, total: int) -> float:
    return 0.0 if total <= 0 else float(count) / float(total)


def _percent(ratio: float) -> str:
    return f"{100.0 * ratio:.1f}%"


def _max_int(samples: Sequence[AlignmentDiagnosticSample], key: str) -> int:
    values = [
        value for value in (_as_int(sample.values.get(key)) for sample in samples)
        if value is not None
    ]
    return max(values) if values else 0


def load_alignment_csv(path: str) -> List[AlignmentDiagnosticSample]:
    samples: List[AlignmentDiagnosticSample] = []
    with open(path, "r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                values = json.loads(row.get("values_json", "{}") or "{}")
            except json.JSONDecodeError:
                values = {}
            samples.append(
                AlignmentDiagnosticSample(
                    stamp_sec=float(row.get("stamp_sec") or "0.0"),
                    level=_as_int(row.get("level")) or 0,
                    message=str(row.get("message", "")),
                    values={str(key): str(value) for key, value in values.items()},
                )
            )
    return samples


def summarize_runtime(samples: Sequence[AlignmentDiagnosticSample]) -> Dict[str, Any]:
    imu_status_counts = Counter(
        sample.values.get("imu_preintegration_status", "") for sample in samples
    )
    scan_time_status_counts = Counter(
        sample.values.get("scan_time_status", "") for sample in samples
    )
    deskew_readiness_counts = Counter(
        sample.values.get("deskew_readiness_status", "") for sample in samples
    )
    registration_seed_source_counts = Counter(
        sample.values.get("registration_seed_source", "not_selected") for sample in samples
    )
    registration_seed_source_present_count = sum(
        1 for sample in samples if "registration_seed_source" in sample.values
    )
    continuous_time_counts = Counter(
        sample.values.get("continuous_time_deskew_status", "") for sample in samples
    )
    imu_active_count = imu_status_counts.get(IMU_ACTIVE_STATUS, 0)
    imu_available_count = imu_status_counts.get(IMU_AVAILABLE_STATUS, 0)
    imu_fallback_count = sum(
        1 for sample in samples
        if sample.values.get("imu_preintegration_status", "") == IMU_FALLBACK_STATUS
        or _as_bool(sample.values.get("imu_preintegration_fallback_mode"))
    )
    imu_preintegration_seed_count = registration_seed_source_counts.get(
        "imu_preintegration", 0
    )
    deskew_applied_count = sum(
        1 for sample in samples
        if sample.values.get("continuous_time_deskew_status", "") == DESKEW_APPLIED_STATUS
        or _as_bool(sample.values.get("continuous_time_deskew_applied"))
    )
    total = len(samples)
    scan_time_durations = _float_values(samples, "scan_time_duration_sec")
    pose_history_coverage = _float_values(
        samples, "continuous_time_deskew_pose_history_coverage_ratio"
    )
    return {
        "sample_count": total,
        "error_level_count": sum(1 for sample in samples if sample.level >= 2),
        "message_counts": dict(Counter(sample.message for sample in samples)),
        "imu_status_counts": dict(imu_status_counts),
        "imu_active_count": imu_active_count,
        "imu_available_count": imu_available_count,
        "imu_active_ratio": _ratio(imu_active_count, total),
        "imu_available_or_active_ratio": _ratio(imu_active_count + imu_available_count, total),
        "imu_fallback_count": imu_fallback_count,
        "imu_integrated_sample_count_max": _max_int(samples, "imu_integrated_sample_count"),
        "imu_received_sample_count_max": _max_int(samples, "imu_received_sample_count"),
        "imu_skipped_sample_count_max": _max_int(samples, "imu_skipped_sample_count"),
        "imu_transform_failure_count_max": _max_int(samples, "imu_transform_failure_count"),
        "imu_invalid_dt_count_max": _max_int(samples, "imu_invalid_dt_count"),
        "imu_non_finite_sample_count_max": _max_int(samples, "imu_non_finite_sample_count"),
        "scan_time_status_counts": dict(scan_time_status_counts),
        "scan_time_duration_sample_count": len(scan_time_durations),
        "scan_time_duration_median_sec": (
            statistics.median(scan_time_durations) if scan_time_durations else None
        ),
        "scan_time_duration_p95_sec": _percentile(scan_time_durations, 0.95),
        "scan_time_duration_max_sec": max(scan_time_durations) if scan_time_durations else None,
        "deskew_readiness_counts": dict(deskew_readiness_counts),
        "registration_seed_source_counts": dict(registration_seed_source_counts),
        "registration_seed_source_present_count": registration_seed_source_present_count,
        "imu_preintegration_seed_count": imu_preintegration_seed_count,
        "imu_preintegration_seed_ratio": _ratio(imu_preintegration_seed_count, total),
        "continuous_time_deskew_status_counts": dict(continuous_time_counts),
        "continuous_time_deskew_applied_count": deskew_applied_count,
        "continuous_time_deskew_applied_ratio": _ratio(deskew_applied_count, total),
        "continuous_time_deskew_point_count_max": _max_int(
            samples, "continuous_time_deskew_point_count"
        ),
        "continuous_time_deskew_skipped_invalid_time_count_max": _max_int(
            samples, "continuous_time_deskew_skipped_invalid_time_count"
        ),
        "continuous_time_deskew_clamped_time_count_max": _max_int(
            samples, "continuous_time_deskew_clamped_time_count"
        ),
        "continuous_time_deskew_pose_history_coverage_sample_count": len(
            pose_history_coverage
        ),
        "continuous_time_deskew_pose_history_coverage_median": (
            statistics.median(pose_history_coverage) if pose_history_coverage else None
        ),
        "continuous_time_deskew_pose_history_coverage_min": (
            min(pose_history_coverage) if pose_history_coverage else None
        ),
    }


def evaluate_runtime_summary(
    summary: Dict[str, Any],
    config: RuntimeValidationConfig,
) -> List[ValidationCheck]:
    checks: List[ValidationCheck] = []
    sample_count = int(summary["sample_count"])
    if sample_count < config.min_samples:
        checks.append(
            ValidationCheck(
                "FAIL",
                f"only {sample_count} alignment_status samples received",
                "Start localization and rosbag playback, then run the validator for a longer duration.",
            )
        )
    else:
        checks.append(ValidationCheck("OK", f"alignment_status samples={sample_count}"))

    active_ratio = float(summary["imu_active_ratio"])
    if active_ratio < config.min_imu_active_ratio:
        checks.append(
            ValidationCheck(
                "FAIL",
                (
                    "IMU preintegration active ratio "
                    f"{_percent(active_ratio)} < {_percent(config.min_imu_active_ratio)}"
                ),
                (
                    "Check imu_preintegration_status counts, /imu remapping, use_sim_time, "
                    "base_frame <- imu_frame TF, and timestamp order before tuning noise."
                ),
            )
        )
    else:
        checks.append(
            ValidationCheck(
                "OK",
                f"IMU preintegration active ratio {_percent(active_ratio)}",
            )
        )

    integrated_max = int(summary["imu_integrated_sample_count_max"])
    if integrated_max < config.min_imu_integrated_samples:
        checks.append(
            ValidationCheck(
                "FAIL",
                (
                    f"max imu_integrated_sample_count {integrated_max} "
                    f"< {config.min_imu_integrated_samples}"
                ),
                "Inspect imu_received_sample_count, imu_skipped_sample_count, transform failures, and invalid dt counters.",
            )
        )
    else:
        checks.append(
            ValidationCheck(
                "OK",
                f"max imu_integrated_sample_count={integrated_max}",
            )
        )

    fallback_count = int(summary["imu_fallback_count"])
    if fallback_count > config.max_imu_fallback_count:
        checks.append(
            ValidationCheck(
                "FAIL",
                f"IMU preintegration fallback count {fallback_count} > {config.max_imu_fallback_count}",
                "Fallback means the correction guard disabled IMU preintegration for this run; inspect preceding alignment warnings.",
            )
        )
    else:
        checks.append(ValidationCheck("OK", f"IMU fallback count={fallback_count}"))

    if config.require_imu_seed_source:
        seed_source_present_count = int(summary["registration_seed_source_present_count"])
        seed_ratio = float(summary["imu_preintegration_seed_ratio"])
        if seed_source_present_count <= 0:
            checks.append(
                ValidationCheck(
                    "FAIL",
                    "registration_seed_source is missing from alignment_status samples",
                    (
                        "Rebuild and rerun with a lidar_localization_ros2 version that publishes "
                        "registration_seed_source, then check whether it reports imu_preintegration."
                    ),
                )
            )
        elif seed_ratio < config.min_imu_seed_source_ratio:
            checks.append(
                ValidationCheck(
                    "FAIL",
                    (
                        "IMU preintegration seed source ratio "
                        f"{_percent(seed_ratio)} < {_percent(config.min_imu_seed_source_ratio)}"
                    ),
                    (
                        "Check registration_seed_source counts, imu_preintegration_status, "
                        "new IMU samples after each scan, and whether another prediction source "
                        "is being selected."
                    ),
                )
            )
        else:
            checks.append(
                ValidationCheck(
                    "OK",
                    f"IMU preintegration seed source ratio {_percent(seed_ratio)}",
                )
            )

    if config.require_deskew_applied:
        deskew_ratio = float(summary["continuous_time_deskew_applied_ratio"])
        if deskew_ratio < config.min_deskew_applied_ratio:
            checks.append(
                ValidationCheck(
                    "FAIL",
                    (
                        "continuous-time deskew applied ratio "
                        f"{_percent(deskew_ratio)} < {_percent(config.min_deskew_applied_ratio)}"
                    ),
                    (
                        "Check scan_time_status, deskew_readiness_status, per-point cloud timing, "
                        "and whether use_continuous_time_deskew is true."
                    ),
                )
            )
        else:
            checks.append(
                ValidationCheck(
                    "OK",
                    f"continuous-time deskew applied ratio {_percent(deskew_ratio)}",
                )
            )
    return checks


def validation_exit_code(checks: Sequence[ValidationCheck]) -> int:
    return 1 if any(check.level == "FAIL" for check in checks) else 0


def render_runtime_report(summary: Dict[str, Any], checks: Sequence[ValidationCheck]) -> List[str]:
    lines = [
        "Runtime IMU / deskew validation",
        f"samples: {summary['sample_count']}",
        (
            "imu active: "
            f"{summary['imu_active_count']}/{summary['sample_count']} "
            f"({_percent(float(summary['imu_active_ratio']))})"
        ),
        f"imu status counts: {json.dumps(summary['imu_status_counts'], sort_keys=True)}",
        (
            "imu counters max: "
            f"received={summary['imu_received_sample_count_max']} "
            f"integrated={summary['imu_integrated_sample_count_max']} "
            f"skipped={summary['imu_skipped_sample_count_max']} "
            f"tf_failures={summary['imu_transform_failure_count_max']} "
            f"invalid_dt={summary['imu_invalid_dt_count_max']} "
            f"non_finite={summary['imu_non_finite_sample_count_max']}"
        ),
        f"scan time counts: {json.dumps(summary['scan_time_status_counts'], sort_keys=True)}",
        (
            "scan time duration sec: "
            f"samples={summary['scan_time_duration_sample_count']} "
            f"median={summary['scan_time_duration_median_sec']} "
            f"p95={summary['scan_time_duration_p95_sec']} "
            f"max={summary['scan_time_duration_max_sec']}"
        ),
        f"deskew readiness counts: {json.dumps(summary['deskew_readiness_counts'], sort_keys=True)}",
        (
            "registration seed sources: "
            f"{json.dumps(summary['registration_seed_source_counts'], sort_keys=True)}"
        ),
        (
            "imu preintegration seed source: "
            f"{summary['imu_preintegration_seed_count']}/{summary['sample_count']} "
            f"({_percent(float(summary['imu_preintegration_seed_ratio']))})"
        ),
        (
            "continuous-time deskew applied: "
            f"{summary['continuous_time_deskew_applied_count']}/{summary['sample_count']} "
            f"({_percent(float(summary['continuous_time_deskew_applied_ratio']))})"
        ),
        (
            "continuous-time deskew counts: "
            f"{json.dumps(summary['continuous_time_deskew_status_counts'], sort_keys=True)}"
        ),
        (
            "continuous-time deskew counters max: "
            f"points={summary['continuous_time_deskew_point_count_max']} "
            f"skipped_invalid_time={summary['continuous_time_deskew_skipped_invalid_time_count_max']} "
            f"clamped_time={summary['continuous_time_deskew_clamped_time_count_max']}"
        ),
        (
            "pose-history coverage: "
            f"samples={summary['continuous_time_deskew_pose_history_coverage_sample_count']} "
            f"median={summary['continuous_time_deskew_pose_history_coverage_median']} "
            f"min={summary['continuous_time_deskew_pose_history_coverage_min']}"
        ),
        "",
    ]
    for check in checks:
        lines.append(f"[{check.level}] {check.message}")
        if check.hint:
            lines.append(f"  hint: {check.hint}")
    return lines


def markdown_report(summary: Dict[str, Any], checks: Sequence[ValidationCheck]) -> str:
    status = "PASS" if validation_exit_code(checks) == 0 else "FAIL"
    lines = [
        f"# Runtime IMU / Deskew Validation: {status}",
        "",
        "| Metric | Value |",
        "| --- | --- |",
        f"| Samples | {summary['sample_count']} |",
        f"| IMU active ratio | {_percent(float(summary['imu_active_ratio']))} |",
        (
            "| IMU preintegration seed source ratio | "
            f"{_percent(float(summary['imu_preintegration_seed_ratio']))} |"
        ),
        f"| Max integrated IMU samples | {summary['imu_integrated_sample_count_max']} |",
        f"| IMU fallback count | {summary['imu_fallback_count']} |",
        f"| Scan time duration median sec | {summary['scan_time_duration_median_sec']} |",
        f"| Scan time duration p95 sec | {summary['scan_time_duration_p95_sec']} |",
        f"| Scan time duration max sec | {summary['scan_time_duration_max_sec']} |",
        (
            "| Continuous-time deskew applied ratio | "
            f"{_percent(float(summary['continuous_time_deskew_applied_ratio']))} |"
        ),
        (
            "| Pose-history coverage median | "
            f"{summary['continuous_time_deskew_pose_history_coverage_median']} |"
        ),
        "",
        "## Checks",
        "",
    ]
    for check in checks:
        lines.append(f"- **{check.level}** {check.message}")
        if check.hint:
            lines.append(f"  - hint: {check.hint}")
    lines.extend([
        "",
        "## Status Counts",
        "",
        "```json",
        json.dumps(
            {
                "imu_status_counts": summary["imu_status_counts"],
                "scan_time_status_counts": summary["scan_time_status_counts"],
                "deskew_readiness_counts": summary["deskew_readiness_counts"],
                "registration_seed_source_counts": summary["registration_seed_source_counts"],
                "continuous_time_deskew_status_counts": summary[
                    "continuous_time_deskew_status_counts"
                ],
            },
            indent=2,
            sort_keys=True,
        ),
        "```",
        "",
    ])
    return "\n".join(lines)
