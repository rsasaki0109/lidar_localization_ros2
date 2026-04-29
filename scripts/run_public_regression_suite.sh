#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the public lidar_localization regression suite.

This suite covers:
  - Autoware Istanbul 60 s default-on no-IMU safety run
  - HDL 60 s IMU safety/throughput regression for use_imu_preintegration false/true

Usage:
  scripts/run_public_regression_suite.sh [options]

Options:
  --output-dir DIR       Working directory for generated inputs and run outputs.
                         Default: /tmp/lidarloc_public_regression_suite
  --report-dir DIR       Directory for summary artifacts.
                         Default: <workspace>/artifacts/public/public_regression_suite
  --ros-domain-base N    Base ROS domain id. Uses N for Istanbul and then
                         two ids per HDL repeat. Default: 200
  --hdl-repeat-count N   Number of HDL baseline/candidate repeats. Default: 2
  --resume               Reuse existing completed outputs when possible.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

if [[ ! -f "${repo_root}/scripts/setup_local_env.sh" ]]; then
  echo "repo root does not look valid: ${repo_root}" >&2
  exit 1
fi

output_dir="/tmp/lidarloc_public_regression_suite"
report_dir="${ws_root}/artifacts/public/public_regression_suite"
ros_domain_base=200
hdl_repeat_count=2
resume=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --report-dir)
      shift
      report_dir="$1"
      ;;
    --ros-domain-base)
      shift
      ros_domain_base="$1"
      ;;
    --hdl-repeat-count)
      shift
      hdl_repeat_count="$1"
      ;;
    --resume)
      resume=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if ! [[ "${hdl_repeat_count}" =~ ^[0-9]+$ ]] || (( hdl_repeat_count < 1 )); then
  echo "--hdl-repeat-count must be a positive integer" >&2
  exit 1
fi

run_allow_failure() {
  local rc=0
  set +e
  "$@"
  rc=$?
  set -e
  if [[ "${rc}" -ne 0 ]]; then
    echo "warning: command failed with rc=${rc}: $*" >&2
  fi
  return 0
}

istanbul_bag="${ws_root}/data/official/autoware_istanbul/localization_rosbag"
istanbul_map="${ws_root}/data/official/autoware_istanbul/pointcloud_map.pcd"
istanbul_cloud_topic="/localization/util/downsample/pointcloud"
istanbul_twist_topic="/localization/twist_estimator/twist_with_covariance"
istanbul_pose_topic="/sensing/gnss/pose_with_covariance"
istanbul_base_template="${repo_root}/param/public_istanbul_60s_benchmark.yaml"

hdl_bag="${ws_root}/data/official/hdl_localization/hdl_400_ros2"
hdl_cloud_topic="/velodyne_points"
hdl_imu_topic="/gpsimu_driver/imu_data"

for required_dir in "${istanbul_bag}" "${hdl_bag}"; do
  if [[ ! -d "${required_dir}" ]]; then
    echo "Required bag directory not found: ${required_dir}" >&2
    exit 1
  fi
done

for required_file in "${istanbul_map}" "${istanbul_base_template}" "${repo_root}/param/hdl_imu_preint.yaml"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Required file not found: ${required_file}" >&2
    exit 1
  fi
done

max_ros_domain_id=$((ros_domain_base + (2 * hdl_repeat_count)))
if (( max_ros_domain_id > 232 )); then
  echo "ROS domain range ${ros_domain_base}..${max_ros_domain_id} exceeds Fast DDS limit 232" >&2
  exit 1
fi

set +u
source "${repo_root}/scripts/setup_local_env.sh"
set -u

inputs_dir="${output_dir}/inputs"
istanbul_dir="${output_dir}/istanbul"
hdl_dir="${output_dir}/hdl"
mkdir -p "${inputs_dir}" "${istanbul_dir}" "${hdl_dir}" "${report_dir}"

istanbul_reference_csv="${inputs_dir}/autoware_istanbul_reference_60s.csv"
istanbul_initial_pose_yaml="${inputs_dir}/autoware_istanbul_initial_pose_60s.yaml"
istanbul_param_yaml="${inputs_dir}/autoware_istanbul_public_benchmark_60s.yaml"
istanbul_run_dir="${istanbul_dir}/default_on_no_imu"
hdl_no_imu_yaml="${inputs_dir}/hdl_imu_preint_false.yaml"
summary_json="${report_dir}/summary.json"
summary_md="${report_dir}/summary.md"

if [[ "${resume}" != "1" || ! -f "${istanbul_reference_csv}" || ! -f "${istanbul_initial_pose_yaml}" ]]; then
  ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
    --bag-path "${istanbul_bag}" \
    --pose-topic "${istanbul_pose_topic}" \
    --sample-topic "${istanbul_cloud_topic}" \
    --bag-duration 60 \
    --initial-pose-skip-sec 0.05 \
    --output-csv "${istanbul_reference_csv}" \
    --output-initial-pose-yaml "${istanbul_initial_pose_yaml}"
fi

python3 - "${istanbul_base_template}" "${istanbul_initial_pose_yaml}" "${istanbul_param_yaml}" "${istanbul_map}" <<'PY'
import sys
from pathlib import Path
import yaml

base_path = Path(sys.argv[1])
init_path = Path(sys.argv[2])
output_path = Path(sys.argv[3])
map_path = str(Path(sys.argv[4]).resolve())

base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
initial_pose = yaml.safe_load(init_path.read_text(encoding="utf-8"))
params = base["/**"]["ros__parameters"]
params.update(initial_pose["/**"]["ros__parameters"])
params["map_path"] = map_path
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(yaml.safe_dump(base, sort_keys=False), encoding="utf-8")
PY

python3 - "${repo_root}/param/hdl_imu_preint.yaml" "${hdl_no_imu_yaml}" <<'PY'
import sys
from pathlib import Path
import yaml

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
data = yaml.safe_load(src.read_text(encoding="utf-8"))
data["/**"]["ros__parameters"]["use_imu_preintegration"] = False
dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY

if [[ "${resume}" != "1" || ! -f "${istanbul_run_dir}/summary.json" ]]; then
  run_allow_failure "${repo_root}/scripts/benchmark_runner" \
    --bag-path "${istanbul_bag}" \
    --output-dir "${istanbul_run_dir}" \
    --ros-domain-id "${ros_domain_base}" \
    --bag-duration 60 \
    --settle-seconds 5 \
    --post-roll-seconds 1 \
    --target-process-pattern lidar_localization_node \
    --record-topic /pcl_pose \
    --record-qos-durability volatile \
    --diagnostic-topic /alignment_status \
    --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=${istanbul_param_yaml} cloud_topic:=${istanbul_cloud_topic} twist_topic:=${istanbul_twist_topic}"
fi

if [[ "${resume}" != "1" || ! -f "${istanbul_run_dir}/trajectory_eval.json" ]]; then
  "${repo_root}/scripts/benchmark_eval_trajectory" \
    --estimated-csv "${istanbul_run_dir}/pose_trace.csv" \
    --reference-csv "${istanbul_reference_csv}" \
    --output-json "${istanbul_run_dir}/trajectory_eval.json" \
    --max-time-diff 0.05
  cp "${istanbul_run_dir}/trajectory_eval.json" "${istanbul_run_dir}/eval.json"
fi

hdl_false_group_dir="${hdl_dir}/no_imu_preintegration"
hdl_true_group_dir="${hdl_dir}/imu_preintegration_with_disable_fallback"
mkdir -p "${hdl_false_group_dir}" "${hdl_true_group_dir}"

for ((repeat_index=1; repeat_index<=hdl_repeat_count; repeat_index++)); do
  run_suffix="$(printf 'r%02d' "${repeat_index}")"
  hdl_false_run_dir="${hdl_false_group_dir}/${run_suffix}"
  hdl_true_run_dir="${hdl_true_group_dir}/${run_suffix}"
  hdl_false_domain_id=$((ros_domain_base + (2 * repeat_index) - 1))
  hdl_true_domain_id=$((ros_domain_base + (2 * repeat_index)))

  if [[ "${resume}" != "1" || ! -f "${hdl_false_run_dir}/summary.json" ]]; then
    run_allow_failure "${repo_root}/scripts/benchmark_runner" \
      --bag-path "${hdl_bag}" \
      --output-dir "${hdl_false_run_dir}" \
      --ros-domain-id "${hdl_false_domain_id}" \
      --bag-duration 60 \
      --settle-seconds 10 \
      --post-roll-seconds 3 \
      --target-process-pattern lidar_localization_node \
      --record-topic /pcl_pose \
      --diagnostic-topic /alignment_status \
      --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=${hdl_no_imu_yaml} cloud_topic:=${hdl_cloud_topic} imu_topic:=${hdl_imu_topic}"
  fi

  if [[ "${resume}" != "1" || ! -f "${hdl_true_run_dir}/summary.json" ]]; then
    run_allow_failure "${repo_root}/scripts/benchmark_runner" \
      --bag-path "${hdl_bag}" \
      --output-dir "${hdl_true_run_dir}" \
      --ros-domain-id "${hdl_true_domain_id}" \
      --bag-duration 60 \
      --settle-seconds 10 \
      --post-roll-seconds 3 \
      --target-process-pattern lidar_localization_node \
      --record-topic /pcl_pose \
      --diagnostic-topic /alignment_status \
      --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=${repo_root}/param/hdl_imu_preint.yaml cloud_topic:=${hdl_cloud_topic} imu_topic:=${hdl_imu_topic}"
  fi
done

python3 - "${istanbul_run_dir}" "${hdl_false_group_dir}" "${hdl_true_group_dir}" "${summary_json}" "${summary_md}" <<'PY'
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_status_counts_and_rows(path: Path):
    counts = {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        counts[row["message"]] = counts.get(row["message"], 0) + 1
    return counts, rows


def count_csv_rows(path: Path) -> int:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return max(0, sum(1 for _ in csv.DictReader(stream)))


def median(values):
    return statistics.median(values) if values else None


def diagnostic_metrics(rows):
    align = []
    imu_active = 0
    for row in rows:
        values = json.loads(row["values_json"])
        if str(values.get("imu_prediction_active", "")).lower() == "true":
            imu_active += 1
        try:
            align.append(float(values["alignment_time_sec"]))
        except Exception:
            pass
    align.sort()
    align_median = median(align)
    align_p95 = align[min(len(align) - 1, math.ceil(len(align) * 0.95) - 1)] if align else None
    return align_median, align_p95, imu_active


def as_float(value):
    if value in (None, "", "None"):
        return None
    return float(value)


def collect_run_dirs(group_dir: Path):
    run_dirs = sorted(child for child in group_dir.iterdir() if child.is_dir() and (child / "summary.json").exists())
    if run_dirs:
        return run_dirs
    if (group_dir / "summary.json").exists():
        return [group_dir]
    raise FileNotFoundError(f"No benchmark run outputs found under {group_dir}")


def load_run_metrics(run_dir: Path):
    summary = load_json(run_dir / "summary.json")
    counts, rows = load_status_counts_and_rows(run_dir / "alignment_status.csv")
    pose_rows = count_csv_rows(run_dir / "pose_trace.csv")
    align_median, align_p95, imu_active = diagnostic_metrics(rows)
    return {
        "run_dir": str(run_dir),
        "summary": summary,
        "status_counts": counts,
        "diagnostic_rows": len(rows),
        "pose_rows": pose_rows,
        "alignment_time_median_sec": align_median,
        "alignment_time_p95_sec": align_p95,
        "imu_prediction_active_rows": imu_active,
        "fallback_events": (
            counts.get("imu_smoother_diverged_imu_disabled", 0) +
            counts.get("imu_prediction_correction_guard_imu_disabled", 0)
        ),
    }


istanbul_run_dir = Path(sys.argv[1])
hdl_false_group_dir = Path(sys.argv[2])
hdl_true_group_dir = Path(sys.argv[3])
summary_json = Path(sys.argv[4])
summary_md = Path(sys.argv[5])

istanbul_summary = load_json(istanbul_run_dir / "summary.json")
istanbul_eval = load_json(istanbul_run_dir / "trajectory_eval.json")
istanbul_counts, istanbul_rows = load_status_counts_and_rows(istanbul_run_dir / "alignment_status.csv")
istanbul_pose_rows = count_csv_rows(istanbul_run_dir / "pose_trace.csv")
istanbul_align_median, istanbul_align_p95, istanbul_imu_active = diagnostic_metrics(istanbul_rows)
istanbul_matched = int(istanbul_eval.get("matched_sample_count") or 0)
istanbul_translation = as_float(istanbul_eval.get("translation_rmse_m"))
istanbul_rotation = as_float(istanbul_eval.get("rotation_rmse_deg"))

istanbul_process_alive_check = not bool(istanbul_summary.get("target_process_died_during_run"))
istanbul_imu_inactive_check = istanbul_imu_active == 0
istanbul_pose_rows_check = istanbul_pose_rows >= 80
istanbul_matched_check = istanbul_matched >= 80
istanbul_translation_check = istanbul_translation is not None and istanbul_translation <= 6.0
istanbul_rotation_check = istanbul_rotation is not None and istanbul_rotation <= 20.0
istanbul_pass = all(
    [
        istanbul_process_alive_check,
        istanbul_imu_inactive_check,
        istanbul_pose_rows_check,
        istanbul_matched_check,
        istanbul_translation_check,
        istanbul_rotation_check,
    ]
)

hdl_false_runs = [load_run_metrics(path) for path in collect_run_dirs(hdl_false_group_dir)]
hdl_true_runs = [load_run_metrics(path) for path in collect_run_dirs(hdl_true_group_dir)]

hdl_false_pose_rows_values = [item["pose_rows"] for item in hdl_false_runs]
hdl_true_pose_rows_values = [item["pose_rows"] for item in hdl_true_runs]
hdl_false_align_median_values = [item["alignment_time_median_sec"] for item in hdl_false_runs if item["alignment_time_median_sec"] is not None]
hdl_true_align_median_values = [item["alignment_time_median_sec"] for item in hdl_true_runs if item["alignment_time_median_sec"] is not None]
hdl_false_align_p95_values = [item["alignment_time_p95_sec"] for item in hdl_false_runs if item["alignment_time_p95_sec"] is not None]
hdl_true_align_p95_values = [item["alignment_time_p95_sec"] for item in hdl_true_runs if item["alignment_time_p95_sec"] is not None]
hdl_true_imu_active_values = [item["imu_prediction_active_rows"] for item in hdl_true_runs]
hdl_true_fallback_values = [item["fallback_events"] for item in hdl_true_runs]

hdl_false_pose_rows = median(hdl_false_pose_rows_values)
hdl_true_pose_rows = median(hdl_true_pose_rows_values)
hdl_false_align_median = median(hdl_false_align_median_values)
hdl_true_align_median = median(hdl_true_align_median_values)
hdl_false_align_p95 = median(hdl_false_align_p95_values)
hdl_true_align_p95 = median(hdl_true_align_p95_values)
hdl_true_imu_active = median(hdl_true_imu_active_values)
hdl_fallback_events = max(hdl_true_fallback_values) if hdl_true_fallback_values else None

hdl_baseline_process_alive_check = all(not bool(item["summary"].get("target_process_died_during_run")) for item in hdl_false_runs)
hdl_candidate_process_alive_check = all(not bool(item["summary"].get("target_process_died_during_run")) for item in hdl_true_runs)
hdl_baseline_pose_floor_check = hdl_false_pose_rows is not None and hdl_false_pose_rows >= 250
hdl_candidate_pose_floor_check = hdl_true_pose_rows is not None and hdl_true_pose_rows >= 250
hdl_align_check = hdl_true_align_median is not None and hdl_true_align_median <= 0.10
hdl_imu_used_check = hdl_true_imu_active is not None and hdl_true_imu_active > 0
hdl_fallback_check = hdl_fallback_events is not None and hdl_fallback_events <= 1
hdl_pass = all(
    [
        hdl_baseline_process_alive_check,
        hdl_candidate_process_alive_check,
        hdl_baseline_pose_floor_check,
        hdl_candidate_pose_floor_check,
        hdl_align_check,
        hdl_imu_used_check,
        hdl_fallback_check,
    ]
)

result = {
    "overall_pass": bool(istanbul_pass and hdl_pass),
    "istanbul": {
        "pass": istanbul_pass,
        "run_dir": str(istanbul_run_dir),
        "pose_rows": istanbul_pose_rows,
        "diagnostic_rows": len(istanbul_rows),
        "alignment_time_median_sec": istanbul_align_median,
        "alignment_time_p95_sec": istanbul_align_p95,
        "imu_prediction_active_rows": istanbul_imu_active,
        "status_counts": istanbul_counts,
        "matched_sample_count": istanbul_matched,
        "translation_rmse_m": istanbul_translation,
        "rotation_rmse_deg": istanbul_rotation,
        "target_process_died_during_run": bool(istanbul_summary.get("target_process_died_during_run")),
        "target_process_died_after_signal": bool(istanbul_summary.get("target_process_died_after_signal")),
        "checks": {
            "target_process_survived_during_run": istanbul_process_alive_check,
            "no_imu_dataset_keeps_imu_prediction_inactive": istanbul_imu_inactive_check,
            "pose_rows_at_least_80": istanbul_pose_rows_check,
            "matched_sample_count_at_least_80": istanbul_matched_check,
            "translation_rmse_at_most_6_0m": istanbul_translation_check,
            "rotation_rmse_at_most_20_0deg": istanbul_rotation_check,
        },
    },
    "hdl": {
        "pass": hdl_pass,
        "baseline_run_dirs": [item["run_dir"] for item in hdl_false_runs],
        "candidate_run_dirs": [item["run_dir"] for item in hdl_true_runs],
        "baseline_run_count": len(hdl_false_runs),
        "candidate_run_count": len(hdl_true_runs),
        "false_pose_rows": hdl_false_pose_rows,
        "true_pose_rows": hdl_true_pose_rows,
        "false_alignment_time_median_sec": hdl_false_align_median,
        "true_alignment_time_median_sec": hdl_true_align_median,
        "false_alignment_time_p95_sec": hdl_false_align_p95,
        "true_alignment_time_p95_sec": hdl_true_align_p95,
        "true_imu_prediction_active_rows": hdl_true_imu_active,
        "true_fallback_events": hdl_fallback_events,
        "pose_row_ratio_true_vs_false": (hdl_true_pose_rows / hdl_false_pose_rows) if hdl_false_pose_rows else None,
        "alignment_time_median_ratio_true_vs_false": (
            hdl_true_align_median / hdl_false_align_median
            if hdl_false_align_median not in (None, 0.0) and hdl_true_align_median is not None
            else None
        ),
        "false_target_process_died_during_run_any": any(bool(item["summary"].get("target_process_died_during_run")) for item in hdl_false_runs),
        "false_target_process_died_after_signal_any": any(bool(item["summary"].get("target_process_died_after_signal")) for item in hdl_false_runs),
        "true_target_process_died_during_run_any": any(bool(item["summary"].get("target_process_died_during_run")) for item in hdl_true_runs),
        "true_target_process_died_after_signal_any": any(bool(item["summary"].get("target_process_died_after_signal")) for item in hdl_true_runs),
        "per_run": {
            "baseline": hdl_false_runs,
            "candidate": hdl_true_runs,
        },
        "checks": {
            "baseline_process_survived_during_run": hdl_baseline_process_alive_check,
            "candidate_process_survived_during_run": hdl_candidate_process_alive_check,
            "baseline_pose_rows_at_least_250": hdl_baseline_pose_floor_check,
            "candidate_pose_rows_at_least_250": hdl_candidate_pose_floor_check,
            "candidate_alignment_median_at_most_0_10s": hdl_align_check,
            "imu_prediction_was_used": hdl_imu_used_check,
            "fallback_events_at_most_one": hdl_fallback_check,
        },
    },
}

summary_json.parent.mkdir(parents=True, exist_ok=True)
summary_json.write_text(json.dumps(result, indent=2, sort_keys=False) + "\n", encoding="utf-8")

summary_md.write_text(
    "\n".join(
        [
            "# Public Regression Suite",
            "",
            f"- overall_pass: `{result['overall_pass']}`",
            "",
            "## Istanbul",
            f"- pass: `{istanbul_pass}`",
            f"- pose_rows: `{istanbul_pose_rows}`",
            f"- matched_sample_count: `{istanbul_matched}`",
            f"- translation_rmse_m: `{istanbul_translation:.3f}`" if istanbul_translation is not None else "- translation_rmse_m: `None`",
            f"- rotation_rmse_deg: `{istanbul_rotation:.3f}`" if istanbul_rotation is not None else "- rotation_rmse_deg: `None`",
            f"- imu_prediction_active_rows: `{istanbul_imu_active}`",
            "",
            "## HDL",
            f"- pass: `{hdl_pass}`",
            f"- runs: `{len(hdl_false_runs)} baseline / {len(hdl_true_runs)} candidate`",
            f"- pose_rows_median: `{hdl_false_pose_rows} -> {hdl_true_pose_rows}`",
            f"- alignment_time_median_sec_median: `{hdl_false_align_median:.6f} -> {hdl_true_align_median:.6f}`",
            f"- imu_prediction_active_rows_median(true): `{hdl_true_imu_active}`",
            f"- imu_smoother_diverged_imu_disabled_max: `{hdl_fallback_events}`",
            "",
            f"- summary_json: `{summary_json}`",
        ]
    )
    + "\n",
    encoding="utf-8",
)

print(json.dumps(result, indent=2, sort_keys=False))
sys.exit(0 if result["overall_pass"] else 1)
PY

echo "summary_json: ${summary_json}"
echo "summary_md: ${summary_md}"
