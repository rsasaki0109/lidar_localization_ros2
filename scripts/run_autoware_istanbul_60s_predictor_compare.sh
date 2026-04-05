#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the Autoware Istanbul 60-second predictor comparison benchmark.

This compares:
  - stable_thr8
  - twist_linear_thr6

Usage:
  scripts/run_autoware_istanbul_60s_predictor_compare.sh [options]

Options:
  --output-dir DIR       Working directory for references, configs, and run outputs.
                         Default: /tmp/lidarloc_autoware_istanbul_60s_predictor_compare
  --report-dir DIR       Directory for drift reports and comparison JSON.
                         Default: <workspace>/artifacts/public/istanbul_60s_predictor_compare
  --ros-domain-base N    Base ROS domain id for the sweep. Default: 220
  --bag-duration SEC     Benchmark window in seconds. Default: 60
  --resume               Reuse completed run directories when possible.
  -h, --help             Show this help.
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_root="$(cd "${repo_root}/.." && pwd)"

bag_path="${ws_root}/data/official/autoware_istanbul/localization_rosbag"
map_path="${ws_root}/data/official/autoware_istanbul/pointcloud_map.pcd"
cloud_topic="/localization/util/downsample/pointcloud"
twist_topic="/localization/twist_estimator/twist_with_covariance"
pose_topic="/sensing/gnss/pose_with_covariance"
sample_topic="${cloud_topic}"

output_dir="/tmp/lidarloc_autoware_istanbul_60s_predictor_compare"
report_dir="${ws_root}/artifacts/public/istanbul_60s_predictor_compare"
ros_domain_base=220
bag_duration=60
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
    --bag-duration)
      shift
      bag_duration="$1"
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

if [[ ! -d "${bag_path}" ]]; then
  echo "Bag path not found: ${bag_path}" >&2
  exit 1
fi

if [[ ! -f "${map_path}" ]]; then
  echo "Map path not found: ${map_path}" >&2
  exit 1
fi

set +u
source "${repo_root}/scripts/setup_local_env.sh"
set -u

input_dir="${output_dir}/inputs"
run_dir="${output_dir}/runs"
mkdir -p "${input_dir}" "${run_dir}" "${report_dir}"

reference_csv="${input_dir}/autoware_istanbul_reference_${bag_duration}s_stable.csv"
initial_pose_yaml="${input_dir}/autoware_istanbul_initial_pose_${bag_duration}s_stable.yaml"
base_param_yaml="${input_dir}/autoware_istanbul_ndt_${bag_duration}s_base.yaml"
compare_json="${report_dir}/comparison.json"

ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
  --bag-path "${bag_path}" \
  --pose-topic "${pose_topic}" \
  --sample-topic "${sample_topic}" \
  --bag-duration "${bag_duration}" \
  --initial-pose-skip-sec 0.05 \
  --output-csv "${reference_csv}" \
  --output-initial-pose-yaml "${initial_pose_yaml}"

python3 - "${repo_root}/param/localization.yaml" "${initial_pose_yaml}" "${base_param_yaml}" "${map_path}" <<'PY'
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

sweep_args=(
  python3 "${repo_root}/scripts/benchmark_sweep_localizer"
  --bag-path "${bag_path}"
  --reference-csv "${reference_csv}"
  --base-param-yaml "${base_param_yaml}"
  --config-json "${repo_root}/param/benchmark/autoware_istanbul_ndt_60s_predictor_compare.json"
  --output-dir "${run_dir}"
  --map-path "${map_path}"
  --cloud-topic "${cloud_topic}"
  --twist-topic "${twist_topic}"
  --bag-duration "${bag_duration}"
  --settle-seconds 5
  --post-roll-seconds 1
  --ros-domain-base "${ros_domain_base}"
)

if [[ "${resume}" == "1" ]]; then
  sweep_args+=(--resume)
fi

"${sweep_args[@]}"

python3 "${repo_root}/scripts/benchmark_compare_runs" \
  --run-dir "${run_dir}/stable_thr8" \
  --run-dir "${run_dir}/twist_linear_thr6" \
  --output-json "${compare_json}"

for name in stable_thr8 twist_linear_thr6; do
  python3 "${repo_root}/scripts/benchmark_drift_correlation_report" \
    --estimated-csv "${run_dir}/${name}/pose_trace.csv" \
    --reference-csv "${reference_csv}" \
    --diagnostic-csv "${run_dir}/${name}/alignment_status.csv" \
    --output-html "${report_dir}/${name}_drift.html" \
    --output-json "${report_dir}/${name}_drift.json"
done

echo "runs: ${run_dir}"
echo "comparison_json: ${compare_json}"
echo "report_dir: ${report_dir}"
