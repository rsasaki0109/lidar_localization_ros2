#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the lightweight public demo: Istanbul 60 s localization replay and report.

This is the star-friendly entry path. It does not run the full HDL regression suite.

Pipeline:
  check build -> (optional) fetch dataset -> replay 60 s -> trajectory eval -> report

Usage:
  scripts/run_public_demo.sh [options]

Options:
  --output-dir DIR     Working directory for run outputs.
                       Default: /tmp/lidarloc_public_demo
  --report-dir DIR     Directory for demo_report.md/json and trajectory_xy.png.
                       Default: <workspace>/artifacts/public/demo/latest
  --ros-domain-id N    ROS domain id for replay. Default: 210
  --skip-fetch         Do not download the Istanbul dataset when missing.
  --skip-build         Do not attempt colcon build when the package is missing.
  --resume             Reuse existing completed outputs when possible.
                       Prefer a full rerun (no --resume) after code changes;
                       Istanbul RMSE can vary run-to-run on identical seed/map.
  -h, --help           Show this help.
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

output_dir="/tmp/lidarloc_public_demo"
report_dir="${ws_root}/artifacts/public/demo/latest"
ros_domain_id=210
skip_fetch=0
skip_build=0
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
    --ros-domain-id)
      shift
      ros_domain_id="$1"
      ;;
    --skip-fetch)
      skip_fetch=1
      ;;
    --skip-build)
      skip_build=1
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

if ! [[ "${ros_domain_id}" =~ ^[0-9]+$ ]] || (( ros_domain_id > 232 )); then
  echo "--ros-domain-id must be an integer in [0, 232]" >&2
  exit 1
fi

demo_start_epoch="$(date +%s)"

istanbul_dataset_dir="${ws_root}/data/official/autoware_istanbul"
istanbul_bag="${istanbul_dataset_dir}/localization_rosbag"
istanbul_map="${istanbul_dataset_dir}/pointcloud_map.pcd"
istanbul_cloud_topic="/localization/util/downsample/pointcloud"
istanbul_twist_topic="/localization/twist_estimator/twist_with_covariance"
istanbul_pose_topic="/sensing/gnss/pose_with_covariance"
istanbul_base_template="${repo_root}/param/public_istanbul_60s_benchmark.yaml"

ensure_build() {
  set +u
  source "${repo_root}/scripts/setup_local_env.sh"
  set -u

  if ros2 pkg prefix lidar_localization_ros2 >/dev/null 2>&1; then
    return 0
  fi

  if [[ "${skip_build}" -eq 1 ]]; then
    echo "lidar_localization_ros2 is not built. Re-run without --skip-build." >&2
    exit 1
  fi

  echo "Building lidar_localization_ros2 in ${ws_root}/build_ws ..."
  (
    cd "${ws_root}/build_ws"
    colcon build --symlink-install --packages-up-to lidar_localization_ros2
  )
  set +u
  source "${repo_root}/scripts/setup_local_env.sh"
  set -u
}

ensure_dataset() {
  if [[ -d "${istanbul_bag}" && -f "${istanbul_map}" ]]; then
    return 0
  fi

  if [[ "${skip_fetch}" -eq 1 ]]; then
    echo "Istanbul dataset is missing and --skip-fetch was set." >&2
    echo "Expected:" >&2
    echo "  ${istanbul_bag}" >&2
    echo "  ${istanbul_map}" >&2
    exit 1
  fi

  echo "Fetching official Autoware Istanbul dataset into ${istanbul_dataset_dir} ..."
  "${repo_root}/scripts/fetch_official_autoware_istanbul_dataset.sh" \
    --output-dir "${istanbul_dataset_dir}"
}

ensure_build
ensure_dataset

for required_path in "${istanbul_bag}" "${istanbul_map}" "${istanbul_base_template}"; do
  if [[ ! -e "${required_path}" ]]; then
    echo "Required path not found: ${required_path}" >&2
    exit 1
  fi
done

inputs_dir="${output_dir}/inputs"
run_dir="${output_dir}/run"
mkdir -p "${inputs_dir}" "${run_dir}" "${report_dir}"

reference_csv="${inputs_dir}/autoware_istanbul_reference_60s.csv"
initial_pose_yaml="${inputs_dir}/autoware_istanbul_initial_pose_60s.yaml"
param_yaml="${inputs_dir}/autoware_istanbul_public_benchmark_60s.yaml"
trajectory_eval_json="${run_dir}/trajectory_eval.json"
summary_json="${run_dir}/summary.json"

if [[ "${resume}" != "1" || ! -f "${reference_csv}" || ! -f "${initial_pose_yaml}" ]]; then
  ros2 run lidar_localization_ros2 benchmark_extract_pose_reference_from_rosbag2 \
    --bag-path "${istanbul_bag}" \
    --pose-topic "${istanbul_pose_topic}" \
    --sample-topic "${istanbul_cloud_topic}" \
    --bag-duration 60 \
    --initial-pose-skip-sec 0.05 \
    --output-csv "${reference_csv}" \
    --output-initial-pose-yaml "${initial_pose_yaml}"
fi

python3 - "${istanbul_base_template}" "${initial_pose_yaml}" "${param_yaml}" "${istanbul_map}" <<'PY'
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

if [[ "${resume}" != "1" || ! -f "${summary_json}" ]]; then
  "${repo_root}/scripts/benchmark_runner" \
    --bag-path "${istanbul_bag}" \
    --output-dir "${run_dir}" \
    --ros-domain-id "${ros_domain_id}" \
    --bag-duration 60 \
    --settle-seconds 5 \
    --post-roll-seconds 1 \
    --target-process-pattern lidar_localization_node \
    --record-topic /pcl_pose \
    --record-qos-durability volatile \
    --diagnostic-topic /alignment_status \
    --system-command "ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=${param_yaml} cloud_topic:=${istanbul_cloud_topic} twist_topic:=${istanbul_twist_topic}"
fi

if [[ "${resume}" != "1" || ! -f "${trajectory_eval_json}" ]]; then
  "${repo_root}/scripts/benchmark_eval_trajectory" \
    --estimated-csv "${run_dir}/pose_trace.csv" \
    --reference-csv "${reference_csv}" \
    --output-json "${trajectory_eval_json}" \
    --max-time-diff 0.05
fi

demo_end_epoch="$(date +%s)"
elapsed_sec="$((demo_end_epoch - demo_start_epoch))"

python3 "${repo_root}/scripts/build_public_demo_report.py" \
  --estimated-csv "${run_dir}/pose_trace.csv" \
  --reference-csv "${reference_csv}" \
  --trajectory-eval-json "${trajectory_eval_json}" \
  --summary-json "${summary_json}" \
  --output-dir "${report_dir}" \
  --elapsed-sec "${elapsed_sec}"

"${repo_root}/scripts/run_public_validation_dashboard.sh" \
  --workspace-root "${ws_root}" || true

cat <<EOF

Public demo complete.

Report:
  ${report_dir}/demo_report.md
  ${report_dir}/demo_report.json
  ${report_dir}/trajectory_xy.png

Run outputs:
  ${run_dir}

Elapsed sec: ${elapsed_sec}
EOF
