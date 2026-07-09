#!/usr/bin/env bash
# Run a small real-bag IMU preintegration + continuous-time deskew smoke on
# Koide's outdoor_hard_01a sequence.
set -euo pipefail

usage() {
  cat <<'EOF'
Run Koide outdoor_hard_01a LiDAR-only / IMU preintegration / deskew smoke.

Usage:
  scripts/run_koide_hard_imu_deskew_smoke.sh [options]

Options:
  --download             Download missing Koide maps, GT, and outdoor_hard_01a bag.
  --force-download       Re-download / re-unzip the sequence archive.
  --data-dir DIR         Dataset root. Default: source-tree data/public/... when available,
                         otherwise ./data/public/koide_hard_localization.
  --output-dir DIR       Output directory. Default: /tmp/lidarloc_koide_outdoor_hard_01a_imu_deskew_smoke<duration>
  --duration SEC         Bag playback duration. Default: 20.
  --ros-domain-id ID     ROS_DOMAIN_ID for replay. Default: ROS_DOMAIN_ID env, or 195.
  --mode NAME            Repeat to run a subset: lidar_only, imu_preintegration, deskew.
  --print-only           Write configs/run_plan and print commands without running ROS.
  --stop-on-failure      Stop at the first failed mode instead of collecting all reports.
  -h, --help             Show this help.

Before running:
  source /opt/ros/<distro>/setup.bash
  source install/setup.bash

The script writes comparison.md under the output directory.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${script_dir}/../scripts" && -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
else
  repo_root="$(pwd)"
fi

data_dir="${repo_root}/data/public/koide_hard_localization"
output_dir=""
duration="20"
ros_domain_id="${ROS_DOMAIN_ID:-195}"
download=0
force_download=0
print_only=0
continue_on_failure=1
declare -a modes=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --download)
      download=1
      ;;
    --force-download)
      download=1
      force_download=1
      ;;
    --data-dir)
      shift
      data_dir="$1"
      ;;
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --duration)
      shift
      duration="$1"
      ;;
    --ros-domain-id)
      shift
      ros_domain_id="$1"
      ;;
    --mode)
      shift
      modes+=("$1")
      ;;
    --print-only)
      print_only=1
      ;;
    --stop-on-failure)
      continue_on_failure=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

duration_label="${duration//./p}"
if [[ -z "${output_dir}" ]]; then
  output_dir="/tmp/lidarloc_koide_outdoor_hard_01a_imu_deskew_smoke${duration_label}"
fi

sequence="outdoor_hard_01a"
bag_path="${data_dir}/sequences/${sequence}"
map_path="${data_dir}/map_outdoor_hard.ply"
gt_path="${data_dir}/gt/traj_lidar_${sequence}.txt"
benchmark_dir="${data_dir}/benchmark/${sequence}"
reference_csv="${benchmark_dir}/reference.csv"
initial_pose_yaml="${benchmark_dir}/initial_pose.yaml"

fetch_script="${script_dir}/fetch_koide_hard_pointcloud_localization_dataset.sh"
converter_script="${script_dir}/tum_trajectory_to_pose_reference_csv.py"
comparison_script="${script_dir}/run_lidar_localization_imu_comparison.py"

if [[ "${download}" -eq 1 ]]; then
  fetch_args=(
    --output-dir "${data_dir}"
    --with-maps
    --with-gt
    --sequence "${sequence}"
  )
  if [[ "${force_download}" -eq 1 ]]; then
    fetch_args+=(--force-sequences)
  fi
  "${fetch_script}" "${fetch_args[@]}"
fi

missing=()
[[ -f "${map_path}" ]] || missing+=("${map_path}")
[[ -f "${bag_path}/metadata.yaml" ]] || missing+=("${bag_path}/metadata.yaml")
[[ -f "${gt_path}" || -f "${reference_csv}" ]] || missing+=("${gt_path}")

if [[ "${#missing[@]}" -gt 0 && "${print_only}" -eq 0 ]]; then
  echo "Missing Koide smoke assets:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "" >&2
  echo "Fetch them with:" >&2
  echo "  ${BASH_SOURCE[0]} --download" >&2
  exit 2
fi

if [[ (! -f "${reference_csv}" || ! -f "${initial_pose_yaml}") && -f "${gt_path}" ]]; then
  mkdir -p "${benchmark_dir}"
  "${converter_script}" \
    --input "${gt_path}" \
    --output-csv "${reference_csv}" \
    --output-initial-pose-yaml "${initial_pose_yaml}" \
    --initial-pose-skip-sec 0.05
fi

yaml_value() {
  local key="$1"
  local path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${path}"
}

initial_pose=(
  "-86.03759"
  "-8.856023000000004"
  "-11.046818"
  "-0.011096"
  "-0.013567"
  "-0.655961"
  "0.754591"
)
if [[ -f "${initial_pose_yaml}" ]]; then
  initial_pose=(
    "$(yaml_value initial_pose_x "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_y "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_z "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_qx "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_qy "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_qz "${initial_pose_yaml}")"
    "$(yaml_value initial_pose_qw "${initial_pose_yaml}")"
  )
fi

mode_args=()
for mode in "${modes[@]}"; do
  mode_args+=(--mode "${mode}")
done

failure_args=()
if [[ "${continue_on_failure}" -eq 1 ]]; then
  failure_args+=(--continue-on-failure)
fi

print_args=()
if [[ "${print_only}" -eq 1 ]]; then
  print_args+=(--print-only)
elif ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not on PATH. Source ROS 2 and this workspace first." >&2
  exit 2
fi

"${comparison_script}" \
  --bag-path "${bag_path}" \
  --map-path "${map_path}" \
  --reference-csv "${reference_csv}" \
  --profile standalone \
  "${mode_args[@]}" \
  --cloud-topic /livox/points \
  --imu-topic /livox/imu \
  --lidar-frame livox_frame \
  --imu-frame livox_frame \
  --base-frame livox_frame \
  --output-dir "${output_dir}" \
  --initial-pose "${initial_pose[@]}" \
  --bag-duration "${duration}" \
  --settle-seconds 2 \
  --post-roll-seconds 1 \
  --ros-domain-id "${ros_domain_id}" \
  --min-imu-active-ratio 0.1 \
  --min-imu-seed-source-ratio 0.1 \
  --min-deskew-applied-ratio 0.1 \
  --scan-time-range-max-duration-ratio 4.0 \
  --enable-open-loop-strict-score-threshold \
  --open-loop-strict-min-accepted-gap-sec 1.0 \
  --open-loop-strict-min-seed-translation-m 0.5 \
  --open-loop-strict-score-threshold 2.0 \
  "${failure_args[@]}" \
  "${print_args[@]}"
