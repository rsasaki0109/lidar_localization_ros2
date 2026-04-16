#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_nav2_reinit_supervisor_sweep.sh [options]

Options:
  --output-dir PATH             default: artifacts/public/nav2_reinit_supervisor_burst_sweep_120
  --map-yaml PATH               default: official Istanbul Nav2 map yaml
  --pcd-map-path PATH           default: official Istanbul pointcloud map
  --bag-path PATH               default: official Istanbul localization rosbag
  --post-goal-observe-sec SEC   default: 120
  --ros-domain-base VALUE       default: 141
  --resume                      skip variants whose alignment CSV already exists
  --help

This runs three configured-initial-pose supervisor variants on the same
Nav2 replay scenario:

  - publish_count=1, cooldown=5.0
  - publish_count=3, cooldown=5.0
  - publish_count=5, cooldown=5.0

After all runs complete, it writes:

  - comparison.json
  - summary.md

using compare_nav2_reinit_supervisor_runs.py.
EOF
}

script_dir="$(cd "$(dirname "$0")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

if [[ ! -f "${repo_root}/scripts/setup_local_env.sh" ]]; then
  printf 'repo root does not look valid: %s\n' "${repo_root}" >&2
  exit 2
fi

output_dir="${ws_root}/artifacts/public/nav2_reinit_supervisor_burst_sweep_120"
map_yaml="${ws_root}/artifacts/public/autoware_istanbul_60s_nav2_map/istanbul_60s_nav2_map.yaml"
pcd_map_path="${ws_root}/data/official/autoware_istanbul/pointcloud_map.pcd"
bag_path="${ws_root}/data/official/autoware_istanbul/localization_rosbag"
post_goal_observe_sec="120"
ros_domain_base="141"
resume="0"

while (($# > 0)); do
  case "$1" in
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --map-yaml)
      map_yaml="$2"
      shift 2
      ;;
    --pcd-map-path)
      pcd_map_path="$2"
      shift 2
      ;;
    --bag-path)
      bag_path="$2"
      shift 2
      ;;
    --post-goal-observe-sec)
      post_goal_observe_sec="$2"
      shift 2
      ;;
    --ros-domain-base)
      ros_domain_base="$2"
      shift 2
      ;;
    --resume)
      resume="1"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${output_dir}"

if [[ ! -f "${map_yaml}" ]]; then
  printf 'map_yaml does not exist: %s\n' "${map_yaml}" >&2
  exit 2
fi
if [[ ! -f "${pcd_map_path}" ]]; then
  printf 'pcd_map_path does not exist: %s\n' "${pcd_map_path}" >&2
  exit 2
fi
if [[ ! -e "${bag_path}" ]]; then
  printf 'bag_path does not exist: %s\n' "${bag_path}" >&2
  exit 2
fi

set +u
source "${repo_root}/scripts/setup_local_env.sh"
set -u

declare -a variant_labels=(
  "configured_pose_count1"
  "configured_pose_count3"
  "configured_pose_count5"
)
declare -a variant_publish_counts=("1" "3" "5")
declare -a failed_labels=()

for index in "${!variant_labels[@]}"; do
  label="${variant_labels[$index]}"
  publish_count="${variant_publish_counts[$index]}"
  log_dir="${output_dir}/${label}"
  ros_domain_id=$((ros_domain_base + index))
  alignment_csv="${log_dir}/alignment_status.csv"

  if [[ "${resume}" == "1" && -f "${alignment_csv}" ]]; then
    printf 'Skipping %s (resume)\n' "${label}"
    continue
  fi

  rm -rf "${log_dir}"
  printf 'Running %s (publish_count=%s, ros_domain_id=%s)\n' \
    "${label}" "${publish_count}" "${ros_domain_id}"

  if ! ros2 run lidar_localization_ros2 run_nav2_replay_smoke \
    --ros-domain-id "${ros_domain_id}" \
    --log-dir "${log_dir}" \
    --map-yaml "${map_yaml}" \
    --pcd-map-path "${pcd_map_path}" \
    --bag-path "${bag_path}" \
    --initial-pose-x 66459.42927936181 \
    --initial-pose-y 43620.21239264418 \
    --initial-pose-z 41.53427549514439 \
    --initial-pose-qx -0.004873999337462572 \
    --initial-pose-qy 0.005545974206531707 \
    --initial-pose-qz 0.959846363599952 \
    --initial-pose-qw -0.2804290366287121 \
    --goal-forward-m 0.5 \
    --goal-timeout-sec 90 \
    --launch-timeout-sec 90 \
    --post-goal-observe-sec "${post_goal_observe_sec}" \
    --enable-reinitialization-supervisor \
    --reinitialization-supervisor-publish-count "${publish_count}" \
    --reinitialization-supervisor-publish-interval-sec 0.2 \
    --reinitialization-supervisor-cooldown-sec 5.0; then
    printf 'Variant failed: %s\n' "${label}" >&2
    failed_labels+=("${label}")
  fi
done

compare_cmd=(
  python3 "${ws_root}/repo/scripts/compare_nav2_reinit_supervisor_runs.py"
  --run "configured_pose_count1=${output_dir}/configured_pose_count1"
  --run "configured_pose_count3=${output_dir}/configured_pose_count3"
  --run "configured_pose_count5=${output_dir}/configured_pose_count5"
  --output-json "${output_dir}/comparison.json"
  --output-md "${output_dir}/summary.md"
)

"${compare_cmd[@]}"

printf 'comparison_json: %s\n' "${output_dir}/comparison.json"
printf 'summary_md: %s\n' "${output_dir}/summary.md"

if ((${#failed_labels[@]} > 0)); then
  printf 'failed_variants: %s\n' "${failed_labels[*]}" >&2
  exit 1
fi
