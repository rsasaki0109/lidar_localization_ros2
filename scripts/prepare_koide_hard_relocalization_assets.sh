#!/usr/bin/env bash
# Prepare Koide outdoor_hard_01a assets for the guarded global-localization
# recovery launch.
set -euo pipefail

usage() {
  cat <<'EOF'
Prepare Koide outdoor_hard_01a relocalization assets.

Usage:
  scripts/prepare_koide_hard_relocalization_assets.sh [options]

Options:
  --download             Download missing Koide maps, GT, and outdoor_hard_01a bag.
  --force-download       Re-download / re-unzip the sequence archive.
  --data-dir DIR         Dataset root. Default: source-tree data/public/... when available,
                         otherwise ./data/public/koide_hard_localization.
  --output-dir DIR       Generated artifact root. Default: DATA_DIR/generated.
  --mode NAME            lidar_only, imu_preintegration, or deskew. Default: imu_preintegration.
  --no-initial-pose      Do not embed the GT initial pose; useful for kidnapped-start tests.
  --reinit-threshold X   Reinit trigger threshold. Default: 0.40 for recovery experiments.
  --reinit-gap-scale X   Accepted-gap score scale in seconds. Default: 10.0.
  --g2-yaw-step-deg X    BBS yaw sampling step. Default: 10.0.
  --g2-max-scan-points N BBS scan point cap. Default: 256.
  --g2-nms-radius-m X    BBS candidate NMS radius. Default: 0.5 to keep
                         near-by/yaw-alternative hypotheses for Koide recovery.
  --supervisor-settle-timeout-sec X
                         Post-reset recovery wait. Default: 8.0; the localizer
                         now consumes /initialpose promptly, so fresh re-query
                         beats walking a stale candidate list.
  --supervisor-recovery-confirmation-samples N
                         Consecutive low-fitness samples required for recovery.
                         Default: 3.
  --supervisor-max-walk-candidates N
                         Candidate ranks to try per query before re-querying. Default: 1.
  --enable-seed-motion-compensation
                         Forward-compensate G2 seeds by query latency. Default: enabled.
  --disable-seed-motion-compensation
                         Disable seed motion compensation in the printed launch command.
  --supervisor-max-seed-speed-mps X
                         Reject faster inferred seed velocities. Default: 3.0.
  --supervisor-max-seed-latency-sec X
                         Clamp seed-motion latency compensation. Default: 30.0.
  --print-only           Print the generation and launch commands without running them.
  -h, --help             Show this help.

Outputs:
  OUTPUT_DIR/koide_outdoor_hard_01a_recovery/localization.yaml
  OUTPUT_DIR/occupancy_outdoor_hard_bbs/outdoor_hard_bbs.yaml

After this script succeeds, source ROS 2 + the workspace and run the printed
global_localization_recovery.launch.py command, then replay the bag with --clock.
EOF
}

quote_command() {
  local quoted=()
  local arg
  for arg in "$@"; do
    quoted+=("$(printf '%q' "${arg}")")
  done
  printf '%s\n' "${quoted[*]}"
}

run_or_print() {
  if [[ "${print_only}" -eq 1 ]]; then
    echo "  $(quote_command "$@")"
  else
    "$@"
  fi
}

yaml_value() {
  local key="$1"
  local path="$2"
  awk -v key="${key}" '$1 == key ":" {print $2; exit}' "${path}"
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${script_dir}/../scripts" && -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
else
  repo_root="$(pwd)"
fi

data_dir="${repo_root}/data/public/koide_hard_localization"
output_dir=""
mode="imu_preintegration"
download=0
force_download=0
print_only=0
embed_initial_pose=1
reinit_threshold="0.40"
reinit_gap_scale_sec="10.0"
reinit_seed_translation_scale_m="100.0"
reinit_reject_streak_scale="200.0"
reinit_fitness_explosion_threshold="1000.0"
g2_yaw_step_deg="10.0"
g2_max_scan_points="256"
g2_nms_radius_m="0.5"
supervisor_query_timeout_sec="20.0"
supervisor_settle_timeout_sec="8.0"
supervisor_recovery_confirmation_samples="3"
supervisor_max_walk_candidates="1"
supervisor_enable_seed_motion_compensation="true"
supervisor_max_seed_speed_mps="3.0"
supervisor_max_seed_latency_sec="30.0"

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
    --mode)
      shift
      mode="$1"
      ;;
    --no-initial-pose)
      embed_initial_pose=0
      ;;
    --reinit-threshold)
      shift
      reinit_threshold="$1"
      ;;
    --reinit-gap-scale)
      shift
      reinit_gap_scale_sec="$1"
      ;;
    --g2-yaw-step-deg)
      shift
      g2_yaw_step_deg="$1"
      ;;
    --g2-max-scan-points)
      shift
      g2_max_scan_points="$1"
      ;;
    --g2-nms-radius-m)
      shift
      g2_nms_radius_m="$1"
      ;;
    --supervisor-max-walk-candidates)
      shift
      supervisor_max_walk_candidates="$1"
      ;;
    --supervisor-settle-timeout-sec)
      shift
      supervisor_settle_timeout_sec="$1"
      ;;
    --supervisor-recovery-confirmation-samples)
      shift
      supervisor_recovery_confirmation_samples="$1"
      ;;
    --enable-seed-motion-compensation)
      supervisor_enable_seed_motion_compensation="true"
      ;;
    --disable-seed-motion-compensation)
      supervisor_enable_seed_motion_compensation="false"
      ;;
    --supervisor-max-seed-speed-mps)
      shift
      supervisor_max_seed_speed_mps="$1"
      ;;
    --supervisor-max-seed-latency-sec)
      shift
      supervisor_max_seed_latency_sec="$1"
      ;;
    --print-only)
      print_only=1
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

case "${mode}" in
  lidar_only|imu_preintegration|deskew)
    ;;
  *)
    echo "Unknown --mode: ${mode}" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ -z "${output_dir}" ]]; then
  output_dir="${data_dir}/generated"
fi

sequence="outdoor_hard_01a"
bag_path="${data_dir}/sequences/${sequence}"
map_path="${data_dir}/map_outdoor_hard.ply"
gt_path="${data_dir}/gt/traj_lidar_${sequence}.txt"
benchmark_dir="${data_dir}/benchmark/${sequence}"
reference_csv="${benchmark_dir}/reference.csv"
initial_pose_yaml="${benchmark_dir}/initial_pose.yaml"
recovery_dir="${output_dir}/koide_outdoor_hard_01a_recovery"
occupancy_dir="${output_dir}/occupancy_outdoor_hard_bbs"
localization_yaml="${recovery_dir}/localization.yaml"
occupancy_yaml="${occupancy_dir}/outdoor_hard_bbs.yaml"

fetch_script="${script_dir}/fetch_koide_hard_pointcloud_localization_dataset.sh"
converter_script="${script_dir}/tum_trajectory_to_pose_reference_csv.py"
config_script="${script_dir}/create_lidar_localization_config.py"
occupancy_script="${script_dir}/generate_occupancy_map_from_pcd.py"

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
  echo "Dataset fetch command:"
  run_or_print "${fetch_script}" "${fetch_args[@]}"
fi

missing=()
[[ -f "${map_path}" ]] || missing+=("${map_path}")
[[ -f "${bag_path}/metadata.yaml" ]] || missing+=("${bag_path}/metadata.yaml")
[[ -f "${gt_path}" || -f "${reference_csv}" ]] || missing+=("${gt_path}")

if [[ "${#missing[@]}" -gt 0 && "${print_only}" -eq 0 ]]; then
  echo "Missing Koide relocalization assets:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "" >&2
  echo "Fetch them with:" >&2
  echo "  ${BASH_SOURCE[0]} --download" >&2
  exit 2
fi

if [[ (! -f "${reference_csv}" || ! -f "${initial_pose_yaml}") && -f "${gt_path}" ]]; then
  if [[ "${print_only}" -eq 0 ]]; then
    mkdir -p "${benchmark_dir}"
  fi
  echo "Reference conversion command:"
  run_or_print "${converter_script}" \
    --input "${gt_path}" \
    --output-csv "${reference_csv}" \
    --output-initial-pose-yaml "${initial_pose_yaml}" \
    --initial-pose-skip-sec 0.05
elif [[ "${print_only}" -eq 1 ]]; then
  echo "Reference conversion command, if ${reference_csv} is missing:"
  echo "  $(quote_command "${converter_script}" \
    --input "${gt_path}" \
    --output-csv "${reference_csv}" \
    --output-initial-pose-yaml "${initial_pose_yaml}" \
    --initial-pose-skip-sec 0.05)"
fi

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

imu_mode="preintegration"
use_imu_preintegration="true"
use_continuous_time_deskew="false"
require_cloud_time_field=()
deskew_args=()
case "${mode}" in
  lidar_only)
    imu_mode="off"
    use_imu_preintegration="false"
    ;;
  deskew)
    use_continuous_time_deskew="true"
    require_cloud_time_field+=(--require-cloud-time-field)
    deskew_args+=(--enable-continuous-time-deskew)
    ;;
esac

config_args=(
  --map-path "${map_path}"
  --output "${localization_yaml}"
  --profile standalone
  --cloud-topic /livox/points
  --imu-topic /livox/imu
  --lidar-frame livox_frame
  --imu-frame livox_frame
  --base-frame livox_frame
  --use-sim-time
  --no-publish-lidar-tf
  --no-publish-imu-tf
  --imu-mode "${imu_mode}"
  "${deskew_args[@]}"
  "${require_cloud_time_field[@]}"
  --score-threshold 6.0
  --enable-open-loop-strict-score-threshold
  --open-loop-strict-min-accepted-gap-sec 1.0
  --open-loop-strict-min-seed-translation-m 0.5
  --open-loop-strict-score-threshold 2.0
  --reinitialization-trigger-threshold "${reinit_threshold}"
  --reinitialization-trigger-gap-scale-sec "${reinit_gap_scale_sec}"
  --reinitialization-trigger-seed-translation-scale-m "${reinit_seed_translation_scale_m}"
  --reinitialization-trigger-reject-streak-scale "${reinit_reject_streak_scale}"
  --reinitialization-trigger-fitness-explosion-threshold "${reinit_fitness_explosion_threshold}"
  --overwrite
)
if [[ "${embed_initial_pose}" -eq 1 ]]; then
  config_args+=(--initial-pose "${initial_pose[@]}")
fi

occupancy_args=(
  --pcd "${map_path}"
  --reference-csv "${reference_csv}"
  --route-padding-m 20.0
  --resolution 0.2
  --output-dir "${occupancy_dir}"
  --map-name outdoor_hard_bbs
)

if [[ "${print_only}" -eq 0 ]]; then
  mkdir -p "${recovery_dir}" "${occupancy_dir}"
fi

echo "Localization config command:"
run_or_print "${config_script}" "${config_args[@]}"

echo "Occupancy map command:"
run_or_print "${occupancy_script}" "${occupancy_args[@]}"

launch_cmd=(
  ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py
  "localization_param_dir:=${localization_yaml}"
  "occupancy_yaml:=${occupancy_yaml}"
  "cloud_topic:=/livox/points"
  "imu_topic:=/livox/imu"
  "global_frame_id:=map"
  "odom_frame_id:=odom"
  "base_frame_id:=livox_frame"
  "lidar_frame_id:=livox_frame"
  "imu_frame_id:=livox_frame"
  "publish_lidar_tf:=false"
  "publish_imu_tf:=false"
  "use_sim_time:=true"
  "use_imu_preintegration:=${use_imu_preintegration}"
  "imu_preintegration_use_base_frame_transform:=false"
  "use_continuous_time_deskew:=${use_continuous_time_deskew}"
  "continuous_time_deskew_reference_time_sec:=0.0"
  "g2_use_cpp_backend:=true"
  "g2_angular_resolution_deg:=${g2_yaw_step_deg}"
  "g2_max_scan_points:=${g2_max_scan_points}"
  "g2_nms_radius_m:=${g2_nms_radius_m}"
  "supervisor_query_timeout_sec:=${supervisor_query_timeout_sec}"
  "supervisor_settle_timeout_sec:=${supervisor_settle_timeout_sec}"
  "supervisor_recovery_confirmation_samples:=${supervisor_recovery_confirmation_samples}"
  "supervisor_max_walk_candidates:=${supervisor_max_walk_candidates}"
  "supervisor_enable_seed_motion_compensation:=${supervisor_enable_seed_motion_compensation}"
  "supervisor_max_seed_speed_mps:=${supervisor_max_seed_speed_mps}"
  "supervisor_max_seed_latency_sec:=${supervisor_max_seed_latency_sec}"
)

echo ""
echo "Prepared paths:"
echo "  localization_yaml: ${localization_yaml}"
echo "  occupancy_yaml:    ${occupancy_yaml}"
echo "  bag_path:          ${bag_path}"
echo ""
echo "Run recovery launch:"
echo "  $(quote_command "${launch_cmd[@]}")"
echo ""
echo "Replay bag:"
echo "  $(quote_command ros2 bag play "${bag_path}" --clock)"
