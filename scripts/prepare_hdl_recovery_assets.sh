#!/usr/bin/env bash
# Prepare HDL hdl_400_ros2 assets for the guarded global-localization recovery launch.
set -euo pipefail

usage() {
  cat <<'EOF'
Prepare HDL hdl_400_ros2 relocalization assets.

Usage:
  scripts/prepare_hdl_recovery_assets.sh [options]

Options:
  --data-dir DIR         Dataset root. Default: <workspace>/data/official/hdl_localization.
  --output-dir DIR       Generated artifact root. Default: DATA_DIR/generated.
  --reinit-threshold X   Reinit trigger threshold. Default: 0.40 for recovery experiments.
  --reinit-gap-scale X   Accepted-gap score scale in seconds. Default: 10.0.
  --reinit-seed-translation-scale-m X
                         Seed-translation scale in meters. Default: 100.0.
  --reinit-reject-streak-scale X
                         Reject-streak scale. Default: 200.0.
  --reinit-fitness-explosion-threshold X
                         Fitness explosion threshold. Default: 1000.0.
  --print-only           Print the generation and launch commands without running them.
  -h, --help             Show this help.

Outputs:
  OUTPUT_DIR/hdl_400_recovery/localization.yaml
  OUTPUT_DIR/occupancy_hdl_bbs/hdl_bbs.yaml

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

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${script_dir}/../scripts" && -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/../.." && pwd)}"
else
  repo_root="$(pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/../.." && pwd)}"
fi

data_dir="${ws_root}/data/official/hdl_localization"
output_dir=""
print_only=0
reinit_threshold="0.40"
reinit_gap_scale_sec="10.0"
reinit_seed_translation_scale_m="100.0"
reinit_reject_streak_scale="200.0"
reinit_fitness_explosion_threshold="1000.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-dir)
      shift
      data_dir="$1"
      ;;
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --reinit-threshold)
      shift
      reinit_threshold="$1"
      ;;
    --reinit-gap-scale)
      shift
      reinit_gap_scale_sec="$1"
      ;;
    --reinit-seed-translation-scale-m)
      shift
      reinit_seed_translation_scale_m="$1"
      ;;
    --reinit-reject-streak-scale)
      shift
      reinit_reject_streak_scale="$1"
      ;;
    --reinit-fitness-explosion-threshold)
      shift
      reinit_fitness_explosion_threshold="$1"
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

if [[ -z "${output_dir}" ]]; then
  output_dir="${data_dir}/generated"
fi

bag_path="${data_dir}/hdl_400_ros2"
map_path="${data_dir}/map.pcd"
recovery_dir="${output_dir}/hdl_400_recovery"
occupancy_dir="${output_dir}/occupancy_hdl_bbs"
localization_yaml="${recovery_dir}/localization.yaml"
occupancy_yaml="${occupancy_dir}/hdl_bbs.yaml"

config_script="${script_dir}/create_lidar_localization_config.py"
occupancy_script="${script_dir}/generate_occupancy_map_from_pcd.py"

missing=()
[[ -f "${map_path}" ]] || missing+=("${map_path}")
[[ -f "${bag_path}/metadata.yaml" ]] || missing+=("${bag_path}/metadata.yaml")

if [[ "${#missing[@]}" -gt 0 && "${print_only}" -eq 0 ]]; then
  echo "Missing HDL relocalization assets:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  exit 2
fi

config_args=(
  --map-path "${map_path}"
  --output "${localization_yaml}"
  --profile standalone
  --cloud-topic /velodyne_points
  --imu-topic /gpsimu_driver/imu_data
  --base-frame base_link
  --lidar-frame velodyne
  --imu-frame base_link
  --use-sim-time
  --no-publish-imu-tf
  --imu-mode off
  --initial-pose 0 0 0 0 0 0 1
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

# Crop to the hdl_400 route (x [-14.1, 7.2], y [0.0, 68.2] from a clean replay pose
# trace) plus 20 m padding: the full campus map makes a BBS query ~9.5 s, long enough
# for the moving robot to outrun the first reset seed.
occupancy_args=(
  --pcd "${map_path}"
  --resolution 0.2
  --x-min -34.0
  --x-max 27.0
  --y-min -20.0
  --y-max 88.0
  --output-dir "${occupancy_dir}"
  --map-name hdl_bbs
)

if [[ "${print_only}" -eq 0 ]]; then
  mkdir -p "${recovery_dir}" "${occupancy_dir}"
fi

echo "Localization config command:"
run_or_print "${config_script}" "${config_args[@]}"

# The standalone profile disables the scan voxel filter (Koide runs that way), but the
# proven HDL profile keeps the node default (enabled): 61k raw Velodyne points push NDT
# to ~0.9 s/scan and the localizer never settles after a reset.
if [[ "${print_only}" -eq 0 ]]; then
  python3 - "${localization_yaml}" <<'PY'
import sys
from pathlib import Path
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8"))
data["/**"]["ros__parameters"]["enable_scan_voxel_filter"] = True
path.write_text(yaml.safe_dump(data, sort_keys=True), encoding="utf-8")
PY
else
  echo "  (post-edit) set enable_scan_voxel_filter: true in ${localization_yaml}"
fi

echo "Occupancy map command:"
run_or_print "${occupancy_script}" "${occupancy_args[@]}"

launch_cmd=(
  ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py
  "localization_param_dir:=${localization_yaml}"
  "occupancy_yaml:=${occupancy_yaml}"
  "map_path:=${map_path}"
  "g2_enable_registration_scoring:=true"
  "g2_registration_refine_candidates:=true"
  "g2_registration_seed_z_m:=1.76"
  "cloud_topic:=/velodyne_points"
  "imu_topic:=/gpsimu_driver/imu_data"
  "global_frame_id:=map"
  "odom_frame_id:=odom"
  "base_frame_id:=base_link"
  "lidar_frame_id:=velodyne"
  "imu_frame_id:=base_link"
  "publish_lidar_tf:=true"
  "publish_imu_tf:=false"
  "use_sim_time:=true"
  "use_imu_preintegration:=false"
  "g2_use_cpp_backend:=true"
  "g2_nms_radius_m:=0.5"
  "supervisor_reset_default_z_m:=1.76"
  "supervisor_prefer_reset_default_z_m:=true"
  "supervisor_enable_seed_motion_compensation:=true"
  "supervisor_max_seed_speed_mps:=3.0"
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
