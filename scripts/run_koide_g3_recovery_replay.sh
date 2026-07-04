#!/usr/bin/env bash
# Run the Koide outdoor_hard_01a G3 recovery replay with kidnap injection.
set -euo pipefail

usage() {
  cat <<'EOF'
Run Koide G3 recovery replay (120 s default) with kidnap injection.

Usage:
  scripts/run_koide_g3_recovery_replay.sh [options]

Options:
  --data-dir DIR         Koide dataset root.
  --output-dir DIR       Artifact output directory.
  --duration-sec N       Bag replay duration. Default: 120.
  --rate X               Bag replay rate. Default: 0.4.
  --ros-domain-id N      ROS domain id. Default: 181.
  --recovery-fitness-threshold X
                         Supervisor post-reset confirm threshold. Default: 1.5.
  --max-walk-candidates N
                         Candidates walked per query before re-query. Default: 4.
                         With g2_nms_radius_m 0.5 the walk candidates are near-
                         duplicates, so each walk step mostly adds seed staleness.
  --skip-prepare         Skip asset regeneration.
  --print-only           Print commands without running.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
data_dir="${repo_root}/data/public/koide_hard_localization"
output_dir=""
duration_sec="120"
play_rate="0.4"
ros_domain_id="181"
recovery_fitness_threshold="1.5"
max_walk_candidates="4"
skip_prepare=0
print_only=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-dir) shift; data_dir="$1" ;;
    --output-dir) shift; output_dir="$1" ;;
    --duration-sec) shift; duration_sec="$1" ;;
    --rate) shift; play_rate="$1" ;;
    --ros-domain-id) shift; ros_domain_id="$1" ;;
    --recovery-fitness-threshold) shift; recovery_fitness_threshold="$1" ;;
    --max-walk-candidates) shift; max_walk_candidates="$1" ;;
    --skip-prepare) skip_prepare=1 ;;
    --print-only) print_only=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ -z "${output_dir}" ]]; then
  output_dir="/tmp/lidarloc_koide_g3_recovery_regscore_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${output_dir}"

if [[ "${skip_prepare}" -eq 0 ]]; then
  echo "Preparing Koide recovery assets..."
  if [[ "${print_only}" -eq 1 ]]; then
    "${script_dir}/prepare_koide_hard_relocalization_assets.sh" \
      --data-dir "${data_dir}" \
      --output-dir "${data_dir}/generated" \
      --mode imu_preintegration \
      --print-only
  else
    "${script_dir}/prepare_koide_hard_relocalization_assets.sh" \
      --data-dir "${data_dir}" \
      --output-dir "${data_dir}/generated" \
      --mode imu_preintegration
  fi
fi

localization_yaml="${data_dir}/generated/koide_outdoor_hard_01a_recovery/localization.yaml"
occupancy_yaml="${data_dir}/generated/occupancy_outdoor_hard_bbs/outdoor_hard_bbs.yaml"
map_path="${data_dir}/map_outdoor_hard.ply"
bag_path="${data_dir}/sequences/outdoor_hard_01a"
alignment_csv="${output_dir}/alignment_status.csv"
supervisor_events_csv="${output_dir}/supervisor_events.csv"
launch_log="${output_dir}/recovery_launch.log"
bag_log="${output_dir}/bag_play.log"
diagnostic_log="${output_dir}/diagnostic_recorder.log"
pose_trace_csv="${output_dir}/pose_trace.csv"
pose_log="${output_dir}/pose_recorder.log"
injector_log="${output_dir}/kidnap_injector.log"
health_json="${output_dir}/recovery_health.json"
pose_gt_json="${output_dir}/pose_gt_check.json"
gt_traj="${data_dir}/gt/traj_lidar_outdoor_hard_01.txt"

for required in "${localization_yaml}" "${occupancy_yaml}" "${map_path}" "${bag_path}/metadata.yaml"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required asset: ${required}" >&2
    exit 2
  fi
done

launch_cmd=(
  ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py
  "localization_param_dir:=${localization_yaml}"
  "occupancy_yaml:=${occupancy_yaml}"
  "map_path:=${map_path}"
  "g2_enable_registration_scoring:=true"
  "g2_registration_seed_z_m:=-11.046818"
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
  "use_imu_preintegration:=true"
  "imu_preintegration_use_base_frame_transform:=false"
  "g2_use_cpp_backend:=true"
  # Fix staleness at reset-publish time is the dominant seed error term: 16
  # candidates scored single-threaded ran 17-21 wall-s (~7-8 bag-s at rate 0.4),
  # putting seeds 2.4-5.8 m off and into along-corridor alias capture. Scoring 8
  # candidates roughly halves that. (G2's ndt_num_threads param stays 1:
  # pclomp NDT_OMP scoring is not thread-safe with its default KDTREE search.)
  "g2_max_candidates:=8"
  "g2_nms_radius_m:=0.5"
  "g2_registration_seed_z_m:=-11.046818"
  "supervisor_reset_default_z_m:=-11.046818"
  "supervisor_prefer_reset_default_z_m:=true"
  "supervisor_enable_seed_motion_compensation:=true"
  "supervisor_max_seed_speed_mps:=3.0"
  "supervisor_recovery_fitness_threshold:=${recovery_fitness_threshold}"
  "supervisor_max_walk_candidates:=${max_walk_candidates}"
  "supervisor_event_log_csv:=${supervisor_events_csv}"
)

injector_cmd=(
  python3 "${script_dir}/inject_koide_kidnap_initialpose.py"
  # trigger_after_first_clock_sec, NOT trigger_sim_sec: the bag clock is
  # epoch-stamped (1694532825...), so an absolute trigger of 22.0 fires on the
  # first clock tick, where the not-yet-tracking localizer drops it -- every
  # archived run before 2026-07-03 silently tested natural loss, not a kidnap.
  # Repeated publishes defeat the in-flight-scan race so the kidnap sticks
  # deterministically.
  --ros-args -p use_sim_time:=true -p trigger_after_first_clock_sec:=22.0 -p z:=-11.046818
  -p repeat_count:=5 -p repeat_period_sec:=1.0
)

recorder_cmd=(
  ros2 run lidar_localization_ros2 benchmark_diagnostic_recorder
  --ros-args
  -p use_sim_time:=true
  -p topic:=/alignment_status
  -p "output_path:=${alignment_csv}"
)

# Pose trace enables ground-truth comparison of confirmed recoveries
# (gt/traj_lidar_outdoor_hard_01.txt); fitness alone cannot distinguish a
# genuine recovery from an along-route alias on this map.
pose_recorder_cmd=(
  ros2 run lidar_localization_ros2 benchmark_pose_recorder
  --ros-args
  -p use_sim_time:=true
  -p topic:=/pcl_pose
  -p "output_path:=${pose_trace_csv}"
)

bag_cmd=(
  ros2 bag play "${bag_path}"
  --clock
  --rate "${play_rate}"
  --playback-duration "${duration_sec}"
  --topics /livox/points /livox/imu /tf_static
)

if [[ "${print_only}" -eq 1 ]]; then
  printf 'output_dir=%s\n' "${output_dir}"
  printf '%q ' "${launch_cmd[@]}"; echo
  printf '%q ' "${injector_cmd[@]}"; echo
  printf '%q ' "${recorder_cmd[@]}"; echo
  printf '%q ' "${pose_recorder_cmd[@]}"; echo
  printf '%q ' "${bag_cmd[@]}"; echo
  exit 0
fi

set +u
# shellcheck source=/dev/null
source "${script_dir}/setup_local_env.sh"
# shellcheck source=/dev/null
source "${repo_root}/../install/setup.bash" 2>/dev/null || source "${repo_root}/../../install/setup.bash" 2>/dev/null || true
if [[ -f "/home/sasaki/workspace/old_~2026/lidarloc_ws/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/home/sasaki/workspace/old_~2026/lidarloc_ws/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ros_domain_id}"
echo "Output directory: ${output_dir}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# Previous replay runs may leave G2/supervisor nodes on the same domain; they steal
# service responses and produce spurious weak_candidate_rejected events.
pkill -9 -f "lidar_localization_ros2/lib/lidar_localization_ros2/" 2>/dev/null || true
pkill -9 -f "global_localization_recovery.launch.py" 2>/dev/null || true
sleep 2

pids=()
cleanup() {
  # Children are setsid leaders, so kill the whole process group of each; a
  # plain kill on the leader leaves localizer/G2/supervisor nodes alive to
  # poison later runs on the same domain.
  for pid in "${pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

setsid stdbuf -oL -eL "${launch_cmd[@]}" >"${launch_log}" 2>&1 &
pids+=("$!")
sleep 8

setsid stdbuf -oL -eL "${recorder_cmd[@]}" >"${diagnostic_log}" 2>&1 &
pids+=("$!")
setsid stdbuf -oL -eL "${pose_recorder_cmd[@]}" >"${pose_log}" 2>&1 &
pids+=("$!")
setsid stdbuf -oL -eL "${injector_cmd[@]}" >"${injector_log}" 2>&1 &
pids+=("$!")

echo "Playing bag for ${duration_sec}s at rate ${play_rate}..."
if setsid stdbuf -oL -eL "${bag_cmd[@]}" >"${bag_log}" 2>&1; then
  bag_status=0
else
  bag_status=$?
fi
sleep 15

echo "Bag play finished with status ${bag_status}"
if [[ -f "${supervisor_events_csv}" ]]; then
  echo "Supervisor events:"
  tail -20 "${supervisor_events_csv}" || true
fi

health_status=0
if [[ -f "${supervisor_events_csv}" ]]; then
  python3 "${script_dir}/check_koide_recovery_health.py" \
    --supervisor-events-csv "${supervisor_events_csv}" \
    --alignment-status-csv "${alignment_csv}" \
    --output-json "${health_json}" || health_status=$?
  cat "${health_json}"
else
  echo "No supervisor event log produced." >&2
  health_status=1
fi

pose_gt_status=0
if [[ -f "${gt_traj}" && -f "${pose_trace_csv}" ]]; then
  echo "Pose ground-truth check:"
  python3 "${script_dir}/check_recovery_pose_gt.py" \
    --pose-trace "${pose_trace_csv}" \
    --gt "${gt_traj}" \
    --output-json "${pose_gt_json}" || pose_gt_status=$?
  if [[ "${pose_gt_status}" -ne 0 ]]; then
    health_status="${pose_gt_status}"
  fi
elif [[ ! -f "${gt_traj}" ]]; then
  echo "Skipping pose GT check: ground truth not found at ${gt_traj}"
else
  echo "Skipping pose GT check: pose trace not found at ${pose_trace_csv}"
fi

exit "${health_status}"
