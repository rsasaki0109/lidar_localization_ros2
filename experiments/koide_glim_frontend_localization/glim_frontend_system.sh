#!/usr/bin/env bash
# External-LIO-front-end system wrapper: a live GLIM ROS node supplies a
# continuous odom -> livox_frame TF (see param/odometry/glim_koide_outdoor_gicp6500_livetf,
# where map_frame_id is renamed to "glim_world" so GLIM's own map -> odom publish
# cannot collide with the localizer's), then the core lidar_localization node
# (+ G2 global_localization_node + G3 reinitialization_supervisor_node, all via
# global_localization_recovery.launch.py) runs with enable_map_odom_tf:=true so it
# looks that TF up and publishes map -> odom = map -> base_link(NDT) * inverse(odom -> base_link).
#
# 2026-07-17 odom-bridge recovery (follow-up to the postmortem below): with
# enable_map_odom_tf, the C++ recovery supervisor now keeps the last ACCEPTED
# map -> odom offset alive (re-stamped) through a dropout instead of letting TF
# go stale (see republishFrozenMapToOdomTransform in lidar_localization_component.cpp),
# and G3 (supervisor_use_odom_bridge_candidate:=true below) tries a candidate
# from that live TF composition -- map -> odom(last accepted) x
# odom -> base_link(now), i.e. GLIM's own low-drift odometry -- before every G2
# BBS query. Zero query latency and (per GLIM's ~0.5-0.8% relative odom drift)
# well under 1 m of error for a dropout on the order of tens of seconds, vs.
# BBS's ~5-25 s query + occupancy-grid-resolution accuracy. Falls back to BBS
# after supervisor_odom_bridge_max_attempts unconfirmed tries per episode.
#
# Intended as a `system: {command: ...}` entry for scripts/benchmark_from_manifest
# (mode: run). Unlike the localizer-only manifest path, `system:` mode never
# generates a localization.yaml for us (benchmark_from_manifest skips
# build_localization_yaml whenever `system` is present) -- this script builds its
# own merged localization.yaml at runtime, mirroring that helper's merge order
# (base param yaml -> initial_pose_yaml -> overrides) so the G2/G3 launch still
# gets score_threshold / voxel / scan-range overrides that
# global_localization_recovery.launch.py does not expose as launch arguments.
#
# 2026-07-17 postmortem (full 380s run diverged: RMSE 166.7m, 139/1248 rows
# stuck in reinitialization_requested, 0 recovered):
#   The suspected root cause going in was a TF dual-parent conflict on `odom`
#   (GLIM's rviz_viewer publishes <map_frame_id> -> odom every raw-odometry
#   frame; the localizer's enable_map_odom_tf publishes map -> odom). That
#   hypothesis did NOT hold up:
#     - No TF_REPEATED_DATA / authority-conflict warnings appear anywhere in
#       any of the 4 full-length benchmark logs (plain NDT, smallgicp,
#       glimfrontend TF-only, glimfrontend odomseed).
#     - The "odom" lookupTransform "does not exist" warnings fire at the same
#       rate per row in the healthy 60s smoke (43/158 rows, RMSE 0.21m) as in
#       the diverged 380s run (274/1248 rows) -- not correlated with
#       divergence, and they are concentrated in the first ~80s (cold start)
#       and around G3's reset-retry cycles, not at the actual lock-loss onset.
#     - The actual lock-loss onset (~t=76-105s, accepted_gap_sec climbing
#       from ~0 to 30s+) happens during a window with almost no TF warnings
#       at all.
#     - The SAME "fitness-exploded reinit that never recovers" failure mode
#       appears in the plain-NDT baseline (outdoor_hard_01a_ndt, no GLIM
#       involved at all, onset t=180.8s) and smallgicp (onset t=289.3s).
#   The real regression: this GLIM-front-end wiring set
#   `use_imu_preintegration: false` (reasoning "GLIM replaces IMU
#   preintegration"), but the TF-only variant never actually feeds GLIM's
#   odometry into prediction (enable_map_odom_tf only affects the TF
#   *publish*, not the NDT seed -- prediction is pure
#   predict_pose_from_previous_delta / twist_prediction, self-referential on
#   the localizer's own accepted matches). So this variant runs with a
#   strictly *weaker* drift-bridging mechanism than the working NDT baseline
#   (which keeps use_imu_preintegration:=true) -- zero prediction help once
#   NDT starts rejecting scans in a hard segment, so the pose free-runs on
#   stale twist, the local_map_crop walks off real map data, and G2/G3 can
#   never re-anchor it. Fix applied: re-enable use_imu_preintegration:=true
#   here (independent mechanism from the GLIM odom TF plumbing -- imu
#   preintegration only needs /livox/imu, and
#   imu_preintegration_use_base_frame_transform stays false so it needs no
#   TF lookup at all) while keeping enable_map_odom_tf:=true / the GLIM
#   odom -> livox_frame TF architecture intact.
#
# Env:
#   OUT_DIR         Required. Output/log directory (benchmark_runner's --output-dir).
#   ROS_DOMAIN_ID   Required. Domain id shared by the GLIM container and the ROS 2 graph.
#   GLIM_IMAGE      Optional. Docker image. Default: lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded.
#   REPO_ROOT       Optional. lidar_localization_ros2 source checkout. Default: the
#                   known workspace path below.
#   USE_ODOM_TF_PREDICTION / PUBLISH_BRIDGE_POSE_WHEN_LOST /
#   SUPERVISOR_USE_ODOM_BRIDGE_CANDIDATE
#                   Optional true/false A/B switches. Defaults: true.
#   LOCALIZER_SCORE_THRESHOLD
#                   Optional NDT acceptance threshold. Defaults to 6.0 so the
#                   bridge-off A/B baseline remains directly comparable.
#   ENABLE_BORDERLINE_SEED_REJECTION_GATE
#                   Optional true/false override. Defaults true to preserve
#                   the baseline; the odom-guarded tuned runs disable it.
set -euo pipefail

: "${OUT_DIR:?OUT_DIR must be set (output/log directory)}"
: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID must be set}"

# CRITICAL: FastDDS shared-memory transport silently drops ALL data between the
# GLIM container and host processes (both directions) even with
# --network=host --ipc=host -- every pre-2026-07-17T23:00 "glimfrontend" run
# actually ran with GLIM receiving zero points and the host receiving zero
# GLIM TF/odom (glim_live.log stuck at module-load, "odom does not exist"
# lookup warnings for the whole run). Forcing UDPv4 on BOTH sides fixes it
# (verified live: host receives /glim_ros/odom + odom->livox_frame TF).
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

repo_root="${REPO_ROOT:-/home/sasaki/workspace/old_~2026/lidarloc_ws/src/lidar_localization_ros2}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
glim_image="${GLIM_IMAGE:-lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded}"
use_odom_tf_prediction="${USE_ODOM_TF_PREDICTION:-true}"
publish_bridge_pose_when_lost="${PUBLISH_BRIDGE_POSE_WHEN_LOST:-true}"
supervisor_use_odom_bridge_candidate="${SUPERVISOR_USE_ODOM_BRIDGE_CANDIDATE:-true}"
localizer_score_threshold="${LOCALIZER_SCORE_THRESHOLD:-6.0}"
odom_correction_guard_translation_m="${ODOM_CORRECTION_GUARD_TRANSLATION_M:-2.0}"
enable_borderline_seed_rejection_gate="${ENABLE_BORDERLINE_SEED_REJECTION_GATE:-true}"
enable_map_odom_anchor_fitness_gate="${ENABLE_MAP_ODOM_ANCHOR_FITNESS_GATE:-true}"
map_odom_anchor_max_fitness="${MAP_ODOM_ANCHOR_MAX_FITNESS:-1.5}"
map_odom_anchor_max_correction_rotation_deg="${MAP_ODOM_ANCHOR_MAX_CORRECTION_ROTATION_DEG:-10.0}"
odom_tf_constraint_mode="${ODOM_TF_CONSTRAINT_MODE:-height_only}"

for value in "${use_odom_tf_prediction}" "${publish_bridge_pose_when_lost}" \
  "${supervisor_use_odom_bridge_candidate}" "${enable_borderline_seed_rejection_gate}"; do
  case "${value}" in
    true|false) ;;
    *)
      echo "Bridge A/B switches must be true or false (got: ${value})" >&2
      exit 2
      ;;
  esac
done

case "${enable_map_odom_anchor_fitness_gate}" in
  true|false) ;;
  *)
    echo "ENABLE_MAP_ODOM_ANCHOR_FITNESS_GATE must be true or false (got: ${enable_map_odom_anchor_fitness_gate})" >&2
    exit 2
    ;;
esac

case "${odom_tf_constraint_mode}" in
  planar|height_only|none) ;;
  *)
    echo "ODOM_TF_CONSTRAINT_MODE must be planar, height_only, or none (got: ${odom_tf_constraint_mode})" >&2
    exit 2
    ;;
esac

data_dir="${repo_root}/data/public/koide_hard_localization"
occupancy_yaml="${data_dir}/generated/occupancy_outdoor_hard_bbs/outdoor_hard_bbs.yaml"
map_path="${data_dir}/map_outdoor_hard.ply"
base_param_yaml="${repo_root}/param/nav2_ndt_urban.yaml"
# Per-sequence initial pose: manifests MUST pass INITIAL_POSE_YAML for any
# sequence other than 01a (before 2026-07-17T23:00 this was hardcoded to 01a,
# so the 01b/02a/02b glimfrontend runs started from the wrong pose).
initial_pose_yaml="${INITIAL_POSE_YAML:-${data_dir}/generated/localization_gif_benchmarks/assets/outdoor_hard_01a_initial_pose.yaml}"
glim_config_dir="${repo_root}/param/odometry/glim_koide_outdoor_gicp6500_livetf"
glim_odom_csv="${GLIM_ODOM_CSV:-}"

for required in "${occupancy_yaml}" "${map_path}" "${base_param_yaml}" \
  "${initial_pose_yaml}" "${glim_config_dir}/config_ros.json"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required asset: ${required}" >&2
    exit 2
  fi
done

mkdir -p "${OUT_DIR}"
generated_dir="${OUT_DIR}/generated"
mkdir -p "${generated_dir}"
localization_yaml="${generated_dir}/localization.yaml"
glim_live_log="${OUT_DIR}/glim_live.log"

# Merge nav2_ndt_urban.yaml + the outdoor_hard_01a initial pose + the
# outdoor_hard full-sequence overrides (score_threshold 6.0, voxel 0.5, scan
# 1-100 m; score threshold is configurable) that
# global_localization_recovery.launch.py has no launch argument
# for -- mirrors scripts/benchmark_from_manifest build_localization_yaml().
python3 - "${base_param_yaml}" "${initial_pose_yaml}" "${map_path}" "${localization_yaml}" \
  "${localizer_score_threshold}" "${enable_borderline_seed_rejection_gate}" \
  "${publish_bridge_pose_when_lost}" "${odom_correction_guard_translation_m}" \
  "${enable_map_odom_anchor_fitness_gate}" "${map_odom_anchor_max_fitness}" \
  "${map_odom_anchor_max_correction_rotation_deg}" "${odom_tf_constraint_mode}" <<'PYEOF'
import math
import sys
import yaml

base_yaml, initial_pose_yaml, map_path, output_yaml, score_threshold_arg, borderline_arg, bridge_output_arg, odom_guard_arg, anchor_gate_arg, anchor_fitness_arg, anchor_rotation_arg, odom_constraint_mode = sys.argv[1:13]
score_threshold = float(score_threshold_arg)
if score_threshold <= 0.0:
    raise ValueError("LOCALIZER_SCORE_THRESHOLD must be positive")
enable_borderline_gate = borderline_arg == "true"
enable_bridge_timer = bridge_output_arg == "true"
odom_correction_guard_translation_m = float(odom_guard_arg)
if odom_correction_guard_translation_m <= 0.0:
    raise ValueError("ODOM_CORRECTION_GUARD_TRANSLATION_M must be positive")
enable_anchor_gate = anchor_gate_arg == "true"
map_odom_anchor_max_fitness = float(anchor_fitness_arg)
if not math.isfinite(map_odom_anchor_max_fitness) or map_odom_anchor_max_fitness < 0.0:
    raise ValueError("MAP_ODOM_ANCHOR_MAX_FITNESS must be non-negative")
map_odom_anchor_max_correction_rotation_deg = float(anchor_rotation_arg)
if (not math.isfinite(map_odom_anchor_max_correction_rotation_deg) or
        map_odom_anchor_max_correction_rotation_deg < 0.0):
    raise ValueError("MAP_ODOM_ANCHOR_MAX_CORRECTION_ROTATION_DEG must be non-negative")

with open(base_yaml, "r", encoding="utf-8") as stream:
    merged = yaml.safe_load(stream)
ros_params = merged["/**"]["ros__parameters"]

with open(initial_pose_yaml, "r", encoding="utf-8") as stream:
    initial_pose = yaml.safe_load(stream)
ros_params.update(initial_pose["/**"]["ros__parameters"])

ros_params["map_path"] = map_path
ros_params.update({
    "use_imu": False,
    "use_imu_preintegration": True,
    "enable_scan_voxel_filter": True,
    "voxel_leaf_size": 0.5,
    "base_frame_id": "livox_frame",
    "scan_min_range": 1.0,
    "scan_max_range": 100.0,
    "score_threshold": score_threshold,
    "enable_borderline_seed_rejection_gate": enable_borderline_gate,
    "enable_timer_publishing": enable_bridge_timer,
    "pose_publish_frequency": 12.5,
    "imu_accel_scale": 9.80665,
    "enable_map_odom_tf": True,
    "enable_map_odom_anchor_fitness_gate": enable_anchor_gate,
    "map_odom_anchor_max_fitness": map_odom_anchor_max_fitness,
    "map_odom_anchor_max_correction_rotation_deg": map_odom_anchor_max_correction_rotation_deg,
    "use_odom": False,
    # The external LIO constraint is sensor/sequence dependent: planar keeps
    # 01a/02a tilt drift from rotating horizontal motion out of the map plane,
    # while height-only preserves materially better live orientation on 02b.
    # Both remain opt-in experiment settings; component defaults stay false.
    "constrain_odom_tf_prediction_to_planar": odom_constraint_mode == "planar",
    "constrain_odom_tf_prediction_height_only": odom_constraint_mode == "height_only",
    "enable_odom_tf_prediction_correction_guard": True,
    "odom_tf_prediction_correction_guard_translation_m": odom_correction_guard_translation_m,
    "odom_tf_prediction_correction_guard_yaw_deg": 30.0,
    "enable_odom_tf_prediction_recovery_correction_guard": False,
    "odom_tf_prediction_recovery_min_rejections": 30,
    "odom_tf_prediction_recovery_max_fitness": 1.5,
    "odom_tf_prediction_recovery_guard_translation_m": 5.0,
    "odom_tf_prediction_recovery_guard_yaw_deg": 30.0,
    # Arm G3 near the beginning of a sustained loss instead of waiting for the
    # old 30 s / 200-reject saturation (or a chance fitness explosion).
    "reinitialization_trigger_gap_scale_sec": 10.0,
    "reinitialization_trigger_reject_streak_scale": 30.0,
})

with open(output_yaml, "w", encoding="utf-8") as stream:
    yaml.safe_dump(merged, stream, sort_keys=False)
PYEOF

echo "Generated localization param yaml: ${localization_yaml}"

set +u
# shellcheck source=/dev/null
source "${repo_root}/scripts/setup_local_env.sh" 2>/dev/null || true
# shellcheck source=/dev/null
source "${repo_root}/../../install/setup.bash" 2>/dev/null || true
if [[ -f "/home/sasaki/workspace/old_~2026/lidarloc_ws/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/home/sasaki/workspace/old_~2026/lidarloc_ws/install/setup.bash"
fi
set -u

echo "Starting GLIM front end (image ${glim_image}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."
# Run detached (-d) with a unique --name instead of an attached background job:
# an attached `docker run &` followed by `exec ros2 launch ...` orphans the
# docker CLI process (exec replaces this script's image, and benchmark_runner's
# process-group SIGTERM/SIGKILL of that group does not reliably reach a
# `docker run --rm` client and stop the daemon-side container -- confirmed
# leaking a container in testing). `docker rm -f` in the trap below is
# unconditional and does not depend on signal propagation into the container.
glim_container=""
glim_log_pid=""
tf_relay_pid=""
csv_tf_pid=""
if [[ -n "${glim_odom_csv}" ]]; then
  if [[ ! -f "${glim_odom_csv}" ]]; then
    echo "Missing GLIM_ODOM_CSV: ${glim_odom_csv}" >&2
    exit 2
  fi
  python3 "${script_dir}/pose_csv_tf_publisher.py" --ros-args \
    -p "pose_csv:=${glim_odom_csv}" >"${OUT_DIR}/glim_csv_tf.log" 2>&1 &
  csv_tf_pid=$!
  echo "Validated GLIM odometry CSV TF source: pid ${csv_tf_pid} (${glim_odom_csv})"
else
glim_container="glim_frontend_$$"
# /tf is remapped to /glim/tf_raw because glim_rosnode unconditionally
# publishes BOTH odom -> livox_frame (wanted) and glim_world -> odom (its SLAM
# correction). The latter gives `odom` a second parent and corrupts the
# localizer's map -> odom -> base_link chain; glim_tf_relay.py (below) filters
# the wanted edge back onto /tf.
docker run -d --network=host --ipc=host \
  --name "${glim_container}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -e "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}" \
  -v "${glim_config_dir}:/glim/config:ro" \
  "${glim_image}" \
  bash -lc ". /opt/ros/jazzy/setup.bash && . /root/ros2_ws/install/setup.bash && \
    ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config \
      -r /tf:=/glim/tf_raw -r /tf_static:=/glim/tf_static_raw" \
  >/dev/null
docker logs -f "${glim_container}" >"${glim_live_log}" 2>&1 &
glim_log_pid=$!
echo "GLIM container: ${glim_container} (log: ${glim_live_log})"

python3 "${script_dir}/glim_tf_relay.py" --ros-args \
  -p use_sim_time:=true >"${OUT_DIR}/glim_tf_relay.log" 2>&1 &
tf_relay_pid=$!
echo "GLIM TF relay: pid ${tf_relay_pid}"
fi

cleanup() {
  [[ -z "${glim_container}" ]] || docker rm -f "${glim_container}" >/dev/null 2>&1 || true
  [[ -z "${glim_log_pid}" ]] || kill "${glim_log_pid}" >/dev/null 2>&1 || true
  [[ -z "${tf_relay_pid}" ]] || kill "${tf_relay_pid}" >/dev/null 2>&1 || true
  [[ -z "${csv_tf_pid}" ]] || kill "${csv_tf_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Give glim_rosnode time to come up and start publishing odom -> livox_frame
# before the localizer starts looking that TF up (enable_map_odom_tf).
sleep 8

echo "Launching localizer + G2 + G3 (map -> odom via external front-end TF)..."
ros2 launch lidar_localization_ros2 global_localization_recovery.launch.py \
  "localization_param_dir:=${localization_yaml}" \
  "occupancy_yaml:=${occupancy_yaml}" \
  "map_path:=${map_path}" \
  "g2_enable_registration_scoring:=true" \
  "cloud_topic:=/livox/points" \
  "imu_topic:=/livox/imu" \
  "global_frame_id:=map" \
  "odom_frame_id:=glim_odom" \
  "base_frame_id:=livox_frame" \
  "lidar_frame_id:=livox_frame" \
  "imu_frame_id:=livox_frame" \
  "publish_lidar_tf:=false" \
  "publish_imu_tf:=false" \
  "use_sim_time:=true" \
  "use_imu_preintegration:=true" \
  "imu_preintegration_use_base_frame_transform:=false" \
  "enable_map_odom_tf:=true" \
  "use_odom:=false" \
  "use_odom_tf_prediction:=${use_odom_tf_prediction}" \
  "publish_bridge_pose_when_lost:=${publish_bridge_pose_when_lost}" \
  "g2_use_cpp_backend:=true" \
  "g2_max_candidates:=8" \
  "g2_nms_radius_m:=0.5" \
  "g2_registration_seed_z_m:=-11.046818" \
  "supervisor_reset_default_z_m:=-11.046818" \
  "supervisor_prefer_reset_default_z_m:=true" \
  "supervisor_enable_seed_motion_compensation:=true" \
  "supervisor_max_seed_speed_mps:=3.0" \
  "supervisor_recovery_fitness_threshold:=1.5" \
  "supervisor_max_attempts:=5" \
  "supervisor_max_walk_candidates:=1" \
  "supervisor_enable_bbs_shadow_motion_gate:=true" \
  "supervisor_bbs_shadow_required_samples:=2" \
  "supervisor_bbs_shadow_max_translation_mismatch_m:=5.0" \
  "supervisor_bbs_shadow_max_yaw_mismatch_deg:=20.0" \
  "supervisor_bbs_shadow_bridge_stamp_tolerance_sec:=2.0" \
  "supervisor_use_odom_bridge_candidate:=${supervisor_use_odom_bridge_candidate}" \
  "supervisor_odom_bridge_max_attempts:=1" \
  "supervisor_odom_bridge_max_age_sec:=2.0" \
  "supervisor_odom_bridge_position_std_m:=0.3" \
  "supervisor_odom_bridge_yaw_std_rad:=0.1" \
  "supervisor_event_log_csv:=${OUT_DIR}/supervisor_events.csv"
launch_status=$?
exit "${launch_status}"
