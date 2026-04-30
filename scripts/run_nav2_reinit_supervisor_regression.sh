#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_nav2_reinit_supervisor_regression.sh [options]

Options:
  --output-dir PATH             default: artifacts/public/nav2_reinit_supervisor_regression_150
  --map-yaml PATH               default: official Istanbul Nav2 map yaml
  --pcd-map-path PATH           default: official Istanbul pointcloud map
  --bag-path PATH               default: official Istanbul localization rosbag
  --post-goal-observe-sec SEC   default: 150
  --ros-domain-base VALUE       default: 151
  --resume                      skip variants whose alignment CSV already exists
  --help

This runs two Nav2 replay variants on the same long replay scenario:

  - no_supervisor
  - configured_initial_pose_count1

It then compares them and writes:

  - comparison.json
  - summary.md
  - regression_result.json

The regression passes when:

  - both runs finish `navigate_to_pose` with `SUCCEEDED`
  - `no_supervisor` actually exercises at least one reinitialization request
  - `configured_initial_pose_count1` keeps reinitialization-request rows bounded
  - `configured_initial_pose_count1` reduces request rows by at least 10x vs `no_supervisor`
  - the compare helper recommends `configured_initial_pose_count1`
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

output_dir="${ws_root}/artifacts/public/nav2_reinit_supervisor_regression_150"
map_yaml="${ws_root}/artifacts/public/autoware_istanbul_60s_nav2_map/istanbul_60s_nav2_map.yaml"
pcd_map_path="${ws_root}/data/official/autoware_istanbul/pointcloud_map.pcd"
bag_path="${ws_root}/data/official/autoware_istanbul/localization_rosbag"
post_goal_observe_sec="150"
ros_domain_base="151"
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

if ((ros_domain_base + 1 > 232)); then
  printf 'ROS domain range %s..%s exceeds Fast DDS limit 232\n' "${ros_domain_base}" "$((ros_domain_base + 1))" >&2
  exit 2
fi

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

run_variant() {
  local label="$1"
  local ros_domain_id="$2"
  shift 2

  local log_dir="${output_dir}/${label}"
  local alignment_csv="${log_dir}/alignment_status.csv"

  if [[ "${resume}" == "1" && -f "${alignment_csv}" ]]; then
    printf 'Skipping %s (resume)\n' "${label}"
    return 0
  fi

  rm -rf "${log_dir}"
  printf 'Running %s (ros_domain_id=%s)\n' "${label}" "${ros_domain_id}"

  ros2 run lidar_localization_ros2 run_nav2_replay_smoke \
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
    "$@"
}

run_variant "no_supervisor" "${ros_domain_base}"
run_variant \
  "configured_initial_pose_count1" \
  "$((ros_domain_base + 1))" \
  --enable-reinitialization-supervisor \
  --reinitialization-supervisor-publish-count 1 \
  --reinitialization-supervisor-publish-interval-sec 0.2 \
  --reinitialization-supervisor-cooldown-sec 5.0

comparison_json="${output_dir}/comparison.json"
summary_md="${output_dir}/summary.md"
regression_result_json="${output_dir}/regression_result.json"

python3 "${repo_root}/scripts/compare_nav2_reinit_supervisor_runs.py" \
  --run "no_supervisor=${output_dir}/no_supervisor" \
  --run "configured_initial_pose_count1=${output_dir}/configured_initial_pose_count1" \
  --output-json "${comparison_json}" \
  --output-md "${summary_md}" \
  > /dev/null

python3 - "${comparison_json}" "${regression_result_json}" <<'PY'
import json
import sys
from pathlib import Path

comparison_path = Path(sys.argv[1])
result_path = Path(sys.argv[2])
comparison = json.loads(comparison_path.read_text(encoding="utf-8"))
runs = {run["label"]: run for run in comparison["runs"]}

required_labels = {"no_supervisor", "configured_initial_pose_count1"}
missing = sorted(required_labels - runs.keys())
if missing:
    raise SystemExit(f"Missing runs in comparison: {missing}")

baseline = runs["no_supervisor"]
candidate = runs["configured_initial_pose_count1"]

checks = []

def add_check(name: str, passed: bool, detail: str) -> None:
    checks.append({"name": name, "passed": passed, "detail": detail})

add_check(
    "baseline_goal_succeeded",
    baseline["goal_status"] == "SUCCEEDED",
    f"goal_status={baseline['goal_status']}",
)
add_check(
    "candidate_goal_succeeded",
    candidate["goal_status"] == "SUCCEEDED",
    f"goal_status={candidate['goal_status']}",
)
add_check(
    "baseline_triggered_reinit",
    baseline["alignment"]["reinitialization_requested_rows"] >= 1,
    f"requested_rows={baseline['alignment']['reinitialization_requested_rows']}",
)
add_check(
    "candidate_requested_rows_at_most_20",
    candidate["alignment"]["reinitialization_requested_rows"] <= 20,
    f"candidate_requested_rows={candidate['alignment']['reinitialization_requested_rows']}",
)
add_check(
    "candidate_requested_rows_reduced_10x",
    candidate["alignment"]["reinitialization_requested_rows"] * 10 <= baseline["alignment"]["reinitialization_requested_rows"],
    "candidate_requested_rows="
    f"{candidate['alignment']['reinitialization_requested_rows']}, "
    "baseline_requested_rows="
    f"{baseline['alignment']['reinitialization_requested_rows']}",
)
add_check(
    "compare_helper_recommends_candidate",
    comparison["recommended_run"] == "configured_initial_pose_count1",
    f"recommended_run={comparison['recommended_run']}",
)

overall_pass = all(check["passed"] for check in checks)
result = {
    "overall_pass": overall_pass,
    "recommended_run": comparison["recommended_run"],
    "checks": checks,
    "baseline": baseline,
    "candidate": candidate,
}
result_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
print(json.dumps(result, indent=2, sort_keys=True))
if not overall_pass:
    raise SystemExit(1)
PY

printf 'comparison_json: %s\n' "${comparison_json}"
printf 'summary_md: %s\n' "${summary_md}"
printf 'regression_result_json: %s\n' "${regression_result_json}"
