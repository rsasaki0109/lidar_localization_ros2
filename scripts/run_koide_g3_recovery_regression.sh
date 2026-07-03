#!/usr/bin/env bash
# Release-style wrapper for the Koide outdoor_hard_01a G3 recovery replay.
set -euo pipefail

usage() {
  cat <<'EOF'
Run the Koide G3 recovery regression (120 s bag, kidnap injection, health rubric).

Writes regression_result.json with overall_pass when the supervisor reports both
recovery_confirmed and at least one stable_recovered_request_window.

Usage:
  scripts/run_koide_g3_recovery_regression.sh [options]

Options:
  --output-dir DIR       Artifact directory. Default: artifacts/public/koide_g3_recovery_regression
  --ros-domain-id N      ROS domain id. Default: 220
  --duration-sec N       Bag replay duration. Default: 120
  --rate X               Bag replay rate. Default: 0.4
  --max-load N           Warn when 1-min load average exceeds N. Default: 5
  --skip-prepare         Skip Koide asset regeneration.
  --resume               Skip replay when recovery_health.json already passes.
  -h, --help             Show this help.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/../CMakeLists.txt" ]]; then
  repo_root="$(cd "${script_dir}/.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${repo_root}/../.." && pwd)}"
else
  package_prefix="$(cd "${script_dir}/../.." && pwd)"
  ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${package_prefix}/../../.." && pwd)}"
  repo_root="${LIDAR_LOCALIZATION_REPO_ROOT:-${ws_root}/repo}"
fi

output_dir="${ws_root}/artifacts/public/koide_g3_recovery_regression"
ros_domain_id="220"
duration_sec="120"
play_rate="0.4"
max_load="5"
skip_prepare=0
resume=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) shift; output_dir="$1" ;;
    --ros-domain-id) shift; ros_domain_id="$1" ;;
    --duration-sec) shift; duration_sec="$1" ;;
    --rate) shift; play_rate="$1" ;;
    --max-load) shift; max_load="$1" ;;
    --skip-prepare) skip_prepare=1 ;;
    --resume) resume=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

data_dir="${repo_root}/data/public/koide_hard_localization"
bag_meta="${data_dir}/sequences/outdoor_hard_01a/metadata.yaml"
health_json="${output_dir}/recovery_health.json"
result_json="${output_dir}/regression_result.json"

if [[ ! -f "${bag_meta}" ]]; then
  cat >"${result_json}" <<EOF
{
  "overall_pass": false,
  "skipped": true,
  "reason": "koide dataset missing (${bag_meta})",
  "output_dir": "${output_dir}"
}
EOF
  echo "Koide G3 regression skipped: dataset not present at ${data_dir}" >&2
  exit 0
fi

load_avg="$(awk '{print $1}' /proc/loadavg)"
load_int="$(awk -v load="${load_avg}" 'BEGIN { printf "%d", load + 0.5 }')"
if (( load_int >= max_load )); then
  echo "warning: 1-min load ${load_avg} >= ${max_load}; replay timing may be unreliable" >&2
fi

mkdir -p "${output_dir}"

if [[ "${resume}" -eq 1 && -f "${health_json}" ]]; then
  if python3 - "${health_json}" <<'PY'
import json, sys
payload = json.loads(open(sys.argv[1], encoding="utf-8").read())
sys.exit(0 if payload.get("ok") else 1)
PY
  then
    cat >"${result_json}" <<EOF
{
  "overall_pass": true,
  "resumed": true,
  "recovery_health_json": "${health_json}",
  "output_dir": "${output_dir}"
}
EOF
    echo "Koide G3 regression resumed from existing pass: ${health_json}"
    exit 0
  fi
fi

replay_args=(
  --output-dir "${output_dir}"
  --duration-sec "${duration_sec}"
  --rate "${play_rate}"
  --ros-domain-id "${ros_domain_id}"
)
if [[ "${skip_prepare}" -eq 1 ]]; then
  replay_args+=(--skip-prepare)
fi

replay_status=0
if ! "${script_dir}/run_koide_g3_recovery_replay.sh" "${replay_args[@]}"; then
  replay_status=$?
fi

if [[ ! -f "${health_json}" ]]; then
  cat >"${result_json}" <<EOF
{
  "overall_pass": false,
  "replay_exit_code": ${replay_status},
  "reason": "recovery_health.json missing",
  "output_dir": "${output_dir}"
}
EOF
  exit 1
fi

python3 - "${health_json}" "${result_json}" "${replay_status}" "${output_dir}" <<'PY'
import json
import sys
from pathlib import Path

health_path = Path(sys.argv[1])
result_path = Path(sys.argv[2])
replay_status = int(sys.argv[3])
output_dir = sys.argv[4]
health = json.loads(health_path.read_text(encoding="utf-8"))
overall_pass = bool(health.get("ok")) and replay_status == 0
payload = {
    "overall_pass": overall_pass,
    "replay_exit_code": replay_status,
    "recovery_health": health,
    "recovery_health_json": str(health_path),
    "output_dir": output_dir,
    "supervisor_events_csv": str(Path(output_dir) / "supervisor_events.csv"),
    "alignment_status_csv": str(Path(output_dir) / "alignment_status.csv"),
}
result_path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")
print(json.dumps(payload, indent=2, sort_keys=False))
sys.exit(0 if overall_pass else 1)
PY
