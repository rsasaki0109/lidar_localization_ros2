#!/bin/bash
# Smoke test: quickstart on a synthetic map, checking each localization status.
#
# Starts the synthetic LiDAR and quickstart with an explicit pose, then switches
# the scan condition and waits for the matching [localization_status] line:
# tracking, weak (sparse scan), lost (kidnap), no scans (stop). Exits non-zero
# with the log tail on the first missing line.
#
#   docker/smoke_status.sh            # without RViz
#   docker/smoke_status.sh --rviz     # RViz under Xvfb as well
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(mktemp -d /tmp/lidarloc_smoke.XXXXXX)"
LOG="${WORK}/quickstart.log"
MODE="${WORK}/mode"
RVIZ=(--no-rviz)
if [[ "${1:-}" == "--rviz" ]]; then
  RVIZ=(--rviz)
  Xvfb :99 -screen 0 1280x800x24 >/dev/null 2>&1 &
  export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
fi
# Keep this run apart from any other ROS graph on the host network.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null; done
  sleep 3
  pkill -f "${WORK}" 2>/dev/null
  pkill -P $$ 2>/dev/null
  pkill Xvfb 2>/dev/null
  true
}
trap cleanup EXIT

wait_for() {
  local label="$1" pattern="$2" timeout="$3" start=${SECONDS}
  while (( SECONDS - start < timeout )); do
    if grep -aqE "${pattern}" "${LOG}"; then
      echo "PASS  ${label}: $(grep -aE "${pattern}" "${LOG}" | tail -1 | sed 's/.*\]: //')"
      return 0
    fi
    sleep 0.5
  done
  echo "FAIL  ${label}: no line matching '${pattern}' within ${timeout} s"
  echo "----- last lines of ${LOG}"
  grep -a "localization_status\]\|startup_initialization\]\|ERROR\|Traceback" "${LOG}" | tail -20
  exit 1
}

echo ok > "${MODE}"
python3 "${HERE}/synthetic_lidar.py" --write-map "${WORK}/map.pcd"
python3 "${HERE}/synthetic_lidar.py" --mode-file "${MODE}" >"${WORK}/lidar.log" 2>&1 &
PIDS+=($!)
sleep 2
ros2 run lidar_localization_ros2 quickstart.py \
  --map "${WORK}/map.pcd" --initial-pose 0 0 1 0 0 0 1 \
  --state-file "${WORK}/pose.json" --output "${WORK}/params.yaml" \
  --no-bringup-check "${RVIZ[@]}" >"${LOG}" 2>&1 &
PIDS+=($!)

wait_for "discovery" "^Discovery: .*cloud=/cloud" 30
wait_for "tracking" "localization_status\]: Tracking\.$" 90
echo sparse > "${MODE}"
wait_for "weak" "localization_status\]: Tracking is weak" 30
echo ok > "${MODE}"
wait_for "recovered" "localization_status\]: Tracking again" 30
echo kidnap > "${MODE}"
wait_for "lost" "localization_status\]: Tracking lost" 40
echo stop > "${MODE}"
wait_for "no scans" "localization_status\]: No scan processed" 30
echo "All status checks passed (log: ${LOG})"
