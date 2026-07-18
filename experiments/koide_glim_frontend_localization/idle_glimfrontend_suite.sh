#!/bin/bash
# Idle-gated rerun of the GLIM-front-end + odom-bridge full-sequence suite.
# Waits for sustained 1-min load < 2.5 (3 consecutive checks, 60 s apart),
# then runs 01a; continues to 01b/02a/02b (re-gating before each) regardless
# of pass/fail so we get the full picture in one quiet window.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${repo_root}/../../install/setup.bash"
set -u
# FastDDS SHM silently drops container<->host data; every process in the
# benchmark graph (bag play, recorders, wrapper children) must use UDPv4.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

wait_for_idle() {
  local ok=0
  while [ $ok -lt 3 ]; do
    local load
    load=$(awk '{print $1}' /proc/loadavg)
    if awk -v l="$load" 'BEGIN{exit !(l < 2.5)}'; then
      ok=$((ok + 1))
    else
      ok=0
    fi
    echo "[idle-gate] $(date +%H:%M:%S) load=$load ok_streak=$ok"
    [ $ok -lt 3 ] && sleep 60
  done
}

cd "${repo_root}"
for seq in 01a 01b 02a 02b; do
  mf="${script_dir}/fullseq_glim_frontend_${seq}.yaml"
  out=$(python3 -c "import yaml;print(yaml.safe_load(open('$mf'))['benchmark']['output_dir'])")
  case "${out}" in
    /tmp/glimfrontend_runs/*) ;;
    *)
      echo "Refusing unexpected output directory: ${out}" >&2
      exit 2
      ;;
  esac
  echo "=== waiting for idle before outdoor_hard_${seq} ==="
  wait_for_idle
  # Paranoia: no leftover front-end containers or localizer nodes.
  docker ps -a --format '{{.Names}}' | grep -i '^glim_frontend' | xargs -r docker rm -f
  echo "=== outdoor_hard_${seq} start $(date +%H:%M:%S) ==="
  if [ -e "${out}" ]; then
    previous_out="${out}.previous.$(date +%Y%m%dT%H%M%S)"
    mv "${out}" "${previous_out}"
    echo "Preserved previous output at ${previous_out}"
  fi
  load_log=$(mktemp "/tmp/glimfrontend_${seq}_load.XXXXXX.log")
  ( while true; do echo "$(date +%s) $(cat /proc/loadavg)"; sleep 20; done ) > "${load_log}" 2>&1 &
  mon=$!
  benchmark_status=0
  python3 scripts/benchmark_from_manifest --manifest "${mf}" || benchmark_status=$?
  kill "${mon}" 2>/dev/null
  if [ -d "${out}" ]; then
    mv "${load_log}" "${out}/load.log" 2>/dev/null
  fi

  coverage_status=0
  if [ -f "${out}/alignment_status.csv" ] && [ -f "${out}/pose_trace.csv" ]; then
    python3 "${script_dir}/summarize_bridge_coverage.py" \
      --alignment-csv "${out}/alignment_status.csv" \
      --pose-csv "${out}/pose_trace.csv" \
      --output-json "${out}/bridge_coverage.json" \
      --min-output-coverage 99 \
      --min-matched-coverage 95 \
      --max-output-gap-sec 1.0 || coverage_status=$?
  else
    coverage_status=2
    echo "Missing alignment_status.csv or pose_trace.csv in ${out}" >&2
  fi
  echo "=== outdoor_hard_${seq} benchmark_exit=${benchmark_status} coverage_exit=${coverage_status} $(date +%H:%M:%S) ==="
done
echo "GLIMFRONTEND_SUITE_DONE"
