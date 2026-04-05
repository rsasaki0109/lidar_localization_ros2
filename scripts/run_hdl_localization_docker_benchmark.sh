#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the official hdl_localization baseline inside Docker and record /odom and /status into CSV.

Usage:
  scripts/run_hdl_localization_docker_benchmark.sh --bag-path PATH --output-dir DIR [--image IMAGE]

Options:
  --bag-path PATH    Path to hdl_400.bag or another ROS1 bag.
  --output-dir DIR   Directory for CSVs and logs.
  --image IMAGE      Docker image tag. Default: lidarloc-hdl-localization:noetic
  -h, --help         Show this help.
EOF
}

image="lidarloc-hdl-localization:noetic"
bag_path=""
output_dir=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-path)
      shift
      bag_path="$1"
      ;;
    --output-dir)
      shift
      output_dir="$1"
      ;;
    --image)
      shift
      image="$1"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if [[ -z "${bag_path}" || -z "${output_dir}" ]]; then
  usage >&2
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_path="$(readlink -f "${bag_path}")"
output_dir="$(readlink -f "${output_dir}")"
bag_dir="$(dirname "${bag_path}")"
bag_file="$(basename "${bag_path}")"

mkdir -p "${output_dir}"

docker run --rm \
  -v "${bag_dir}:/input:ro" \
  -v "${output_dir}:/output" \
  -v "${repo_root}/scripts/record_hdl_localization_outputs.py:/tools/record_hdl_localization_outputs.py:ro" \
  "${image}" \
  bash -lc '
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
export ROS_LOG_DIR=/output/ros_logs
roscore > /output/roscore.stdout.log 2> /output/roscore.stderr.log &
ROSCORE_PID=$!
for _ in $(seq 1 30); do
  if rosparam set use_sim_time true >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
python3 /tools/record_hdl_localization_outputs.py --output-dir /output > /output/recorder.stdout.log 2> /output/recorder.stderr.log &
REC_PID=$!
roslaunch hdl_localization hdl_localization.launch use_global_localization:=false use_imu:=false > /output/system.stdout.log 2> /output/system.stderr.log &
LAUNCH_PID=$!
sleep 5
rosbag play --clock "/input/'"${bag_file}"'" > /output/bag.stdout.log 2> /output/bag.stderr.log
sleep 3
kill -INT "${REC_PID}" || true
kill -INT "${LAUNCH_PID}" || true
kill -INT "${ROSCORE_PID}" || true
wait || true
'
