#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/bootstrap_colcon_workspace.sh [--build] [--ros-distro DISTRO]

Fetch the source dependencies needed to build lidar_localization_ros2 in a
standard ROS workspace:

  lidarloc_ws/
    src/
      lidar_localization_ros2/
      ndt_omp_ros2/

Options:
  --build              Run colcon build after fetching dependencies.
  --ros-distro DISTRO  Source /opt/ros/DISTRO/setup.bash before building.
                       Defaults to ROS_DISTRO, then jazzy, then humble.
  -h, --help           Show this help.
EOF
}

have_command() {
  command -v "$1" >/dev/null 2>&1
}

die_missing_command() {
  local command_name="$1"
  local install_hint="$2"
  echo "Required command not found: ${command_name}" >&2
  echo "Install hint: ${install_hint}" >&2
  exit 1
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
src_root="$(cd "${repo_root}/.." && pwd)"
repos_file="${repo_root}/dependencies.repos"

if [[ "$(basename "${src_root}")" != "src" ]]; then
  echo "Expected this repository to live under a ROS workspace src/ directory:" >&2
  echo "  <workspace>/src/lidar_localization_ros2" >&2
  echo "Current path: ${repo_root}" >&2
  exit 1
fi

ws_root="$(cd "${src_root}/.." && pwd)"
build_after_fetch=false
ros_distro="${ROS_DISTRO:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      build_after_fetch=true
      shift
      ;;
    --ros-distro)
      if [[ $# -lt 2 ]]; then
        echo "--ros-distro requires a value" >&2
        exit 1
      fi
      ros_distro="$2"
      shift 2
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
done

if [[ -z "${ros_distro}" ]]; then
  for candidate in jazzy humble; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      ros_distro="${candidate}"
      break
    fi
  done
fi

echo "Workspace: ${ws_root}"

ndt_dir="${src_root}/ndt_omp_ros2"
if [[ -d "${ndt_dir}/.git" ]]; then
  echo "Dependency already present: ${ndt_dir}"
else
  if [[ -e "${ndt_dir}" ]]; then
    echo "Cannot clone ndt_omp_ros2: ${ndt_dir} exists but is not a git checkout" >&2
    exit 1
  fi
  have_command git || die_missing_command "git" "sudo apt install git"
  if [[ -f "${repos_file}" ]] && have_command vcs; then
    echo "Importing dependencies from ${repos_file}"
    vcs import "${src_root}" < "${repos_file}"
  else
    if [[ -f "${repos_file}" ]]; then
      echo "vcs is not installed; falling back to git clone for ndt_omp_ros2"
    fi
    git clone --branch humble https://github.com/rsasaki0109/ndt_omp_ros2.git "${ndt_dir}"
  fi
fi

if [[ ! -f "${ndt_dir}/package.xml" ]]; then
  echo "Dependency checkout looks incomplete: ${ndt_dir}/package.xml is missing" >&2
  exit 1
fi

if [[ "${build_after_fetch}" != true ]]; then
  cat <<EOF

Dependencies are ready. Build with:
  cd ${ws_root}
  source /opt/ros/${ros_distro:-<distro>}/setup.bash
  colcon build --symlink-install --packages-up-to lidar_localization_ros2
  source install/setup.bash
EOF
  exit 0
fi

if [[ -z "${ros_distro}" || ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "No ROS 2 setup file found. Pass --ros-distro DISTRO or install ROS 2." >&2
  exit 1
fi

have_command colcon || die_missing_command "colcon" "sudo apt install python3-colcon-common-extensions"

set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u
cd "${ws_root}"
colcon build --symlink-install --packages-up-to lidar_localization_ros2

cat <<EOF

Build complete. Use this overlay with:
  source ${ws_root}/install/setup.bash
EOF
