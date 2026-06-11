#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script instead of executing it:"
  echo "  source scripts/setup_local_env.sh"
  exit 1
fi

_lidarloc_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_lidarloc_ws_root="${LIDAR_LOCALIZATION_WS_ROOT:-$(cd "${_lidarloc_repo_root}/.." && pwd)}"
_lidarloc_prefix="${LIDAR_LOCALIZATION_LOCAL_PREFIX:-${_lidarloc_ws_root}/local_prefix}"
_lidarloc_overlay="${LIDAR_LOCALIZATION_OVERLAY:-${_lidarloc_ws_root}/build_ws/install/setup.bash}"

_lidarloc_ros_distro=""
for _lidarloc_candidate in "${LIDAR_LOCALIZATION_ROS_DISTRO:-}" "${ROS_DISTRO:-}" humble jazzy; do
  if [[ -n "${_lidarloc_candidate}" && -f "/opt/ros/${_lidarloc_candidate}/setup.bash" ]]; then
    _lidarloc_ros_distro="${_lidarloc_candidate}"
    break
  fi
done
unset _lidarloc_candidate

if [[ -z "${_lidarloc_ros_distro}" ]]; then
  echo "No ROS 2 installation found under /opt/ros (checked LIDAR_LOCALIZATION_ROS_DISTRO, ROS_DISTRO, humble, jazzy)" >&2
  return 1
fi

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-}"
export COLCON_TRACE="${COLCON_TRACE-}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE-}"

source "/opt/ros/${_lidarloc_ros_distro}/setup.bash"

export LIDAR_LOCALIZATION_WS_ROOT="${_lidarloc_ws_root}"
export LIDAR_LOCALIZATION_LOCAL_PREFIX="${_lidarloc_prefix}"

export CMAKE_PREFIX_PATH="${_lidarloc_prefix}/usr:${_lidarloc_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PKG_CONFIG_PATH="${_lidarloc_prefix}/usr/lib/x86_64-linux-gnu/pkgconfig:${_lidarloc_prefix}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export LIBRARY_PATH="${_lidarloc_prefix}/usr/lib/x86_64-linux-gnu:${_lidarloc_prefix}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${_lidarloc_prefix}/usr/lib/x86_64-linux-gnu:${_lidarloc_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CPATH="${_lidarloc_prefix}/usr/include${CPATH:+:${CPATH}}"
export CPLUS_INCLUDE_PATH="${_lidarloc_prefix}/usr/include${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}"
export CMAKE_INCLUDE_PATH="${_lidarloc_prefix}/usr/include${CMAKE_INCLUDE_PATH:+:${CMAKE_INCLUDE_PATH}}"

if [[ -f "${_lidarloc_overlay}" ]]; then
  source "${_lidarloc_overlay}"
fi

unset _lidarloc_ros_distro
unset _lidarloc_repo_root
unset _lidarloc_ws_root
unset _lidarloc_prefix
unset _lidarloc_overlay
