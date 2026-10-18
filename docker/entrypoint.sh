#!/bin/bash
# Source ROS and the built workspace, then run the given command.
set -e
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source /ws/install/setup.bash
exec "$@"
