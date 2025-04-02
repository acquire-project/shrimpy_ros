#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# setup workspace environment, if it exists
if [ -f "/opt/ros/custom/setup.bash" ]; then
    source "/opt/ros/custom/setup.bash" --
fi

exec "$@"
