#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Execute the command passed to the container
exec "$@"
