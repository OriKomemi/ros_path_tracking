#!/bin/bash
# Launch ROS 2 nodes with virtual environment activated

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/install/setup.bash"

# Launch the application
ros2 launch autonomous_car_sim autonomous_car.launch.py
