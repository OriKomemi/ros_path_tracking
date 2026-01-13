#!/bin/bash
# Reset car to track start position and clear control commands

echo "=== Resetting BGR Car to Track Start ==="
echo ""

# Stop all control commands first
echo "1. Stopping control commands..."
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" &
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]" &
sleep 0.5

echo "2. Setting car position to track start..."
# Position from racing_line_trackdrive.npz start point
# x: 45.751, y: 81.3, z: 1.0 (slightly elevated)
# Orientation: nearly aligned with initial track direction

gz service -s /world/empty/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: 'bgr', position: {x: 45.751, y: 81.3, z: 1.0}, orientation: {x: -0.0001, y: -0.0007, z: 1.232886347679596e-06, w: 0.9999}"
