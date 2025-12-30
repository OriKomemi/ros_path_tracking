# Reset Car to Track Start

**Purpose:** Reset the BGR car to the racing line start position and clear all control commands.

---

## Quick Usage

### Option 1: Bash Script (Simplest)

```bash
./reset_to_start.sh
```

### Option 2: ROS 2 Node (Recommended)

```bash
# After building the package
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run autonomous_car_sim reset_position
```

---

## What It Does

The reset sequence performs these steps:

1. **Stops Control Commands**
   - Sets steering to 0.0 radians
   - Sets all wheel velocities to 0.0 rad/s
   - Publishes to Gazebo control topics

2. **Resets Position**
   - Uses Gazebo service to set car pose
   - Position: (45.751, 81.3, 1.0) meters
   - Matches `start` point from `racing_line_trackdrive.npz`

3. **Waits for Physics**
   - Allows Gazebo physics to settle
   - Ensures stable initial state

4. **Confirms Success**
   - Verifies position was set
   - Ready for autonomous control

---

## Track Start Position

### Coordinates (from racing_line_trackdrive.npz)

```python
Position:
  x: 45.751 meters
  y: 81.3 meters
  z: 1.0 meters (slightly elevated to prevent ground collision)

Orientation (Quaternion):
  x: -0.0001
  y: -0.0007
  z: 1.232886347679596e-06
  w: 0.9999

# Approximately aligned with initial track direction
# Yaw ≈ 0.0 radians (facing along track)
```

### Why This Position?

- Matches the `start` array in `racing_line_trackdrive.npz`
- First waypoint of the racing line
- Optimal starting point for path tracking
- Car faces along the track direction

---

## Method 1: Bash Script

### File: `reset_to_start.sh`

```bash
#!/bin/bash
# Reset car to track start position and clear control commands

# Stop control commands
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0]"
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"

# Reset position using Gazebo service
gz service -s /world/empty/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: 'bgr', position: {x: 45.751, y: 81.3, z: 1.0}, \
         orientation: {x: -0.0001, y: -0.0007, z: 1.232886347679596e-06, w: 0.9999}"
```

### Usage:

```bash
# Make executable (first time only)
chmod +x reset_to_start.sh

# Run
./reset_to_start.sh
```

### Requirements:

- Gazebo running with BGR robot
- `gz` CLI tool available
- `ros2` CLI tool available

---

## Method 2: ROS 2 Node (Recommended)

### File: `reset_position.py`

A proper ROS 2 node with better error handling and integration.

### Features:

- ✅ Proper ROS 2 publisher setup
- ✅ Error handling for Gazebo service calls
- ✅ Logging with timestamps
- ✅ Confirmation messages
- ✅ Automatic timing and sequencing

### Usage:

```bash
# Build the package
colcon build --packages-select autonomous_car_sim --symlink-install

# Source workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run reset node
ros2 run autonomous_car_sim reset_position
```

### Output:

```
[INFO] [reset_position]: Position Resetter initialized
[INFO] [reset_position]: === Starting Reset Sequence ===
[INFO] [reset_position]: Stopping control commands...
[INFO] [reset_position]: Resetting car to track start position...
[INFO] [reset_position]: ✅ Car reset to start position: (45.751, 81.3, 1.0)
[INFO] [reset_position]: Gazebo service confirmed successful pose update
[INFO] [reset_position]: Waiting for physics to settle...
[INFO] [reset_position]: === Reset Complete ===
[INFO] [reset_position]: Car is ready at track start position
[INFO] [reset_position]: You can now start autonomous control
```

---

## Integration with Launch File

### Add to Your Workflow

```bash
# 1. Start Gazebo with BGR robot
# (your Gazebo launch command)

# 2. Reset car to start
ros2 run autonomous_car_sim reset_position

# 3. Start autonomous control
ros2 launch autonomous_car_sim autonomous_car.launch.py
```

### Create Combined Launch (Optional)

You can create a launch file that includes reset as a one-shot action:

```python
# In your launch file
from launch.actions import ExecuteProcess

reset_action = ExecuteProcess(
    cmd=['ros2', 'run', 'autonomous_car_sim', 'reset_position'],
    output='screen'
)

# Add to LaunchDescription before starting controller
```

---

## Gazebo Service Details

### Service Name

```
/world/empty/set_pose
```

### Message Types

```
Request:  gz.msgs.Pose
Response: gz.msgs.Boolean
```

### Request Format

```protobuf
name: 'bgr'
position {
  x: 45.751
  y: 81.3
  z: 1.0
}
orientation {
  x: -0.0001
  y: -0.0007
  z: 1.232886347679596e-06
  w: 0.9999
}
```

### Response

```protobuf
data: true  # Success
data: false # Failure
```

---

## Control Commands Reset

### Steering Command

```bash
# Topic: /forward_position_controller/commands
# Type: std_msgs/msg/Float64MultiArray
# Value: [0.0]

ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0]"
```

### Wheel Velocity Commands

```bash
# Topic: /forward_velocity_controller/commands
# Type: std_msgs/msg/Float64MultiArray
# Value: [0.0, 0.0, 0.0, 0.0] (4 wheels)

ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
```

---

## Troubleshooting

### Issue: "Gazebo service timeout"

**Cause:** Gazebo is not running or world name is different

**Solution:**
```bash
# Check Gazebo is running
gz world list

# If world name is different, update the service path:
gz service -s /world/YOUR_WORLD_NAME/set_pose ...
```

### Issue: "gz command not found"

**Cause:** Gazebo is not installed or not in PATH

**Solution:**
```bash
# Install Gazebo
sudo apt install gz-harmonic  # or your version

# Or add to PATH
export PATH=$PATH:/usr/bin
```

### Issue: Car doesn't move after reset

**Possible causes:**
1. Controllers not running
2. Topics not connected
3. Car needs physics activation

**Solution:**
```bash
# Check topics
ros2 topic list | grep forward

# Verify controller nodes
ros2 node list

# Try small movement command to activate
ros2 topic pub --once /forward_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"
```

### Issue: Position reset but car falls through ground

**Cause:** Z position too low

**Solution:**
- Increase z in reset command: `z: 1.0` → `z: 1.5`
- Check ground plane in Gazebo world

---

## Verification

### Check Car Position

```bash
# Using Gazebo CLI
gz model -m bgr -p

# Expected output near:
# x: 45.751
# y: 81.3
# z: ~1.0 (may settle slightly)
```

### Check Control Topics

```bash
# Monitor steering
ros2 topic echo /forward_position_controller/commands

# Monitor velocities
ros2 topic echo /forward_velocity_controller/commands

# Should show zeros after reset
```

### Visual Check in Gazebo

1. Open Gazebo GUI
2. Look for BGR robot
3. Should be at coordinates (45.751, 81.3)
4. Should be near the racing line start
5. Oriented along the track

---

## Advanced Usage

### Reset to Custom Position

Edit `reset_position.py` or use parameters:

```python
# In reset_position.py, change:
self.start_x = 45.751  # Your X
self.start_y = 81.3    # Your Y
self.start_z = 1.0     # Your Z
```

Or use Gazebo service directly:

```bash
gz service -s /world/empty/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: 'bgr', position: {x: YOUR_X, y: YOUR_Y, z: YOUR_Z}"
```

### Reset During Runtime

The reset node can be called while the car is moving:

```bash
# Car is running autonomous control
# Emergency reset:
ros2 run autonomous_car_sim reset_position
```

The reset will:
1. Stop commands
2. Reset position
3. Autonomous control can resume automatically

---

## Files Reference

| File | Type | Purpose |
|------|------|---------|
| [reset_to_start.sh](reset_to_start.sh) | Bash | Simple script for manual reset |
| [reset_position.py](src/autonomous_car_sim/autonomous_car_sim/reset_position.py) | Python | ROS 2 node for reset |
| [setup.py](src/autonomous_car_sim/setup.py) | Config | Added reset_position entry point |

---

## Best Practices

1. **Always reset before starting** a new autonomous run
2. **Use ROS 2 node** for better integration and error handling
3. **Wait for physics** to settle (1-2 seconds)
4. **Verify position** before starting autonomous control
5. **Keep Gazebo GUI open** to visually confirm reset

---

## Integration with Racing Line

The reset position matches the racing line start:

```python
# From racing_line_trackdrive.npz
start_point = [45.751, 81.3]  # First waypoint

# Reset position
reset_position = (45.751, 81.3, 1.0)  # Same X, Y + elevation

# Controller will immediately start tracking from waypoint 0
```

This ensures:
- ✅ Car starts on the racing line
- ✅ Controller finds closest waypoint immediately
- ✅ No large initial correction needed
- ✅ Smooth autonomous start

---

## Summary

**Two methods to reset the car:**

1. **Bash Script** - Quick and simple
   ```bash
   ./reset_to_start.sh
   ```

2. **ROS 2 Node** - Recommended for integration
   ```bash
   ros2 run autonomous_car_sim reset_position
   ```

Both methods:
- ✅ Stop all control commands
- ✅ Reset car to track start (45.751, 81.3, 1.0)
- ✅ Ready for autonomous control

**The car is now at the optimal position to begin racing line tracking!** 🏁
