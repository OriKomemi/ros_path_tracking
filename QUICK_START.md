# Quick Start Guide

**Complete workflow to run the BGR autonomous racing car on Gazebo**

---

## Prerequisites

- ✅ Gazebo running with BGR robot model
- ✅ ROS 2 Jazzy installed
- ✅ Package built: `colcon build --symlink-install`

---

## Complete Workflow

### 1. Build the Package

```bash
cd /home/bgr/dev/ros_path_tracking
colcon build --symlink-install
```

### 2. Source the Workspace

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 3. Reset Car to Track Start

```bash
# Using ROS 2 node (recommended)
ros2 run autonomous_car_sim reset_position
```

**Expected output:**
```
[INFO] [position_resetter]: === Starting Reset Sequence ===
[INFO] [position_resetter]: Stopping control commands...
[INFO] [position_resetter]: ✅ Car reset to start position: (45.751, 81.3, 1.0)
[INFO] [position_resetter]: Gazebo service confirmed successful pose update
[INFO] [position_resetter]: === Reset Complete ===
[INFO] [position_resetter]: Car is ready at track start position
```

### 4. Launch Autonomous Control

```bash
ros2 launch autonomous_car_sim autonomous_car.launch.py
```

**What runs:**
- ✅ SuperStateSpy - Processes Gazebo odometry → publishes full state
- ✅ PathPlanner - Loads racing line (643 waypoints) → publishes path
- ✅ VehicleController - Pure Pursuit tracking → publishes Gazebo commands

**Expected output:**
```
[INFO] [super_state_spy]: SuperStateSpy node initialized
[INFO] [super_state_spy]: Subscribing to: /model/bgr/odometry
[INFO] [super_state_spy]: Publishing to: /robot/full_state
[INFO] [path_planner]: Loaded racing line: 643 waypoints
[INFO] [path_planner]: Track bounds: X=[-18.89, 97.76], Y=[-3.87, 81.86]
[INFO] [path_planner]: Path Planner started - using racing_line path
[INFO] [vehicle_controller]: Vehicle Controller started
[INFO] [vehicle_controller]: Control: steering=X.XX°, wheel_vel=16.67 rad/s, target_vel=5.00 m/s
```

---

## Monitoring (Optional)

### In Separate Terminals

**Terminal 1: Watch steering commands**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /forward_position_controller/commands
```

**Terminal 2: Watch wheel velocities**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /forward_velocity_controller/commands
```

**Terminal 3: Watch full state**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /robot/full_state
```

**Terminal 4: Watch planned path**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /planned_path
```

---

## Common Commands

### List Active Nodes
```bash
ros2 node list
```

**Expected:**
```
/super_state_spy
/path_planner
/vehicle_controller
```

### List Active Topics
```bash
ros2 topic list
```

**Key topics:**
```
/model/bgr/odometry                        # From Gazebo
/robot/full_state                          # From SuperStateSpy
/planned_path                              # From PathPlanner
/forward_position_controller/commands      # To Gazebo (steering)
/forward_velocity_controller/commands      # To Gazebo (wheels)
```

### Check Topic Rates
```bash
ros2 topic hz /robot/full_state
ros2 topic hz /forward_position_controller/commands
ros2 topic hz /forward_velocity_controller/commands
```

---

## Configuration Options

### Change Path Type

**Circle path:**
```bash
ros2 launch autonomous_car_sim autonomous_car.launch.py \
    path_type:=circle \
    radius:=20.0
```

**Figure-8 path:**
```bash
ros2 launch autonomous_car_sim autonomous_car.launch.py \
    path_type:=figure8 \
    radius:=30.0
```

**Custom racing line:**
```bash
ros2 launch autonomous_car_sim autonomous_car.launch.py \
    path_type:=racing_line \
    racing_line_file:=my_track.npz
```

### Adjust Controller Parameters

```bash
ros2 launch autonomous_car_sim autonomous_car.launch.py \
    target_velocity:=10.0 \
    lookahead_distance:=8.0
```

**Parameters:**
- `target_velocity` - Desired speed (m/s), default: 5.0
- `lookahead_distance` - Pure Pursuit lookahead (m), default: 5.0
- `path_type` - Path to follow, default: racing_line
- `radius` - For circle/figure8 paths (m), default: 20.0

---

## Troubleshooting

### Issue: "No executable found"

**Solution:**
```bash
# Rebuild and create symlinks
colcon build --symlink-install
cd install/autonomous_car_sim/lib/autonomous_car_sim
ln -sf ../../bin/reset_position reset_position
ln -sf ../../bin/super_state_spy super_state_spy
ln -sf ../../bin/path_planner path_planner
ln -sf ../../bin/vehicle_controller vehicle_controller
```

### Issue: "Gazebo service timeout"

**Check Gazebo is running:**
```bash
gz world list
```

**If world name is different:**
```bash
# Update in reset_position.py or use correct world name
gz service -s /world/YOUR_WORLD_NAME/set_pose ...
```

### Issue: Car not moving

**Check topics are connected:**
```bash
ros2 topic info /forward_position_controller/commands -v
ros2 topic info /forward_velocity_controller/commands -v
```

**Manually test Gazebo commands:**
```bash
# Test steering
ros2 topic pub --once /forward_position_controller/commands \
    std_msgs/msg/Float64MultiArray "data: [0.1]"

# Test wheels
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "data: [5.0, 5.0, 5.0, 5.0]"
```

### Issue: Racing line not loading

**Check file exists:**
```bash
ls src/autonomous_car_sim/autonomous_car_sim/racing_line_trackdrive.npz
```

**View path planner logs:**
```bash
# Look for:
# [INFO] [path_planner]: Loaded racing line: 643 waypoints
# If you see "Falling back to circle path" - file is missing
```

---

## System Architecture

```
┌─────────────────────────────────────────┐
│         GAZEBO SIMULATION               │
│  - BGR robot model                      │
│  - Physics engine                       │
│  - /model/bgr/odometry publisher        │
└────────────────┬────────────────────────┘
                 │ Odometry (ground truth)
                 ▼
┌─────────────────────────────────────────┐
│       SuperStateSpy Node                │
│  - Processes odometry                   │
│  - Computes acceleration                │
│  - 12-element state vector              │
└────────────────┬────────────────────────┘
                 │ /robot/full_state
                 ▼
┌─────────────────────────────────────────┐
│       VehicleController Node            │
│  - Pure Pursuit algorithm               │
│  - Uses current_state (not pose!)      │
│  - Computes steering + velocities       │
└────────┬────────────────────────────────┘
         │
         ├──> /forward_position_controller/commands
         │    (Steering)
         │
         └──> /forward_velocity_controller/commands
              (4 wheel velocities)
              │
              ▼
┌─────────────────────────────────────────┐
│      GAZEBO Controller Plugins          │
│  - Position controller (steering)       │
│  - Velocity controller (wheels)         │
└─────────────────────────────────────────┘
```

---

## Complete Example Session

```bash
# Terminal 1: Build and launch
cd /home/bgr/dev/ros_path_tracking
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Reset to start
ros2 run autonomous_car_sim reset_position

# Start autonomous control
ros2 launch autonomous_car_sim autonomous_car.launch.py

# Terminal 2: Monitor steering
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /forward_position_controller/commands

# Terminal 3: Monitor state
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /robot/full_state

# Watch the car follow the racing line! 🏎️
```

---

## Key Files

| File | Purpose |
|------|---------|
| [super_state_spy.py](src/autonomous_car_sim/autonomous_car_sim/super_state_spy.py) | State publisher |
| [path_planner.py](src/autonomous_car_sim/autonomous_car_sim/path_planner.py) | Racing line loader |
| [vehicle_controller.py](src/autonomous_car_sim/autonomous_car_sim/vehicle_controller.py) | Pure Pursuit controller |
| [reset_position.py](src/autonomous_car_sim/autonomous_car_sim/reset_position.py) | Reset to start |
| [autonomous_car.launch.py](src/autonomous_car_sim/launch/autonomous_car.launch.py) | Launch file |
| [racing_line_trackdrive.npz](src/autonomous_car_sim/autonomous_car_sim/racing_line_trackdrive.npz) | Racing line data |

---

## Documentation

- [TEST_RESULTS.md](TEST_RESULTS.md) - Full system testing
- [GAZEBO_ONLY_MODE.md](GAZEBO_ONLY_MODE.md) - VehicleSimulator removal details
- [RACING_LINE_INTEGRATION.md](RACING_LINE_INTEGRATION.md) - Racing line setup
- [RESET_TO_START.md](RESET_TO_START.md) - Reset functionality
- [PLANNER_CONTROL_INTEGRATION.md](PLANNER_CONTROL_INTEGRATION.md) - Integration guide

---

## Success Indicators

### You know it's working when:

✅ All 3 nodes start without errors
✅ SuperStateSpy publishes at ~300+ Hz
✅ Controller logs steering commands every 2.5 seconds
✅ Gazebo topics show data flow
✅ Car position changes in Gazebo
✅ Car follows the racing line path

### Expected Performance:

- **State updates**: ~300-400 Hz
- **Control commands**: ~50 Hz
- **Path updates**: 1 Hz
- **Steering range**: ±30° max
- **Velocity**: 5.0 m/s default (configurable)

---

**Ready to race!** 🏁
