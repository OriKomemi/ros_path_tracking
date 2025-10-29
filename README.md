# ROS 2 Jazzy Autonomous Car Simulator

A simple Docker-based autonomous car simulation project using ROS 2 Jazzy. This project demonstrates basic autonomous vehicle concepts including path planning and path-following control.

## Features

- **Vehicle Simulator**: Simple 2D kinematic bicycle model
- **Path Planner**: Generates reference trajectories (circle, figure-8, straight line)
- **Vehicle Controller**: Pure Pursuit path-following controller
- **Visualization**: RViz2 markers and path visualization
- **Dockerized**: Complete ROS 2 Jazzy environment in Docker

## Project Structure

```
.
├── Dockerfile                          # ROS 2 Jazzy Docker image
├── docker-compose.yml                  # Docker Compose configuration
├── docker-entrypoint.sh               # Container entrypoint script
├── README.md                          # This file
└── src/
    └── autonomous_car_sim/            # Main ROS 2 package
        ├── package.xml                # Package manifest
        ├── setup.py                   # Python package setup
        ├── autonomous_car_sim/        # Python source code
        │   ├── __init__.py
        │   ├── vehicle_simulator.py   # Vehicle dynamics simulation
        │   ├── path_planner.py        # Reference path generation
        │   └── vehicle_controller.py  # Pure Pursuit controller
        ├── launch/                    # Launch files
        │   └── autonomous_car.launch.py
        └── config/                    # Configuration files
            └── params.yaml
```

## Prerequisites

- Docker
- Docker Compose
- X11 server (for visualization on Linux)

## Quick Start

### 1. Build the Docker Image

```bash
make build
# or
docker-compose build
```

### 2. Run the Simulation

```bash
# Start the simulation
make up
# or
docker-compose up
```

The simulation will start three ROS 2 nodes:
- **vehicle_simulator**: Simulates the vehicle dynamics
- **path_planner**: Generates the reference path
- **vehicle_controller**: Controls the vehicle to follow the path

### 3. Visualize in RViz2

You have two options for visualization:

#### Option A: Pre-configured RViz2 (Recommended)

In a new terminal, run:

```bash
make rviz-config
```

This will launch RViz2 with all visualizations already configured:
- **Blue arrow**: The autonomous vehicle
- **Green circle**: The planned path (20m radius)
- **Red trail**: The vehicle's actual trajectory

#### Option B: Manual RViz2 Setup

```bash
make rviz
```

Then manually configure RViz2:
1. Set **Fixed Frame** to `odom` (in left panel under "Global Options")
2. Click **Add** button and add:
   - **Marker** → Topic: `/vehicle/marker` (blue arrow showing vehicle)
   - **Path** → Topic: `/planned_path` (green planned trajectory)
   - **Odometry** → Topic: `/vehicle/odometry` (red trail showing movement)
3. Adjust the view: Use mouse to zoom/pan the camera to see the circular path

## Running Different Path Types

You can run the simulation with different path types:

### Circle Path (Default)
```bash
docker-compose up
```

### Figure-8 Path
```bash
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2_autonomous_car \
  ros2 launch autonomous_car_sim autonomous_car.launch.py path_type:=figure8
```

### Straight Line Path
```bash
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2_autonomous_car \
  ros2 launch autonomous_car_sim autonomous_car.launch.py path_type:=straight
```

### Custom Parameters
```bash
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2_autonomous_car \
  ros2 launch autonomous_car_sim autonomous_car.launch.py \
    path_type:=circle \
    radius:=30.0 \
    target_velocity:=8.0 \
    lookahead_distance:=7.0
```

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vehicle/cmd_vel` | `geometry_msgs/Twist` | Control commands to vehicle |
| `/vehicle/odometry` | `nav_msgs/Odometry` | Vehicle odometry |
| `/vehicle/pose` | `geometry_msgs/PoseStamped` | Vehicle pose |
| `/vehicle/marker` | `visualization_msgs/Marker` | Vehicle visualization |
| `/planned_path` | `nav_msgs/Path` | Planned trajectory |

## Nodes

### vehicle_simulator
Simulates vehicle dynamics using a simple kinematic bicycle model.

**Published Topics:**
- `/vehicle/odometry` (nav_msgs/Odometry)
- `/vehicle/pose` (geometry_msgs/PoseStamped)
- `/vehicle/marker` (visualization_msgs/Marker)

**Subscribed Topics:**
- `/vehicle/cmd_vel` (geometry_msgs/Twist)

### path_planner
Generates reference trajectories for the vehicle to follow.

**Parameters:**
- `path_type`: Type of path (circle, figure8, straight)
- `radius`: Path radius in meters (default: 20.0)
- `num_points`: Number of points in the path (default: 100)

**Published Topics:**
- `/planned_path` (nav_msgs/Path)

### vehicle_controller
Pure Pursuit controller for path following.

**Parameters:**
- `lookahead_distance`: Lookahead distance in meters (default: 5.0)
- `target_velocity`: Target velocity in m/s (default: 5.0)
- `wheelbase`: Vehicle wheelbase in meters (default: 2.5)

**Subscribed Topics:**
- `/vehicle/pose` (geometry_msgs/PoseStamped)
- `/planned_path` (nav_msgs/Path)

**Published Topics:**
- `/vehicle/cmd_vel` (geometry_msgs/Twist)

## Development

### Interactive Development

To develop and test interactively inside the container:

```bash
# Start container with bash
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/src:/ros2_ws/src:rw \
  ros2_autonomous_car bash

# Inside container, rebuild after changes
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run nodes individually
ros2 run autonomous_car_sim vehicle_simulator
ros2 run autonomous_car_sim path_planner
ros2 run autonomous_car_sim vehicle_controller
```

### Monitor Topics

Using the Makefile:

```bash
# List all active topics
make topics

# View container logs
make logs

# Open shell in container for manual commands
make shell
```

Or manually inside the container:

```bash
# Access container shell
docker exec -it ros2_autonomous_car bash

# Inside the container:
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /vehicle/pose

# Check publishing rate
ros2 topic hz /vehicle/odometry

# View topic info
ros2 topic info /planned_path
```

## Makefile Commands Reference

The project includes a Makefile for convenient operations:

| Command | Description |
|---------|-------------|
| `make help` | Show all available commands |
| `make build` | Build the Docker image (enables X11 forwarding) |
| `make up` | Start the simulation |
| `make down` | Stop the simulation |
| `make clean` | Remove containers and images |
| `make shell` | Open bash shell in the running container |
| `make rviz` | Launch RViz2 (requires manual setup) |
| `make rviz-config` | Launch RViz2 with pre-configured visualization ⭐ |
| `make topics` | List all active ROS 2 topics |
| `make logs` | Show simulation logs (follow mode) |

**Quick workflow:**
```bash
make build          # Build once
make up             # Start simulation in terminal 1
make rviz-config    # Start visualization in terminal 2
make logs           # Monitor logs in terminal 3 (optional)
```

## Configuration

Edit `src/autonomous_car_sim/config/params.yaml` to modify default parameters:

- Vehicle dynamics (wheelbase, max velocity, etc.)
- Path parameters (type, radius, number of points)
- Controller parameters (lookahead distance, target velocity)

## Troubleshooting

### Display Issues (Linux)

If you can't see RViz2:
```bash
xhost +local:docker
export DISPLAY=:0
```

### Build Issues

If you encounter build errors:
```bash
# Rebuild from scratch
docker-compose build --no-cache
```

### Container Access

To access a running container:
```bash
docker exec -it ros2_autonomous_car bash
```

## License

Apache-2.0

## Future Enhancements

- Add obstacle avoidance
- Implement dynamic path replanning
- Add sensor simulation (LIDAR, camera)
- Implement more advanced controllers (MPC, LQR)
- Add multi-vehicle simulation
- Integrate with Gazebo for 3D simulation
