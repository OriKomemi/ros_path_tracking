.PHONY: help build up down clean shell rviz rviz-config logs topics figure-8 circle build-local test-local

help:
	@echo "ROS 2 Autonomous Car Simulator - Available commands:"
	@echo "  make build       - Build Docker image"
	@echo "  make up          - Start simulation"
	@echo "  make down        - Stop simulation"
	@echo "  make clean       - Remove containers and images"
	@echo "  make shell       - Open bash shell in container"
	@echo "  make rviz        - Launch RViz2 visualization (manual setup required)"
	@echo "  make rviz-config - Launch RViz2 with pre-configured visualization"
	@echo "  make figure-8    - Launch simulation with figure-8 path and RViz"
	@echo "  make circle      - Launch simulation with circle path and RViz"
	@echo "  make topics      - List all active ROS topics"
	@echo "  make logs        - Show container logs"
	@echo "  make build-local - Build ROS 2 workspace locally (no Docker)"
	@echo "  make test-local  - Test launch file locally (no Docker)"

build:
	xhost +local:docker
	docker compose build

up:
	xhost +local:docker
	docker compose up

down:
	docker compose down

clean:
	docker compose down --rmi all --volumes

shell:
	docker exec -it ros2_autonomous_car bash

rviz:
	@echo "Launching RViz2 (you'll need to manually add visualizations)..."
	@echo "See README.md for setup instructions"
	docker exec -e DISPLAY=${DISPLAY} ros2_autonomous_car bash -c \
		"source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2"

rviz-config:
	@echo "Copying RViz config to container..."
	@docker cp autonomous_car.rviz ros2_autonomous_car:/tmp/
	@echo "Launching RViz2 with pre-configured visualization..."
	docker exec -e DISPLAY=${DISPLAY} ros2_autonomous_car bash -c \
		"source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2 -d /tmp/autonomous_car.rviz"

topics:
	@echo "Active ROS 2 Topics:"
	@docker exec ros2_autonomous_car bash -c \
		"source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list"

logs:
	docker compose logs -f

figure-8:
	@echo "Stopping any running containers..."
	@docker compose down 2>/dev/null || true
	@echo "Starting simulation with figure-8 path..."
	xhost +local:docker
	@docker compose run -d --name ros2_autonomous_car autonomous_car \
		ros2 launch autonomous_car_sim autonomous_car.launch.py path_type:=figure8
	@echo "Waiting for simulation to initialize..."
	@sleep 3
	@echo "Copying RViz config to container..."
	@docker cp autonomous_car.rviz ros2_autonomous_car:/tmp/
	@echo "Launching RViz2 with pre-configured visualization..."
	@docker exec -e DISPLAY=${DISPLAY} ros2_autonomous_car bash -c \
		"source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2 -d /tmp/autonomous_car.rviz"

circle:
	@echo "Stopping any running containers..."
	@docker compose down 2>/dev/null || true
	@echo "Starting simulation with circle path..."
	xhost +local:docker
	@docker compose run -d --name ros2_autonomous_car autonomous_car \
		ros2 launch autonomous_car_sim autonomous_car.launch.py path_type:=circle
	@echo "Waiting for simulation to initialize..."
	@sleep 3
	@echo "Copying RViz config to container..."
	@docker cp autonomous_car.rviz ros2_autonomous_car:/tmp/
	@echo "Launching RViz2 with pre-configured visualization..."
	@docker exec -e DISPLAY=${DISPLAY} ros2_autonomous_car bash -c \
		"source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2 -d /tmp/autonomous_car.rviz"

build-local:
	@echo "Building ROS 2 workspace locally..."
	colcon build --symlink-install

test-local:
	@echo "Testing launch file locally..."
	@echo "Make sure you have sourced your ROS 2 environment first:"
	@echo "  source /opt/ros/<distro>/setup.bash"
	@echo "  source install/setup.bash"
	@echo ""
	@echo "Then run:"
	@echo "  ros2 launch autonomous_car_sim autonomous_car.launch.py"
