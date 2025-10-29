.PHONY: help build up down clean shell rviz rviz-config logs topics figure-8

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
	@echo "  make topics      - List all active ROS topics"
	@echo "  make logs        - Show container logs"

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
