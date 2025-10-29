FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source files
COPY src/ /ros2_ws/src/

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install && \
    mkdir -p install/autonomous_car_sim/lib/autonomous_car_sim && \
    ln -sf ../../bin/vehicle_simulator install/autonomous_car_sim/lib/autonomous_car_sim/vehicle_simulator && \
    ln -sf ../../bin/path_planner install/autonomous_car_sim/lib/autonomous_car_sim/path_planner && \
    ln -sf ../../bin/vehicle_controller install/autonomous_car_sim/lib/autonomous_car_sim/vehicle_controller

# Source the workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]

CMD ["bash"]
