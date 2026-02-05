#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import os
import sys

# Add the ft-fsd-path-planning directory to Python path
# fsd_path = os.path.join(os.path.dirname(__file__), 'ft-fsd-path-planning')
# if fsd_path not in sys.path:
#     sys.path.insert(0, fsd_path)

# from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from std_msgs.msg import Float64MultiArray

# from bgr_description.srv import GetTrack
# from bgr_description.msg import Cone
 


class Planner(Node):
    """
    Path planner that loads racing line from NPZ file or generates geometric paths.
    Default: Loads racing_line_trackdrive.npz from package data.
    """

    def __init__(self):
        super().__init__('path_planner')
        
        # planner parameters
        # self.path_planner = PathPlanner(MissionTypes.trackdrive)
        self.path_planner = None
        self.car_position = None
        self.car_direction = None
        self.cones = None

        # # service client to get cones
        # self.cones_service_client = self.create_client(
        #     GetTrack,
        #     'get_track'
        # )

        # while not self.cones_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f'service {self.cones_service_client.srv_name} not available, waiting...')

        # self.req_track = GetTrack.Request()
        # self.req_track.track_name = "CompetitionMap1"
        # future = self.cones_service_client.call_async(self.req_track)
        # future.add_done_callback(self.load_cones)

        # Declare parameters
        self.declare_parameter('path_type', 'racing_line')  # racing_line, circle, figure8, straight
        self.declare_parameter('racing_line_file', 'racing_line_trackdrive.npz')
        self.declare_parameter('radius', 20.0)
        self.declare_parameter('num_points', 100)

        # Get parameters
        self.path_type = self.get_parameter('path_type').value
        self.racing_line_file = self.get_parameter('racing_line_file').value
        self.radius = self.get_parameter('radius').value
        self.num_points = self.get_parameter('num_points').value

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/robot/full_state',
            self.state_callback,
            10
        )

        # Load racing line if needed
        self.racing_line_waypoints = None
        if self.path_type == 'racing_line':
            self.load_racing_line()

        # Generate and publish path periodically
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(f'Path Planner started - using {self.path_type} path')

    def state_callback(self, msg):
        """
        Receive full state from SuperStateSpy.

        State vector layout (12 elements):
        0: pos_x, 1: pos_y, 2: pos_z
        3: roll, 4: pitch, 5: yaw
        6: vel_x, 7: vel_y, 8: vel_z
        9: acc_x, 10: acc_y, 11: acc_z
        """
        x = msg.data[0]
        y = msg.data[1]
        self.car_position = np.array([x, y])

        yaw = msg.data[5]
        self.car_direction = np.array([np.cos(yaw), np.sin(yaw)])

    def load_racing_line(self):
        """Load racing line waypoints from NPZ file"""
        try:
            # First try to find the file in the package source directory
            package_dir = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(package_dir, self.racing_line_file)

            if not os.path.exists(file_path):
                self.get_logger().error(f'Racing line file not found: {file_path}')
                self.get_logger().warn('Falling back to circle path')
                self.path_type = 'circle'
                return

            # Load the NPZ file
            data = np.load(file_path)

            if 'path' not in data:
                self.get_logger().error('NPZ file does not contain "path" key')
                self.get_logger().warn('Falling back to circle path')
                self.path_type = 'circle'
                return

            # Extract waypoints (assuming [x, y] format)
            self.racing_line_waypoints = data['path']

            self.get_logger().info(
                f'Loaded racing line: {len(self.racing_line_waypoints)} waypoints from {file_path}'
            )

            # Log some statistics
            x_coords = self.racing_line_waypoints[:, 0]
            y_coords = self.racing_line_waypoints[:, 1]
            self.get_logger().info(
                f'Track bounds: X=[{x_coords.min():.2f}, {x_coords.max():.2f}], '
                f'Y=[{y_coords.min():.2f}, {y_coords.max():.2f}]'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load racing line: {str(e)}')
            self.get_logger().warn('Falling back to circle path')
            self.path_type = 'circle'

    def load_cones(self, future):
        try:
            response = future.result()
            self.cones = response.cones # list of Cone messages
            self.get_logger().info(f'Loaded {len(self.cones)} cones from track service.')
            print("\n\n\n\n", self.cones[0], "\n\n\n\n")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        




        
# -----------------------------------------------------------------------------

    def generate_racing_line_path(self):
        """Generate path from loaded racing line waypoints"""
        if self.racing_line_waypoints is None:
            self.get_logger().error('Racing line waypoints not loaded!')
            return self.generate_circle_path()

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        num_waypoints = len(self.racing_line_waypoints)

        for i in range(num_waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            # Position from racing line
            pose.pose.position.x = float(self.racing_line_waypoints[i, 0])
            pose.pose.position.y = float(self.racing_line_waypoints[i, 1])
            pose.pose.position.z = 0.0

            # Calculate orientation from path tangent
            # Use next waypoint to determine heading direction
            next_i = (i + 1) % num_waypoints
            dx = self.racing_line_waypoints[next_i, 0] - self.racing_line_waypoints[i, 0]
            dy = self.racing_line_waypoints[next_i, 1] - self.racing_line_waypoints[i, 1]
            yaw = math.atan2(dy, dx)

            # Convert yaw to quaternion (simplified for 2D)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        return path

    def generate_circle_path(self):
        """Generate a circular path"""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.num_points):
            angle = 2 * math.pi * i / self.num_points
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = self.radius * math.cos(angle)
            pose.pose.position.y = self.radius * math.sin(angle)
            pose.pose.position.z = 0.0

            # Orientation tangent to circle
            yaw = angle + math.pi / 2
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        return path

    def generate_figure8_path(self):
        """Generate a figure-8 (lemniscate) path"""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.num_points):
            t = 2 * math.pi * i / self.num_points
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            # Lemniscate of Gerono
            pose.pose.position.x = self.radius * math.cos(t)
            pose.pose.position.y = self.radius * math.sin(t) * math.cos(t)
            pose.pose.position.z = 0.0

            # Calculate orientation from path tangent
            dx = -self.radius * math.sin(t)
            dy = self.radius * (math.cos(2*t))
            yaw = math.atan2(dy, dx)

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        return path

    def generate_straight_path(self):
        """Generate a straight line path"""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0

            # Orientation along x-axis
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        return path

    def generate_auto_cross_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # calc path
        data = self.path_planner.calculate_path_in_global_frame(self.cones, self.car_position, self.car_direction)

        # fill massage
        for x,y in zip(data[:, 1], data[:, 2]):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0

            path.poses.append(pose)

        return path

# -----------------------------------------------------------------------------

    def publish_path(self):
        """Publish the planned path"""
        if self.path_type == 'racing_line':
            path = self.generate_racing_line_path()
        elif self.path_type == 'auto_cross':
            path = self.generate_auto_cross_path()
        elif self.path_type == 'circle':
            path = self.generate_circle_path()
        elif self.path_type == 'figure8':
            path = self.generate_figure8_path()
        elif self.path_type == 'straight':
            path = self.generate_straight_path()
        else:
            self.get_logger().warn(f'Unknown path type: {self.path_type}, using racing_line')
            path = self.generate_racing_line_path()

        self.path_pub.publish(path)

    def planning(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Planner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
