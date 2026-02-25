#!/usr/bin/env python3

from ast import Add

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import os
import sys
import matplotlib.pyplot as plt

# Add the ft-fsd-path-planning directory to Python path
#change to parent directory of this file
fsd_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'ft-fsd-path-planning')
if fsd_path not in sys.path:
    sys.path.insert(0, fsd_path)

from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
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
        self.path_planner = PathPlanner(MissionTypes.trackdrive)
        # self.path_planner = None
        self.car_position = None
        self.car_direction = None
        self.cones = None
        self.lidar_detections: PointCloud2 = None
        self.lidar_cones = None
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

        self.state_sub = self.create_subscription(
            PointCloud2,
            '/lidar/detections',
            self.lidar_detection_callback,
            qos_profile_sensor_data
        )


        
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        plt.ion()
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Path')
        self.cone_unknown_sc = self.ax.scatter([], [], c='gray', s=30, label='Unknown cones', zorder=5)
        self.cone_left_sc = self.ax.scatter([], [], c='yellow', s=40, edgecolors='black', label='Left cones', zorder=5)
        self.cone_right_sc = self.ax.scatter([], [], c='blue', s=40, label='Right cones', zorder=5)
        self.car_marker, = self.ax.plot([], [], 'r^', markersize=10, label='Car', zorder=6)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Auto Cross Path")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.legend(loc='upper right')
        plt.show(block=False)

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

    def lidar_detection_callback(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return  # no detections — keep lidar_cones as-is so the car won't start driving
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        # x is at byte offset 0, y at byte offset 4 — grab exactly those 8 bytes
        # regardless of point_step (avoids wrong reshape when step != 12)
        if self.car_position is None:
            self.get_logger().warn('Received lidar detections but car position is unknown, ignoring.')
            return
        car_xy = np.array(self.car_position[:2], dtype=np.float32)

        # Extract XY from raw lidar points
        xy = raw[:, :8].view(np.float32).reshape(n, 2).copy()

        # Add car position safely
        xy += car_xy


        cones_by_type = [np.zeros((0, 2)) for _ in range(len(ConeTypes))] # the outer list has 5 elements, one for each ConeTypes, then each element is an array of shape (N, 2)
        cones_by_type[ConeTypes.LEFT] = np.array([])
        cones_by_type[ConeTypes.RIGHT] = np.array([])
        cones_by_type[ConeTypes.START_FINISH_LINE] = np.array([])
        cones_by_type[ConeTypes.START_FINISH_AREA] = np.array([])
        cones_by_type[ConeTypes.UNKNOWN] = np.array(xy)
        self.get_logger().info(f'Received lidar detection: {len(xy)} points')
        self.lidar_cones = cones_by_type

        # Update cone scatter plot immediately on new lidar data
        def _set(sc, arr):
            if arr is not None and arr.ndim == 2 and len(arr) > 0:
                sc.set_offsets(arr)
            else:
                sc.set_offsets(np.empty((0, 2)))

        _set(self.cone_unknown_sc, self.lidar_cones[ConeTypes.UNKNOWN])
        _set(self.cone_left_sc,    self.lidar_cones[ConeTypes.LEFT])
        _set(self.cone_right_sc,   self.lidar_cones[ConeTypes.RIGHT])
        if self.car_position is not None:
            self.car_marker.set_xdata([self.car_position[0]])
            self.car_marker.set_ydata([self.car_position[1]])

        # ax.relim() ignores scatter (PathCollection) — compute bounds manually
        all_pts = [xy]
        if self.car_position is not None:
            all_pts.append(self.car_position.reshape(1, 2))
        pts = np.vstack(all_pts)
        pad = 5.0
        self.ax.set_xlim(pts[:, 0].min() - pad, pts[:, 0].max() + pad)
        self.ax.set_ylim(pts[:, 1].min() - pad, pts[:, 1].max() + pad)
        self.fig.canvas.draw_idle()


      
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
        
        if self.lidar_cones is None or self.car_position is None or self.car_direction is None:
            return path

        # Don't plan until we have actual cone detections
        unknown_cones = self.lidar_cones[ConeTypes.UNKNOWN]
        if unknown_cones is None or len(unknown_cones) == 0:
            return path

        data = self.path_planner.calculate_path_in_global_frame(
            self.lidar_cones,
            self.car_position,
            self.car_direction
        ) 

        # Extract XY
        x_vals = data[:, 1]
        y_vals = data[:, 2]

        # Update path line
        self.path_line.set_xdata(x_vals)
        self.path_line.set_ydata(y_vals)

        # Update cone scatter plots
        def _set_scatter(sc, arr):
            if arr is not None and arr.ndim == 2 and len(arr) > 0:
                sc.set_offsets(arr)
            else:
                sc.set_offsets(np.empty((0, 2)))

        _set_scatter(self.cone_unknown_sc, self.lidar_cones[ConeTypes.UNKNOWN])
        _set_scatter(self.cone_left_sc, self.lidar_cones[ConeTypes.LEFT])
        _set_scatter(self.cone_right_sc, self.lidar_cones[ConeTypes.RIGHT])

        # Update car position
        if self.car_position is not None:
            self.car_marker.set_xdata([self.car_position[0]])
            self.car_marker.set_ydata([self.car_position[1]])

        self.fig.canvas.draw_idle()

        for x, y in zip(x_vals, y_vals):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # ← fix: valid quaternion

            path.poses.append(pose)

        return path


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
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
