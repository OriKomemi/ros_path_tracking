#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class PathPlanner(Node):
    """
    Simple path planner that generates a reference trajectory.
    Publishes a circular or figure-8 path for the vehicle to follow.
    """

    def __init__(self):
        super().__init__('path_planner')

        # Declare parameters
        self.declare_parameter('path_type', 'circle')
        self.declare_parameter('radius', 20.0)
        self.declare_parameter('num_points', 100)

        # Get parameters
        self.path_type = self.get_parameter('path_type').value
        self.radius = self.get_parameter('radius').value
        self.num_points = self.get_parameter('num_points').value

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Generate and publish path periodically
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(f'Path Planner started - generating {self.path_type} path')

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

    def publish_path(self):
        """Publish the planned path"""
        if self.path_type == 'circle':
            path = self.generate_circle_path()
        elif self.path_type == 'figure8':
            path = self.generate_figure8_path()
        elif self.path_type == 'straight':
            path = self.generate_straight_path()
        else:
            self.get_logger().warn(f'Unknown path type: {self.path_type}, using circle')
            path = self.generate_circle_path()

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
