#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math


class VehicleSimulator(Node):
    """
    Simple 2D vehicle simulator with basic bicycle model dynamics.
    Subscribes to control commands and publishes vehicle state.
    """

    def __init__(self):
        super().__init__('vehicle_simulator')

        # Vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading angle
        self.velocity = 0.0
        self.angular_velocity = 0.0

        # Vehicle parameters
        self.wheelbase = 2.5  # meters
        self.max_velocity = 10.0  # m/s
        self.max_steering_angle = math.radians(30)  # radians

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle/pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/vehicle/marker', 10)

        # Subscriber for control commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/vehicle/cmd_vel',
            self.cmd_callback,
            10
        )

        # Timer for simulation update
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Vehicle Simulator started')

    def cmd_callback(self, msg):
        """Receive control commands (linear and angular velocity)"""
        self.velocity = max(-self.max_velocity, min(msg.linear.x, self.max_velocity))
        self.angular_velocity = msg.angular.z

    def update(self):
        """Update vehicle state using simple bicycle model"""
        # Simple kinematic bicycle model
        self.x += self.velocity * math.cos(self.theta) * self.dt
        self.y += self.velocity * math.sin(self.theta) * self.dt
        self.theta += self.angular_velocity * self.dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish state
        self.publish_odometry()
        self.publish_pose()
        self.publish_marker()

    def publish_odometry(self):
        """Publish vehicle odometry"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = self.velocity
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

    def publish_pose(self):
        """Publish vehicle pose"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(self.theta / 2.0)
        pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pose_pub.publish(pose)

    def publish_marker(self):
        """Publish vehicle visualization marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'vehicle'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0

        marker.pose.orientation.z = math.sin(self.theta / 2.0)
        marker.pose.orientation.w = math.cos(self.theta / 2.0)

        marker.scale.x = 2.0
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
