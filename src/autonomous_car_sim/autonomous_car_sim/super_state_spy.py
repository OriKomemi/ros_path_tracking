#!/usr/bin/env python3
"""
SuperStateSpy - State Publisher Node

Subscribes to Gazebo's odometry topic and publishes a comprehensive state vector
containing position, orientation, velocity, and acceleration information.

This provides everything a planner and controller needs for autonomous vehicle control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion
import numpy as np


class SuperStateSpy(Node):
    """
    Processes odometry and computes derived quantities (acceleration).

    Subscribed Topic:
        /model/bgr/odometry (nav_msgs/msg/Odometry)
            - Ground truth odometry from Gazebo simulation
            - Uses BEST_EFFORT QoS to match Gazebo's publisher

    Published Topic:
        /robot/full_state (std_msgs/msg/Float64MultiArray)
            - 12-element array with complete state information
            - Published at simulation rate (~1000 Hz in Gazebo)

    State Vector Layout (12 elements):
        Index | Data    | Units   | Description
        ------|---------|---------|----------------------------------
        0     | pos_x   | meters  | Global X position
        1     | pos_y   | meters  | Global Y position
        2     | pos_z   | meters  | Global Z position
        3     | roll    | radians | Rotation around X axis
        4     | pitch   | radians | Rotation around Y axis
        5     | yaw     | radians | Rotation around Z axis (heading)
        6     | vel_x   | m/s     | Linear velocity in X
        7     | vel_y   | m/s     | Linear velocity in Y
        8     | vel_z   | m/s     | Linear velocity in Z
        9     | acc_x   | m/s²    | Linear acceleration in X
        10    | acc_y   | m/s²    | Linear acceleration in Y
        11    | acc_z   | m/s²    | Linear acceleration in Z
    """

    def __init__(self):
        super().__init__('super_state_spy')

        # QoS profile to match Gazebo's BEST_EFFORT policy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to odometry from simulator
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/bgr/odometry',
            self.odom_callback,
            qos_profile
        )

        # Publish full state vector
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/robot/full_state',
            10
        )

        # Previous velocity for acceleration computation
        self.prev_velocity = np.array([0.0, 0.0, 0.0])
        self.prev_time = None

        # Filter window for acceleration (moving average)
        self.accel_window_size = 5
        self.accel_history = []

        self.get_logger().info('SuperStateSpy node initialized')
        self.get_logger().info('Subscribing to: /model/bgr/odometry')
        self.get_logger().info('Publishing to: /robot/full_state')

    def odom_callback(self, msg: Odometry):
        """
        Process odometry and publish full state vector.

        Args:
            msg: Odometry message from simulator
        """
        # Extract position
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        # Extract orientation (convert quaternion to euler angles)
        orientation = msg.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Extract velocity
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z

        # Compute acceleration via numerical differentiation
        current_time = self.get_clock().now()
        current_velocity = np.array([vel_x, vel_y, vel_z])

        if self.prev_time is not None:
            # Calculate time delta
            dt = (current_time - self.prev_time).nanoseconds / 1e9

            if dt > 0:
                # Numerical differentiation: a = (v - v_prev) / dt
                raw_accel = (current_velocity - self.prev_velocity) / dt

                # Apply moving average filter to reduce noise
                self.accel_history.append(raw_accel)
                if len(self.accel_history) > self.accel_window_size:
                    self.accel_history.pop(0)

                filtered_accel = np.mean(self.accel_history, axis=0)
                acc_x, acc_y, acc_z = filtered_accel
            else:
                acc_x = acc_y = acc_z = 0.0
        else:
            # First message - no acceleration available
            acc_x = acc_y = acc_z = 0.0

        # Update previous values for next iteration
        self.prev_velocity = current_velocity
        self.prev_time = current_time

        # Construct state vector (12 elements)
        state_msg = Float64MultiArray()
        state_msg.data = [
            pos_x,   # 0: X position
            pos_y,   # 1: Y position
            pos_z,   # 2: Z position
            roll,    # 3: Roll angle
            pitch,   # 4: Pitch angle
            yaw,     # 5: Yaw angle (heading)
            vel_x,   # 6: X velocity
            vel_y,   # 7: Y velocity
            vel_z,   # 8: Z velocity
            acc_x,   # 9: X acceleration
            acc_y,   # 10: Y acceleration
            acc_z    # 11: Z acceleration
        ]

        # Publish state vector
        self.state_pub.publish(state_msg)


def main(args=None):
    """Main entry point for SuperStateSpy node."""
    rclpy.init(args=args)
    node = SuperStateSpy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
