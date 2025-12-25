#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import math
import numpy as np


class VehicleController(Node):
    """
    Pure Pursuit path-following controller for Gazebo.
    Uses full state from SuperStateSpy and publishes Gazebo control commands.
    No dependency on VehicleSimulator - works directly with Gazebo odometry.
    """

    def __init__(self):
        super().__init__('vehicle_controller')

        # Declare parameters
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('target_velocity', 5.0)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('max_steering_angle', 0.524)  # ~30 degrees

        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.target_velocity = self.get_parameter('target_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering = self.get_parameter('max_steering_angle').value

        # Vehicle state (using only SuperStateSpy full_state, not VehicleSimulator pose)
        self.current_state = None
        self.planned_path = None

        # Publishers
        # Gazebo control publishers (primary output)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/robot/full_state',
            self.state_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Vehicle Controller started')

    def state_callback(self, msg):
        """
        Receive full state from SuperStateSpy.

        State vector layout (12 elements):
        0: pos_x, 1: pos_y, 2: pos_z
        3: roll, 4: pitch, 5: yaw
        6: vel_x, 7: vel_y, 8: vel_z
        9: acc_x, 10: acc_y, 11: acc_z
        """
        self.current_state = {
            'x': msg.data[0],
            'y': msg.data[1],
            'z': msg.data[2],
            'roll': msg.data[3],
            'pitch': msg.data[4],
            'yaw': msg.data[5],
            'vx': msg.data[6],
            'vy': msg.data[7],
            'vz': msg.data[8],
            'ax': msg.data[9],
            'ay': msg.data[10],
            'az': msg.data[11],
            'speed': np.sqrt(msg.data[6]**2 + msg.data[7]**2)
        }

    def path_callback(self, msg):
        """Receive planned path"""
        self.planned_path = msg

    def find_lookahead_point(self):
        """Find the lookahead point on the path using current_state from SuperStateSpy"""
        if self.current_state is None or self.planned_path is None or len(self.planned_path.poses) == 0:
            return None

        # Get current position from state vector
        current_x = self.current_state['x']
        current_y = self.current_state['y']

        min_dist = float('inf')
        closest_idx = 0

        # Find closest point on path
        for i, pose in enumerate(self.planned_path.poses):
            dist = math.sqrt(
                (pose.pose.position.x - current_x) ** 2 +
                (pose.pose.position.y - current_y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Find lookahead point
        for i in range(closest_idx, len(self.planned_path.poses)):
            pose = self.planned_path.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - current_x) ** 2 +
                (pose.pose.position.y - current_y) ** 2
            )

            if dist >= self.lookahead_distance:
                return pose

        # If no point found, return the last point
        return self.planned_path.poses[-1]

    def pure_pursuit_control(self, target_pose):
        """
        Pure pursuit controller to calculate steering angle using current_state.
        Returns the required angular velocity to reach the target.
        """
        # Get current position and heading from state vector
        current_x = self.current_state['x']
        current_y = self.current_state['y']
        theta = self.current_state['yaw']  # Heading angle from SuperStateSpy

        # Transform target point to vehicle frame
        dx = target_pose.pose.position.x - current_x
        dy = target_pose.pose.position.y - current_y

        # Transform to vehicle frame
        target_x = math.cos(theta) * dx + math.sin(theta) * dy
        target_y = -math.sin(theta) * dx + math.cos(theta) * dy

        # Pure pursuit formula
        alpha = math.atan2(target_y, target_x)
        ld = math.sqrt(target_x ** 2 + target_y ** 2)

        if ld < 0.1:
            return 0.0

        # Steering angle
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)

        # Convert to angular velocity (simplified)
        angular_velocity = (2.0 * self.target_velocity * math.sin(alpha)) / ld

        return angular_velocity

    def control_loop(self):
        """Main control loop - uses only current_state from SuperStateSpy"""
        if self.current_state is None or self.planned_path is None:
            return

        # Find lookahead point
        target_pose = self.find_lookahead_point()
        if target_pose is None:
            return

        # Calculate control command
        angular_vel = self.pure_pursuit_control(target_pose)

        # Calculate steering angle from angular velocity
        # For Ackermann steering: delta = atan(L * omega / v)
        if abs(self.target_velocity) > 0.1:
            steering_angle = math.atan2(self.wheelbase * angular_vel, self.target_velocity)
        else:
            steering_angle = 0.0

        # Saturate steering angle to reasonable limits
        steering_angle = max(-self.max_steering, min(steering_angle, self.max_steering))

        # Publish Gazebo steering command
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_angle]
        self.steering_pub.publish(steering_msg)

        # Publish Gazebo velocity command (4 wheels)
        # Convert linear velocity to wheel angular velocity: omega_wheel = v / r
        wheel_angular_vel = self.target_velocity / self.wheel_radius
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [wheel_angular_vel, wheel_angular_vel,
                            wheel_angular_vel, wheel_angular_vel]
        self.velocity_pub.publish(velocity_msg)

        # Log control commands (throttled to avoid spam)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0

        if self._log_counter % 50 == 0:  # Log every 50 cycles (~2.5 seconds at 20Hz)
            self.get_logger().info(
                f'Control: steering={math.degrees(steering_angle):.2f}°, '
                f'wheel_vel={wheel_angular_vel:.2f} rad/s, '
                f'target_vel={self.target_velocity:.2f} m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
