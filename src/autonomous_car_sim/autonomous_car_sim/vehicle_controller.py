#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math


class VehicleController(Node):
    """
    Simple path-following controller using Pure Pursuit algorithm.
    Subscribes to vehicle pose and planned path, publishes control commands.
    """

    def __init__(self):
        super().__init__('vehicle_controller')

        # Declare parameters
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('target_velocity', 5.0)
        self.declare_parameter('wheelbase', 2.5)

        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.target_velocity = self.get_parameter('target_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value

        # Vehicle state
        self.current_pose = None
        self.planned_path = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/vehicle/cmd_vel', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle/pose',
            self.pose_callback,
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

    def pose_callback(self, msg):
        """Receive current vehicle pose"""
        self.current_pose = msg

    def path_callback(self, msg):
        """Receive planned path"""
        self.planned_path = msg

    def find_lookahead_point(self):
        """Find the lookahead point on the path"""
        if self.planned_path is None or len(self.planned_path.poses) == 0:
            return None

        min_dist = float('inf')
        closest_idx = 0

        # Find closest point on path
        for i, pose in enumerate(self.planned_path.poses):
            dist = math.sqrt(
                (pose.pose.position.x - self.current_pose.pose.position.x) ** 2 +
                (pose.pose.position.y - self.current_pose.pose.position.y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Find lookahead point
        for i in range(closest_idx, len(self.planned_path.poses)):
            pose = self.planned_path.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - self.current_pose.pose.position.x) ** 2 +
                (pose.pose.position.y - self.current_pose.pose.position.y) ** 2
            )

            if dist >= self.lookahead_distance:
                return pose

        # If no point found, return the last point
        return self.planned_path.poses[-1]

    def pure_pursuit_control(self, target_pose):
        """
        Pure pursuit controller to calculate steering angle.
        Returns the required angular velocity to reach the target.
        """
        # Transform target point to vehicle frame
        dx = target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.pose.position.y

        # Get vehicle heading from quaternion
        q = self.current_pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

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
        """Main control loop"""
        if self.current_pose is None or self.planned_path is None:
            return

        # Find lookahead point
        target_pose = self.find_lookahead_point()
        if target_pose is None:
            return

        # Calculate control command
        angular_vel = self.pure_pursuit_control(target_pose)

        # Publish command
        cmd = Twist()
        cmd.linear.x = self.target_velocity
        cmd.angular.z = angular_vel

        self.cmd_pub.publish(cmd)

        # Update vehicle heading based on angular velocity
        # This is a simplification - in reality the vehicle simulator handles this
        if self.current_pose is not None:
            dt = 0.05
            q = self.current_pose.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            new_theta = theta + angular_vel * dt

            # Update the simulator's theta through the velocity command
            # The simulator will integrate this


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
