#!/usr/bin/env python3
"""
Reset BGR car to track start position.
Publishes zero commands and uses Gazebo service to set pose.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import subprocess
import time


class PositionResetter(Node):
    """Node to reset car position to track start"""

    def __init__(self):
        super().__init__('position_resetter')

        # Publishers for control commands
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

        # Track start position (from racing_line_trackdrive.npz)
        self.start_x = 45.751
        self.start_y = 81.3
        self.start_z = 1.0

        # Start orientation (nearly aligned with track)
        self.start_qx = -0.0001
        self.start_qy = -0.0007
        self.start_qz = 1.232886347679596e-06
        self.start_qw = 0.9999

        self.get_logger().info('Position Resetter initialized')

    def stop_commands(self):
        """Publish zero commands to stop the car"""
        self.get_logger().info('Stopping control commands...')

        # Zero steering
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0]
        self.steering_pub.publish(steering_msg)

        # Zero velocities for all 4 wheels
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.velocity_pub.publish(velocity_msg)

        # Wait for commands to take effect
        time.sleep(0.5)

    def reset_to_start(self):
        """Reset car to track start position using Gazebo service"""
        self.get_logger().info('Resetting car to track start position...')

        # Build Gazebo service command
        cmd = [
            'gz', 'service',
            '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req',
            f"name: 'bgr', "
            f"position: {{x: {self.start_x}, y: {self.start_y}, z: {self.start_z}}}, "
            f"orientation: {{x: {self.start_qx}, y: {self.start_qy}, "
            f"z: {self.start_qz}, w: {self.start_qw}}}"
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                self.get_logger().info(f'✅ Car reset to start position: ({self.start_x}, {self.start_y}, {self.start_z})')
                if 'true' in result.stdout.lower():
                    self.get_logger().info('Gazebo service confirmed successful pose update')
            else:
                self.get_logger().error(f'Failed to reset position: {result.stderr}')

        except subprocess.TimeoutExpired:
            self.get_logger().error('Gazebo service timeout - is Gazebo running?')
        except FileNotFoundError:
            self.get_logger().error('Gazebo (gz) command not found - is Gazebo installed?')
        except Exception as e:
            self.get_logger().error(f'Error calling Gazebo service: {str(e)}')

    def execute_reset(self):
        """Execute full reset sequence"""
        self.get_logger().info('=== Starting Reset Sequence ===')

        # Step 1: Stop commands
        self.stop_commands()

        # Step 2: Reset position
        self.reset_to_start()

        # Step 3: Wait for physics to settle
        self.get_logger().info('Waiting for physics to settle...')
        time.sleep(1.0)

        # Step 4: Publish zero commands again to be safe
        self.stop_commands()

        self.get_logger().info('=== Reset Complete ===')
        self.get_logger().info('Car is ready at track start position')
        self.get_logger().info('You can now start autonomous control')


def main(args=None):
    rclpy.init(args=args)

    node = PositionResetter()

    try:
        # Execute the reset sequence
        node.execute_reset()

        # Keep node alive briefly to ensure messages are sent
        time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
