#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import osqp
from scipy import sparse


class VehicleController(Node):
    """
    MPC path-following controller for Gazebo.
    Uses full state from SuperStateSpy and publishes Gazebo control commands.
    Uses OSQP-based Model Predictive Control for lateral control.
    """

    def __init__(self):
        super().__init__('vehicle_controller')

        # Declare parameters
        self.declare_parameter('target_velocity', 50.0)
        self.declare_parameter('wheelbase', 1.5)
        self.declare_parameter('wheel_radius', 0.165)  # 13-inch rim radius
        self.declare_parameter('max_steering_angle', 0.524)  # ~30 degrees
        self.declare_parameter('max_wheel_vel', 350.0)  # rad/s
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('dt', 0.05)
        # PID gains for velocity control
        self.declare_parameter('pid_kp', 5.0)
        self.declare_parameter('pid_ki', 1.0)
        self.declare_parameter('pid_kd', 0.2)

        # Get parameters
        self.target_velocity = self.get_parameter('target_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self.N = self.get_parameter('mpc_horizon').value
        self.dt = self.get_parameter('dt').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value

        # MPC weights (reduced for smoother steering)
        self.Q = np.diag([3.0, 1.0])   # [ey, epsi] - lateral error, heading error
        self.R = np.array([[1.5]])     # steering input weight (higher = smoother)

        # PID state
        self.vel_error_integral = 0.0
        self.vel_error_prev = 0.0

        # Vehicle state
        self.current_state = None
        self.planned_path = None
        self._log_counter = 0

        # Publishers
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )

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
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('MPC Vehicle Controller started')

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

    def compute_errors(self):
        """Compute lateral error (ey) and heading error (epsi) from nearest path point"""
        x = self.current_state['x']
        y = self.current_state['y']
        yaw = self.current_state['yaw']

        # Find nearest path point
        min_d = float('inf')
        ref = None

        for p in self.planned_path.poses:
            dx = x - p.pose.position.x
            dy = y - p.pose.position.y
            d = dx * dx + dy * dy
            if d < min_d:
                min_d = d
                ref = p

        # Get reference yaw from path orientation
        yaw_ref = self.yaw_from_quat(ref.pose.orientation)

        # Compute errors in Frenet frame
        dx = x - ref.pose.position.x
        dy = y - ref.pose.position.y

        # Lateral error (cross-track error)
        ey = -math.sin(yaw_ref) * dx + math.cos(yaw_ref) * dy
        # Heading error
        epsi = self.normalize_angle(yaw - yaw_ref)

        return ey, epsi

    def build_model(self, v):
        """Build discrete-time lateral dynamics model"""
        # State: [ey, epsi], Input: [delta]
        # ey_dot = v * sin(epsi) ≈ v * epsi
        # epsi_dot = v / L * tan(delta) ≈ v / L * delta
        A = np.array([[1.0, v * self.dt],
                      [0.0, 1.0]])

        B = np.array([[0.0],
                      [v * self.dt / self.wheelbase]])
        return A, B

    def solve_mpc(self, A, B, ey, epsi):
        """Solve MPC using OSQP"""
        nx, nu = 2, 1
        N = self.N

        x0 = np.array([ey, epsi])

        # Build block diagonal weight matrices
        Qbar = sparse.block_diag([self.Q] * N)
        Rbar = sparse.block_diag([self.R] * N)

        # Build prediction matrices
        Ax = np.zeros((nx * N, nx))
        Bu = np.zeros((nx * N, nu * N))

        for i in range(N):
            Ai = np.linalg.matrix_power(A, i + 1)
            Ax[i * nx:(i + 1) * nx, :] = Ai
            for j in range(i + 1):
                Aij = np.linalg.matrix_power(A, i - j)
                Bu[i * nx:(i + 1) * nx, j * nu:(j + 1) * nu] = Aij @ B

        # QP matrices: min 0.5 * u' * H * u + f' * u
        H = Bu.T @ Qbar @ Bu + Rbar
        f = Bu.T @ Qbar @ (Ax @ x0)

        # Steering constraints
        Aineq = sparse.eye(N)
        l = -self.max_steering * np.ones(N)
        u = self.max_steering * np.ones(N)

        # Solve QP
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix(H),
            q=f,
            A=Aineq,
            l=l,
            u=u,
            verbose=False
        )

        res = solver.solve()

        if res.info.status != 'solved':
            self.get_logger().warn('OSQP failed to solve')
            return 0.0

        return res.x[0]

    def velocity_pid_control(self, current_speed):
        """PID controller for velocity tracking"""
        # Compute error
        vel_error = self.target_velocity - current_speed

        # Integral term with anti-windup
        self.vel_error_integral += vel_error * self.dt
        max_integral = self.max_wheel_vel / (self.pid_ki + 1e-6)
        self.vel_error_integral = np.clip(
            self.vel_error_integral, -max_integral, max_integral
        )

        # Derivative term
        vel_error_derivative = (vel_error - self.vel_error_prev) / self.dt
        self.vel_error_prev = vel_error

        # PID output (desired linear velocity adjustment)
        vel_cmd = (
            self.pid_kp * vel_error +
            self.pid_ki * self.vel_error_integral +
            self.pid_kd * vel_error_derivative
        )

        # Convert to wheel angular velocity and add feedforward
        feedforward = self.target_velocity / self.wheel_radius
        wheel_angular_vel = feedforward + vel_cmd / self.wheel_radius

        # Clamp to limits
        wheel_angular_vel = float(np.clip(wheel_angular_vel, 0.0, self.max_wheel_vel))

        return wheel_angular_vel, vel_error

    def control_loop(self):
        """Main MPC control loop"""
        # Always publish velocity even without path (allows car to start moving)
        if self.current_state is None:
            return

        # If no path yet, just drive straight with PID velocity control
        if self.planned_path is None or len(self.planned_path.poses) == 0:
            # Publish zero steering
            steering_msg = Float64MultiArray()
            steering_msg.data = [0.0]
            self.steering_pub.publish(steering_msg)

            # PID velocity control
            current_speed = self.current_state['speed']
            wheel_angular_vel, _ = self.velocity_pid_control(current_speed)

            velocity_msg = Float64MultiArray()
            velocity_msg.data = [wheel_angular_vel] * 4
            self.velocity_pub.publish(velocity_msg)
            return

        # Use target velocity for model (allows starting from rest)
        v = max(self.current_state['speed'], self.target_velocity * 0.5)

        # Compute tracking errors
        ey, epsi = self.compute_errors()

        # Build linearized model
        A, B = self.build_model(v)

        # Solve MPC for optimal steering
        steering_angle = self.solve_mpc(A, B, ey, epsi)

        # Publish steering command
        steering_msg = Float64MultiArray()
        steering_msg.data = [float(steering_angle)]
        self.steering_pub.publish(steering_msg)

        # PID velocity control
        current_speed = self.current_state['speed']
        wheel_angular_vel, vel_error = self.velocity_pid_control(current_speed)

        # Publish velocity command (4 wheels)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [wheel_angular_vel] * 4
        self.velocity_pub.publish(velocity_msg)

        # Throttled logging
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(
                f'MPC Control: steering={math.degrees(steering_angle):.2f}°, '
                f'ey={ey:.3f}m, epsi={math.degrees(epsi):.2f}°, '
                f'vel={current_speed:.2f}/{self.target_velocity:.2f} m/s, '
                f'wheel_vel={wheel_angular_vel:.2f} rad/s'
            )

    @staticmethod
    def yaw_from_quat(q):
        """Extract yaw angle from quaternion"""
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def normalize_angle(a):
        """Normalize angle to [-pi, pi]"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


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
