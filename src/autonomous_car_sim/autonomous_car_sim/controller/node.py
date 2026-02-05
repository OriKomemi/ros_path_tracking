#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray

from .pid import PID
from .params import declare_params, load_params
from .math_utils import clamp
from .path_utils import (
    speed_from_vxy,
    closest_index,
    curvature_at_index,
    find_lookahead_point,
    pure_pursuit_delta,
)


class VehicleController(Node):
    """
    Same logic as your single-file controller, just cleaner + split into modules.
    """

    # /robot/full_state layout (12 elements)
    IDX_X = 0
    IDX_Y = 1
    IDX_YAW = 5
    IDX_VX = 6
    IDX_VY = 7
    MIN_STATE_LEN = 12

    def __init__(self):
        super().__init__('vehicle_controller')

        # Params
        declare_params(self)
        self.cfg = load_params(self)

        self.pid = PID(
            kp=self.cfg['pid_kp'],
            ki=self.cfg['pid_ki'],
            kd=self.cfg['pid_kd'],
            i_min=self.cfg['pid_i_min'],
            i_max=self.cfg['pid_i_max'],
        )

        # State (kept simple dict, no custom structures)
        self.state = None  # {'x','y','yaw','vx','vy','speed'}
        self.path = None   # nav_msgs/Path
        self.v_cmd = 0.0

        # Publishers
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )

        # Subscribers
        self.create_subscription(Float64MultiArray, '/robot/full_state', self._on_state, 10)
        self.create_subscription(Path, '/planned_path', self._on_path, 10)

        # Timer
        self.create_timer(self.cfg['control_dt'], self.control_loop)

        # Logging throttle
        self._last_log_time = self.get_clock().now()

        self.get_logger().info('Vehicle Controller started')

    def _on_state(self, msg: Float64MultiArray):
        if len(msg.data) < self.MIN_STATE_LEN:
            self.get_logger().warn(f'/robot/full_state length={len(msg.data)} < {self.MIN_STATE_LEN}')
            return

        vx = float(msg.data[self.IDX_VX])
        vy = float(msg.data[self.IDX_VY])

        self.state = {
            'x': float(msg.data[self.IDX_X]),
            'y': float(msg.data[self.IDX_Y]),
            'yaw': float(msg.data[self.IDX_YAW]),
            'vx': vx,
            'vy': vy,
            'speed': speed_from_vxy(vx, vy),
        }

    def _on_path(self, msg: Path):
        self.path = msg

    # --- same math as before ---
    def target_speed_from_curvature(self, kappa: float) -> float:
        k = max(abs(kappa), self.cfg['kappa_eps'])
        v = math.sqrt(max(self.cfg['a_lat_max'], 0.0) / k)
        return clamp(v, self.cfg['v_min'], self.cfg['v_max'])

    def dynamic_lookahead(self, v_meas: float, kappa: float) -> float:
        base = self.cfg['lookahead_min'] + self.cfg['lookahead_speed_gain'] * max(v_meas, 0.0)
        base = clamp(base, self.cfg['lookahead_min'], self.cfg['lookahead_max'])

        scale = 1.0 + self.cfg['lookahead_curv_gain'] * abs(kappa)
        ld = base / max(scale, 1e-6)

        return clamp(ld, self.cfg['lookahead_min'], self.cfg['lookahead_max'])

    # --- ROS publish ---
    def _publish_commands(self, steering_angle: float, v_cmd: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_angle]
        self.steering_pub.publish(steering_msg)

        wheel_omega = v_cmd / max(self.cfg['wheel_radius'], 1e-6)
        vel_msg = Float64MultiArray()
        vel_msg.data = [wheel_omega, wheel_omega, wheel_omega, wheel_omega]
        self.velocity_pub.publish(vel_msg)

    def _maybe_log(self, text: str):
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 < self.cfg['log_period_s']:
            return
        self._last_log_time = now
        self.get_logger().info(text)

    def control_loop(self):
        if self.state is None or self.path is None or not self.path.poses:
            return

        x = self.state['x']
        y = self.state['y']
        yaw = self.state['yaw']
        v_meas = self.state['speed']

        ci = closest_index(x, y, self.path)
        if ci is None:
            return

        kappa_local = curvature_at_index(self.path, ci)
        Ld = self.dynamic_lookahead(v_meas, kappa_local)

        target_pose, target_idx = find_lookahead_point(x, y, self.path, Ld)
        if target_pose is None or target_idx is None:
            return

        delta = pure_pursuit_delta(x, y, yaw, target_pose, self.cfg['wheelbase'])
        steering_angle = clamp(delta, -self.cfg['max_steering_angle'], self.cfg['max_steering_angle'])

        kappa_tgt = curvature_at_index(self.path, target_idx)
        v_tgt = self.target_speed_from_curvature(kappa_tgt)

        v_err = v_tgt - v_meas
        a_cmd = self.pid.update(v_err, self.cfg['control_dt'])

        dv_max = self.cfg['speed_rate_limit'] * self.cfg['control_dt']
        dv = clamp(a_cmd * self.cfg['control_dt'], -dv_max, dv_max)

        self.v_cmd = clamp(self.v_cmd + dv, 0.0, self.cfg['v_max'])

        self._publish_commands(steering_angle, self.v_cmd)

        self._maybe_log(
            f"Ld={Ld:.2f}m | kappa_local={kappa_local:.4f} 1/m | kappa_tgt={kappa_tgt:.4f} 1/m | "
            f"v_tgt={v_tgt:.2f} | v_meas={v_meas:.2f} | v_cmd={self.v_cmd:.2f} | "
            f"steer={math.degrees(steering_angle):.1f}deg"
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
