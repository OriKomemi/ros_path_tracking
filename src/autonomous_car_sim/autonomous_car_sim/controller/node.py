#!/usr/bin/env python3
import math
import numpy as np
import time
import matplotlib.pyplot as plt
from collections import deque
# scipy import for smoother plotting, but not required
try:
    from scipy.interpolate import make_interp_spline
    _SCIPY_AVAILABLE = True
except Exception:
    _SCIPY_AVAILABLE = False

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
    torque_vectoring,
    mean_curvature_at_index,
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

        # Initialize speed plotting (timestamps, measured, target)
        self.init_speed_plot()

        # Smoothed values for EMA (initialized when first data arrives)
        self._v_meas_smoothed = None
        self._v_tgt_smoothed = None
        self._last_plot_save_time = time.time()

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
        #vel_msg.data = [wheel_omega, wheel_omega, wheel_omega, wheel_omega]
        vel_msg.data = torque_vectoring(steering_angle,wheel_omega,self.cfg['torque_const'])
        self.velocity_pub.publish(vel_msg)

    def _maybe_log(self, text: str):
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 < self.cfg['log_period_s']:
            return
        self._last_log_time = now
        self.get_logger().info(text)


    def init_speed_plot(self):
        """Initialize speed vs time plotting buffers and figure."""
        self._plot_time = deque(maxlen=2000)
        self._plot_v_meas = deque(maxlen=2000)
        self._plot_v_tgt = deque(maxlen=2000)

        try:
            self._plot_fig, self._plot_ax = plt.subplots()
            (self._plot_line_meas,) = self._plot_ax.plot([], [], label='v_meas')
            (self._plot_line_tgt,) = self._plot_ax.plot([], [], label='v_tgt')
            self._plot_ax.set_xlabel('Time [s]')
            self._plot_ax.set_ylabel('Speed [m/s]')
            self._plot_ax.legend()
            
            # Rolling window plot: show last N seconds (60s default)
            self._plot_window_size = 60.0  # seconds
            self._plot_ax.set_ylim([0, 12])  # fixed y-axis for speed
            
            self._plot_start_time = time.time()
            self.create_timer(0.5, self._update_speed_plot)
        except Exception:
            self._plot_fig = None
            self._plot_ax = None
            self._plot_line_meas = None
            self._plot_line_tgt = None
            self._plot_start_time = time.time()

    def _update_speed_plot(self):
        """Non-blocking redraw of the live speed plot with rolling window (fixed scale)."""
        if not getattr(self, '_plot_fig', None) or not self._plot_time:
            return
        try:
            t = np.array(self._plot_time)
            v_meas_list = np.array(self._plot_v_meas)
            v_tgt_list = np.array(self._plot_v_tgt)

            # Rolling window: show only the last N seconds
            window_size = getattr(self, '_plot_window_size', 60.0)
            if len(t) > 0:
                t_max = t[-1]
                t_min = max(0, t_max - window_size)
                
                # Filter data within window
                mask = (t >= t_min) & (t <= t_max)
                t_window = t[mask]
                v_meas_window = v_meas_list[mask]
                v_tgt_window = v_tgt_list[mask]
            else:
                t_window = t
                v_meas_window = v_meas_list
                v_tgt_window = v_tgt_list

            # Use B-spline interpolation for smooth curves when available and enabled
            if _SCIPY_AVAILABLE and self.cfg.get('plot_interp', True) and t_window.size >= 4 and (t_window[-1] - t_window[0]) > 1e-6:
                # create dense time grid
                num_out = max(int(self.cfg.get('plot_interp_points', 200)), t_window.size * 10)
                t_new = np.linspace(t_window[0], t_window[-1], num_out)
                try:
                    spline_meas = make_interp_spline(t_window, v_meas_window, k=3)
                    v_meas_smooth = spline_meas(t_new)
                except Exception:
                    t_new = t_window
                    v_meas_smooth = v_meas_window

                try:
                    spline_tgt = make_interp_spline(t_window, v_tgt_window, k=3)
                    v_tgt_smooth = spline_tgt(t_new)
                except Exception:
                    t_new = t_window
                    v_tgt_smooth = v_tgt_window

                self._plot_line_meas.set_data(t_new, v_meas_smooth)
                self._plot_line_tgt.set_data(t_new, v_tgt_smooth)
            else:
                # Fallback: plot raw (or EMA-smoothed) points
                self._plot_line_meas.set_data(t_window, v_meas_window)
                self._plot_line_tgt.set_data(t_window, v_tgt_window)

            # Set fixed x and y axes (no autoscaling)
            if len(t) > 0:
                t_max = t[-1]
                t_min = max(0, t_max - window_size)
                self._plot_ax.set_xlim([t_min, t_max])
            
            self._plot_ax.set_ylim([0, 12])  # Fixed y-axis
            
            self._plot_fig.canvas.draw()
            plt.pause(0.001)
        except Exception:
            return


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

        #kappa_tgt = curvature_at_index(self.path, target_idx)
        kappa_tgt = mean_curvature_at_index(self.path, ci, target_idx,int(self.cfg['static_lookahead_curv']))  # smoother curvature for speed target
        v_tgt = self.target_speed_from_curvature(kappa_tgt)

        v_err = v_tgt - v_meas
        a_cmd = self.pid.update(v_err, self.cfg['control_dt'])

        dv_max = self.cfg['speed_rate_limit'] * self.cfg['control_dt']
        dv = clamp(a_cmd * self.cfg['control_dt'], -dv_max, dv_max)

        self.v_cmd = clamp(self.v_cmd + dv, 0.0, self.cfg['v_max'])
        self._publish_commands(steering_angle, self.v_cmd)

        # Record speeds for plotting (safe if plotting disabled)
        try:
            now = time.time() - getattr(self, '_plot_start_time', time.time())
            # Exponential smoothing for measured and target speeds
            a_meas = float(self.cfg.get('plot_smooth_alpha', 0.3))
            a_tgt = float(self.cfg.get('plot_tgt_smooth_alpha', 0.3))

            if self._v_meas_smoothed is None:
                self._v_meas_smoothed = v_meas
            else:
                self._v_meas_smoothed = a_meas * v_meas + (1.0 - a_meas) * self._v_meas_smoothed

            if self._v_tgt_smoothed is None:
                self._v_tgt_smoothed = v_tgt
            else:
                self._v_tgt_smoothed = a_tgt * v_tgt + (1.0 - a_tgt) * self._v_tgt_smoothed

            self._plot_time.append(now)
            self._plot_v_meas.append(self._v_meas_smoothed)
            self._plot_v_tgt.append(self._v_tgt_smoothed)
        except Exception:
            pass

        self._maybe_log(
            f"Ld={Ld:.2f}m | kappa_local={kappa_local:.4f} 1/m | kappa_tgt={kappa_tgt:.4f} 1/m | "
            f"v_tgt={v_tgt:.2f} | v_meas={v_meas:.2f} | v_cmd={self.v_cmd:.2f} | "
            f"steer={math.degrees(steering_angle):.1f}deg"
        )

        # Save plot PNG periodically (every 5 seconds)
        try:
            now = time.time()
            if now - self._last_plot_save_time > 5.0:
                fig = getattr(self, '_plot_fig', None)
                if fig is not None:
                    fig.savefig('speed_plot.png', dpi=100)
                    self._last_plot_save_time = now
        except Exception:
            pass




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

        # Keep plot open after shutdown
        try:
            fig = getattr(node, '_plot_fig', None)
            if fig is not None:
                fig.savefig('speed_plot.png', dpi=150)
                plt.ioff()  
                plt.show(block = True)  # blocks until window closed
        except Exception:
            pass


if __name__ == '__main__':
    main()
