from rclpy.node import Node


# ymal params -> declare + load functions (best practice)
def declare_params(node: Node) -> None:
    """
    Best practice: declare parameters even when YAML supplies them.
    Keeps types/defaults + prevents silent typos.
    """
    node.declare_parameters(
        namespace='',
        parameters=[
            # Vehicle
            ('wheelbase', 2.0368),
            ('wheel_radius', 0.24),
            ('max_steering_angle', 0.524),

            # Dynamic lookahead
            ('lookahead_min', 2.0),
            ('lookahead_max', 12.0),
            ('lookahead_speed_gain', 0.8),
            ('lookahead_curv_gain', 8.0),

            # Speed & curvature
            ('v_max', 10.0),
            ('v_min', 1.0),
            ('a_lat_max', 3.0),
            ('kappa_eps', 1e-4),
            ('speed_rate_limit', 2.0),

            # PID
            ('pid_kp', 1.2),
            ('pid_ki', 0.1),
            ('pid_kd', 0.05),
            ('pid_i_min', -3.0),
            ('pid_i_max', 3.0),

            # Loop + logging
            ('control_dt', 0.05),
            ('log_period_s', 2.5),
        ]
    )


def load_params(node: Node) -> dict:
    """
    Returns a plain dict (simple, no dataclasses).
    """
    def p(name: str) -> float:
        return float(node.get_parameter(name).value)

    return {
        'wheelbase': p('wheelbase'),
        'wheel_radius': p('wheel_radius'),
        'max_steering_angle': p('max_steering_angle'),

        'lookahead_min': p('lookahead_min'),
        'lookahead_max': p('lookahead_max'),
        'lookahead_speed_gain': p('lookahead_speed_gain'),
        'lookahead_curv_gain': p('lookahead_curv_gain'),

        'v_max': p('v_max'),
        'v_min': p('v_min'),
        'a_lat_max': p('a_lat_max'),
        'kappa_eps': p('kappa_eps'),
        'speed_rate_limit': p('speed_rate_limit'),

        'pid_kp': p('pid_kp'),
        'pid_ki': p('pid_ki'),
        'pid_kd': p('pid_kd'),
        'pid_i_min': p('pid_i_min'),
        'pid_i_max': p('pid_i_max'),

        'control_dt': p('control_dt'),
        'log_period_s': p('log_period_s'),
    }
