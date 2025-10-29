#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for autonomous car simulation.
    Starts vehicle simulator, path planner, and vehicle controller.
    """

    # Declare launch arguments
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='circle',
        description='Type of path to generate (circle, figure8, straight)'
    )

    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='20.0',
        description='Radius of the path (meters)'
    )

    target_velocity_arg = DeclareLaunchArgument(
        'target_velocity',
        default_value='5.0',
        description='Target velocity for the vehicle (m/s)'
    )

    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='5.0',
        description='Lookahead distance for pure pursuit controller (meters)'
    )

    # Get launch configurations
    path_type = LaunchConfiguration('path_type')
    radius = LaunchConfiguration('radius')
    target_velocity = LaunchConfiguration('target_velocity')
    lookahead_distance = LaunchConfiguration('lookahead_distance')

    # Vehicle Simulator Node
    vehicle_simulator = Node(
        package='autonomous_car_sim',
        executable='vehicle_simulator',
        name='vehicle_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Path Planner Node
    path_planner = Node(
        package='autonomous_car_sim',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'path_type': path_type,
            'radius': radius,
            'num_points': 100,
            'use_sim_time': False,
        }]
    )

    # Vehicle Controller Node
    vehicle_controller = Node(
        package='autonomous_car_sim',
        executable='vehicle_controller',
        name='vehicle_controller',
        output='screen',
        parameters=[{
            'lookahead_distance': lookahead_distance,
            'target_velocity': target_velocity,
            'wheelbase': 2.5,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        path_type_arg,
        radius_arg,
        target_velocity_arg,
        lookahead_distance_arg,
        vehicle_simulator,
        path_planner,
        vehicle_controller,
    ])
