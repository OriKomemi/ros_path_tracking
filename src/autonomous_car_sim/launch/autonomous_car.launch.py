#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----------------------------
    # Launch arguments (presets)
    # ----------------------------
    planner_preset_arg = DeclareLaunchArgument(
        'planner_preset',
        default_value='racing',  # racing | circle | figure8 | straight
        description='Planner preset name (maps to config/planner_<name>.yaml)'
    )

    controller_preset_arg = DeclareLaunchArgument(
        'controller_preset',
        default_value='safe',  # safe | fast | race
        description='Controller preset name (maps to config/controller_<name>.yaml)'
    )

    planner_preset = LaunchConfiguration('planner_preset')
    controller_preset = LaunchConfiguration('controller_preset')

    pkg_share = FindPackageShare('autonomous_car_sim')

    # ----------------------------
    # Param files (best practice: base + overlay)
    # ----------------------------
    controller_base_yaml = PathJoinSubstitution([pkg_share, 'config', 'controller_base.yaml'])
    controller_yaml = PathJoinSubstitution([pkg_share, 'config', ['controller_', controller_preset, '.yaml']])

    planner_yaml = PathJoinSubstitution([pkg_share, 'config', ['planner_', planner_preset, '.yaml']])

    # ----------------------------
    # Nodes
    # ----------------------------
    path_planner = Node(
        package='autonomous_car_sim',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            planner_yaml
        ],
    )

    vehicle_controller = Node(
        package='autonomous_car_sim',
        executable='vehicle_controller',
        name='vehicle_controller',
        output='screen',
        parameters=[
            controller_base_yaml,
            controller_yaml,
        ],
    )

    return LaunchDescription([
        planner_preset_arg,
        controller_preset_arg,
        path_planner,
        vehicle_controller,
    ])
