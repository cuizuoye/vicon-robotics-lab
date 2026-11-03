"""Hardware-isolated Vicon path-recovery software-in-the-loop demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_vicon_control')
    control_config = os.path.join(package_share, 'config', 'control.yaml')
    recovery_config = os.path.join(package_share, 'config', 'path_recovery.yaml')
    description_share = get_package_share_directory('mecanum_description')
    description_launch = PythonLaunchDescriptionSource(
        os.path.join(description_share, 'launch', 'display.launch.py'))
    recovery_rviz = os.path.join(
        description_share, 'rviz', 'disturbance_recovery.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rectangle_width_m', default_value='2.0'),
        DeclareLaunchArgument('rectangle_height_m', default_value='1.0'),
        DeclareLaunchArgument('nominal_speed_mps', default_value='0.10'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            description_launch,
            launch_arguments={
                'start_rviz': 'false',
                'wheelbase': '0.122',
                'track_width': '0.170',
                'wheel_diameter': '0.060',
            }.items(),
        ),
        Node(
            package='mecanum_vicon_control',
            executable='simulated_vicon_car',
            output='screen',
            parameters=[control_config, {'position_noise_std_m': 0.0}],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_state_bridge',
            output='screen',
            parameters=[control_config],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='path_recovery_controller',
            output='screen',
            parameters=[recovery_config, {
                'nominal_speed_mps': ParameterValue(
                    LaunchConfiguration('nominal_speed_mps'), value_type=float),
                'max_linear_speed_mps': ParameterValue(
                    LaunchConfiguration('nominal_speed_mps'), value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='safety_watchdog',
            output='screen',
            parameters=[control_config, {
                'start_enabled': ParameterValue(
                    LaunchConfiguration('start_enabled'), value_type=bool),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='trajectory_generator',
            output='screen',
            parameters=[{
                'shape': 'rectangle',
                'world_frame': 'map',
                'width_m': ParameterValue(
                    LaunchConfiguration('rectangle_width_m'), value_type=float),
                'height_m': ParameterValue(
                    LaunchConfiguration('rectangle_height_m'), value_type=float),
                'waypoint_count': 8,
                'loop_count': 1,
                'heading_mode': 'fixed',
                'fixed_yaw_rad': 0.0,
                'relative_to_start': True,
                'start_enabled': ParameterValue(
                    LaunchConfiguration('start_enabled'), value_type=bool),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_visualizer',
            output='screen',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='recovery_visualizer',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', recovery_rviz],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
