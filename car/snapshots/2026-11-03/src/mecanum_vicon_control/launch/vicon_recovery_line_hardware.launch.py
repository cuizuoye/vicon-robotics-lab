"""Inert first hardware migration for Vicon disturbance recovery.

This launch starts the 4WGB safe-rearm transport and the new path controller,
but both the ROS safety gate and the STM32 motion gate remain disabled.  It
defines one 1 m forward segment at 0.15 m/s; a separate one-shot run guard is
still required to authorize motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_vicon_control')
    control_config = os.path.join(package_share, 'config', 'control.yaml')
    bounded_config = os.path.join(package_share, 'config', 'bounded_motion.yaml')
    recovery_config = os.path.join(package_share, 'config', 'path_recovery.yaml')

    vicon_pose_topic = LaunchConfiguration('vicon_pose_topic')
    esp_ip = LaunchConfiguration('esp_ip')
    network_interface = LaunchConfiguration('network_interface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vicon_pose_topic',
            default_value='/vicon/car01/car01',
            description='Validated PoseStamped topic for the car01 rigid body',
        ),
        DeclareLaunchArgument('esp_ip', default_value='192.168.50.1'),
        DeclareLaunchArgument('network_interface', default_value='wlx000000000000'),
        DeclareLaunchArgument('line_length_m', default_value='1.0'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.15'),
        DeclareLaunchArgument(
            'base_offset_yaw_rad', default_value='2.356194490192345'),
        DeclareLaunchArgument(
            'invert_body_xy', default_value='false',
            description='4WGB keeps the ground-verified ROS-to-STM32 direction calibration',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_state_bridge',
            output='screen',
            parameters=[control_config, {
                'input_pose_topic': vicon_pose_topic,
                'base_offset_yaw_rad': ParameterValue(
                    LaunchConfiguration('base_offset_yaw_rad'), value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='path_recovery_controller',
            output='screen',
            parameters=[recovery_config, {
                'nominal_speed_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'tracking_cross_speed_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'recovery_cross_speed_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'max_linear_speed_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='safety_watchdog',
            output='screen',
            parameters=[control_config, bounded_config, {'start_enabled': False}],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='trajectory_generator',
            output='screen',
            parameters=[{
                'shape': 'line',
                'world_frame': 'map',
                'width_m': ParameterValue(
                    LaunchConfiguration('line_length_m'), value_type=float),
                'waypoint_count': 2,
                'loop_count': 1,
                'heading_mode': 'fixed',
                'fixed_yaw_rad': 0.0,
                'relative_to_start': True,
                'start_pose_timeout_s': 0.20,
                'start_enabled': False,
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='cmd_vel_udp_bridge',
            output='screen',
            parameters=[control_config, bounded_config, {
                'esp_ip': esp_ip,
                'network_interface': network_interface,
                'udp_low_latency_qos': False,
                'required_motion_tag': '4WGB',
                'wire_timeout_ms': 600,
                'invert_body_xy': ParameterValue(
                    LaunchConfiguration('invert_body_xy'), value_type=bool),
                'max_vx_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'max_vy_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
            }],
        ),
    ])
