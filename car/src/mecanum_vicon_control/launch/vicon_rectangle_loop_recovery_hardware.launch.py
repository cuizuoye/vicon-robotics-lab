"""Inert 3 m x 2 m continuous Vicon recovery rectangle for 4WGB.

Launching starts only monitoring and zero commands.  Motion requires an
explicit /car/start_loop service call.  /car/stop closes the trajectory, ROS
speed gate and STM32 gate.  Each completed lap is paused and safely rearmed so
the STM32 150 s independent run limit is never disabled or bypassed.
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
    max_linear_mps = LaunchConfiguration('max_linear_mps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vicon_pose_topic', default_value='/vicon/car01/car01'),
        DeclareLaunchArgument('esp_ip', default_value='192.168.50.1'),
        DeclareLaunchArgument(
            'network_interface', default_value='wlx000000000000'),
        DeclareLaunchArgument('rectangle_width_m', default_value='3.0'),
        DeclareLaunchArgument('rectangle_height_m', default_value='2.0'),
        DeclareLaunchArgument('rectangle_waypoints', default_value='4'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.15'),
        DeclareLaunchArgument(
            'max_recoverable_deviation_m', default_value='0.65'),
        DeclareLaunchArgument(
            'base_offset_yaw_rad', default_value='2.356194490192345'),
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
                    max_linear_mps, value_type=float),
                'tracking_cross_speed_mps': ParameterValue(
                    max_linear_mps, value_type=float),
                'recovery_cross_speed_mps': ParameterValue(
                    max_linear_mps, value_type=float),
                'max_linear_speed_mps': ParameterValue(
                    max_linear_mps, value_type=float),
                'max_recoverable_deviation_m': ParameterValue(
                    LaunchConfiguration('max_recoverable_deviation_m'),
                    value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='safety_watchdog',
            output='screen',
            parameters=[control_config, bounded_config, {
                'start_enabled': False,
                'require_run_lease': True,
                'run_lease_timeout_s': 0.30,
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
                'waypoint_count': ParameterValue(
                    LaunchConfiguration('rectangle_waypoints'), value_type=int),
                'loop_count': 0,
                'waypoint_dwell_s': 0.80,
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
                'invert_body_xy': False,
                'max_vx_mps': ParameterValue(max_linear_mps, value_type=float),
                'max_vy_mps': ParameterValue(max_linear_mps, value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='continuous_loop_supervisor',
            output='screen',
            parameters=[{
                'required_motion_tag': '4WGB',
                'lap_timeout_s': 140.0,
                'lease_rate_hz': 20.0,
            }],
        ),
    ])
