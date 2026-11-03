"""Real-car Vicon rectangle stack.

This launch file is deliberately inert: both the ROS safety gate and the STM32
motion gate start disabled.  Run ``rectangle_run_guard`` only after the car is
placed in a clear test area and an operator can remove motor power.
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

    common_parameters = [control_config, bounded_config]
    vicon_pose_topic = LaunchConfiguration('vicon_pose_topic')
    esp_ip = LaunchConfiguration('esp_ip')
    network_interface = LaunchConfiguration('network_interface')
    required_motion_tag = LaunchConfiguration('required_motion_tag')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vicon_pose_topic',
            default_value='/vicon/car01/car01',
            description='Validated PoseStamped topic for the car01 rigid body',
        ),
        DeclareLaunchArgument('esp_ip', default_value='192.168.50.1'),
        DeclareLaunchArgument('network_interface', default_value='wlx000000000000'),
        DeclareLaunchArgument('rectangle_width_m', default_value='0.30'),
        DeclareLaunchArgument('rectangle_height_m', default_value='0.20'),
        DeclareLaunchArgument('rectangle_waypoints', default_value='8'),
        DeclareLaunchArgument('required_motion_tag', default_value='4WG7'),
        DeclareLaunchArgument(
            'invert_body_xy', default_value='true',
            description=(
                'Rotate ROS body translations by 180 degrees at the STM32 wire boundary; '
                'measured on the installed car by hand-push versus powered Vicon motion'),
        ),
        DeclareLaunchArgument(
            'max_linear_mps', default_value='0.03',
            description='Profile-specific linear cap; 4WG7<=0.05, 4WG8<=0.20 m/s',
        ),
        DeclareLaunchArgument(
            'base_offset_yaw_rad',
            default_value='2.356194490192345',
            description='Provisional car01-to-base heading offset from hand-push calibration',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_state_bridge',
            output='screen',
            parameters=common_parameters + [{
                'input_pose_topic': vicon_pose_topic,
                'base_offset_yaw_rad': ParameterValue(
                    LaunchConfiguration('base_offset_yaw_rad'), value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='pose_controller',
            output='screen',
            parameters=common_parameters + [{
                'max_vx_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'max_vy_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
            }],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='safety_watchdog',
            output='screen',
            parameters=common_parameters + [{'start_enabled': False}],
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
            parameters=common_parameters + [{
                'esp_ip': esp_ip,
                'network_interface': network_interface,
                'udp_low_latency_qos': False,
                'required_motion_tag': required_motion_tag,
                'invert_body_xy': ParameterValue(
                    LaunchConfiguration('invert_body_xy'), value_type=bool),
                'max_vx_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
                'max_vy_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float),
            }],
        ),
    ])
