"""Real Sign to isolated software car; source receiver must already be running."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mecanum_wand_control'), 'config', 'sign_live_sim.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument('auto_engage', default_value='true', choices=['true', 'false']),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '99'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='mecanum_wand_control', executable='wand_sim_car', namespace='wand_sim',
             output='screen', parameters=[{
                 'cmd_topic': 'cmd_vel', 'pose_topic': 'car_pose', 'disturbance_topic': 'disturbance',
                 'frame_id': 'wand_sim_map', 'position_noise_std_m': 0.0, 'command_timeout_s': .30,
             }]),
        Node(package='mecanum_wand_control', executable='wand_sim_controller',
             namespace='wand_sim', output='screen',
             parameters=[{'scale': 2.0, 'display_label': 'REAL Sign -> SIM CAR'}]),
        Node(package='mecanum_wand_control', executable='sign_to_simulation',
             namespace='wand_sim', output='screen', arguments=[
                 '--record-dir', LaunchConfiguration('record_dir'),
                 '--auto-engage', LaunchConfiguration('auto_engage')]),
        Node(package='rviz2', executable='rviz2', namespace='wand_sim', name='sign_live_sim_rviz',
             arguments=['-d', config], condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
