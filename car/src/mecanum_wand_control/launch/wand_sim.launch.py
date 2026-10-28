"""Offline only. No include of hardware launch files or physical transport."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('mecanum_wand_control'), 'config', 'wand_sim.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='97', choices=['97', '98', '99']),
        DeclareLaunchArgument('scripted', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='mecanum_wand_control', executable='wand_sim_car',
             namespace='wand_sim', output='screen', parameters=[{
                 'cmd_topic': 'cmd_vel', 'pose_topic': 'car_pose',
                 'disturbance_topic': 'disturbance', 'frame_id': 'wand_sim_map',
                 'position_noise_std_m': 0.0, 'command_timeout_s': 0.30,
             }]),
        Node(package='mecanum_wand_control', executable='wand_sim_controller',
             namespace='wand_sim', output='screen', parameters=[{'scale': 2.0}]),
        Node(package='mecanum_wand_control', executable='wand_demo',
             namespace='wand_sim', output='screen',
             condition=IfCondition(LaunchConfiguration('scripted'))),
        Node(package='rviz2', executable='rviz2', namespace='wand_sim',
             arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
