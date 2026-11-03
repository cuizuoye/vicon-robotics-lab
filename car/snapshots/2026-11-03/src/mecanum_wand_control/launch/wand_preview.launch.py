"""Subscribe to an already-running Vicon receiver; publish preview graphics only."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mecanum_wand_control'),
                          'config', 'wand_preview.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument('car_yaw_offset_rad', default_value='2.356194490192345'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        Node(package='mecanum_wand_control', executable='sign_readonly_preview',
             namespace='wand_preview', output='screen', parameters=[{
                 'wand_topic': '/vicon/Sign/Sign', 'car_topic': '/vicon/car01/car01',
                 'world_frame': 'map', 'scale': 2.0, 'auto_capture': True,
                 'record_dir': ParameterValue(LaunchConfiguration('record_dir'), value_type=str),
                 'car_yaw_offset_rad': ParameterValue(
                     LaunchConfiguration('car_yaw_offset_rad'), value_type=float),
             }]),
        Node(package='rviz2', executable='rviz2', namespace='wand_preview',
             name='sign_readonly_rviz', arguments=['-d', config],
             condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
