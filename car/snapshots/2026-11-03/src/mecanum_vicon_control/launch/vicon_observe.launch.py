"""无ESP32的Vicon纯观测：不创建控制、使能、轨迹发生或硬件通信节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('mecanum_description')
    return LaunchDescription([
        # 实际segment名称由Tracker/ROS确认，不猜测car01/base或沿用旧T2。
        DeclareLaunchArgument('vicon_pose_topic', description='实际小车刚体PoseStamped话题，必填'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('base_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('base_offset_y_m', default_value='0.0'),
        DeclareLaunchArgument('base_offset_yaw_rad', default_value='0.0'),
        # Include参数默认写入同一上下文；必须隔离，防止内部false覆盖本入口start_rviz。
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'display.launch.py')),
                launch_arguments={'start_rviz': 'false', 'wheelbase': '0.122',
                                  'track_width': '0.170', 'wheel_diameter': '0.060'}.items(),
            ),
        ]),
        Node(package='mecanum_vicon_control', executable='car_state_bridge', output='screen',
             parameters=[{
                 'input_pose_topic': LaunchConfiguration('vicon_pose_topic'),
                 'world_frame': 'map', 'base_frame': 'base_footprint', 'pose_timeout_s': 0.2,
                 'base_offset_x_m': ParameterValue(LaunchConfiguration('base_offset_x_m'), value_type=float),
                 'base_offset_y_m': ParameterValue(LaunchConfiguration('base_offset_y_m'), value_type=float),
                 'base_offset_yaw_rad': ParameterValue(LaunchConfiguration('base_offset_yaw_rad'), value_type=float),
             }]),
        Node(package='mecanum_vicon_control', executable='car_visualizer', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(description_share, 'rviz', 'mecanum_observe.rviz')],
             condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
