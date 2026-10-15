"""6x5m定航向矩形：仅仿真或Vicon预览，不加载UDP硬件桥。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('mecanum_vicon_control')
    common = {
        'use_trajectory': 'true',
        'trajectory_shape': 'rectangle',
        'trajectory_width': LaunchConfiguration('forward_m'),
        'trajectory_height': LaunchConfiguration('left_m'),
        'trajectory_waypoints': LaunchConfiguration('waypoints'),
        'trajectory_loops': LaunchConfiguration('loops'),
        'trajectory_heading_mode': 'fixed',
        'trajectory_fixed_yaw': '0.0',
        'trajectory_relative_to_start': 'true',
        'start_rviz': LaunchConfiguration('start_rviz'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='simulation',
                              choices=['simulation', 'vicon_preview']),
        DeclareLaunchArgument('forward_m', default_value='6.0'),
        DeclareLaunchArgument('left_m', default_value='5.0'),
        DeclareLaunchArgument('waypoints', default_value='8'),
        DeclareLaunchArgument('loops', default_value='0',
                              description='0 repeats until operator cancels; positive stops after N loops'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('vicon_pose_topic', default_value='/vicon/car01/base'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'software_in_loop.launch.py')),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'simulation')),
            # 先等话题就绪，再由操作者启动；不自动解除看门狗锁止。
            launch_arguments={**common, 'use_demo_goal': 'false',
                              'simulation_start_enabled': 'false',
                              'trajectory_start_enabled': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'vicon_control.launch.py')),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'vicon_preview')),
            launch_arguments={**common, 'vicon_pose_topic': LaunchConfiguration('vicon_pose_topic'),
                              'trajectory_start_enabled': 'false'}.items(),
        ),
    ])
