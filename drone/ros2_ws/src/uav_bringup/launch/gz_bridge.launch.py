"""启动 Gazebo <-> ROS 2 桥，并可选把 Gazebo 真值里程计发布成 TF。

用法:
    ros2 launch uav_bringup gz_bridge.launch.py
    ros2 launch uav_bringup gz_bridge.launch.py bridge_config:=/path/to/gz_bridge.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_bringup')

    default_bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'uav_sim.rviz')

    bridge_config = LaunchConfiguration('bridge_config')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_gz_tf = LaunchConfiguration('use_gz_tf')
    use_px4_odom = LaunchConfiguration('use_px4_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config', default_value=default_bridge_config,
            description='ros_gz_bridge 的配置 yaml'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否同时启动 RViz2'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='RViz2 配置文件'),
        DeclareLaunchArgument(
            'use_gz_tf', default_value='true',
            description='把 Gazebo 真值里程计发布为 TF (world->base_link)'),
        DeclareLaunchArgument(
            'use_px4_odom', default_value='true',
            description='把 PX4 里程计转成 ENU 并发 TF (map->base_link)，'
                        'Agent 没起时自动无数据'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),

        # Gazebo 真值里程计 -> TF，方便在 RViz2 里看机体姿态
        Node(
            package='uav_coord_bridge',
            executable='odom_to_tf',
            name='gz_odom_to_tf',
            output='screen',
            condition=IfCondition(use_gz_tf),
            parameters=[{
                'odom_topic': '/gz/odometry',
                'parent_frame': 'odom',
                'child_frame': 'gz_base_link',
            }],
        ),

        # PX4 EKF2 里程计 (NED/FRD) -> ROS ENU/FLU + TF map->base_link
        Node(
            package='uav_coord_bridge',
            executable='px4_odometry_to_ros',
            name='px4_odometry_to_ros',
            output='screen',
            condition=IfCondition(use_px4_odom),
            parameters=[{
                'parent_frame': 'map',
                'child_frame': 'base_link',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
