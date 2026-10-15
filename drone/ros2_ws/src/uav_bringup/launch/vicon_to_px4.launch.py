"""把 Vicon (动作捕捉) 位姿送进 PX4 EKF2 作为外部位姿源。

前提:
  1. PX4 端已使能 mocap:  见 config/px4_mocap_params.txt
  2. MicroXRCEAgent 已运行 (scripts/41_run_ros2_bridge.sh 或 sim_ros.launch.py)
  3. Vicon 接收节点已发布 geometry_msgs/PoseStamped

用法:
    ros2 launch uav_bringup vicon_to_px4.launch.py
    ros2 launch uav_bringup vicon_to_px4.launch.py pose_topic:=/vicon/drone/drone
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_bringup')
    mocap_params = os.path.join(pkg_share, 'config', 'px4_mocap_params.yaml')

    pose_topic = LaunchConfiguration('pose_topic')
    twist_topic = LaunchConfiguration('twist_topic')

    return LaunchDescription([
        DeclareLaunchArgument('pose_topic', default_value='/vicon/drone/drone',
                              description='Vicon PoseStamped 话题'),
        DeclareLaunchArgument('twist_topic', default_value='/vicon/drone/drone_twist',
                              description='Vicon TwistStamped 话题（可留空表示不订阅）'),

        Node(
            package='uav_coord_bridge',
            executable='vicon_pose_to_px4_odometry',
            name='vicon_pose_to_px4_odometry',
            output='screen',
            parameters=[mocap_params, {
                'pose_topic': pose_topic,
                'twist_topic': twist_topic,
            }],
        ),
    ])
