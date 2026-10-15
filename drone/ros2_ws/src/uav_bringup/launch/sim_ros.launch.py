"""一条命令启动 ROS 2 侧全部通信组件：

  1. MicroXRCEAgent  (PX4 <-> ROS 2 的 uXRCE-DDS 桥, udp4:8888)
  2. ros_gz_bridge   (Gazebo <-> ROS 2)
  3. RViz2 (可选)

PX4 SITL 与 Gazebo 本体请先用 scripts/40_run_sitl.sh 启动（或本 launch 用
start_px4:=true 让 PX4 作为子进程启动）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_bringup')

    agent_bin = LaunchConfiguration('agent_bin')
    agent_port = LaunchConfiguration('agent_port')
    start_agent = LaunchConfiguration('start_agent')
    start_px4 = LaunchConfiguration('start_px4')
    px4_dir = LaunchConfiguration('px4_dir')
    px4_model = LaunchConfiguration('px4_model')
    use_rviz = LaunchConfiguration('use_rviz')

    # drone_sim/ 根目录: <...>/install/uav_bringup/share/uav_bringup -> ../../../../
    drone_sim_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..', '..'))
    default_agent = os.path.join(drone_sim_root, 'build', 'bin', 'MicroXRCEAgent')
    default_px4_dir = os.path.join(drone_sim_root, 'px4_firmware')

    return LaunchDescription([
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('agent_bin', default_value=default_agent),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('start_px4', default_value='false',
                              description='本 launch 内一并启动 PX4 SITL + Gazebo'),
        DeclareLaunchArgument('px4_dir', default_value=default_px4_dir),
        DeclareLaunchArgument('px4_model', default_value='gz_x500'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        ExecuteProcess(
            cmd=[agent_bin, 'udp4', '-p', agent_port],
            output='screen',
            condition=IfCondition(start_agent),
        ),

        ExecuteProcess(
            cmd=['make', 'px4_sitl', px4_model],
            cwd=px4_dir,
            output='screen',
            condition=IfCondition(start_px4),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gz_bridge.launch.py')),
            launch_arguments={'use_rviz': use_rviz}.items(),
        ),
    ])
