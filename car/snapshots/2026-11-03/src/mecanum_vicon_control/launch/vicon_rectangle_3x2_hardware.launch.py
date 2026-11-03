"""Inert 3 m forward x 2 m left rectangle configuration for 4WG9.

This keeps the full Vicon position and heading validation enabled.  Both
motion gates start disabled; rectangle_run_guard is the only start path.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_vicon_control')
    base = PythonLaunchDescriptionSource(
        package_share + '/launch/vicon_rectangle_hardware.launch.py')
    forwarded = (
        'vicon_pose_topic', 'esp_ip', 'network_interface', 'base_offset_yaw_rad')
    return LaunchDescription([
        DeclareLaunchArgument('vicon_pose_topic', default_value='/vicon/car01/car01'),
        DeclareLaunchArgument('esp_ip', default_value='192.168.50.1'),
        DeclareLaunchArgument('network_interface', default_value='wlx000000000000'),
        DeclareLaunchArgument('base_offset_yaw_rad', default_value='2.356194490192345'),
        IncludeLaunchDescription(
            base,
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in forwarded},
                'rectangle_width_m': '3.0',
                'rectangle_height_m': '2.0',
                'rectangle_waypoints': '8',
                'max_linear_mps': '0.20',
                'required_motion_tag': '4WG9',
                'invert_body_xy': 'false',
            }.items(),
        ),
    ])
