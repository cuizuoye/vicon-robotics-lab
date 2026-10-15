from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_vicon_control')
    config = os.path.join(package_share, 'config', 'control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'esp_ip',
            default_value='',
            description='ESP32-S3 address on the same LAN; empty disables UDP',
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='',
            description='Optional per-socket interface binding, e.g. wlp114s0',
        ),
        DeclareLaunchArgument(
            'udp_low_latency_qos', default_value='false',
            description='Opt-in command-socket EF/priority-6 marking; no system network change',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='cmd_vel_udp_bridge',
            output='screen',
            parameters=[config, {
                'esp_ip': LaunchConfiguration('esp_ip'),
                'network_interface': LaunchConfiguration('network_interface'),
                'udp_low_latency_qos': ParameterValue(
                    LaunchConfiguration('udp_low_latency_qos'), value_type=bool),
            }],
        ),
    ])
