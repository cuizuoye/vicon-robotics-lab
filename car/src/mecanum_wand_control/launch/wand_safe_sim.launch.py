"""Only offline safety nodes in domain96. No auto-enable or hardware includes."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import secrets


def generate_launch_description():
    channel = {'WAND_SAFE_CHANNEL_KEY': secrets.token_hex(32)}
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '96'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='mecanum_wand_control', executable='wand_safe_plant',
             name='safe_sim_plant', namespace='wand_safe_sim', output='screen', additional_env=channel),
        Node(package='mecanum_wand_control', executable='wand_safe_supervisor',
             name='safe_wand_supervisor', namespace='wand_safe_sim', output='screen'),
        Node(package='mecanum_wand_control', executable='wand_safe_gate',
             name='safe_wand_gate', namespace='wand_safe_sim', output='screen', additional_env=channel),
    ])
