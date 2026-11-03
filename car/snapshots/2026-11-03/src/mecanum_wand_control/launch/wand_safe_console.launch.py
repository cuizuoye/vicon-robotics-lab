"""Virtual wand console and existing isolated safety backend; no real Vicon."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    backend = os.path.join(get_package_share_directory('mecanum_wand_control'),
                           'launch', 'wand_safe_sim.launch.py')
    console = Node(package='mecanum_wand_control', executable='wand_safe_console',
                   name='safe_sim_operator', namespace='wand_safe_sim', output='screen')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '96'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(backend)),
        console,
        RegisterEventHandler(OnProcessExit(target_action=console,
            on_exit=[EmitEvent(event=Shutdown(reason='offline console closed'))])),
    ])
