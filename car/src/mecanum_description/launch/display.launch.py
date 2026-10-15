import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_description')
    model = os.path.join(package_share, 'urdf', 'mecanum_car.urdf.xacro')
    rviz_config = os.path.join(package_share, 'rviz', 'mecanum_car.rviz')

    names_and_defaults = {
        'start_rviz': 'true',
        'chassis_length': '0.24',
        'chassis_width': '0.20',
        'chassis_height': '0.06',
        'wheelbase': '0.18',
        'track_width': '0.16',
        'wheel_diameter': '0.060',
        'wheel_width': '0.0306',
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in names_and_defaults.items()
    ]

    robot_description = Command([
        'xacro ', model,
        ' chassis_length:=', LaunchConfiguration('chassis_length'),
        ' chassis_width:=', LaunchConfiguration('chassis_width'),
        ' chassis_height:=', LaunchConfiguration('chassis_height'),
        ' wheelbase:=', LaunchConfiguration('wheelbase'),
        ' track_width:=', LaunchConfiguration('track_width'),
        ' wheel_diameter:=', LaunchConfiguration('wheel_diameter'),
        ' wheel_width:=', LaunchConfiguration('wheel_width'),
    ])

    return LaunchDescription(declarations + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # 当前没有真实轮角反馈，先发布四个轮关节的零位。
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])

