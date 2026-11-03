from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_vicon_control')
    config = os.path.join(package_share, 'config', 'control.yaml')
    vicon_pose_topic = LaunchConfiguration('vicon_pose_topic')
    use_trajectory = LaunchConfiguration('use_trajectory')
    start_rviz = LaunchConfiguration('start_rviz')
    description_launch = os.path.join(
        get_package_share_directory('mecanum_description'),
        'launch', 'display.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vicon_pose_topic',
            default_value='/vicon/T2/T2',
            description='PoseStamped topic for the Vicon rigid-body center',
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_trajectory', default_value='false'),
        DeclareLaunchArgument('trajectory_shape', default_value='circle'),
        DeclareLaunchArgument('trajectory_center_x', default_value='0.0'),
        DeclareLaunchArgument('trajectory_center_y', default_value='0.0'),
        DeclareLaunchArgument('trajectory_radius', default_value='0.5'),
        DeclareLaunchArgument('trajectory_width', default_value='1.0'),
        DeclareLaunchArgument('trajectory_height', default_value='0.6'),
        DeclareLaunchArgument('trajectory_waypoints', default_value='32'),
        DeclareLaunchArgument('trajectory_loops', default_value='1'),
        DeclareLaunchArgument('trajectory_heading_mode', default_value='tangent'),
        DeclareLaunchArgument('trajectory_fixed_yaw', default_value='0.0'),
        DeclareLaunchArgument('trajectory_relative_to_start', default_value='false'),
        DeclareLaunchArgument('trajectory_start_enabled', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'start_rviz': start_rviz}.items(),
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_state_bridge',
            output='screen',
            parameters=[config, {'input_pose_topic': vicon_pose_topic}],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='pose_controller',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='safety_watchdog',
            output='screen',
            parameters=[config, {'start_enabled': False}],
        ),
        Node(
            package='mecanum_vicon_control',
            executable='car_visualizer',
            output='screen',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='rviz_goal_bridge',
            output='screen',
        ),
        Node(
            package='mecanum_vicon_control',
            executable='trajectory_generator',
            output='screen',
            condition=IfCondition(use_trajectory),
            parameters=[{
                'shape': LaunchConfiguration('trajectory_shape'),
                'center_x_m': ParameterValue(
                    LaunchConfiguration('trajectory_center_x'), value_type=float),
                'center_y_m': ParameterValue(
                    LaunchConfiguration('trajectory_center_y'), value_type=float),
                'radius_m': ParameterValue(
                    LaunchConfiguration('trajectory_radius'), value_type=float),
                'width_m': ParameterValue(
                    LaunchConfiguration('trajectory_width'), value_type=float),
                'height_m': ParameterValue(
                    LaunchConfiguration('trajectory_height'), value_type=float),
                'waypoint_count': ParameterValue(
                    LaunchConfiguration('trajectory_waypoints'), value_type=int),
                'loop_count': ParameterValue(
                    LaunchConfiguration('trajectory_loops'), value_type=int),
                'heading_mode': LaunchConfiguration('trajectory_heading_mode'),
                'fixed_yaw_rad': ParameterValue(
                    LaunchConfiguration('trajectory_fixed_yaw'), value_type=float),
                'relative_to_start': ParameterValue(
                    LaunchConfiguration('trajectory_relative_to_start'), value_type=bool),
                'start_enabled': ParameterValue(
                    LaunchConfiguration('trajectory_start_enabled'), value_type=bool),
            }],
        ),
    ])
