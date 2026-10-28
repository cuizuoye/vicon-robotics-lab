from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.10.1',
        description='Vicon server hostname or IP address'
    )
    buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='200',
        description='Buffer size for Vicon data'
    )
    topic_namespace_arg = DeclareLaunchArgument(
        'topic_namespace',
        default_value='vicon',
        description='Topic namespace for Vicon messages'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='World frame for the tf2 transformations'
    )
    vicon_frame_arg = DeclareLaunchArgument(
        'vicon_frame',
        default_value='vicon',
        description='Vicon frame for the tf2 transformations'
    )
    map_xyz_arg = DeclareLaunchArgument(
        'map_xyz',
        default_value='[0.0, 0.0, 0.0]',
        description='XYZ translation for coordinate frame mapping'
    )
    map_rpy_arg = DeclareLaunchArgument(
        'map_rpy',
        default_value='[0.0, 0.0, 0.0]',
        description='RPY rotation for coordinate frame mapping'
    )
    map_rpy_in_degrees_arg = DeclareLaunchArgument(
        'map_rpy_in_degrees',
        default_value='false',
        description='Whether RPY values are in degrees (true) or radians (false)'
    )
    stream_mode_arg = DeclareLaunchArgument(
        'stream_mode',
        default_value='ServerPush',
        description='Vicon DataStream mode: ServerPush (lowest latency), ClientPull or ClientPullPreFetch'
    )
    enable_marker_ray_data_arg = DeclareLaunchArgument(
        'enable_marker_ray_data',
        default_value='false',
        description='Enable the heavy marker ray stream (adds per-frame latency)'
    )
    enable_debug_data_arg = DeclareLaunchArgument(
        'enable_debug_data',
        default_value='false',
        description='Enable the debug data stream (adds per-frame latency)'
    )
    enable_device_data_arg = DeclareLaunchArgument(
        'enable_device_data',
        default_value='false',
        description='Enable device data (force plates, EMG, ...)'
    )
    publish_latency_arg = DeclareLaunchArgument(
        'publish_latency',
        default_value='true',
        description='Publish the Vicon-reported total latency on <namespace>/latency'
    )

    # Get launch configuration values
    hostname = LaunchConfiguration('hostname')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('topic_namespace')
    world_frame = LaunchConfiguration('world_frame')
    vicon_frame = LaunchConfiguration('vicon_frame')
    map_xyz = LaunchConfiguration('map_xyz')
    map_rpy = LaunchConfiguration('map_rpy')
    map_rpy_in_degrees = LaunchConfiguration('map_rpy_in_degrees')
    stream_mode = LaunchConfiguration('stream_mode')
    enable_marker_ray_data = LaunchConfiguration('enable_marker_ray_data')
    enable_debug_data = LaunchConfiguration('enable_debug_data')
    enable_device_data = LaunchConfiguration('enable_device_data')
    publish_latency = LaunchConfiguration('publish_latency')

    return LaunchDescription([
        # Launch arguments
        hostname_arg,
        buffer_size_arg,
        topic_namespace_arg,
        world_frame_arg,
        vicon_frame_arg,
        map_xyz_arg,
        map_rpy_arg,
        map_rpy_in_degrees_arg,
        stream_mode_arg,
        enable_marker_ray_data_arg,
        enable_debug_data_arg,
        enable_device_data_arg,
        publish_latency_arg,
        
        # Node
        Node(
            package='vicon_receiver', 
            executable='vicon_client', 
            output='screen',
            parameters=[{
                'hostname': hostname, 
                'buffer_size': buffer_size, 
                'namespace': topic_namespace,
                'world_frame': world_frame,
                'vicon_frame': vicon_frame,
                'map_xyz': map_xyz,
                'map_rpy': map_rpy,
                'map_rpy_in_degrees': map_rpy_in_degrees,
                'stream_mode': stream_mode,
                'enable_marker_ray_data': enable_marker_ray_data,
                'enable_debug_data': enable_debug_data,
                'enable_device_data': enable_device_data,
                'publish_latency': publish_latency
            }]
        )
    ])
