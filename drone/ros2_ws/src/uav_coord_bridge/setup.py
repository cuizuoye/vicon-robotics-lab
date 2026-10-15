from setuptools import find_packages, setup

package_name = 'uav_coord_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='czy',
    maintainer_email='czy@example.com',
    description='Coordinate transfer nodes between Vicon, PX4 and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vicon_pose_to_px4_odometry = uav_coord_bridge.vicon_pose_to_px4_odometry:main',
            'px4_odometry_to_ros = uav_coord_bridge.px4_odometry_to_ros:main',
            'odom_to_tf = uav_coord_bridge.odom_to_tf:main',
            'offboard_takeoff_land = uav_coord_bridge.offboard_takeoff_land:main',
            'vicon_watchdog = uav_coord_bridge.vicon_watchdog:main',
            'uav_emergency_control = uav_coord_bridge.uav_emergency_control:main',
            'gz_pose_mirror = uav_coord_bridge.gz_pose_mirror:main',
            'cf_real_twin = uav_coord_bridge.cf_real_twin:main',
            'vicon_to_poses = uav_coord_bridge.vicon_to_poses:main',
            'vicon_mavlink_preview = uav_coord_bridge.vicon_mavlink_preview:main',
        ],
    },
)
