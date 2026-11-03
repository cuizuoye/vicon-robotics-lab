from glob import glob
from setuptools import find_packages, setup


package_name = 'mecanum_vicon_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czy',
    maintainer_email='czy@example.com',
    description='Vicon pose closed-loop control scaffold for a mecanum car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_state_bridge = mecanum_vicon_control.car_state_bridge:main',
            'car_visualizer = mecanum_vicon_control.car_visualizer:main',
            'cmd_vel_udp_bridge = mecanum_vicon_control.cmd_vel_udp_bridge:main',
            'continuous_loop_supervisor = mecanum_vicon_control.continuous_loop_supervisor:main',
            'demo_goal_publisher = mecanum_vicon_control.demo_goal_publisher:main',
            'drive_plan_dryrun = mecanum_vicon_control.drive_plan:main',
            'pose_controller = mecanum_vicon_control.pose_controller:main',
            'path_recovery_controller = mecanum_vicon_control.path_recovery_controller:main',
            'rectangle_run_guard = mecanum_vicon_control.rectangle_run_guard:main',
            'recovery_visualizer = mecanum_vicon_control.recovery_visualizer:main',
            'rviz_goal_bridge = mecanum_vicon_control.rviz_goal_bridge:main',
            'safety_watchdog = mecanum_vicon_control.safety_watchdog:main',
            'simulated_vicon_car = mecanum_vicon_control.simulated_vicon_car:main',
            'trajectory_generator = mecanum_vicon_control.trajectory_generator:main',
        ],
    },
)
