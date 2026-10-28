from glob import glob
from setuptools import find_packages, setup

setup(
    name='mecanum_wand_4ws9_control',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mecanum_wand_4ws9_control']),
        ('share/mecanum_wand_4ws9_control', ['package.xml', 'README.md']),
        ('share/mecanum_wand_4ws9_control/launch', glob('launch/*.launch.py')),
        ('share/mecanum_wand_4ws9_control/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czy',
    maintainer_email='czy@example.com',
    description='Offline relative-position wand teleoperation, without hardware transport',
    license='MIT',
    entry_points={'console_scripts': [
        'wand_sim_controller = mecanum_wand_4ws9_control.ros_nodes:controller_main',
        'wand_demo = mecanum_wand_4ws9_control.ros_nodes:demo_main',
        'wand_sim_car = mecanum_wand_4ws9_control.ros_nodes:sim_car_main',
        'wand_offline_suite = mecanum_wand_4ws9_control.offline_suite:main',
        'wand_ros_smoke = mecanum_wand_4ws9_control.ros_smoke:main',
        'sign_readonly_preview = mecanum_wand_4ws9_control.readonly_preview:main',
        'sign_to_simulation = mecanum_wand_4ws9_control.live_bridge:main',
        'wand_safe_supervisor = mecanum_wand_4ws9_control.safety_nodes:supervisor_main',
        'wand_safe_gate = mecanum_wand_4ws9_control.safety_nodes:gate_main',
        'wand_safe_plant = mecanum_wand_4ws9_control.safety_nodes:plant_main',
        'wand_safe_suite = mecanum_wand_4ws9_control.safety_suite:main',
        'wand_safe_ros_smoke = mecanum_wand_4ws9_control.safety_ros_smoke:main',
        'wand_safe_console = mecanum_wand_4ws9_control.operator_console:main',
    ]},
)
