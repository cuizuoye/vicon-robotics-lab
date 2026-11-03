from glob import glob
from setuptools import setup, find_packages

setup(name='mecanum_wand_4wsb_hardware', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/mecanum_wand_4wsb_hardware']),
                ('share/mecanum_wand_4wsb_hardware', ['package.xml', 'README.md']),
                ('share/mecanum_wand_4wsb_hardware/launch', glob('launch/*.launch.py')),
                ('share/mecanum_wand_4wsb_hardware/config', glob('config/*'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='czy', maintainer_email='czy@example.com',
    description='Explicit opt-in Sign to real car commissioning, default shadow mode', license='MIT',
    entry_points={'console_scripts': [
        'wand_pilot_observer = mecanum_wand_4wsb_hardware.pilot_observer:main',
        'wand_acceptance_source = mecanum_wand_4wsb_hardware.optimized_nodes:source_main',
        'wand_acceptance_supervisor = mecanum_wand_4wsb_hardware.optimized_nodes:supervisor_main',
        'wand_acceptance_gate = mecanum_wand_4wsb_hardware.optimized_nodes:gate_main',
        'wand_acceptance_shadow = mecanum_wand_4wsb_hardware.nodes:shadow_main',
        'wand_acceptance_transport = mecanum_wand_4wsb_hardware.nodes:hardware_main',
        'wand_acceptance_console = mecanum_wand_4wsb_hardware.console:main',
        'wand_acceptance_smoke = mecanum_wand_4wsb_hardware.acceptance_smoke:main',
        'wand_acceptance_preflight = mecanum_wand_4wsb_hardware.preflight:main',
        'wand_acceptance_review = mecanum_wand_4wsb_hardware.preflight:review_main',
    ]})
