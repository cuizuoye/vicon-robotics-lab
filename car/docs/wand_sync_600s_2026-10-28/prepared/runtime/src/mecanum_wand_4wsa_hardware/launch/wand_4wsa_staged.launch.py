"""Sign safety acceptance. Default shadow; hardware opt-in is validated first."""

import os
import secrets

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    from mecanum_wand_4wsa_hardware.commissioning import Runtime, DOMAIN, NAMESPACE
    mode, profile, approval, consent, gui = [LaunchConfiguration(k).perform(context)
        for k in ('mode', 'profile', 'approved_sha256', 'hardware_consent', 'gui')]
    # Validation precedes Node actions: a draft can never open a real socket.
    runtime = Runtime(mode, profile, approval, consent)
    channel = {'WAND_4WSA_CHANNEL_KEY': secrets.token_hex(32)}
    env = [SetEnvironmentVariable(k, v) for k, v in dict(
        ROS_DOMAIN_ID=str(DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        WAND_4WSA_MODE=mode, WAND_4WSA_PROFILE=profile,
        WAND_PROFILE_APPROVAL=approval, WAND_HARDWARE_CONSENT=consent).items()]
    def node(executable, name, signed=False, parameters=None):
        return Node(package='mecanum_wand_4wsa_hardware', executable=executable, name=name,
            namespace=NAMESPACE, output='screen', additional_env=channel if signed else {},
            parameters=parameters or [])
    nodes = [node('wand_acceptance_source', 'wand_sign_source'),
             node('wand_acceptance_supervisor', 'safe_wand_supervisor'),
             node('wand_acceptance_gate', 'safe_wand_gate', True)]
    if mode == 'shadow':
        nodes += [node('wand_acceptance_shadow', 'safe_sim_plant', True)]
    else:
        nodes += [node('wand_acceptance_transport', 'wand_hardware_bridge', True, [{
            'esp_ip': runtime.ip, 'network_interface': runtime.interface,
            'command_port': runtime.command_port, 'telemetry_port': runtime.telemetry_port,
            'bounded_motion_mode': True, 'required_motion_tag': runtime.motion_tag, 'wire_timeout_ms': 600,
            'max_vx_mps': runtime.config.max_speed_mps, 'max_vy_mps': runtime.config.max_speed_mps, 'max_wz_radps': runtime.config.max_yaw_rate_radps,
            'command_timeout_s': .10, 'link_receive_timeout_s': .20,
            'invert_body_xy': False, 'udp_low_latency_qos': False}])]
    if gui not in ('true', 'false'): raise ValueError('gui must be true or false')
    if gui == 'true': nodes += [node('wand_acceptance_console', 'wand_real_operator')]
    nodes += [node('wand_pilot_observer', 'wand_pilot_observer')]
    # Never leave a transport running after any critical component exits.
    hooks = [RegisterEventHandler(OnProcessExit(target_action=n,
        on_exit=[EmitEvent(event=Shutdown(reason='acceptance component exited'))])) for n in nodes]
    return [*env, *nodes, *hooks]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='shadow',
            description='shadow: real Sign/software car; emulator: loopback only; hardware: reviewed opt-in'),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('approved_sha256', default_value=''),
        DeclareLaunchArgument('hardware_consent', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=build),
    ])
