"""Hypothetical 1.40m/s SOFTWARE profile, never a 4WSB firmware capability."""
from dataclasses import dataclass,asdict,replace
import math
import os
from pathlib import Path
import re
import sys
from types import FunctionType,SimpleNamespace as NS

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'candidates/wand_native_acceptance_20261030'))
from common import config,protected,save
from mecanum_wand_4wsb_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4wsb_hardware import commissioning as authority

SCOPE='PRIVATE_SIM14_HYPOTHETICAL_V1_'
CHANGES=dict(max_speed_mps=1.4,max_accel_mps2=4.,max_yaw_rate_radps=2.,max_yaw_accel_radps2=12.)


@dataclass(frozen=True)
class SimConfig(TrackingSafetyConfig):
    def __post_init__(self):
        expected=dict(asdict(config(6)),**CHANGES)
        if asdict(self)!=expected:raise ValueError('exact SIM14 profile; all non-motion guards pinned')
    @property
    def stage(self):return 6  # Internal legacy API slot, NOT hardware compatibility.
    @property
    def limits(self):return NS(speed=1.4,accel=4.,yaw_rate=2.,yaw_accel=12.,target_rpm=800.,feedback_rpm=880.)
    def follower_config(self):
        return replace(config(6).follower_config(),**CHANGES,max_wheel_rpm=800.)


def sim_config():return SimConfig(**dict(asdict(config(6)),**CHANGES))


def scoped(function):
    result=FunctionType(function.__code__,dict(function.__globals__,SCOPE=SCOPE),
        function.__name__,function.__defaults__,function.__closure__)
    result.__kwdefaults__=function.__kwdefaults__
    return result


class Signer(authority.AuthoritySigner):
    encode=scoped(authority.AuthoritySigner.encode)
    def __init__(self,key,stage=6):
        if stage!=6:raise ValueError('SIM14 internal slot only')
        super().__init__(key,6)


class Verifier(authority.AuthorityVerifier):
    decode=scoped(authority.AuthorityVerifier.decode)
    def __init__(self,key):
        super().__init__(key,6);self.limits=sim_config().limits


class Policy(authority.TransportPolicy):
    def __init__(self,key):
        super().__init__(key,6);self.verifier=Verifier(key)


def runtime(namespace):
    if (os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or not re.fullmatch('/host4wsb_[a-f0-9]{10}',namespace)):
        raise ValueError('private localhost software domain required')
    def guard(node):
        if (node.context.get_domain_id()!=191 or node.get_namespace()!=namespace
                or node.get_parameter('use_sim_time').value or node.get_name()=='wand_hardware_bridge'):
            raise ValueError('SIM14 cannot escape private software scope')
    return NS(namespace=namespace,world='map',config=sim_config(),mode='hardware',software_only=True,
        experiment='SIM14_HYPOTHETICAL_NO_C_FIRMWARE',guard=guard,yaw_offset=0.,wand_reference_margin_m=0.,
        expected=dict(car_pose='wand_sign_source',wand_pose='wand_sign_source',rotation_packet='wand_sign_source',
            plant_feedback='safe_native_twin',operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
            cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate'))


def install():
    # Process-local override only; production and sealed files stay immutable.
    from mecanum_wand_4wsb_hardware import optimized_nodes
    optimized_nodes.AuthoritySigner=Signer


def contract():
    return dict(profile='SIM14_HYPOTHETICAL',speed_mps=1.4,accel_mps2=4.,yaw_radps=2.,yaw_accel_radps2=12.,
        wheel_rpm_cap=800.,mapping='1:1 XY and relative yaw; Z ignored',fixed_fence=False,
        session_s=600.,pose_timeout_s=.2,command_timeout_s=.1,operator_timeout_s=.15,feedback_timeout_s=.3,
        original_4wsb_firmware_used=False,hardware_output=False,physical_qualification=False,
        actuator_model='nominal first-order tau=0.12s; no measured motor/current/thermal dynamics')
