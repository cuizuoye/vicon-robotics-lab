"""Isolated SOFTWARE direct-target profile. Never a hardware firmware tier."""
from dataclasses import asdict, dataclass, replace
import math
import os
from pathlib import Path
import re
import sys
from types import FunctionType, SimpleNamespace as NS

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
PREPARED = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'candidates/wand_sim12_20261030'))
for package in ('mecanum_wand_4wsb_control', 'mecanum_wand_4wsb_hardware'):
    sys.path.insert(0, str(PREPARED/'runtime/src'/package))
sys.path.insert(0, str(ROOT/'src/mecanum_vicon_control'))
from mecanum_wand_4wsb_control.tracking_config import TrackingSafetyConfig
# Importing the reused helper afterward cannot redirect an already loaded
# package's __path__. Assert this rather than relying on sys.path ordering.
sys.path.insert(0, str(ROOT/'candidates/wand_native_acceptance_20261030'))
from common import config, protected, save
from mecanum_wand_4wsb_hardware import commissioning as authority
import mecanum_wand_4wsb_control as control_package
if not str(Path(control_package.__file__).resolve()).startswith(str(PREPARED/'runtime')):
    raise ValueError('refuse fallback to sealed governed controller')

SCOPE = 'PRIVATE_BROWSER_DIRECT_SIGN_1TO1_V1_'
CHANGES = dict(max_speed_mps=1.2, max_accel_mps2=4., max_yaw_rate_radps=2., max_yaw_accel_radps2=12.)
COMMAND_RPM = 1.2*60/(2*math.pi*.03)


@dataclass(frozen=True)
class SimConfig(TrackingSafetyConfig):
    def __post_init__(self):
        if asdict(self) != dict(asdict(config(6)), **CHANGES):
            raise ValueError('exact direct-Sign software profile required')
    @property
    def direct_sign_target(self): return True
    @property
    def stage(self): return 6  # Legacy API slot only, not hardware compatibility.
    @property
    def limits(self): return NS(speed=1.2, accel=4., yaw_rate=2., yaw_accel=12., target_rpm=COMMAND_RPM, feedback_rpm=420.)
    def follower_config(self):
        return replace(config(6).follower_config(), **CHANGES,
                       max_wheel_rpm=COMMAND_RPM, wand_deadband_m=0.)


def sim_config(): return SimConfig(**dict(asdict(config(6)), **CHANGES))


def scoped(f):
    result = FunctionType(f.__code__, dict(f.__globals__, SCOPE=SCOPE), f.__name__, f.__defaults__, f.__closure__)
    result.__kwdefaults__ = f.__kwdefaults__
    return result


class Signer(authority.AuthoritySigner):
    encode = scoped(authority.AuthoritySigner.encode)
    def __init__(self, key, stage=6):
        if stage != 6: raise ValueError('software slot only')
        super().__init__(key, 6)


class Verifier(authority.AuthorityVerifier):
    decode = scoped(authority.AuthorityVerifier.decode)
    def __init__(self, key):
        super().__init__(key, 6)
        self.limits = sim_config().limits


class Policy(authority.TransportPolicy):
    def __init__(self, key):
        super().__init__(key, 6)
        self.verifier = Verifier(key)


def runtime(namespace):
    if (os.environ.get('ROS_DOMAIN_ID') != '191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or not re.fullmatch('/host4wsb_[a-f0-9]{10}', namespace)):
        raise ValueError('private localhost software domain required')
    def guard(node):
        if (node.context.get_domain_id() != 191 or node.get_namespace() != namespace
                or node.get_parameter('use_sim_time').value or node.get_name() == 'wand_hardware_bridge'):
            raise ValueError('direct-Sign runtime cannot escape software scope')
    return NS(namespace=namespace, world='map', config=sim_config(), mode='hardware', software_only=True,
        experiment='DIRECT_SIGN_NOMINAL_NO_C_FIRMWARE', guard=guard, yaw_offset=0., wand_reference_margin_m=0.,
        expected=dict(car_pose='wand_sign_source', wand_pose='wand_sign_source', rotation_packet='wand_sign_source',
            plant_feedback='safe_native_twin', operator_lease='wand_real_operator', candidate='safe_wand_supervisor',
            cmd_vel='safe_wand_gate', actuator_command='safe_wand_gate'))


def install():
    from mecanum_wand_4wsb_hardware import optimized_nodes
    from geometry12 import install_geometry
    install_geometry()
    optimized_nodes.AuthoritySigner = Signer


def contract():
    from geometry12 import SHA
    return dict(profile='BROWSER_DIRECT_SIGN_1TO1', mapping='direct 1:1 XY and unwrapped relative yaw; Z ignored',
        target_governor=False, intermediate_reference=False, reference_lead_limit=False,
        proxy_tracking_distance_stop=False, source_jump_stop=True, no_progress_stop=True,
        speed_mps=1.2, diagonal_speed_mps=1.2/math.sqrt(2), accel_mps2=4., yaw_radps=2., yaw_accel_radps2=12.,
        position_gain_per_s=1.4, velocity_window_s=.04, wheel_rpm_cap=COMMAND_RPM,
        sign_geometry_sha256=SHA, sign_bar=['Sign1','Sign4','Sign3'], sign_junction='Sign4',
        fixed_fence=False, session_s=600., browser_timeout_s=.1, pose_timeout_s=.2,
        command_timeout_s=.1, operator_timeout_s=.15, feedback_timeout_s=.3,
        hardware_output=False, physical_qualification=False,
        actuator_model='nominal first-order tau=0.12s; not measured motor dynamics')
