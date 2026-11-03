"""Private 4WSC domain/runtime. Software rehearsal can only address loopback."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
from mecanum_wand_4wsc_control.core import Pose
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig
from .authorization import endpoint
from .ground_admission import verify_runtime_profile,common,CONSENT
from .stages import get_stage

DOMAIN,NAMESPACE,WORLD=129,'/wand_stopfix_stage6_long600','map'
SUSPENDED_CONSENT='4WSC_SUSPENDED_SINGLE5S_CLEAR_POWER_CUT_READY'

class Runtime:
    def __init__(self,mode='hardware',profile='',approval='',consent=''):
        if mode=='hardware':
            from .deployment_record import verify_host_snapshot
            verify_host_snapshot()
        raw=Path(profile).read_bytes()
        if len(raw)>16384:raise ValueError('profile size limit')
        self.profile=json.loads(raw);common(self.profile);self.profile_hash=hashlib.sha256(raw).hexdigest()
        if approval!=self.profile_hash:raise ValueError('exact reviewed profile hash required')
        self.mode=mode;self.purpose=self.profile.get('purpose');self.software_only=mode=='rehearsal'
        if mode=='hardware':
            manifest=json.loads((Path(__file__).resolve().parents[4]/'HOST_MANIFEST.json').read_text())
            if manifest.get('host_offline_passed') is not True or manifest.get('identity')!='4WSC':
                raise ValueError('qualified 4WSC host required before any ROS/socket construction')
            from .deployment_record import verify_migration
            verify_migration(self.profile)
            verify_runtime_profile(self.profile,self.purpose)
            expected=CONSENT if self.purpose=='sign' else SUSPENDED_CONSENT
            if consent!=expected:raise ValueError('explicit purpose-specific hardware consent required')
            self.domain,self.namespace=DOMAIN,NAMESPACE
        elif mode=='rehearsal':
            if self.profile.get('software_rehearsal') is not True or consent!='LOOPBACK_ONLY_C_REHEARSAL':raise ValueError('explicit software-only rehearsal required')
            self.domain=196;self.namespace=self.profile['namespace']
            if not re.fullmatch('/wand_4wsc_test_[a-f0-9]{10}',self.namespace):raise ValueError('private rehearsal namespace')
        else:raise ValueError('hardware or loopback rehearsal only')
        self.ip,self.interface,self.command_port,self.telemetry_port=endpoint(self.profile,rehearsal=self.software_only)
        self.config=TrackingSafetyConfig(**self.profile['safety']);get_stage(self.profile['stage'])
        if self.config.stage!=self.profile['stage']:raise ValueError('stage/config mismatch')
        self.motion_tag='4WSC';self.motion_status_size=208;self.world=WORLD;self.source_domain=0
        self.yaw_offset=float(self.profile['base_yaw_offset_rad']);self.wand_reference_margin_m=0.
        if not math.isfinite(self.yaw_offset):raise ValueError('finite verified car heading offset required')
        self.initial_pose=Pose(0.,0.,self.config.ground_z_m,0.)
        self.expected=dict(car_pose='wand_sign_source',wand_pose='wand_sign_source',rotation_packet='wand_sign_source',
            plant_feedback='wand_hardware_bridge',operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
            cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate')

    def guard(self,node):
        if node.context.get_domain_id()!=self.domain or node.get_namespace()!=self.namespace or node.get_parameter('use_sim_time').value:
            raise RuntimeError('4WSC private domain/namespace/real clocks required')
        for name in (*self.expected,'supervisor_status','gate_status','mode_claim','source_status','transport_status',
                'start','pause','stop','acknowledge','recover','estop'):
            if node.resolve_topic_name(name)!=self.namespace+'/'+name:raise RuntimeError('topic remapping forbidden')
            if name in ('start','pause','stop','acknowledge','recover','estop') and node.resolve_service_name(name)!=self.namespace+'/'+name:
                raise RuntimeError('service remapping forbidden')
