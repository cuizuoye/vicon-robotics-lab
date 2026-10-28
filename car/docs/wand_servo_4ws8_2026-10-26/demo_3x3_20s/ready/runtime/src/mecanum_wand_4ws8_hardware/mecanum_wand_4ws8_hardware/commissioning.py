"""Reviewable real-car profile and signed authority; no sockets or ROS writes."""

from dataclasses import asdict
import hashlib
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import uuid

from mecanum_wand_4ws8_control.core import Pose, Velocity
from mecanum_wand_4ws8_control.safety_core import ZERO, fresh_envelope
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig as SafetyConfig
from .stages import STAGES,get_stage,SCOPE
from .authorization import verify_runtime_profile
from mecanum_wand_4ws8_control.safety_wire import canonical, finite_number, load_json, secret_bytes

DOMAIN, NAMESPACE, WORLD = 120, '/wand_4ws8_ground_pilot', 'map'
CONSENT = '4WS8_DEMO_3X3_20S_CLEAR_POWER_CUT_READY'
REQUIRED_CONFIRMATIONS = ('fixed_arena_checked', 'footprint_measured', 'ground_height_measured',
                          'heading_checked', 'power_cut_ready', 'wiring_secured',
                          'battery_protection_checked', 'wire_watchdog_600ms_documented')
MEASURED_FIELDS = ('arena_x_min', 'arena_x_max', 'arena_y_min', 'arena_y_max',
                   'ground_z_m', 'half_length_m', 'half_width_m', 'boundary_margin_m',
                   'arena_origin_x_m', 'arena_origin_y_m', 'arena_heading_rad',
                   'arena_forward_max_m', 'arena_left_max_m', 'arena_buffer_m')


class Runtime:
    namespace, world = NAMESPACE, WORLD

    def __init__(self, mode='shadow', profile='', approval='', consent=''):
        if mode != 'hardware': raise ValueError('invalid acceptance mode')
        self.mode = mode
        candidate = json.loads(Path(profile).read_text())
        from .pilot_authorization import KIND
        if candidate.get('trial_review', {}).get('kind') != KIND or candidate.get('stage') != 4:
            raise ValueError('this private runtime accepts only the finite stage4 pilot')
        self.profile_hash = ''
        self.profile = {}
        self.yaw_offset = 2.356194490192345  # Existing car01 installation; still requires physical confirmation.
        self.ip, self.interface, self.command_port, self.telemetry_port = '', '', 46000, 46001
        self.source_domain = 93 if mode == 'emulator' else 0
        if mode == 'hardware':
            verify_runtime_profile(json.loads(Path(profile).read_text()), 'sign')
            raw = Path(profile).read_bytes()
            if len(raw) > 16384: raise ValueError('oversized commissioning profile')
            self.profile_hash = hashlib.sha256(raw).hexdigest()
            if approval != self.profile_hash or consent != CONSENT:
                raise ValueError('hardware requires the exact reviewed profile hash and explicit consent')
            self.profile = json.loads(raw)
            if self.profile.get('schema_version') != 1: raise ValueError('profile schema')
            confirmations = self.profile.get('confirmations', {})
            if any(confirmations.get(k) is not True for k in REQUIRED_CONFIRMATIONS):
                raise ValueError('physical commissioning confirmations are incomplete')
            values = self.profile['safety']
            if set(values)-set(MEASURED_FIELDS)-{'max_speed_mps', 'max_session_s',
                                                'max_accel_mps2', 'max_goal_radius_m', 'goal_boundary_policy',
                                                'goal_reference_policy'}:
                raise ValueError('commissioning profile cannot relax established timing/tilt guards')
            if any(type(values.get(k)) not in (int, float) for k in MEASURED_FIELDS):
                raise ValueError('measured arena, footprint and ground height are required')
            self.config = SafetyConfig(**values)
            if self.config.boundary_margin_m < .20:
                raise ValueError('commissioning requires at least 0.20m footprint margin')
            self.yaw_offset = finite_number(self.profile['base_yaw_offset_rad'])
            hw = self.profile['hardware']
            if hw['required_motion_tag'] not in ('4WS8',) or hw['wire_timeout_ms'] != 600:
                raise ValueError('this adapter requires an exact 4WS8 identity and 600ms lease')
            self.ip = str(ipaddress.IPv4Address(hw['esp_ip']))
            if not ipaddress.IPv4Address(self.ip).is_private or ipaddress.IPv4Address(self.ip).is_loopback:
                raise ValueError('real-car endpoint must be an explicitly reviewed private IPv4 address')
            self.interface = hw['network_interface']
            if not isinstance(self.interface, str) or not self.interface or len(self.interface) > 15:
                raise ValueError('an explicit network interface is required')
        else:
            if not profile:
                raise ValueError('explicit 3x3 arena profile required even for preview')
            self.config = None
            if mode in ('shadow', 'emulator') and profile:
                raw = Path(profile).read_bytes()
                if len(raw) > 16384: raise ValueError('oversized shadow profile')
                self.profile_hash = hashlib.sha256(raw).hexdigest()
                self.profile = json.loads(raw)
                self.config = SafetyConfig(**self.profile['safety'])
                self.yaw_offset = finite_number(self.profile['base_yaw_offset_rad'])
            if mode == 'emulator':
                self.ip, self.command_port, self.telemetry_port = '127.0.0.1', 54600, 54601
        hw = self.profile.get('hardware', {})
        if self.profile.get('arena_revision') != 'fixed_corner_3x3_buffer050_v1':
            raise ValueError('explicit fixed 3x3/0.50m arena profile required')
        if mode == 'hardware':
            required = ('extended_arena_and_buffer_clear_and_tracked',
                        'saved_start_and_heading_match', 'immediate_power_cut_ready_now')
            if any(self.profile.get('arena_confirmations', {}).get(k) is not True for k in required):
                raise ValueError('ARENA_3X3_SITE_CONFIRMATION_REQUIRED')
        if self.profile and self.profile.get('mapping_scale') != 1.0:
            raise ValueError('this independent entry requires an explicit 1:1 profile')
        self.motion_tag = hw.get('required_motion_tag', '4WS8')
        if self.motion_tag == '4WS8':
            if hw.get('diagnostic_payload_bytes') != 184 or hw.get('output_authority_version') != 4:
                raise ValueError('4WS8 requires reviewed 184-byte v4 output authority diagnostics')
        elif self.motion_tag != '4WS8':
            raise ValueError('unsupported Sign hardware identity')
        self.motion_status_size = 184 if self.motion_tag == '4WS8' else 100
        if self.config.max_speed_mps > .70 or self.config.max_yaw_rate_radps > .10:
            raise ValueError('fixed stage speed and 0.10rad/s limits required')
        # Explicitly reviewed 2x2m operation needs more than the old 0.50m/30s
        # first-motion budget. Defaults and historical profiles remain unchanged;
        # neither spatial scope nor session duration can disable short watchdogs.
        if self.config.max_session_s > 120: raise ValueError('acceptance session is at most 120s')
        if self.config.max_goal_radius_m > 4.75: raise ValueError('acceptance target radius is at most 4.75m; goals still fixed to 3x3m')
        self.wand_reference_margin_m = 0.
        if self.config.goal_boundary_policy == 'clamp':
            self.wand_reference_margin_m = finite_number(self.profile.get(
                'input_workspace', {}).get('reference_margin_m', .50))
            if self.wand_reference_margin_m < .50:
                raise ValueError('wide-input reference must extend at least 0.50m on every side')
        self.initial_pose = Pose((self.config.arena_x_min+self.config.arena_x_max)/2,
                                 (self.config.arena_y_min+self.config.arena_y_max)/2,
                                 self.config.ground_z_m,
                                 finite_number(self.profile.get('evidence', {}).get('car_heading_rad', 0.)))
        # This seeds the software preview only. Real hardware still consumes
        # fresh car01 poses; preparing never moves/recentres the fixed fence.
        preview_xy = self.profile.get('evidence', {}).get('preview_start_xy_m')
        if preview_xy is not None:
            if not isinstance(preview_xy, list) or len(preview_xy) != 2:
                raise ValueError('preview start must be an explicit XY pair')
            self.initial_pose = Pose(*(finite_number(v) for v in preview_xy),
                                     self.config.ground_z_m, self.initial_pose.yaw)
            if not self.config.inside(self.initial_pose):
                raise ValueError('preview start is outside the fixed footprint-safe arena')
        self.expected = dict(wand_pose='wand_sign_source', operator_lease='wand_real_operator',
            car_pose='safe_sim_plant' if mode == 'shadow' else 'wand_sign_source',
            plant_feedback='safe_sim_plant' if mode == 'shadow' else 'wand_hardware_bridge',
            candidate='safe_wand_supervisor', cmd_vel='safe_wand_gate', actuator_command='safe_wand_gate')

    def guard(self, node):
        if (os.environ.get('ROS_DOMAIN_ID') != str(DOMAIN) or node.context.get_domain_id() != DOMAIN
                or node.get_namespace() != NAMESPACE or node.get_parameter('use_sim_time').value):
            raise RuntimeError('acceptance requires fixed domain119, namespace and real clocks')
        for name in (*self.expected, 'supervisor_status', 'gate_status', 'mode_claim', 'source_status',
                     'start', 'pause', 'stop', 'acknowledge', 'recover', 'estop'):
            if node.resolve_topic_name(name) != NAMESPACE+'/'+name:
                raise RuntimeError('acceptance topic remapping is forbidden')
            if name in ('start', 'pause', 'stop', 'acknowledge', 'recover', 'estop'):
                if node.resolve_service_name(name) != NAMESPACE+'/'+name:
                    raise RuntimeError('acceptance service remapping is forbidden')


def runtime_from_environment():
    return Runtime(os.environ.get('WAND_4WS8_MODE', 'shadow'),
        os.environ.get('WAND_4WS8_PROFILE', ''), os.environ.get('WAND_PROFILE_APPROVAL', ''),
        os.environ.get('WAND_HARDWARE_CONSENT', ''))


class AuthoritySigner:
    def __init__(self, key, stage=0):
        self.stage=stage;get_stage(stage)
        self.key, self.owner, self.sequence = secret_bytes(key), uuid.uuid4().hex, 0

    def encode(self, command, steady, ros, *, epoch='', state='STANDBY', permit=False, estop=False):
        self.sequence += 1
        p = dict(source=self.owner, sequence=self.sequence, issued_steady=steady, issued_ros=ros,
                 epoch=epoch, state=state, permit=permit, estop=estop, command=asdict(command),scope=SCOPE+str(self.stage))
        return json.dumps(dict(payload=p, signature=hmac.new(self.key, canonical(p), hashlib.sha256).hexdigest()))


class AuthorityVerifier:
    """Authenticates permission AND velocity, never an unsigned Bool/Twist."""
    def __init__(self, key, stage=0):
        self.stage=stage;self.limits=get_stage(stage)
        self.key, self.owner, self.sequence = secret_bytes(key), None, -1

    def decode(self, text, steady, ros):
        env = load_json(text)
        if set(env) != {'payload', 'signature'} or not isinstance(env['payload'], dict):
            raise ValueError('authority envelope schema')
        p, signature = env['payload'], env['signature']
        if not isinstance(signature, str) or not hmac.compare_digest(signature,
                hmac.new(self.key, canonical(p), hashlib.sha256).hexdigest()):
            raise ValueError('authority signature')
        if set(p) != {'source', 'sequence', 'issued_steady', 'issued_ros', 'epoch', 'state', 'permit', 'estop', 'command', 'scope'} or p.get('scope')!=SCOPE+str(self.stage):
            raise ValueError('authority fields')
        if (type(p['sequence']) is not int or p['sequence'] <= self.sequence
                or type(p['permit']) is not bool or type(p['estop']) is not bool
                or not isinstance(p['epoch'], str) or len(p['epoch']) > 80
                or not isinstance(p['source'], str) or not 0 < len(p['source']) <= 80):
            raise ValueError('authority types or replay')
        if not fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']), steady, ros, .10):
            raise ValueError('authority expired')
        if set(p['command']) != {'vx', 'vy', 'wz'}: raise ValueError('command schema')
        command = Velocity(**{k: finite_number(v) for k, v in p['command'].items()})
        if math.hypot(command.vx, command.vy) > self.limits.speed+1e-9 or abs(command.wz) > .10+1e-9:
            raise ValueError('hardware acceptance hard speed limit')
        states = ('STANDBY', 'ARMED', 'ACTIVE', 'STOPPING', 'STOP_UNCONFIRMED', 'PAUSED', 'FAULT')
        if p['state'] not in states: raise ValueError('authority state')
        if p['permit'] and (not p['epoch'] or p['state'] not in ('ARMED', 'ACTIVE') or p['estop']):
            raise ValueError('authority permit mismatch')
        if command != ZERO and (not p['permit'] or p['state'] != 'ACTIVE'):
            raise ValueError('nonzero command without active authority')
        if self.owner is None:
            if p['permit'] or command != ZERO: raise ValueError('initial disabled baseline required')
            self.owner = p['source']
        if p['source'] != self.owner: raise ValueError('authority owner changed')
        self.sequence = p['sequence']
        return p, command


class TransportPolicy:
    """No automatic resume of a timed-out, released or replaced arm epoch."""
    def __init__(self, key, stage=0):
        self.verifier = AuthorityVerifier(key,stage)
        self.envelope, self.command, self.last = None, ZERO, -math.inf
        self.epoch, self.blocked = '', set()
        self.fault, self.hard_fault = '', False

    def revoke(self, reason, hard=False):
        if self.epoch: self.blocked.add(self.epoch)
        self.command = ZERO
        self.fault, self.hard_fault = reason, self.hard_fault or hard

    def receive(self, text, steady, ros):
        try:
            p, command = self.verifier.decode(text, steady, ros)
        except (ValueError, TypeError, KeyError, AttributeError):
            self.revoke('SIGNED_AUTHORITY_INVALID', hard=True)
            return False
        self.envelope, self.last = p, steady
        if p['estop']:
            self.revoke('ESTOP', hard=True)
        elif not p['permit']:
            if self.epoch: self.blocked.add(self.epoch)
            self.epoch, self.command = '', ZERO
            if not self.hard_fault: self.fault = ''
        elif p['state'] == 'ARMED' and p['epoch'] not in self.blocked and not self.hard_fault:
            self.epoch, self.command, self.fault = p['epoch'], ZERO, ''
        elif p['epoch'] != self.epoch or p['epoch'] in self.blocked or self.fault:
            self.command = ZERO
        else:
            self.command = command
        return not self.hard_fault

    def active(self, steady):
        if self.epoch and not 0 <= steady-self.last <= .10:
            self.revoke('SIGNED_AUTHORITY_TIMEOUT')
        return bool(not self.fault and not self.hard_fault and self.epoch and self.envelope
                    and self.envelope['permit'] and self.envelope['state'] == 'ACTIVE'
                    and self.envelope['epoch'] == self.epoch and self.epoch not in self.blocked)


class McuSampleClock:
    """Track real MCU sample progress across HELLOs, with uint32 wrap handling.

    Mapping starts with the first received MCU sample, not synchronized camera
    time. Repeated timestamps cannot keep feedback alive; later backlog is aged
    against the fixed local baseline instead of being stamped fresh on receipt.
    """
    def __init__(self):
        self.latest_tick = self.local_baseline = None
        self.elapsed_ms, self.by_kind, self.accepted_at = 0, {}, {}

    def observe(self, kind, tick, now):
        if type(tick) is not int or not 0 <= tick <= 0xffffffff: return None
        previous = self.by_kind.get(kind)
        if previous is not None and not 0 < ((tick-previous) & 0xffffffff) < 0x80000000: return None
        if self.latest_tick is None:
            self.latest_tick, self.local_baseline = tick, now
        delta = (tick-self.latest_tick) & 0xffffffff
        if delta >= 0x80000000: return None
        sample_at = self.local_baseline+(self.elapsed_ms+delta)/1000.
        if not -.02 <= now-sample_at <= .20: return None
        self.latest_tick, self.elapsed_ms = tick, self.elapsed_ms+delta
        # Only AFTER passing the original replay/order and [-20ms,+200ms]
        # sample-age checks, refine the unsynchronised clock offset toward an
        # earlier observed receipt. Capping just this sample at receipt left
        # the first delayed packet's offset in place; later good wheel packets
        # then oscillated over the -20ms boundary and starved host feedback.
        # The baseline NEVER moves later: old/backlogged samples only get
        # older, accepted_at caches are not renewed, and a genuinely excessive
        # future jump remains rejected by the unchanged check above.
        if sample_at > now:
            self.local_baseline -= sample_at-now
            sample_at = now
        self.by_kind[kind], self.accepted_at[kind] = tick, sample_at
        return sample_at
