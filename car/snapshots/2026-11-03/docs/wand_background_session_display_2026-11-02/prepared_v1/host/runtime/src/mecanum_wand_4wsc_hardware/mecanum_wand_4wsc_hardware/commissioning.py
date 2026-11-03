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

from mecanum_wand_4wsc_control.core import Pose, Velocity
from mecanum_wand_4wsc_control.safety_core import ZERO, fresh_envelope
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig as SafetyConfig
from .stages import STAGES,get_stage,SCOPE
from .authorization import verify_runtime_profile
from mecanum_wand_4wsc_control.safety_wire import canonical, finite_number, load_json, secret_bytes

from .host_profile import Runtime, DOMAIN, NAMESPACE, WORLD, CONSENT

def runtime_from_environment():
    return Runtime(os.environ.get('WAND_4WSC_MODE', 'hardware'),
        os.environ.get('WAND_4WSC_PROFILE', ''), os.environ.get('WAND_PROFILE_APPROVAL', ''),
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

    def decode(self, text, steady, ros, *, ignore_expired_disabled=False):
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
        fresh = fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']), steady, ros, .10)
        if set(p['command']) != {'vx', 'vy', 'wz'}: raise ValueError('command schema')
        command = Velocity(**{k: finite_number(v) for k, v in p['command'].items()})
        if math.hypot(command.vx, command.vy) > self.limits.speed+1e-9 or abs(command.wz) > self.limits.yaw_rate+1e-9:
            raise ValueError('hardware acceptance hard speed limit')
        from mecanum_wand_4wsc_control.rotation_servo import wheel_rpm
        if max(map(abs, wheel_rpm(command))) > self.limits.target_rpm+1e-6:
            raise ValueError('signed output exceeds coupled wheel RPM envelope')
        states = ('STANDBY', 'ARMED', 'ACTIVE', 'STOPPING', 'STOP_UNCONFIRMED', 'PAUSED', 'FAULT')
        if p['state'] not in states: raise ValueError('authority state')
        if p['permit'] and (not p['epoch'] or p['state'] not in ('ARMED', 'ACTIVE') or p['estop']):
            raise ValueError('authority permit mismatch')
        if command != ZERO and (not p['permit'] or p['state'] != 'ACTIVE'):
            raise ValueError('nonzero command without active authority')
        if not fresh:
            from .cold_authority import discardable_zero
            if ignore_expired_disabled and discardable_zero(self,p,command,steady,ros):return None
            raise ValueError('authority expired')
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
            from .cold_authority import virgin_transport
            decoded = self.verifier.decode(text, steady, ros, ignore_expired_disabled=virgin_transport(self))
            if decoded is None:return False
            p, command = decoded
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
