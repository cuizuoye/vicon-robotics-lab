"""Same-host, per-launch signed simulation output. Not a hardware protocol.

Jazzy rclpy MessageInfo here does not expose publisher_gid. The actuator never
subscribes to raw Twist; it verifies this channel and pins its signed owner.
This prevents accidental bypass/replay, not a malicious local account or DoS.
"""

from dataclasses import asdict
import hashlib
import hmac
import json
import math
import uuid

from .core import Velocity
from .safety_core import ZERO, fresh_envelope


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result: raise ValueError('duplicate JSON field')
        result[key] = value
    return result


def load_json(text):
    if len(text) > 4096: raise ValueError('oversized envelope')
    result = json.loads(text, object_pairs_hook=unique_object,
                        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    if not isinstance(result, dict): raise ValueError('object required')
    return result


def finite_number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite numeric value required')
    return float(value)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def secret_bytes(hex_key):
    value = bytes.fromhex(hex_key)
    if len(value) != 32: raise ValueError('per-launch 256-bit key required')
    return value


class OutputSigner:
    def __init__(self, hex_key):
        self.key = secret_bytes(hex_key)
        self.owner, self.sequence = uuid.uuid4().hex, 0

    def encode(self, command, steady, ros):
        self.sequence += 1
        payload = dict(source=self.owner, sequence=self.sequence,
                       issued_steady=steady, issued_ros=ros, command=asdict(command))
        signature = hmac.new(self.key, canonical(payload), hashlib.sha256).hexdigest()
        return json.dumps(dict(payload=payload, signature=signature), allow_nan=False)


class OutputVerifier:
    def __init__(self, hex_key):
        self.key = secret_bytes(hex_key)
        self.owner, self.sequence = None, -1

    def decode(self, text, steady, ros):
        envelope = load_json(text)
        if set(envelope) != {'payload', 'signature'}: raise ValueError('invalid signed envelope')
        p, signature = envelope['payload'], envelope['signature']
        if not isinstance(p, dict) or not isinstance(signature, str): raise ValueError('invalid signature fields')
        expected = hmac.new(self.key, canonical(p), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(expected, signature): raise ValueError('output signature mismatch')
        if set(p) != {'source', 'sequence', 'issued_steady', 'issued_ros', 'command'}:
            raise ValueError('invalid payload schema')
        if (not isinstance(p['source'], str) or not 0 < len(p['source']) <= 80
                or type(p['sequence']) is not int or p['sequence'] <= self.sequence):
            raise ValueError('replayed or invalid signed output')
        if not fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']),
                              steady, ros, .10):
            raise ValueError('expired or future signed output')
        if set(p['command']) != {'vx', 'vy', 'wz'}: raise ValueError('invalid velocity schema')
        command = Velocity(**{k: finite_number(v) for k, v in p['command'].items()})
        if math.hypot(command.vx, command.vy) > .05+1e-9 or abs(command.wz) > .10+1e-9:
            raise ValueError('simulated actuator hard limit')
        if self.owner is None:
            if command != ZERO: raise ValueError('initial signed disabled-zero required')
            self.owner = p['source']
        if p['source'] != self.owner: raise ValueError('signed output owner changed')
        self.sequence = p['sequence']
        return command
