"""Scoped same-host SIM ONLY signed channel; never an MCU protocol.

The original .20m/s verifier is untouched and cannot decode this new schema.
Only this separate software plant accepts the .70m/s simulation envelope.
"""
from dataclasses import asdict
import hashlib
import hmac
import json
import math

from mecanum_wand_servo_control.core import Velocity
from mecanum_wand_servo_control.safety_core import ZERO, fresh_envelope
from mecanum_wand_servo_control.safety_wire import (
    OutputSigner, OutputVerifier, canonical, finite_number, load_json)

SCOPE = 'SIM_ONLY_SIGN_MATCHING_V070_A200_V1'


class MatchingOutputSigner(OutputSigner):
    def encode(self, command, steady, ros):
        self.sequence += 1
        payload = dict(source=self.owner, sequence=self.sequence, issued_steady=steady,
                       issued_ros=ros, command=asdict(command), simulation_scope=SCOPE)
        signature = hmac.new(self.key, canonical(payload), hashlib.sha256).hexdigest()
        return json.dumps(dict(payload=payload, signature=signature), allow_nan=False)


class MatchingOutputVerifier(OutputVerifier):
    def decode(self, text, steady, ros):
        envelope = load_json(text)
        if set(envelope) != {'payload', 'signature'}:
            raise ValueError('invalid signed envelope')
        p, signature = envelope['payload'], envelope['signature']
        if not isinstance(p, dict) or not isinstance(signature, str):
            raise ValueError('invalid signature fields')
        expected = hmac.new(self.key, canonical(p), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(expected, signature):
            raise ValueError('output signature mismatch')
        if set(p) != {'source', 'sequence', 'issued_steady', 'issued_ros', 'command', 'simulation_scope'}:
            raise ValueError('invalid SIM ONLY payload schema')
        if p['simulation_scope'] != SCOPE:
            raise ValueError('wrong SIM ONLY channel scope')
        if (not isinstance(p['source'], str) or not 0 < len(p['source']) <= 80
                or type(p['sequence']) is not int or p['sequence'] <= self.sequence):
            raise ValueError('replayed or invalid signed output')
        if not fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']),
                              steady, ros, .10):
            raise ValueError('expired or future signed output')
        if not isinstance(p['command'], dict) or set(p['command']) != {'vx', 'vy', 'wz'}:
            raise ValueError('invalid velocity schema')
        command = Velocity(**{k: finite_number(v) for k, v in p['command'].items()})
        if math.hypot(command.vx, command.vy) > .70+1e-9 or abs(command.wz) > .10+1e-9:
            raise ValueError('SIM ONLY actuator hard limit')
        if self.owner is None:
            if command != ZERO:
                raise ValueError('initial signed disabled-zero required')
            self.owner = p['source']
        if p['source'] != self.owner:
            raise ValueError('signed output owner changed')
        self.sequence = p['sequence']
        return command
