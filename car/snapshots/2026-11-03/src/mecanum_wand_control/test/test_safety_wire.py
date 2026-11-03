import json

import pytest

from mecanum_wand_control.core import Velocity
from mecanum_wand_control.safety_core import ZERO
from mecanum_wand_control.safety_wire import OutputSigner, OutputVerifier

KEY = 'a1'*32


def test_per_launch_signed_disabled_baseline_then_velocity():
    signer, verifier = OutputSigner(KEY), OutputVerifier(KEY)
    assert verifier.decode(signer.encode(ZERO, 10., 1000.), 10., 1000.) == ZERO
    moving = signer.encode(Velocity(.03, .02, .05), 10.02, 1000.02)
    assert verifier.decode(moving, 10.03, 1000.03) == Velocity(.03, .02, .05)
    with pytest.raises(ValueError): verifier.decode(moving, 10.04, 1000.04)


@pytest.mark.parametrize('failure', ['wrong_key', 'tamper', 'expired', 'future', 'new_owner', 'no_baseline', 'overspeed'])
def test_actuator_identity_replay_age_and_bounds(failure):
    signer, verifier = OutputSigner(KEY), OutputVerifier(KEY)
    if failure != 'no_baseline':
        verifier.decode(signer.encode(ZERO, 10., 1000.), 10., 1000.)
    if failure == 'wrong_key': signer = OutputSigner('b2'*32)
    if failure == 'new_owner':
        signer = OutputSigner(KEY)
        signer.sequence = 10
    velocity = Velocity(.06, 0, 0) if failure == 'overspeed' else Velocity(.02, 0, 0)
    message = signer.encode(velocity, 10.02, 1000.02)
    if failure == 'tamper':
        value = json.loads(message)
        value['payload']['command']['vx'] = .04
        message = json.dumps(value)
    at = 10.5 if failure == 'expired' else (9.9 if failure == 'future' else 10.03)
    with pytest.raises(ValueError): verifier.decode(message, at, 1000.+at-10)


def test_plant_never_subscribes_to_unstamped_twist():
    import ast
    from pathlib import Path
    tree = ast.parse((Path(__file__).resolve().parents[1]/'mecanum_wand_control/safety_nodes.py').read_text())
    plant = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PlantNode')
    subscriptions = [call for call in ast.walk(plant) if isinstance(call, ast.Call)
                     and isinstance(call.func, ast.Attribute) and call.func.attr == 'create_subscription']
    assert {ast.literal_eval(call.args[1]) for call in subscriptions} == {'actuator_command', 'inject'}
