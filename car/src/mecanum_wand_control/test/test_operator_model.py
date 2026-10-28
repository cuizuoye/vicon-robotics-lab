import math

import pytest

from mecanum_wand_control.operator_model import OperatorModel


def ready(state='ARMED'):
    model = OperatorModel()
    model.focus(True, 10.)
    model.receive_status(dict(state=state, gate_ready=True, ready_to_start=True), 10.)
    model.receive_gate(dict(fault='', hard_fault=False), 10.)
    model.tick(10.)
    return model


def test_default_never_enables():
    model = OperatorModel()
    assert not model.press(10.) and not model.held


def test_prepare_is_not_motion_and_held_start_is_refused():
    model = ready('STANDBY')
    assert model.can_start(10.)
    assert not model.press(10.)
    assert not model.can_start(10.)
    model.release(10.01)
    assert model.can_start(10.01)


def test_initial_focus_change_cannot_show_prepare_while_input_still_locked():
    model = ready('STANDBY')
    model.focus(False, 10.01)
    model.focus(True, 10.02)
    assert not model.can_start(10.02)
    assert '轻按并松开' in model.instruction(10.02)
    assert not model.press(10.03)
    model.release(10.04)
    assert model.can_start(10.04)
    model.receive_status(dict(state='ARMED', gate_ready=True), 10.05)
    assert '准备就绪' in model.instruction(10.05)
    assert model.press(10.06)


def test_press_before_gate_handshake_explains_release_without_another_prepare():
    model = ready()
    model.status['gate_ready'] = False
    assert '暂不要按空格' in model.instruction(10.)
    assert not model.press(10.01)
    model.status['gate_ready'] = True
    assert '不用再点准备' in model.instruction(10.02)
    model.release(10.03)
    assert '准备就绪' in model.instruction(10.03)
    assert model.press(10.04)


def test_gui_gap_requires_observed_release_before_new_prepare():
    model = ready('STANDBY')
    model.receive_status(model.status, 10.15)
    model.receive_gate(model.gate, 10.15)
    model.tick(10.15)
    assert not model.can_start(10.15)
    model.release(10.16)
    assert model.can_start(10.16)


def test_instruction_never_labels_latched_fault_or_stale_feedback_ready():
    model = ready()
    assert '不可使能' in model.instruction(10.5)
    model.status.update(state='FAULT', hard_fault=True)
    assert '硬故障' in model.instruction(10.)


def test_instruction_handles_enable_and_stop_transitions_without_false_lock():
    model = ready()
    assert model.press(10.)
    assert '使能已发送' in model.instruction(10.)
    model.status['state'] = 'ACTIVE'
    assert '正在跟随' in model.instruction(10.)
    model.release(10.01)
    assert '等待后端停车确认' in model.instruction(10.01)


def test_early_hold_and_autorepeat_cannot_enable_when_backend_later_arms():
    model = ready('STANDBY')
    assert not model.press(10.)
    model.receive_status(dict(state='ARMED', gate_ready=True), 10.01)
    assert not model.press(10.02) and not model.held
    model.release(10.03)
    assert model.press(10.04)


def test_release_and_focus_loss_never_auto_resume():
    model = ready()
    assert model.press(10.)
    model.focus(False, 10.01)
    assert not model.held
    model.focus(True, 10.02)
    assert not model.press(10.03)
    model.release(10.04)
    assert model.press(10.05)
    model.release(10.06)
    assert not model.held


@pytest.mark.parametrize('fault', ['stale', 'gui_gap', 'clock_backwards', 'closing', 'backend_fault'])
def test_operator_revokes_on_its_own_failures(fault):
    model = ready()
    assert model.press(10.)
    if fault == 'stale': model.tick(10.30)
    elif fault == 'gui_gap':
        model.receive_status(model.status, 10.15)
        model.receive_gate(model.gate, 10.15)
        model.tick(10.15)
    elif fault == 'clock_backwards': model.tick(9.99)
    elif fault == 'closing':
        model.closing = True
        model.tick(10.02)
    else:
        model.receive_status(dict(state='FAULT'), 10.02)
        model.tick(10.02)
    assert not model.held and model.blocked


def test_motion_is_2d_bounded_device_not_a_velocity_command():
    model = ready()
    assert model.move(.1, .2)
    assert (model.x, model.y) == (.1, .2)
    assert model.move(2., 2.)
    assert math.hypot(model.x, model.y) == pytest.approx(.245)
    assert not model.move(math.nan, 0.)
    assert not model.held


def test_virtual_input_remains_within_radius_after_nonzero_rebase():
    model = ready('ACTIVE')
    model.status['anchors'] = {'wand_origin': {'x': -.2, 'y': -.4}}
    model.move(-.24, 0.)
    assert math.hypot(model.x-.2, model.y) <= .245+1e-9
    assert math.hypot(model.x, model.y) <= .245+1e-9


def test_console_launch_only_includes_fixed_safe_backend_and_one_operator():
    import ast
    from pathlib import Path
    tree = ast.parse((Path(__file__).resolve().parents[1]/'launch/wand_safe_console.launch.py').read_text())
    nodes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'Node']
    assert len(nodes) == 1
    values = {k.arg: k.value for k in nodes[0].keywords}
    assert ast.literal_eval(values['executable']) == 'wand_safe_console'
    assert ast.literal_eval(values['namespace']) == 'wand_safe_sim'
    assert 'wand_safe_sim.launch.py' in ast.unparse(tree)
    assert 'hardware' not in ast.unparse(tree).lower()
