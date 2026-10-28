from dataclasses import replace
import math
import time

import pytest

from mecanum_wand_control.core import Pose
from mecanum_wand_control.operator_guidance import center_bounds, real_instruction, reason_text, wand_preview
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.safety_core import SafetyConfig, ZERO
from mecanum_wand_control.safety_suite import Harness


def site_config():
    return SafetyConfig(arena_x_min=-.7, arena_x_max=.7, arena_y_min=-.7,
                        arena_y_max=.7, half_length_m=.19, half_width_m=.21)


@pytest.mark.parametrize('yaw', [0., math.pi/2, 1.5164931576573302])
def test_display_inner_bounds_match_authoritative_footprint_guard(yaw):
    config = site_config()
    xmin, xmax, ymin, ymax = center_bounds(config, yaw)
    assert config.inside(Pose(xmin, ymin, yaw=yaw))
    assert config.inside(Pose(xmax, ymax, yaw=yaw))
    assert not config.inside(Pose(xmin-1e-6, 0., yaw=yaw))
    assert not config.inside(Pose(0., ymax+1e-6, yaw=yaw))


def test_site_budget_does_not_repeat_misleading_quarter_metre_hint():
    p = wand_preview(site_config(), Pose(0, 0, .1, 1.5164931576573302), Pose(2, 3, .03), Pose(2, 3, 1.2))
    assert .13 < p['radial_budget_m'] < .15
    assert p['suggested_step_m'] == .10
    assert p['goal'] == Pose(0, 0, .1, 1.5164931576573302) and p['inside']
    assert p['dz'] == pytest.approx(1.17)


def test_same_xy_mapping_is_independent_of_wand_height():
    config, car, origin = site_config(), Pose(0, 0, .1), Pose(2, 3, .03)
    low = wand_preview(config, car, origin, Pose(2.04, 3.10, .03))
    high = wand_preview(config, car, origin, Pose(2.04, 3.10, 1.8))
    assert high['goal'] == low['goal'] and high['inside'] == low['inside']
    assert high['goal'].z == car.z


def test_pickup_with_horizontal_motion_can_cross_boundary_but_z_alone_cannot():
    config, car, origin = site_config(), Pose(0, 0, .1, 1.5164931576573302), Pose(0, 0, .035)
    vertical = wand_preview(config, car, origin, Pose(0, 0, .353))
    observed = wand_preview(config, car, origin, Pose(-.0394, .1803, .353))
    assert vertical['inside'] and vertical['goal'] == car
    assert not observed['inside']
    assert '不是Z' in reason_text('GOAL_OUTSIDE_FIXED_ARENA')


def test_full_supervisor_and_independent_gate_ignore_pure_wand_z():
    h = Harness(site_config())
    h.start()
    for i in range(150):
        h.hand = replace(h.hand, z=.03+1.5*abs(math.sin(i*.04)), yaw=i*.02)
        assert h.step() == ZERO
        assert h.sup.state == 'ACTIVE' and not h.gate.fault
    assert h.sup.follower.goal == h.plant.pose


def test_xy_following_works_while_wand_height_varies():
    h = Harness(site_config())
    h.start()
    initial = h.plant.pose
    for i in range(100):
        h.hand = replace(h.hand, x=-.4+.001*(i+1), z=.03+abs(math.sin(i*.04)))
        h.step()
        assert h.sup.state == 'ACTIVE' and not h.gate.fault
    h.run(6.)
    assert h.sup.follower.goal.x == pytest.approx(initial.x+.2, abs=1e-6)
    assert abs(h.plant.pose.x-initial.x-.2) < .025


def test_recoverable_fault_explains_release_then_ack_not_estop():
    model = OperatorModel()
    model.status = dict(state='FAULT', reason='GOAL_OUTSIDE_FIXED_ARENA', stop_confirmed=True)
    model.key_down = True
    assert '先松开空格' in real_instruction(model, time.monotonic())
    model.key_down = False
    assert '确认可恢复故障' in real_instruction(model, time.monotonic())
    assert '不要点击急停' in real_instruction(model, time.monotonic())


def test_real_guidance_does_not_hide_blocked_key_interlock():
    model = OperatorModel()
    now = time.monotonic()
    model.focused, model.blocked = True, True
    model.receive_status(dict(state='ARMED', gate_ready=True), now)
    model.receive_gate(dict(fault='', hard_fault=False), now)
    assert '松开一次空格' in real_instruction(model, now)


def test_hardware_hard_fault_never_suggests_simulation_only_restart():
    model = OperatorModel()
    model.status = dict(state='FAULT', reason='FINAL_GATE: ESTOP', hard_fault=True)
    text = real_instruction(model, time.monotonic(), hardware=True)
    assert '实车保护已锁存' in text and '不能用确认按钮解除' in text
    assert '本仿真' not in text
