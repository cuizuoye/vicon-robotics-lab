from dataclasses import replace
import math

import pytest

from mecanum_wand_control.core import Pose, Velocity
from mecanum_wand_control.safety_core import SafetyConfig, ZERO
from mecanum_wand_control.safety_core import Lease, FinalGate
from mecanum_wand_control.safety_suite import FAULT_CASES, Harness, fault_case, normal_case, startup_case


@pytest.mark.parametrize('name', FAULT_CASES)
def test_fault_injection(name):
    assert fault_case(name)[0]['passed']


def test_normal_closed_loop_pause_and_rebase():
    assert normal_case()[0]['passed']


def test_startup_never_arms_from_pose_or_held_enable():
    assert startup_case()[0]['passed']


def test_cold_idle_scheduling_gap_resets_readiness_without_granting_authority():
    h = Harness()
    h.run(.4)
    h.step(dt=.3)
    assert h.sup.state == 'STANDBY' and h.output == ZERO and not h.sup.hard_fault
    assert not h.sup.request_start(h.now, h.ros)[0]
    h.run(1.8)
    assert h.sup.request_start(h.now, h.ros)[0]


@pytest.mark.parametrize('changes', [dict(max_speed_mps=.151), dict(max_yaw_rate_radps=.201),
    dict(max_accel_mps2=.31), dict(max_session_s=141), dict(lease_timeout_s=math.inf),
    dict(arena_x_min=2), dict(boundary_margin_m=2), dict(command_timeout_s=.2)])
def test_configuration_refuses_unsafe_overrides(changes):
    with pytest.raises(ValueError): SafetyConfig(**changes)


def test_new_arm_requires_release_stable_feedback_and_new_epoch():
    h = Harness()
    h.moving()
    old_epoch = h.sup.epoch
    h.held = False
    h.step()
    assert not h.sup.request_start(h.now, h.ros)[0]
    h.run(2.)
    assert h.sup.state == 'PAUSED' and h.sup.stop_confirmed
    assert h.sup.request_start(h.now, h.ros)[0]
    h.run(.1)
    assert h.sup.epoch != old_epoch
    assert h.gate.primed_epoch == h.sup.epoch
    h.held = True
    h.step()
    assert h.sup.state == 'ACTIVE' and h.output == ZERO


def test_data_recovery_needs_acknowledgement_then_new_manual_start():
    h = Harness()
    h.moving()
    h.pose_on['wand'] = False
    h.run(.4)
    h.pose_on['wand'] = True
    h.run(2.)
    assert h.sup.state == 'FAULT'
    assert not h.sup.acknowledge(h.now, h.ros)[0]  # Hold still pressed.
    h.held = False
    h.run(.2)
    assert h.sup.acknowledge(h.now, h.ros)[0]
    h.run(.8)
    assert h.sup.request_start(h.now, h.ros)[0]
    h.run(.1)
    h.held = True
    h.step()
    assert h.sup.state == 'ACTIVE' and h.output == ZERO


def test_actuator_fault_and_estop_cannot_be_software_acknowledged():
    for kind in ('actuator', 'estop'):
        h = Harness()
        h.moving()
        if kind == 'actuator': h.fault_bits = 1
        else: h.gate.trip('ESTOP', hard=True)
        h.run(2.)
        h.fault_bits = 0
        h.held = False
        h.run(.8)
        assert h.sup.hard_fault and not h.sup.acknowledge(h.now, h.ros)[0]


def test_valid_frame_after_bad_pose_in_same_tick_does_not_mask_fault():
    h = Harness()
    h.moving()
    for inputs in (h.sup.inputs, h.gate.inputs):
        inputs.observe_pose('car', replace(h.plant.pose, z=.5), h.ros+.001,
                            h.now+.001, h.ros+.001)
    h.step()
    assert h.output == ZERO and h.sup.fault_latched


def test_release_and_repress_within_one_tick_cannot_mask_release():
    h = Harness()
    h.moving()
    for inputs in (h.sup.inputs, h.gate.inputs):
        inputs.observe_lease(Lease('operator-test', h.sequence+1, h.now, h.ros, False), h.now, h.ros)
        inputs.observe_lease(Lease('operator-test', h.sequence+2, h.now, h.ros, True), h.now, h.ros)
    h.sequence += 2
    h.step()
    assert h.output == ZERO and h.sup.state != 'ACTIVE'


def test_independent_gate_rejects_without_disabled_baseline_or_with_own_pose_missing():
    h = Harness()
    h.moving()
    fresh = FinalGate()
    assert not fresh.receive(h.frame, h.now, h.ros)
    h.gate.inputs.poses.pop('wand')
    assert h.sup.state == 'ACTIVE' and h.frame.enabled
    assert h.gate.tick(h.now, h.ros) == ZERO


def test_footprint_yaw_and_margin_are_enforced_at_goal_and_actual_car():
    c = SafetyConfig()
    assert c.inside(Pose(1.10, 0, yaw=0))
    assert not c.inside(Pose(1.10, 0, yaw=math.pi/4))
    assert not c.inside(Pose(1.30, 0))


def test_one_old_replayed_command_never_renews_gate_deadline():
    h = Harness()
    h.moving()
    old = h.frame
    h.modify_frame = lambda _: old
    h.run(.14)
    assert h.output == ZERO and h.gate.fault


def test_gate_refuses_new_owner_and_no_baseline_enabled_frame():
    h = Harness()
    h.moving()
    assert not h.gate.receive(replace(h.frame, source='other-process', sequence=9999), h.now, h.ros)
    assert h.output != ZERO  # Core output is returned by next safety tick.
    assert h.gate.output == ZERO
    h.step()
    assert h.output == ZERO


def test_stop_without_fresh_feedback_stays_unconfirmed_not_timed_standby():
    h = Harness()
    h.moving()
    h.feedback_on = False
    h.sup.request_stop(h.now)
    h.run(2.)
    assert h.sup.state == 'STOP_UNCONFIRMED' and not h.sup.stop_confirmed
    h.feedback_on = True
    h.held = False
    h.run(1.)
    assert h.sup.state == 'PAUSED' and h.sup.stop_confirmed
