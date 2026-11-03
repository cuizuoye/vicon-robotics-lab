"""Deterministic PC-only unit checks: prescribed inputs, no plant/ROS/socket."""
from dataclasses import replace
import math

import pytest

from mecanum_wand_control.core import Config, Pose, WandSession
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.safety_core import SafetyConfig, Supervisor, FinalGate, Frame, Lease, ZERO


def follower(**kwargs):
    config = Config(governed_targets=True, max_speed_mps=.05,
                    max_accel_mps2=.08, max_goal_radius_m=3., **kwargs)
    s = WandSession(config)
    s.observe('car', Pose(0., 0., .1), 10.)
    s.observe('wand', Pose(0., 0., 1.), 10.)
    assert s.engage(10.)[0]
    return s


def sample(s, t, car_x, wand_x, car_y=0., wand_y=0.):
    s.observe('car', Pose(car_x, car_y, .1), t)
    s.observe('wand', Pose(wand_x, wand_y, 1.+.3*math.sin(t)), t)
    return s.tick(t)


def test_large_final_target_preserved_but_reference_and_vector_rate_bounded():
    s = follower()
    previous, command = s.goal, ZERO
    for i in range(1, 2701):
        elapsed = i*.02
        # Prescribed measurement trace, not a command-driven vehicle model.
        out = sample(s, 10.+elapsed, min(2., .04*elapsed), min(1., .5*elapsed))
        assert s.state == 'ACTIVE', s.reason
        assert math.hypot(s.goal.x-previous.x, s.goal.y-previous.y) <= .001+1e-8
        assert math.hypot(out.vx, out.vy) <= .05+1e-9
        assert math.hypot(out.vx-command.vx, out.vy-command.vy) <= .0016+1e-9
        assert abs(s.goal.x-min(2., .04*elapsed)) <= .201
        previous, command = s.goal, out
    assert s.mapped_goal.x == pytest.approx(2.)
    assert s.goal.x == pytest.approx(2., abs=.015)
    assert s.mapping.car_origin == Pose(0., 0., .1)


@pytest.mark.parametrize('axis', ['forward', 'diagonal', 'left'])
def test_stalled_car_is_not_hidden_by_reference_cap_or_target_updates(axis):
    s = follower()
    for i in range(1, 251):
        d = min(.8, i*.01)+.002*math.sin(i)
        wx, wy = (d, 0.) if axis == 'forward' else (0., d) if axis == 'left' else (d, d)
        sample(s, 10+i*.02, 0., wx, wand_y=wy)
        if s.state == 'FAULT':
            break
        assert math.hypot(s.goal.x, s.goal.y) <= .20+1e-8
    assert s.state == 'FAULT' and s.reason == 'CAR_PROGRESS_TIMEOUT'
    assert s.command == ZERO


def test_returned_wand_changes_final_target_without_rebase():
    s = follower()
    for i in range(1, 1251):
        sample(s, 10+i*.02, min(.8, i*.02*.04), min(.4, i*.01))
    assert s.state == 'ACTIVE' and s.mapped_goal.x == pytest.approx(.8)
    for i in range(1, 1251):
        sample(s, 35+i*.02, max(0., .8-i*.02*.04), max(0., .4-i*.01))
        assert s.state == 'ACTIVE', s.reason
    assert s.mapped_goal.x == pytest.approx(0., abs=1e-9)
    assert s.goal.x == pytest.approx(0., abs=.015)
    assert s.mapping.car_origin.x == 0.


def test_pause_discards_pending_target_and_new_engage_reanchors_only_explicitly():
    s = follower()
    for i in range(1, 51): sample(s, 10+i*.02, .02*i/50, min(.4, i*.01))
    assert s.mapped_goal.x > .7
    s.pause()
    assert s.goal is None and s.mapped_goal is None and s.tick(11.01) == ZERO
    sample(s, 11.02, .02, .4)
    assert s.engage(11.02)[0]
    assert s.goal.x == .02 and s.mapped_goal.x == .02 and s.command == ZERO


def test_legacy_follower_policy_is_unchanged_and_policy_validation_is_closed():
    assert not Config().governed_targets
    assert SafetyConfig().goal_reference_policy == 'direct'
    with pytest.raises(ValueError): SafetyConfig(goal_reference_policy='anything')
    with pytest.raises(ValueError): Config(governed_targets='true')


class StaticInputs:
    """Explicit fresh/withheld zero feedback samples; no simulated vehicle."""
    def __init__(self, core):
        self.core, self.now, self.seq = core, 10., 0

    def step(self, *, held=False, feedback=True, fault=0, feedback_speed=0.):
        self.now += .02
        self.seq += 1
        t, r = self.now, 90+self.now
        i = self.core.inputs
        i.observe_pose('car', Pose(0., 0., .1), r, t, r)
        i.observe_pose('wand', Pose(0., 0., 1.), r, t, r)
        i.observe_lease(Lease('operator', self.seq, t, r, held), t, r)
        if feedback:
            i.observe_feedback(self.seq, t, r, t, r, output_zero=True,
                enabled=False, speed=feedback_speed, yaw_rate=0., fault_bits=fault)
        if isinstance(self.core, Supervisor):
            self.core.observe_gate(steady=t, ros=r, epoch=self.core.epoch, fault='', hard_fault=False)
        return self.core.tick(t, r)

    def steps(self, n, **kw):
        for _ in range(n): self.step(**kw)


def stopped_fault():
    s = Supervisor(SafetyConfig(goal_reference_policy='governed'))
    feed = StaticInputs(s)
    feed.steps(100)
    assert s.request_start(feed.now, 90+feed.now)[0]
    old_epoch = s.epoch
    feed.step(held=True)
    assert s.state == 'ACTIVE'
    feed.steps(15, held=True, feedback=False)
    assert s.state in ('STOPPING', 'STOP_UNCONFIRMED') and not s.epoch
    feed.steps(70)
    assert s.state == 'FAULT' and s.stop_confirmed
    return s, feed, old_epoch


def test_feedback_return_alone_never_arms_then_explicit_recovery_only_arms_new_epoch():
    s, f, old = stopped_fault()
    f.steps(100)
    assert s.state == 'FAULT'
    assert s.request_recover(f.now, 90+f.now)[0]
    f.steps(40)
    assert s.state == 'ARMED' and s.epoch and s.epoch != old
    assert s.command == ZERO and s.active_started is None
    f.step(held=True)
    assert s.state == 'ACTIVE' and s.command == ZERO


@pytest.mark.parametrize('failure', ['held', 'feedback', 'stop', 'actuator', 'timer'])
def test_pending_recovery_cancels_instead_of_enabling_on_new_fault(failure):
    s, f, old = stopped_fault()
    assert s.request_recover(f.now, 90+f.now)[0]
    if failure == 'held': f.step(held=True)
    elif failure == 'feedback': f.steps(12, feedback=False)
    elif failure == 'stop': s.request_stop(f.now)
    elif failure == 'actuator': f.step(fault=1)
    elif failure == 'timer':
        s.tick(f.now+.2, 90+f.now+.2)
    assert s.recovery_deadline is None and s.state != 'ACTIVE'
    f.steps(70)
    assert s.epoch != old and s.state != 'ARMED'


@pytest.mark.parametrize('guard', ['held', 'unconfirmed', 'unhealthy', 'hard'])
def test_recovery_request_cannot_bypass_safety(guard):
    s, f, _ = stopped_fault()
    if guard == 'held': f.step(held=True)
    if guard == 'unconfirmed': s.stop_confirmed = False
    if guard == 'unhealthy': s.inputs.feedback = None
    if guard == 'hard': s.hard_fault = True
    assert not s.request_recover(f.now, 90+f.now)[0]


def test_recovery_button_is_not_available_with_stale_held_or_hard_state():
    m = OperatorModel()
    m.focused = True
    m.receive_status(dict(state='FAULT', stop_confirmed=True, hard_fault=False, input_health=''), 10.)
    m.receive_gate(dict(fault='FEEDBACK_TIMEOUT', hard_fault=False), 10.)
    assert m.can_recover(10.) and not m.can_recover(10.3)
    m.key_down = True
    assert not m.can_recover(10.)
    m.key_down = False
    m.status['recovery_pending'] = True
    assert not m.can_recover(10.) and not m.can_start(10.)


def test_recovery_wait_is_bounded_and_cannot_use_stationary_pose_to_hide_spinning_wheels():
    s, f, old = stopped_fault()
    assert s.request_recover(f.now, 90+f.now)[0]
    f.steps(260, feedback_speed=.02)
    assert s.recovery_deadline is None and s.reason == 'RECOVERY_CANCELLED'
    assert s.state == 'PAUSED' and not s.stop_confirmed and not s.epoch
    f.steps(100)
    assert s.state == 'PAUSED' and not s.epoch  # Fresh zero alone is not a new request.


def test_governor_lead_cap_does_not_drag_reference_to_an_externally_displaced_car():
    s = follower()
    for i in range(1, 30): sample(s, 10+i*.02, 0., min(.4, i*.01))
    reference = s.goal
    sample(s, 10.60, -.8, .4)
    assert s.goal == reference and s.reference_limited
    assert abs(s.goal.x-s.samples['car'][0].x) > .65


def test_independent_gate_checks_reference_step_between_valid_active_frames():
    c = SafetyConfig(goal_reference_policy='governed')
    g = FinalGate(c)
    feed = StaticInputs(g)
    for i in range(100):
        t, r = feed.now+.02, 90+feed.now+.02
        g.receive(Frame('owner', i, t, r, 'epoch', 'ARMED', False, ZERO, None), t, r)
        feed.step()
    for seq, x in [(101, 0.), (102, .002)]:
        t, r = feed.now+.02, 90+feed.now+.02
        g.receive(Frame('owner', seq, t, r, 'epoch', 'ACTIVE', True, ZERO, Pose(x, 0, .1)), t, r)
        feed.step(held=True)
    assert g.fault == 'FINAL_REFERENCE_STEP_LIMIT' and g.output == ZERO


def test_independent_gate_rejects_large_first_reference_even_with_valid_source_and_speed():
    c = SafetyConfig(goal_reference_policy='governed', max_goal_radius_m=2.)
    g = FinalGate(c)
    feed = StaticInputs(g)
    for i in range(100):
        t, r = feed.now+.02, 90+feed.now+.02
        g.receive(Frame('owner', i, t, r, 'epoch', 'ARMED', False, ZERO, None), t, r)
        feed.step()
    assert g.primed_epoch == 'epoch'
    t, r = feed.now+.02, 90+feed.now+.02
    g.receive(Frame('owner', 101, t, r, 'epoch', 'ACTIVE', True, ZERO, Pose(.3, 0, .1)), t, r)
    feed.step(held=True)
    assert g.output == ZERO and g.fault == 'FINAL_REFERENCE_STEP_LIMIT'
