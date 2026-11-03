import math
import random

import pytest

from mecanum_wand_control.core import Config, Pose, RelativeMapping, Velocity, WandSession
from mecanum_wand_control.offline_suite import scenarios, simulate_case


@pytest.mark.parametrize('yaw', [0, math.pi/2, math.pi/3, -1.2])
@pytest.mark.parametrize('local', [(.5, 0), (0, .25), (.4, .3), (0, 0), (-.2, -.1)])
def test_fixed_start_frame_exact_scale_and_return(yaw, local):
    car, wand = Pose(3, -2, .1, yaw), Pose(-1, 4, 1.2, -.8)
    mapping = RelativeMapping(car, wand, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    f, l = local
    moved = Pose(wand.x+c*f-s*l, wand.y+s*f+c*l, 8, 2.4)
    assert mapping.local_displacement(moved) == pytest.approx(local)
    goal = mapping.goal(moved)
    assert (goal.x, goal.y) == pytest.approx((car.x+2*(c*f-s*l), car.y+2*(s*f+c*l)))
    assert goal.yaw == yaw and goal.z == car.z
    assert mapping.goal(wand) == car


def ready():
    session = WandSession()
    session.observe('car', Pose(0, 0), 1.0)
    session.observe('wand', Pose(3, 4, 1), 1.0)
    return session


def test_default_inert_and_refuses_missing_or_old_poses():
    session = WandSession()
    assert not session.engage(1)[0]
    assert session.tick(1) == Velocity()
    session = ready()
    assert not session.engage(1.21)[0]
    assert session.state == 'STANDBY'


@pytest.mark.parametrize('lost', ['wand', 'car'])
def test_each_input_loss_latches_zero_and_never_auto_resumes(lost):
    session = ready()
    assert session.engage(1)[0]
    other = 'car' if lost == 'wand' else 'wand'
    session.observe(other, session.samples[other][0], 1.3)
    assert session.tick(1.3) == Velocity()
    assert session.state == 'FAULT'
    for source in ('car', 'wand'):
        session.observe(source, Pose(0, 0), 1.4)
    assert session.tick(1.4) == Velocity() and session.state == 'FAULT'
    assert session.engage(1.4)[0]
    assert session.tick(1.4) == Velocity()


def test_duplicate_or_out_of_order_source_frames_cannot_refresh():
    session = ready()
    assert session.engage(1)[0]
    assert not session.observe('wand', Pose(3, 4), 1.0)
    assert not session.observe('wand', Pose(3, 4), .9)
    session.observe('car', Pose(0, 0), 1.3)
    session.tick(1.3)
    assert session.state == 'FAULT'


def test_pause_reengage_rebases_measured_car_and_hand_without_jump():
    session = ready()
    session.engage(1)
    session.observe('wand', Pose(3.2, 4), 1.02)
    session.observe('car', Pose(0, 0), 1.02)
    assert session.tick(1.02).vx > 0
    session.pause()
    session.observe('wand', Pose(-5, -3), 1.1)
    session.observe('car', Pose(.1, -.2, yaw=.3), 1.1)
    assert session.tick(1.1) == Velocity()
    assert session.engage(1.1)[0]
    assert session.goal == Pose(.1, -.2, yaw=.3)
    assert session.tick(1.1) == Velocity()
    session.pause(stop=True)
    assert session.goal is None and session.state == 'STANDBY'


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_invalid_pose_closes_output(value):
    session = ready()
    session.engage(1)
    session.observe('wand', Pose(value, 0), 1.02)
    assert session.tick(1.02) == Velocity() and session.state == 'FAULT'


def test_wand_jump_and_workspace_limit_are_explicit_faults():
    session = ready()
    session.engage(1)
    session.observe('wand', Pose(3.5, 4), 1.02)
    assert session.state == 'FAULT' and 'jump' in session.reason
    session = ready()
    session.engage(1)
    for i in range(1, 12):
        t = 1+.02*i
        session.observe('car', Pose(0, 0), t)
        session.observe('wand', Pose(3+.1*i, 4), t)
        session.tick(t)
    assert session.state == 'FAULT' and 'workspace' in session.reason


def test_hand_tremor_height_and_orientation_do_not_move_target():
    rng = random.Random(42)
    session = ready()
    session.engage(1)
    for i in range(1, 101):
        t = 1+.02*i
        session.observe('car', Pose(0, 0), t)
        session.observe('wand', Pose(3+rng.uniform(-.001, .001),
                                    4+rng.uniform(-.001, .001), 1+math.sin(t), t), t)
        assert session.tick(t) == Velocity()
        assert session.goal == Pose(0, 0)


def test_heading_is_latched_even_when_car_is_rotated_by_disturbance():
    session = ready()
    session.engage(1)
    session.observe('car', Pose(0, 0, yaw=math.pi/2), 1.02)
    session.observe('wand', Pose(3.1, 4), 1.02)
    command = session.tick(1.02)
    assert command.vy < 0 and abs(command.vx) < 1e-9
    assert command.wz < 0 and session.goal.yaw == 0


@pytest.mark.parametrize('changes', [{'scale': 0}, {'scale': math.nan}, {'max_speed_mps': -.1}])
def test_bad_config_rejected(changes):
    with pytest.raises(ValueError):
        Config(**changes)


@pytest.mark.parametrize('case', scenarios(), ids=[s[0] for s in scenarios()])
def test_closed_loop_scenarios(case):
    summary, _ = simulate_case(*case)
    assert summary['passed'], summary
