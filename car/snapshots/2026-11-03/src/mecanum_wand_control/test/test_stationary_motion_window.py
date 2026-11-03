from dataclasses import replace
import math

import pytest

from mecanum_wand_control.core import Pose
from mecanum_wand_control.safety_core import Inputs, SafetyConfig, ZERO
from mecanum_wand_control.safety_suite import Harness


def test_static_pose_noise_does_not_become_false_speed_from_callback_batching():
    inputs = Inputs(SafetyConfig())
    for i in range(1, 201):
        source = 1000.+i*.01
        receipt = 10.+i*.01+(.003 if i%2 else 0.)
        pose = Pose(.00003*math.sin(i*1.7), .00003*math.cos(i*2.1), .1,
                    .00035*math.sin(i*1.3))
        assert inputs.observe_pose('car', pose, source, receipt, source+.01)
    assert inputs.car_speed < .005 and inputs.car_yaw_rate < .02


@pytest.mark.parametrize('mode', ['translation', 'rotation'])
def test_actual_continuous_motion_cannot_pass_the_same_stationarity_threshold(mode):
    inputs = Inputs(SafetyConfig())
    for i in range(1, 101):
        t = i*.01
        pose = Pose(.02*t if mode == 'translation' else 0., 0., .1,
                    .04*t if mode == 'rotation' else 0.)
        inputs.observe_pose('car', pose, 1000.+t, 10.+t, 1000.+t)
    assert (inputs.car_speed > .005 if mode == 'translation' else inputs.car_yaw_rate > .02)


def test_out_and_back_motion_is_not_hidden_by_equal_endpoints():
    inputs = Inputs(SafetyConfig())
    for i in range(51):
        t = i*.01
        x = .008*math.sin(math.pi*i/50)
        inputs.observe_pose('car', Pose(x, 0., .1), 1000.+t, 10.+t, 1000.+t)
    assert inputs.car_speed > .005


def test_fresh_pose_after_gap_must_rebuild_motion_window():
    inputs = Inputs(SafetyConfig())
    for i in range(60):
        inputs.observe_pose('car', Pose(0., 0., .1), 1000.+i*.01, 10.+i*.01, 1000.+i*.01)
    assert inputs.car_speed == 0.
    inputs.observe_pose('car', Pose(0., 0., .1), 1001., 11., 1001.)
    assert math.isinf(inputs.car_speed) and inputs.stationary_since is None


def test_yaw_wrap_does_not_look_like_full_rotation():
    inputs = Inputs(SafetyConfig())
    for i in range(60):
        yaw = math.pi-.0001 if i%2 else -math.pi+.0001
        inputs.observe_pose('car', Pose(0., 0., .1, yaw), 1000.+i*.01, 10.+i*.01, 1000.+i*.01)
    assert inputs.car_yaw_rate < .02


def test_full_chain_confirms_noisy_static_car_but_still_requires_actual_zero_feedback():
    h = Harness()
    for i in range(100):
        h.plant.pose = replace(h.plant.pose, x=.00003*math.sin(i*1.7),
                              y=.00003*math.cos(i*2.1), yaw=.00035*math.sin(i*1.3))
        h.step()
    assert h.sup.stop_confirmed
    assert h.sup.request_start(h.now, h.ros)[0]
    h.run(.1)
    h.held = True
    h.step()
    assert h.sup.state == 'ACTIVE' and h.output == ZERO


def test_raw_lift_is_not_smoothed_away_by_standstill_estimator():
    h = Harness()
    h.start()
    h.plant.pose = replace(h.plant.pose, z=.4)
    h.step()
    assert h.output == ZERO and h.sup.state != 'ACTIVE'
