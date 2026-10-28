"""Pure-clock SIM tests. No ROS, hardware imports, transport or device access."""
import ast
from dataclasses import FrozenInstanceError, asdict, replace
import math
from pathlib import Path

import pytest
import matching_sim as m
from mecanum_wand_servo_control.core import Pose, Velocity


@pytest.mark.parametrize('values', [(float('nan'), .3), (.7, float('inf')), (True, .3),
                                  (.71, 1.2), (.7, -1.), (.7, 20.)])
def test_only_enumerated_finite_sim_limits(values):
    with pytest.raises(ValueError):
        m.SimOnlyLimits(*values)


def test_only_speed_and_acceleration_differ_no_global_or_hardware_relaxation():
    baseline = m.accepted_config()
    c = m.SimOnlyLimits(.7, 1.2)
    for key, value in asdict(baseline).items():
        if key not in ('max_speed_mps', 'max_accel_mps2'):
            assert getattr(c, key) == value
    original = asdict(baseline.follower_config())
    new = asdict(c.follower_config())
    assert {k for k in original if original[k] != new[k]} == {'max_speed_mps', 'max_accel_mps2'}
    for change in ({'max_speed_mps': .7}, {'max_accel_mps2': 1.2}):
        with pytest.raises(ValueError):
            replace(baseline, **change)
    with pytest.raises(FrozenInstanceError):
        c.max_speed_mps = 1.
    assert not any(hasattr(c, name) for name in ('ip', 'socket', 'mode', 'hardware'))


def test_entrypoint_has_no_ros_network_or_device_imports_and_no_launch_mode():
    source = Path(m.__file__).read_text()
    tree = ast.parse(source)
    imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    imports += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
    assert not any(p and p.split('.')[0] in ('rclpy', 'socket', 'serial', 'subprocess') for p in imports)
    assert '--mode' not in source and '--hardware' not in source


def test_recorded_replay_never_interpolates_and_retains_duplicate_source_stamps():
    cases = m.recorded_cases()
    assert len(cases) == 5
    for c in cases:
        assert m.digest(Path(c.provenance['path'])) == c.provenance['sha256']
        assert all(b.t > a.t for a, b in zip(c.samples, c.samples[1:]))
        assert all(b.source_offset >= a.source_offset for a, b in zip(c.samples, c.samples[1:]))
    # Occlusion lead-in contains repeated samples; their exact stamp, XY stay
    # repeated rather than being refreshed/reconstructed by the replay.
    center3 = next(c for c in cases if c.name == 'real3_center')
    repeats = [(a, b) for a, b in zip(center3.samples, center3.samples[1:])
               if a.source_offset == b.source_offset]
    assert repeats and all((a.forward, a.left) == (b.forward, b.left) for a, b in repeats)


@pytest.mark.parametrize('speed,accel', [(.2, .08), (.5, 1.2), (.7, 1.2), (.7, 2.), (1., 2.)])
def test_smooth_xy_limits_mapping_ignored_z_yaw_and_release(speed, accel):
    c = m.SimOnlyLimits(speed, accel)
    for case in m.synthetic_cases()[:2]:
        r, _ = m.run_case(c, case)
        assert r['passed'] and r['raw_mapping_max_error_m'] < 1e-10
        assert r['max_command_speed_mps'] <= speed+1e-7
        assert r['max_command_accel_mps2'] <= accel+1e-6
        assert r['release_gate_zero_within_s'] <= .02


FAULTS = ('wand_loss', 'car_loss', 'source_replay', 'feedback_loss', 'operator_loss',
    'operator_release', 'wand_jump', 'car_jump', 'lift', 'tilt', 'boundary', 'mode_conflict',
    'source_conflict', 'command_loss', 'supervisor_loss', 'delayed_command', 'command_replay',
    'overspeed', 'acceleration_jump', 'reference_jump', 'owner_changed', 'actuator_fault',
    'estop', 'session_timeout', 'clock_backwards', 'clock_forward', 'timer_gap')


@pytest.mark.parametrize('speed,accel', [(.2, .08), (.5, 1.2), (.7, 1.2), (.7, 2.), (1., 2.)])
@pytest.mark.parametrize('fault', FAULTS)
def test_guards_not_removed_at_faster_model_limits(speed, accel, fault):
    c = m.SimOnlyLimits(speed, accel)
    h = m.Harness(c)
    if speed == .2:
        h.moving()
    else:
        h.start()
        for i in range(1, 101):
            h.set_hand(.4*i*.02, 0.)
            h.step()
        assert h.sup.state == 'ACTIVE' and math.hypot(h.output.vx, h.output.vy) > .3
    start = h.now
    if fault == 'wand_loss': h.pose_on['wand'] = False
    elif fault == 'car_loss': h.pose_on['car'] = False
    elif fault == 'source_replay': h.pose_stamp['wand'] = h.ros
    elif fault == 'feedback_loss': h.feedback_on = False
    elif fault == 'operator_loss': h.lease_on = False
    elif fault == 'operator_release': h.held = False
    elif fault == 'wand_jump': h.hand = replace(h.hand, x=h.hand.x+.3)
    elif fault == 'car_jump': h.plant.pose = replace(h.plant.pose, x=h.plant.pose.x+.21)
    elif fault == 'lift': h.plant.pose = replace(h.plant.pose, z=c.ground_z_m+.09)
    elif fault == 'tilt': h.roll = .3
    elif fault == 'boundary': h.plant.pose = Pose(*c.geometry().world(3.31, 1.5), c.ground_z_m, c.arena_heading_rad)
    elif fault == 'mode_conflict':
        for i in (h.sup.inputs, h.gate.inputs): i.mode = 'rectangle'
    elif fault == 'source_conflict':
        for i in (h.sup.inputs, h.gate.inputs): i.graph_ok = False
    elif fault == 'command_loss': h.command_link_on = False
    elif fault == 'supervisor_loss': h.supervisor_on = False
    elif fault == 'delayed_command': h.modify_frame = lambda f: replace(f, issued_steady=f.issued_steady-.15, issued_ros=f.issued_ros-.15)
    elif fault == 'command_replay':
        frame = h.gate.frame
        h.modify_frame = lambda f: frame
    elif fault == 'overspeed': h.modify_frame = lambda f: replace(f, command=Velocity(speed+.001))
    elif fault == 'acceleration_jump': h.modify_frame = lambda f: replace(f, command=Velocity(speed))
    elif fault == 'reference_jump': h.modify_frame = lambda f: replace(f, goal=replace(f.goal, x=f.goal.x+.10)) if f.goal else f
    elif fault == 'owner_changed': h.modify_frame = lambda f: replace(f, source='foreign-owner')
    elif fault == 'actuator_fault': h.fault_bits = 0x20
    elif fault == 'estop': h.sup.request_stop(h.now, 'sim estop', fault=True, hard=True)
    elif fault == 'session_timeout': h.sup.active_started = h.now-c.max_session_s
    zero_at = None
    for i in range(20):
        output = h.step(dt=.12 if fault == 'timer_gap' and i == 0 else .02,
                        ros_dt=-.02 if fault == 'clock_backwards' and i == 0 else
                        .4 if fault == 'clock_forward' and i == 0 else None)
        if output == m.ZERO and zero_at is None:
            zero_at = h.now-start
    assert zero_at is not None and zero_at <= .24+1e-8
    h.pose_on = {'car': True, 'wand': True}
    h.pose_stamp.clear()
    h.feedback_on = h.lease_on = h.supervisor_on = h.command_link_on = True
    h.modify_frame = lambda f: f
    h.roll = h.pitch = 0.
    h.fault_bits = 0
    for i in (h.sup.inputs, h.gate.inputs):
        i.mode = 'wand'
        i.graph_ok = True
    h.run(.6)
    assert h.output == m.ZERO and h.sup.state not in ('ACTIVE', 'ARMED')


@pytest.mark.parametrize('speed,accel', [(.5, 1.2), (.7, 1.2), (.7, 2.), (1., 2.)])
def test_occlusion_manual_reprepare_never_reuses_old_target(speed, accel):
    h = m.Harness(m.SimOnlyLimits(speed, accel))
    h.moving()
    h.pose_on['wand'] = False
    h.run(.3)
    assert h.output == m.ZERO and h.sup.fault_latched
    h.pose_on['wand'] = True
    h.run(2.)
    assert h.sup.state == 'FAULT' and not h.sup.request_recover(h.now, h.ros)[0]
    h.held = False
    h.run(1.8)
    assert h.sup.request_recover(h.now, h.ros)[0]
    h.run(1.)
    assert h.sup.state == 'ARMED' and h.output == m.ZERO
    parked = h.plant.pose
    h.held = True
    h.run(.4)
    assert h.sup.state == 'ACTIVE' and h.output == m.ZERO and h.plant.pose == parked


@pytest.mark.parametrize('speed,accel', [(.5, 1.2), (.7, 1.2), (.7, 2.), (1., 2.)])
def test_boundary_clamps_goal_without_expanding_or_recentering_fence(speed, accel):
    c = m.SimOnlyLimits(speed, accel)
    # Start in the saved corner, move nominal rod goal beyond 3m at .35m/s.
    case = m.Case('boundary', tuple(m.Sample(i*.02, min(i*.007, 3.5), min(i*.007, 3.5))
                                  for i in range(1, 1001)), origin=(0., 0.))
    polygon = c.goal_polygon()
    result, rows = m.run_case(c, case, keep_rows=True)
    assert result['passed'] and result['target_clamped_fraction'] > 0
    assert c.goal_polygon() == polygon
    assert all(-.30 <= r['car_local_f'] <= 3.30 and -.30 <= r['car_local_l'] <= 3.30 for r in rows)
    assert all(c.goal_inside(Pose(r['target_x'], r['target_y'], c.ground_z_m, c.arena_heading_rad)) for r in rows)


def test_performance_screen_does_not_use_failed_runs_or_hide_missing_reversals():
    assert m.select_candidate([]) is None
