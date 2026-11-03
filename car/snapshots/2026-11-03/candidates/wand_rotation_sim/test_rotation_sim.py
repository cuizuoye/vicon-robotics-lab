"""Pure software control/gate/geometry tests. Never initializes hardware I/O."""
import ast
from dataclasses import FrozenInstanceError,asdict,replace
import json
import math
from pathlib import Path
import numpy as np
import pytest

from bootstrap import ACCEPTED,ROOT
from rotation_core import (SimRotationLimits,reference_pose,wheel_rpm,coupled_shape,
    RotationMapping,MAX_RPM,Pose,Velocity,ZERO)
from harness import RotationHarness
from experiments import run_case,cases
from preview_core import rotation


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    import socket
    def forbidden(*a,**kw):pytest.fail('offline rotation test attempted external I/O')
    monkeypatch.setattr(socket,'socket',forbidden)
    try:
        import rclpy
        monkeypatch.setattr(rclpy,'init',forbidden)
    except ImportError:pass


@pytest.mark.parametrize('limits',[(True,4.),(.11,4.),(float('nan'),4.),(2.,0.),(2.,7.),(2.,float('inf'))])
def test_only_enumerated_simulation_parameters_are_allowed(limits):
    with pytest.raises(ValueError):SimRotationLimits(*limits)


def test_original_configuration_and_all_nonangular_guards_remain_unchanged():
    original=asdict(ACCEPTED);c=SimRotationLimits(2.,4.)
    for key,value in original.items():
        if key not in ('max_yaw_rate_radps','max_yaw_accel_radps2'):assert getattr(c,key)==value
    assert asdict(ACCEPTED)==original
    with pytest.raises(FrozenInstanceError):c.max_yaw_rate_radps=5.
    with pytest.raises(ValueError):replace(ACCEPTED,max_yaw_rate_radps=2.)
    assert not any(hasattr(c,key) for key in ('ip','mode','socket','hardware','runtime_approval'))


def test_measured_sign5_offset_corrects_a_pure_rotation_about_the_crossbar_control_point():
    definition=json.loads((ROOT/'docs/wand_rotation_preview_2026-10-26/readonly_uppz8hn2/geometry_confirmed.json').read_text())
    point=np.array([1.,2.,1.2]);raw_origins=[];headings=[]
    for angle in np.linspace(0.,2*math.pi,121):
        q=(0.,0.,math.sin(angle/2),math.cos(angle/2));r=rotation(q)
        origin=point-r@np.array(definition['local_points']['Sign5']);raw_origins.append(origin)
        p=reference_pose(origin,q,definition);headings.append(p.yaw)
        assert (p.x,p.y,p.z)==pytest.approx(point,abs=1e-12)
    assert np.linalg.norm(raw_origins[0]-raw_origins[60])>.14 # Real ~75mm offset, not zero.
    assert sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(headings,headings[1:]))==pytest.approx(2*math.pi)


@pytest.mark.parametrize('direction',[1.,-1.])
def test_pure_relative_rotation_90_return_without_xy_drift(direction):
    h=RotationHarness(SimRotationLimits(2.,4.));h.start();h.car_wrap=True
    for i in range(1,151):h.set_gesture(0.,0.,direction*math.pi/2*min(i/100,1.));h.step()
    h.run(3.)
    assert h.sup.state=='ACTIVE' and not h.gate.fault
    assert abs(h.sup.follower.car_continuous-h.origin.yaw-direction*math.pi/2)<.035
    for i in range(1,101):h.set_gesture(0.,0.,direction*math.pi/2*(1-i/100));h.step()
    h.run(3.)
    assert abs(h.sup.follower.car_continuous-h.origin.yaw)<.035
    assert math.hypot(h.plant.pose.x-h.origin.x,h.plant.pose.y-h.origin.y)<1e-10


def test_translation_reference_does_not_rotate_with_the_rod_or_car():
    h=RotationHarness(SimRotationLimits(2.,4.));h.start()
    for i in range(1,151):
        h.set_gesture(.20*min(i/100,1.),.10*min(i/100,1.),math.pi/2*min(i/100,1.));h.step()
    mapping=h.sup.follower.mapping
    expected_x=h.origin.x+.20*math.cos(h.origin.yaw)-.10*math.sin(h.origin.yaw)
    expected_y=h.origin.y+.20*math.sin(h.origin.yaw)+.10*math.cos(h.origin.yaw)
    goal=mapping.goal(h.hand)
    assert (goal.x,goal.y)==pytest.approx((expected_x,expected_y))
    h.run(3.)
    assert (h.plant.pose.x,h.plant.pose.y)==pytest.approx((expected_x,expected_y),abs=.02)


def test_acceleration_is_one_effective_combined_constraint_and_wheel_cap_is_common():
    c=SimRotationLimits(2.5,6.);previous=ZERO;dt=.02
    for desired in (Velocity(.495,.495,2.5),Velocity(-.495,-.495,-2.5),ZERO):
        for _ in range(100):
            current=coupled_shape(desired,previous,dt,c)
            assert math.hypot(current.vx,current.vy)<=.70+1e-7
            assert math.hypot(current.vx-previous.vx,current.vy-previous.vy)<=.80*dt+1e-8
            assert abs(current.wz-previous.wz)<=6.*dt+1e-8
            assert max(abs(v) for v in wheel_rpm(current))<=MAX_RPM+1e-6
            previous=current


def moving():
    h=RotationHarness(SimRotationLimits(2.,4.));h.start()
    for i in range(1,61):h.set_gesture(.001*i,.0008*i,.004*i);h.step()
    assert h.sup.state=='ACTIVE' and abs(h.output.wz)>.1
    return h


FAULTS=('wand_loss','car_loss','replay_source','feedback_loss','operator_loss','release',
    'wand_xy_jump','wand_angle_jump','car_jump','car_angle_jump','boundary','lift','tilt',
    'source_conflict','mode_conflict','command_loss','supervisor_loss','delayed_frame','command_replay',
    'yaw_overspeed','yaw_acceleration_jump','yaw_reference_jump','actuator_fault','estop',
    'session_timeout','clock_back','timer_gap')


@pytest.mark.parametrize('fault',FAULTS)
def test_full_supervisor_and_independent_gate_fail_stop_without_automatic_restart(fault):
    h=moving();start=h.now
    if fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='replay_source':h.pose_stamp['wand']=h.ros
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='operator_loss':h.lease_on=False
    elif fault=='release':h.held=False
    elif fault=='wand_xy_jump':h.hand=replace(h.hand,x=h.hand.x+.3)
    elif fault=='wand_angle_jump':h.hand=replace(h.hand,yaw=h.hand.yaw+math.pi)
    elif fault=='car_jump':h.plant.pose=replace(h.plant.pose,x=h.plant.pose.x+.3)
    elif fault=='car_angle_jump':h.plant.pose=replace(h.plant.pose,yaw=h.plant.pose.yaw+math.pi)
    elif fault=='boundary':h.plant.pose=Pose(*h.c.geometry().world(2.,2.),h.c.ground_z_m,h.origin.yaw)
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.1)
    elif fault=='tilt':h.roll=.3
    elif fault=='source_conflict':
        for inputs in (h.sup.inputs,h.gate.inputs):inputs.graph_ok=False
    elif fault=='mode_conflict':
        for inputs in (h.sup.inputs,h.gate.inputs):inputs.mode='rectangle'
    elif fault=='command_loss':h.command_link_on=False
    elif fault=='supervisor_loss':h.supervisor_on=False
    elif fault=='delayed_frame':h.modify_frame=lambda f:replace(f,issued_steady=f.issued_steady-.15,issued_ros=f.issued_ros-.15)
    elif fault=='command_replay':
        frame=h.gate.frame;h.modify_frame=lambda f:frame
    elif fault=='yaw_overspeed':h.modify_frame=lambda f:replace(f,command=replace(f.command,wz=2.01))
    elif fault=='yaw_acceleration_jump':h.modify_frame=lambda f:replace(f,command=replace(f.command,wz=f.command.wz+1.))
    elif fault=='yaw_reference_jump':h.modify_frame=lambda f:replace(f,goal=replace(f.goal,yaw=f.goal.yaw+.20)) if f.goal else f
    elif fault=='actuator_fault':h.fault_bits=0x20
    elif fault=='estop':h.sup.request_stop(h.now,'simulation estop',fault=True,hard=True)
    elif fault=='session_timeout':h.sup.active_started=h.now-h.c.max_session_s
    first_zero=None
    for i in range(20):
        output=h.step(dt=.12 if fault=='timer_gap' and i==0 else .02,
            ros_dt=-.02 if fault=='clock_back' and i==0 else None)
        if output==ZERO and first_zero is None:first_zero=h.now-start
    assert first_zero is not None and first_zero<=.24+1e-7,(fault,h.sup.reason,h.gate.fault)
    h.pose_on={'car':True,'wand':True};h.pose_stamp.clear();h.lease_on=h.feedback_on=True
    h.command_link_on=h.supervisor_on=True;h.modify_frame=lambda f:f;h.roll=0.;h.fault_bits=0;h.held=False
    for inputs in (h.sup.inputs,h.gate.inputs):inputs.graph_ok=True;inputs.mode='wand'
    h.run(1.8)
    assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert not h.sup.request_start(h.now,h.ros)[0] and not h.sup.request_recover(h.now,h.ros)[0]


def test_rotation_without_actual_progress_is_stopped_not_left_turning_until_timeout():
    h=RotationHarness(SimRotationLimits(2.,4.));h.start();h.plant.gains=(.98,.88,0.)
    for i in range(1,251):
        h.set_gesture(0.,0.,min(i*.008,1.4));h.step()
        if h.sup.state!='ACTIVE':break
    assert h.sup.state!='ACTIVE' and h.output==ZERO and 'ROTATION_NO_PROGRESS' in h.sup.reason


def test_prepare_and_pickup_do_not_capture_origins_until_first_real_engagement():
    h=RotationHarness(SimRotationLimits(2.,4.));h.run(1.8)
    assert h.sup.request_start(h.now,h.ros)[0];h.run(.1)
    for i in range(1,101):h.set_gesture(.004*i,.001*i,.015*i);assert h.step()==ZERO
    h.held=True;assert h.step()==ZERO
    assert h.sup.state=='ACTIVE' and h.sup.follower.mapping.turn==0.
    h.run(.2);assert h.output==ZERO


def test_final_gate_also_owns_120_second_budget_even_with_replayed_supervisor_timer():
    h=moving();h.gate.angle_started=h.now-120.
    assert h.step()==ZERO and h.gate.fault=='ROTATION_TIME_LIMIT_120S'


def test_final_gate_rejects_a_small_wrong_goal_even_inside_rate_and_acceleration_limits():
    h=moving()
    h.modify_frame=lambda f:replace(f,goal=replace(f.goal,yaw=f.goal.yaw+.001)) if f.goal else f
    assert h.step()==ZERO and h.gate.fault=='FINAL_REFERENCE_STEP_LIMIT'


def test_same_limits_feedforward_ablation_improves_normal_hand_tracking():
    c=SimRotationLimits(2.,4.);case=cases()[1]
    ff,_=run_case(c,case);position_only,_=run_case(c,case,feedforward=False)
    assert ff['passed'] and position_only['passed']
    assert ff['angular_rms_deg']<position_only['angular_rms_deg']*.70


@pytest.mark.parametrize('case',cases(),ids=lambda c:c.name)
def test_all_rotation_and_mixed_trajectories_keep_command_and_footprint_limits(case):
    result,_=run_case(SimRotationLimits(2.5,6.),case)
    assert result['passed'] and result['release_output_zero_immediate']
    assert result['software_feedback_stop_confirmed'] and result['max_wheel_rpm']<=MAX_RPM+1e-6


def test_offline_runtime_imports_no_ros_or_transport_and_has_no_hardware_launch_flags():
    for file in ('rotation_core.py','experiments.py','harness.py','bootstrap.py'):
        text=(Path(__file__).parent/file).read_text();tree=ast.parse(text)
        for n in ast.walk(tree):
            if isinstance(n,(ast.Import,ast.ImportFrom)):
                modules=[a.name for a in n.names] if isinstance(n,ast.Import) else [n.module or '']
                assert not any(m.split('.')[0] in ('socket','rclpy','serial','subprocess') or 'hardware' in m for m in modules)
        assert '--launch' not in text and '--hardware' not in text


def test_both_source_and_car_angles_can_wrap_through_two_complete_rotations():
    h=RotationHarness(SimRotationLimits(2.,6.));h.start();h.car_wrap=True
    for i in range(1,1001):
        h.set_gesture(0.,0.,4*math.pi*min(i/900,1.))
        h.hand=replace(h.hand,yaw=math.atan2(math.sin(h.hand.yaw),math.cos(h.hand.yaw)))
        h.step()
        assert h.sup.state=='ACTIVE' and not h.gate.fault
    h.run(3.)
    assert abs(h.sup.follower.car_continuous-h.origin.yaw-4*math.pi)<.035


def test_fast_challenge_is_not_reported_as_tracking_acceptance_just_because_it_did_not_fault():
    result,_=run_case(SimRotationLimits(2.,6.),cases()[2])
    assert result['passed'] and not result['tracking_meets_normal_threshold']
    assert result['angular_p95_deg']>15.
