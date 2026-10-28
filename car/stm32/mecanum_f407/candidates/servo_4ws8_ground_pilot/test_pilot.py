"""Virtual-clock fault injection; no ROS init, network, programmer or device."""
from dataclasses import replace
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import pytest

GENERATED = Path(os.environ['STAGED_GENERATED'])
ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(GENERATED/name))
from mecanum_wand_4ws8_control.pilot_core import PilotMonitor, PilotSupervisor, PilotFinalGate, POLICY
from mecanum_wand_4ws8_control.core import Pose, Velocity
from mecanum_wand_4ws8_control.safety_core import Inputs, ZERO
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
from mecanum_wand_4ws8_hardware.authorization import verify_trial_review
from mecanum_wand_4ws8_hardware.pilot_authorization import KIND

spec = importlib.util.spec_from_file_location('mecanum_wand_4ws8_control.pilot_harness',
    ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests/offline_harness.py')
harness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(harness)
harness.Supervisor, harness.FinalGate = PilotSupervisor, PilotFinalGate


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    import socket
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('offline: socket forbidden'))


def config(stage=4):
    p=GENERATED/'mecanum_wand_4ws8_hardware/config'/f'stage{stage}.NOT_APPROVED.json'
    return StagedSafetyConfig(**json.loads(p.read_text())['safety'])


def live():
    h=harness.Harness(config(),origin_local=(0,0))
    h.start()
    for i in range(1,16):
        h.set_hand(.003*i,0.)
        h.step()
    assert h.output != ZERO,(h.sup.reason,h.gate.fault)
    return h


def test_small_step_is_accepted_then_two_seconds_revoke_and_consumes():
    h=live()
    h.run(2.0)
    assert h.output==ZERO and h.sup.pilot.used and h.gate.pilot.used
    assert 'PILOT_TIME_LIMIT' in h.sup.reason or 'PILOT_TIME_LIMIT' in h.gate.fault
    h.held=False;h.run(2.)
    assert not h.sup.request_start(h.now,h.ros)[0]
    assert not h.sup.request_recover(h.now,h.ros)[0]
    h.held=True;h.run(.5)
    assert h.output==ZERO


@pytest.mark.parametrize('fault',['release','wand_loss','car_loss','feedback_loss','lease_loss',
    'actuator_fault','estop','lift','tilt','clock','supervisor_loss','gate_loss',
    'spoofed_owner','overspeed','acceleration_jump','reference_jump','mode_conflict'])
def test_original_protections_still_stop(fault):
    h=live()
    if fault=='release':h.held=False
    elif fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='lease_loss':h.lease_on=False
    elif fault=='actuator_fault':h.fault_bits=0x20
    elif fault=='estop':h.sup.request_stop(h.now,'ESTOP',fault=True,hard=True)
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.09)
    elif fault=='tilt':h.roll=.3
    elif fault=='supervisor_loss':h.supervisor_on=False
    elif fault=='gate_loss':h.gate_on=False
    elif fault=='spoofed_owner':h.modify_frame=lambda f:replace(f,source='foreign')
    elif fault=='overspeed':h.modify_frame=lambda f:replace(f,command=Velocity(.71))
    elif fault=='acceleration_jump':h.modify_frame=lambda f:replace(f,command=Velocity(.65))
    elif fault=='reference_jump':h.modify_frame=lambda f:replace(f,goal=replace(f.goal,x=f.goal.x+.1)) if f.goal else f
    elif fault=='mode_conflict':
        for i in (h.sup.inputs,h.gate.inputs):i.mode='rectangle'
    for i in range(20):
        h.step(ros_dt=-.02 if fault=='clock' and not i else None)
    assert h.plant_target==ZERO,(fault,h.sup.reason,h.gate.fault)
    h.pose_on={'car':True,'wand':True};h.feedback_on=h.lease_on=True
    h.run(.5)
    assert h.plant_target==ZERO


@pytest.mark.parametrize('which',['supervisor','gate'])
@pytest.mark.parametrize('fault',['hand','goal','car','path','lateral','reverse','heading','time','nan','epoch'])
def test_independent_bounds_even_if_other_controller_compromised(which,fault):
    h=live()
    core=h.sup if which=='supervisor' else h.gate
    m=core.pilot
    # Isolate added monitor from base jump/tilt limits, which would also stop.
    poses={k:dict(v) for k,v in core.inputs.poses.items()}
    inputs=Inputs(config());inputs.poses=poses
    car=m.car;goal=car;stamp=m.started+.1;epoch=m.epoch
    if fault=='hand':inputs.poses['wand']['pose']=replace(m.wand,x=m.wand.x+.151)
    elif fault=='goal':goal=replace(car,x=car.x+.151)
    elif fault=='car':inputs.poses['car']['pose']=replace(car,x=car.x+.201)
    elif fault=='path':m.path=.251
    elif fault=='lateral':inputs.poses['car']['pose']=replace(car,x=car.x-.101*math.sin(car.yaw),y=car.y+.101*math.cos(car.yaw))
    elif fault=='reverse':inputs.poses['car']['pose']=replace(car,x=car.x-.041*math.cos(car.yaw),y=car.y-.041*math.sin(car.yaw))
    elif fault=='heading':inputs.poses['car']['pose']=replace(car,yaw=car.yaw+.201)
    elif fault=='time':stamp=m.started+2.001
    elif fault=='nan':goal=replace(car,x=math.nan)
    elif fault=='epoch':epoch='unauthorized-second-epoch'
    assert m.active(epoch,inputs,goal,stamp).startswith('PILOT_')


def test_gate_independently_revokes_signed_permission_if_supervisor_ignores_timeout():
    h=live()
    h.sup.pilot.started=h.now+100  # compromised supervisor monitor, not gate
    # Instead suppress the supervisor add-on altogether, keeping its original
    # algorithm/heartbeats active, then expire only the independent final gate.
    h.sup.pilot.active=lambda *a:''
    h.gate.pilot.started=h.now-2.01
    assert h.step()==ZERO
    assert h.gate.primed_epoch=='' and h.gate.fault=='PILOT_TIME_LIMIT_2S'


def test_release_blocks_second_enable_even_with_replaced_supervisor():
    h=live();h.held=False;h.run(2.)
    assert h.output==ZERO
    assert h.gate.pilot.arm('new-epoch',h.gate.inputs).startswith('PILOT_CONSUMED')


@pytest.mark.parametrize('stage',[0,1,2,3,5,6])
def test_other_stages_rejected_by_both_wrappers(stage):
    for cls in (PilotSupervisor,PilotFinalGate):
        with pytest.raises(ValueError):cls(config(stage))


def actual_review():
    p=ROOT/'docs/wand_servo_4ws8_2026-10-23/requested_suspended_050_070/host_ready/stage4_suspended_l79694pa/bench/SESSION_RESULT.json'
    u=Path(__file__).with_name('USER_AUTHORIZATION.json')
    sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    return dict(kind=KIND,identity='4WS8',authorized_stage=4,purpose='sign',human_authorized=True,
        image_sha256=json.loads(p.read_text())['image_sha256'],pilot_policy=POLICY,
        continuous_ground_qualified=False,electrical_thermal_trial_reviewed=False,
        suspended_evidence=str(p),suspended_evidence_sha256=sha(p),
        user_authorization=str(u),user_authorization_sha256=sha(u))


def test_real_evidence_is_accepted_only_for_finite_pilot():
    r=actual_review()
    assert verify_trial_review(r,4,'sign',r['image_sha256'])
    for stage,purpose in ((3,'sign'),(6,'sign'),(4,'suspended')):
        with pytest.raises(ValueError):verify_trial_review(r,stage,purpose,r['image_sha256'])
    # Normal ground reviews still cannot skip measurements or stages.
    r.update(kind='BOUNDED_TRIAL_NOT_QUALIFICATION')
    with pytest.raises(ValueError):verify_trial_review(r,4,'sign',r['image_sha256'])


@pytest.mark.parametrize('field,value', [('human_authorized',False),('continuous_ground_qualified',True),
    ('electrical_thermal_trial_reviewed',True),('suspended_evidence_sha256','bad'),
    ('user_authorization_sha256','bad'),('pilot_policy',dict(POLICY,max_active_s=120))])
def test_review_drift_and_unearned_qualification_rejected(field,value):
    r=actual_review();r[field]=value
    with pytest.raises(ValueError):verify_trial_review(r,4,'sign',r['image_sha256'])


def test_ros_nodes_use_wrappers_and_original_core_is_byte_identical():
    from mecanum_wand_4ws8_control import safety_nodes
    assert safety_nodes.Supervisor is PilotSupervisor and safety_nodes.FinalGate is PilotFinalGate
    path='mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/safety_core.py'
    assert (GENERATED/path).read_bytes()==(ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/runtime/src'/path).read_bytes()
