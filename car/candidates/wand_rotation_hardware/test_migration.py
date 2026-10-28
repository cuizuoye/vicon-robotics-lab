"""Pure migrated cores/packet/fault regressions; no real ROS or external IO."""
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
import pytest
from collections import OrderedDict,deque
from types import SimpleNamespace as NS

OUT=Path(os.environ['ROTATION_HW_OUTPUT']);ROOT=Path(os.environ['PUBLIC_CAR_ROOT'])
sys.path.insert(0,str(OUT))
for package in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(OUT/'runtime/src'/package))
from mecanum_wand_4ws8_control.rotation_core import RotationSafetyConfig,RotationSupervisor,RotationFinalGate
from mecanum_wand_4ws8_control.rotation_servo import RotationMapping,coupled_shape,wheel_rpm
from mecanum_wand_4ws8_control.rotation_source import Packet,Raw,PacketCache,decode_packet,definition
from mecanum_wand_4ws8_control.rotation_geometry import REQUIRED,rotation
from mecanum_wand_4ws8_control.core import Pose,Velocity,RelativeMapping,wrap_angle
from mecanum_wand_4ws8_control.safety_core import Lease,ZERO
from mecanum_wand_4ws8_control.offline_suite import Plant


def config():
    return RotationSafetyConfig(**json.loads((OUT/'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])


def packet(now=10.,ros=1000.,yaw=0.,point=(1.,2.,1.2),roll=0.):
    d=definition();cr,sr=math.cos(roll/2),math.sin(roll/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    q=(sr*cy,sr*sy,cr*sy,cr*cy);r=rotation(q)
    origin=np.array(point)-r@np.array(d['local_points']['Sign5'])
    return Packet((Raw('sign',tuple(origin),q,ros,now),*
        (Raw(k,tuple(origin+r@np.array(d['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket,subprocess,rclpy
    def forbidden(*a,**kw):pytest.fail('pure regression attempted external I/O')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)
    monkeypatch.setattr(subprocess,'Popen',forbidden)


def test_actual_hardware_configuration_not_a2_software_limits():
    c=config();assert (c.stage,c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(4,.7,.8,.1,.4)
    assert (c.pose_timeout_s,c.feedback_timeout_s,c.command_timeout_s,c.lease_timeout_s)==(.2,.2,.1,.15)
    assert (c.arena_forward_min_m,c.arena_forward_max_m,c.arena_left_min_m,c.arena_left_max_m)==(-1.5,.5,-.4,.5)
    for kw in ({'max_yaw_rate_radps':2.},{'max_yaw_accel_radps2':12.},{'pose_timeout_s':.3}):
        with pytest.raises(ValueError):replace(c,**kw)


def test_sign5_reference_is_stationary_through_a_full_turn():
    p0=packet().pose(10.,1000.);angles=[]
    for yaw in np.linspace(-math.pi,math.pi,101):
        p=packet(yaw=yaw).pose(10.,1000.)
        assert (p.x,p.y,p.z)==pytest.approx((p0.x,p0.y,p0.z),abs=1e-12)
        angles.append(p.yaw)
    assert sum(wrap_angle(b-a) for a,b in zip(angles,angles[1:]))==pytest.approx(2*math.pi)


@pytest.mark.parametrize('fault',['stale','future','mixed','geometry','flip','digest','marker_nan','schema','vertical'])
def test_raw_source_must_not_be_stale_mixed_or_invented(fault):
    p=packet();v=p.payload()
    if fault=='stale':v['samples'][0]['received']=9.7
    elif fault=='future':v['samples'][0]['stamp']=1000.1
    elif fault=='mixed':v['samples'][2]['stamp']=999.999
    elif fault=='geometry':v['samples'][2]['position']=tuple(x+.02 for x in v['samples'][2]['position'])
    elif fault=='flip':v['samples'][0]['quaternion']=(0.,0.,1.,0.)
    elif fault=='digest':v['digest']='0'*64
    elif fault=='marker_nan':v['samples'][2]['position']=(math.nan,0.,0.)
    elif fault=='schema':v['ignored_extra']='forbidden'
    else:v=packet(roll=math.pi/2).payload()
    with pytest.raises((ValueError,TypeError)):decode_packet(v,10.,1000.)


def test_replay_cannot_refresh_receipt_and_gate_selects_exact_packet():
    c=PacketCache();p=packet();assert c.add(p,10.,1000.)
    assert not c.add(p,10.1,1000.1)
    assert c.exact(p.digest,p.stamp,10.1,1000.1)==p
    assert c.exact('0'*64,p.stamp,10.1,1000.1) is None
    with pytest.raises(ValueError):c.exact(p.digest,p.stamp-.01,10.1,1000.1)
    with pytest.raises(ValueError):c.latest(10.21,1000.21)


def test_mapping_serializes_and_uses_start_frame_not_current_rotation():
    m=RotationMapping(RelativeMapping(Pose(1.,2.,.1,.7),Pose(3.,4.,1.,.2),1.))
    m.turn=math.pi/2;g=m.goal(Pose(3.2,4.1,2.,1.7))
    assert (g.x,g.y,g.z,g.yaw)==pytest.approx((1.2,2.1,.1,.7+math.pi/2))
    assert asdict(m)['turn']==math.pi/2


def test_combined_shaping_honors_xy_yaw_and_wheel_envelope():
    c=config();last=ZERO
    for wanted in (Velocity(.7,.7,.1),Velocity(-.7,-.7,-.1),ZERO):
        for _ in range(100):
            v=coupled_shape(wanted,last,.02,c)
            assert math.hypot(v.vx,v.vy)<=.7+1e-8 and abs(v.wz)<=.1+1e-8
            assert math.hypot(v.vx-last.vx,v.vy-last.vy)<=.8*.02+1e-8
            assert abs(v.wz-last.wz)<=.4*.02+1e-8
            assert max(map(abs,wheel_rpm(v)))<=400.+1e-8;last=v


class Harness:
    def __init__(self):
        self.c=config();self.sup=RotationSupervisor(self.c);self.gate=RotationFinalGate(self.c)
        self.origin=Pose(*self.c.geometry().world(-.5,.05),self.c.ground_z_m,self.c.arena_heading_rad)
        self.hand=Pose(1.,2.,1.1,.7);self.plant=Plant(self.origin)
        self.now=10.;self.ros=1000.;self.seq=0;self.output=ZERO;self.held=False
        self.pose_on={'car':True,'wand':True};self.feedback_on=self.lease_on=self.command_on=True
        self.roll=self.pitch=0.;self.fault_bits=0;self.modify=lambda f:f
    def step(self):
        self.now+=.02;self.ros+=.02;self.seq+=1
        for inputs in (self.sup.inputs,self.gate.inputs):
            if self.pose_on['car']:
                p=replace(self.plant.pose,yaw=wrap_angle(self.plant.pose.yaw))
                inputs.observe_pose('car',p,self.ros,self.now,self.ros,self.roll,self.pitch)
            if self.pose_on['wand']:inputs.observe_pose('wand',self.hand,self.ros,self.now,self.ros)
            if self.feedback_on:inputs.observe_feedback(self.seq,self.now,self.ros,self.now,self.ros,
                output_zero=self.output==ZERO,enabled=self.output!=ZERO,
                speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz),fault_bits=self.fault_bits)
            if self.lease_on:inputs.observe_lease(Lease('pure-test',self.seq,self.now,self.ros,self.held),self.now,self.ros)
        frame=self.sup.tick(self.now,self.ros)
        if self.command_on:self.gate.receive(self.modify(frame),self.now,self.ros)
        self.output=self.gate.tick(self.now,self.ros)
        self.sup.observe_gate(steady=self.now,ros=self.ros,epoch=self.gate.primed_epoch,
            fault=self.gate.fault,hard_fault=self.gate.hard_fault)
        self.plant.step(self.output,.02)
        return frame
    def run(self,seconds):
        for _ in range(round(seconds/.02)):self.step()
    def start(self,pick_up=False):
        self.run(1.8);ok,why=self.sup.request_start(self.now,self.ros);assert ok,why
        self.run(.2)
        if pick_up:
            for _ in range(100):
                self.hand=replace(self.hand,x=self.hand.x+.002,yaw=wrap_angle(self.hand.yaw+.01),z=self.hand.z+.002)
                self.step()
            self.run(.3)
        assert self.sup.state=='ARMED'
        self.engage_hand=self.hand;self.held=True;f=self.step()
        assert self.sup.state=='ACTIVE' and f.command==ZERO and self.output==ZERO,(self.sup.reason,self.gate.fault)


def test_prepare_then_pickup_and_rotate_before_enable_captures_neutral_origins():
    h=Harness();h.start(pick_up=True)
    assert h.sup.follower.mapping.wand_origin==h.engage_hand
    assert h.gate.pilot.wand==h.engage_hand
    h.run(.2);assert h.output==ZERO and not h.gate.fault


@pytest.mark.parametrize('direction',[1.,-1.])
def test_90_degree_rotation_return_and_xy_superposition(direction):
    h=Harness();h.start();hand=h.hand
    # Normal hand speed is purposely faster than existing MCU rotation cap.
    # The governed target leads by <=.50rad; it must catch up without latching.
    for i in range(1,101):
        h.hand=replace(hand,x=hand.x+.15*i/100,y=hand.y+.08*i/100,
            yaw=wrap_angle(hand.yaw+direction*math.pi/2*i/100));h.step()
    h.run(25.)
    assert h.sup.state=='ACTIVE' and not h.gate.fault,(h.sup.reason,h.gate.fault)
    assert abs(h.sup.follower.car_continuous-h.origin.yaw-direction*math.pi/2)<.04
    assert (h.plant.pose.x,h.plant.pose.y)==pytest.approx((h.origin.x+.15,h.origin.y+.08),abs=.02)
    for i in range(1,101):
        h.hand=replace(hand,x=hand.x+.15*(1-i/100),y=hand.y+.08*(1-i/100),
            yaw=wrap_angle(hand.yaw+direction*math.pi/2*(1-i/100)));h.step()
    h.run(25.)
    assert h.sup.state=='ACTIVE' and not h.gate.fault,(h.sup.reason,h.gate.fault)
    assert abs(h.sup.follower.car_continuous-h.origin.yaw)<.04
    assert (h.plant.pose.x,h.plant.pose.y)==pytest.approx((h.origin.x,h.origin.y),abs=.02)
    h.held=False;h.run(1.);assert h.output==ZERO and h.sup.stop_confirmed
    assert not h.sup.request_start(h.now,h.ros)[0] # one-shot pilot, no auto-retry.


@pytest.mark.parametrize('yaw',np.linspace(-math.pi,math.pi,17))
def test_reference_projection_checks_rotated_footprint(yaw):
    c=config();g=c.geometry();origin=Pose(*g.world(-.5,.05),c.ground_z_m,c.arena_heading_rad)
    for xy in ((-3.,-2.),(3.,2.),(-1.5,-.4),(.5,.5)):
        p=Pose(*g.world(*xy),origin.z,float(yaw))
        projected=c.project_reference(p,origin,4.75)
        assert c.inside(projected) and projected.yaw==yaw


@pytest.mark.parametrize('fault',['wand_loss','car_loss','feedback_loss','lease_loss','release','yaw_jump',
    'lift','tilt','mode','source','actuator','reference_spoof','overspeed','accel','command_loss','estop','timeout'])
def test_faults_fail_stop_and_never_resume_on_input_return(fault):
    h=Harness();h.start()
    for i in range(1,41):h.hand=replace(h.hand,x=h.hand.x+.0005,yaw=wrap_angle(h.hand.yaw+.001));h.step()
    assert h.sup.state=='ACTIVE'
    if fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='lease_loss':h.lease_on=False
    elif fault=='release':h.held=False
    elif fault=='yaw_jump':h.hand=replace(h.hand,yaw=wrap_angle(h.hand.yaw+math.pi))
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.1)
    elif fault=='tilt':h.roll=.3
    elif fault=='mode':
        for v in (h.sup.inputs,h.gate.inputs):v.mode='rectangle'
    elif fault=='source':
        for v in (h.sup.inputs,h.gate.inputs):v.graph_ok=False
    elif fault=='actuator':h.fault_bits=0x20
    elif fault=='reference_spoof':h.modify=lambda f:replace(f,goal=replace(f.goal,yaw=f.goal.yaw+.001)) if f.goal else f
    elif fault=='overspeed':h.modify=lambda f:replace(f,command=replace(f.command,wz=.11))
    elif fault=='accel':h.modify=lambda f:replace(f,command=replace(f.command,wz=-.1))
    elif fault=='command_loss':h.command_on=False
    elif fault=='estop':h.gate.trip('ESTOP',hard=True)
    elif fault=='timeout':h.sup.active_started=h.now-120.
    h.run(.4)
    assert h.output==ZERO and h.sup.state!='ACTIVE',(fault,h.sup.state,h.gate.fault)
    h.pose_on={'car':True,'wand':True};h.feedback_on=h.lease_on=h.command_on=True;h.modify=lambda f:f;h.held=False
    h.run(.6)
    assert h.output==ZERO and not h.sup.request_start(h.now,h.ros)[0]


def test_old_translation_review_cannot_approve_new_rotation_scope():
    import importlib.util
    spec=importlib.util.spec_from_file_location('rotation_app',OUT/'oneclick_app.py')
    app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)
    m=json.loads((OUT/'build_manifest.json').read_text());review=app.make_review(m)
    from mecanum_wand_4ws8_hardware.authorization import verify_trial_review
    assert verify_trial_review(review,4,'sign',m['firmware']['sha256'])
    for key,value in [('kind','BOUNDED_SPACE_TOGGLE_120S_NOT_QUALIFICATION'),
        ('rotation_physical_qualified',True),('rotation_request_sha256','0'*64),('rotation_mapping_scale',2.)]:
        with pytest.raises(ValueError):verify_trial_review(dict(review,**{key:value}),4,'sign',m['firmware']['sha256'])


def gate_adapter_fixture():
    from mecanum_wand_4ws8_hardware.rotation_nodes import RotationGateNode
    h=Harness();h.start();published=[]
    value=NS(core=h.gate,packet_cache=PacketCache(),cars=OrderedDict(),pending=deque(),
        clocks=lambda:(h.now,h.ros),publish_output=published.append)
    return h,value,published,RotationGateNode


def test_missing_source_frame_cannot_be_overwritten_by_later_good_frame():
    from std_msgs.msg import String
    h,g,published,cls=gate_adapter_fixture();f=h.sup.frame(h.now,h.ros)
    p=packet(h.now,h.ros);g.packet_cache.add(p,h.now,h.ros)
    g.cars[h.ros]=(h.plant.pose,h.ros,h.now,0.,0.)
    def msg(frame,digest):return String(data=json.dumps(dict(frame=asdict(frame),sign_digest=digest,
        sign_stamp=p.stamp,car_stamp=h.ros)))
    cls.on_candidate(g,msg(f,'0'*64));cls.on_candidate(g,msg(replace(f,sequence=f.sequence+1),p.digest))
    assert len(g.pending)==2 and g.pending[0][0]['sign_digest']=='0'*64
    h.now+=.041;h.ros+=.041;cls.process_pending(g)
    assert not g.pending and g.core.fault=='ROTATION_SELECTED_SOURCE_INVALID' and published[-1]==ZERO


def test_stop_bypasses_pending_source_and_publishes_zero_immediately():
    from std_msgs.msg import String
    h,g,published,cls=gate_adapter_fixture();f=h.sup.frame(h.now,h.ros)
    def msg(frame):return String(data=json.dumps(dict(frame=asdict(frame),sign_digest='0'*64,
        sign_stamp=h.ros,car_stamp=h.ros)))
    cls.on_candidate(g,msg(f));assert g.pending
    stop=replace(f,sequence=f.sequence+1,enabled=False,state='STOPPING',command=ZERO,goal=None,epoch='')
    cls.on_candidate(g,msg(stop))
    assert not g.pending and published[-1]==ZERO and g.core.frame.state=='STOPPING'
