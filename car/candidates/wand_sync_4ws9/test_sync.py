"""Offline 4WS9 policy, real C wire, coupled control and fault regressions."""
import ctypes
from dataclasses import asdict,replace
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['SYNC_OUTPUT'])
for name in ('mecanum_wand_4ws9_control','mecanum_wand_4ws9_hardware'):
    sys.path.insert(0,str(OUT/'runtime/src'/name))
from mecanum_wand_4ws9_control.core import Pose,Velocity,wrap_angle
from mecanum_wand_4ws9_control.rotation_core import RotationSafetyConfig,RotationSupervisor,RotationFinalGate
from mecanum_wand_4ws9_control.rotation_servo import coupled_shape,wheel_rpm
from mecanum_wand_4ws9_control.offline_suite import Plant
from mecanum_wand_4ws9_control.safety_core import Lease,ZERO
from mecanum_wand_4ws9_hardware.stages import STAGES
from mecanum_wand_4ws9_hardware.stage0_session import Stage0Session
from mecanum_wand_4ws9_hardware.stage0_contract import decode_status,SIZE
from mecanum_wand_4ws9_hardware.commissioning import AuthoritySigner,AuthorityVerifier
from mecanum_vicon_control.transport_protocol import MessageType,FrameFlags,encode_frame,decode_frame,ProtocolError

def config(stage=6):
    p=OUT/f'runtime/src/mecanum_wand_4ws9_hardware/config/stage{stage}.NOT_APPROVED.json'
    return RotationSafetyConfig(**json.loads(p.read_text())['safety'])

@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,subprocess,rclpy
    def denied(*a,**kw):pytest.fail('Offline test attempted socket/ROS/device process startup')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(rclpy,'init',denied)
    monkeypatch.setattr(subprocess,'Popen',denied)

@pytest.mark.parametrize('stage',range(7))
def test_exact_tier_and_preserved_safety(stage):
    c=config(stage);s=STAGES[stage]
    assert (c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(s.speed,s.accel,s.yaw_rate,s.yaw_accel)
    assert c.stage==stage and c.follower_config().max_wheel_rpm==s.target_rpm
    assert (c.pose_timeout_s,c.feedback_timeout_s,c.command_timeout_s,c.lease_timeout_s,c.max_session_s)==(.2,.2,.1,.15,120.)
    assert (c.arena_forward_min_m,c.arena_forward_max_m,c.arena_left_min_m,c.arena_left_max_m,c.arena_buffer_m)==(-1.5,.5,-.4,.5,.5)
    for kw in ({'max_yaw_rate_radps':s.yaw_rate+.01},{'max_yaw_accel_radps2':s.yaw_accel+.01},
            {'pose_timeout_s':.3},{'tilt_limit_rad':.4},{'boundary_margin_m':.1}):
        with pytest.raises(ValueError):replace(c,**kw)

@pytest.mark.parametrize('stage',range(7))
def test_shared_xy_yaw_ramp_and_wheel_caps(stage):
    c=config(stage);last=ZERO
    for desired in (Velocity(.7,.7,2.),Velocity(-.7,.7,-2.),Velocity(.7,-.7,2.),ZERO):
        for _ in range(100):
            v=coupled_shape(desired,last,.02,c.follower_config())
            assert math.hypot(v.vx,v.vy)<=c.max_speed_mps+1e-7
            assert abs(v.wz)<=c.max_yaw_rate_radps+1e-7
            assert math.hypot(v.vx-last.vx,v.vy-last.vy)<=c.max_accel_mps2*.02+1e-7
            assert abs(v.wz-last.wz)<=c.max_yaw_accel_radps2*.02+1e-7
            assert max(map(abs,wheel_rpm(v)))<=c.limits.target_rpm+1e-7;last=v

class Oracle:
    def __init__(self):
        self.lib=ctypes.CDLL(str(OUT/'native/libstage0_oracle.so'))
        self.lib.oracle_init.argtypes=[];self.lib.oracle_tick.argtypes=[ctypes.c_uint32]
        self.lib.oracle_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.lib.oracle_request.restype=ctypes.c_size_t
        self.lib.oracle_packet.argtypes=[ctypes.c_uint,ctypes.c_void_p];self.lib.oracle_packet.restype=ctypes.c_size_t
        self.lib.oracle_init();self.now=0
    def request(self,raw):
        src=ctypes.create_string_buffer(raw);dest=ctypes.create_string_buffer(512)
        n=self.lib.oracle_request(src,len(raw),self.now,dest);return dest.raw[:n]
    def packet(self):
        buf=ctypes.create_string_buffer(512);n=self.lib.oracle_packet(int(MessageType.LINK_STATUS),buf)
        return decode_frame(buf.raw[:n])
    def tick(self):self.now+=10;self.lib.oracle_tick(self.now)

def arm(stage):
    o=Oracle();s=Stage0Session(lambda:1234,stage=stage)
    s.accept(o.request(s.new_hello(o.now)))
    for _ in range(60):
        o.request(s.cmd_vel(0,0,0,600,False,o.now));o.tick()
    o.request(s.select_mode(o.now));o.request(s.control_enable(True,False,o.now))
    assert decode_status(o.packet().payload)['permitted'];return o,s

@pytest.mark.parametrize('stage',range(7))
def test_actual_c_mcu_frames_decode_with_matching_angular_limits(stage):
    o,s=arm(stage);limits=STAGES[stage]
    for _ in range(170):
        o.request(s.cmd_vel(0.,0.,limits.yaw_rate,600,True,o.now));o.tick()
    v=decode_status(o.packet().payload)
    assert SIZE==200 and v['tag']=='4WS9' and v['output_authority_version']==5 and v['stage']==stage
    assert v['yaw_rate_limit']==pytest.approx(limits.yaw_rate) and v['yaw_accel_limit']==pytest.approx(limits.yaw_accel)
    assert v['body_si'][2]==pytest.approx(limits.yaw_rate,abs=1e-5)
    for offset in (0,153,184,188,192,196):
        raw=bytearray(o.packet().payload);raw[offset]^=0x40
        with pytest.raises((ValueError,struct.error)):decode_status(bytes(raw))
    with pytest.raises(ValueError):decode_status(o.packet().payload[:184])

@pytest.mark.parametrize('stage',range(7))
def test_host_rejects_out_of_tier_commands_and_old_selection_words(stage):
    o,s=arm(stage);limit=STAGES[stage]
    with pytest.raises(ProtocolError):s.cmd_vel(0,0,limit.yaw_rate+.01,600,True,o.now)
    with pytest.raises(ProtocolError):s.cmd_vel(limit.speed+.01,0,0,600,True,o.now)
    for enabled in (False,True):
        raw=s.control_enable(enabled,False,o.now)
        e=struct.unpack('<BBH',decode_frame(raw).payload)
        assert e[2]==(0x7900+stage if enabled else 0)

def test_signed_authority_is_version_tier_and_owner_bound():
    key='ab'*32;signer=AuthoritySigner(key,6);verify=AuthorityVerifier(key,6)
    verify.decode(signer.encode(ZERO,10.,1000.),10.,1000.)
    for w in (1.5,-2.):
        p,c=verify.decode(signer.encode(Velocity(.2,.1,w),10.,1000.,epoch='new',state='ACTIVE',permit=True),10.,1000.)
        assert c.wz==w and p['scope']=='HARDWARE_4WS9_V5_XY_YAW_STAGE_6'
    with pytest.raises(ValueError):AuthorityVerifier(key,0).decode(signer.encode(ZERO,10.,1000.),10.,1000.)
    from mecanum_wand_4ws9_hardware.authorization import verify_trial_review,verify_flash_record
    with pytest.raises(ValueError):verify_trial_review({'identity':'4WS8','human_authorized':True},0,'suspended','0'*64)
    with pytest.raises(ValueError):verify_flash_record(None,None)

@pytest.mark.parametrize('endpoint,interface',[('8.8.8.8','wlx000000000000'),('127.0.0.1','lo'),
    ('0.0.0.0','wlx000000000000'),('192.168.50.1',''),('192.168.50.1','bad/interface')])
def test_new_trial_entry_does_not_relax_endpoint_or_interface_guards(endpoint,interface):
    from mecanum_wand_4ws9_hardware.authorization import verify_runtime_profile
    p=json.loads((OUT/'runtime/src/mecanum_wand_4ws9_hardware/config/stage0.NOT_APPROVED.json').read_text())
    p['hardware'].update(esp_ip=endpoint,network_interface=interface)
    with pytest.raises(ValueError,match='private endpoint'):verify_runtime_profile(p,'suspended')

@pytest.mark.parametrize('stage,x,y,w',[(2,.353,.353,.6),(6,.494,.494,2.)])
def test_real_c_applies_one_common_scale_at_mixed_motion_wheel_saturation(stage,x,y,w):
    o,s=arm(stage)
    for _ in range(200):o.request(s.cmd_vel(x,y,w,600,True,o.now));o.tick()
    d=decode_status(o.packet().payload);a,b,c=d['body_si']
    assert d['permitted'] and 0<a<x and 0<b<y and 0<c<w
    assert a/x==pytest.approx(b/y,rel=1e-5) and a/x==pytest.approx(c/w,rel=1e-5)

def test_start_threshold_is_preserved_but_new_first_tier_can_start_pure_rotation():
    o,s=arm(0)
    for _ in range(100):o.request(s.cmd_vel(0,0,.10,600,True,o.now));o.tick()
    d=decode_status(o.packet().payload);assert d['permitted'] and not d['started_ms'] and not any(d['target_cps'])
    for _ in range(100):o.request(s.cmd_vel(0,0,.20,600,True,o.now));o.tick()
    d=decode_status(o.packet().payload);assert d['permitted'] and d['started_ms'] and d['body_si'][2]>.19

class Harness:
    def __init__(self,stage=6):
        self.c=config(stage);self.sup=RotationSupervisor(self.c);self.gate=RotationFinalGate(self.c)
        self.origin=Pose(*self.c.geometry().world(-.5,.05),self.c.ground_z_m,self.c.arena_heading_rad)
        self.hand=Pose(1.,2.,1.1,.7);self.plant=Plant(self.origin)
        self.now=10.;self.ros=1000.;self.seq=0;self.output=ZERO;self.held=False
        self.pose_on={'car':True,'wand':True};self.feedback_on=self.lease_on=self.command_on=True
        self.roll=self.pitch=0.;self.fault_bits=0;self.modify=lambda f:f
    def step(self):
        self.now+=.02;self.ros+=.02;self.seq+=1
        for inputs in (self.sup.inputs,self.gate.inputs):
            if self.pose_on['car']:inputs.observe_pose('car',replace(self.plant.pose,yaw=wrap_angle(self.plant.pose.yaw)),self.ros,self.now,self.ros,self.roll,self.pitch)
            if self.pose_on['wand']:inputs.observe_pose('wand',self.hand,self.ros,self.now,self.ros)
            if self.feedback_on:inputs.observe_feedback(self.seq,self.now,self.ros,self.now,self.ros,
                output_zero=self.output==ZERO,enabled=self.output!=ZERO,speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz),fault_bits=self.fault_bits)
            if self.lease_on:inputs.observe_lease(Lease('offline-click',self.seq,self.now,self.ros,self.held),self.now,self.ros)
        f=self.sup.tick(self.now,self.ros)
        if self.command_on:self.gate.receive(self.modify(f),self.now,self.ros)
        self.output=self.gate.tick(self.now,self.ros)
        self.sup.observe_gate(steady=self.now,ros=self.ros,epoch=self.gate.primed_epoch,fault=self.gate.fault,hard_fault=self.gate.hard_fault)
        self.plant.step(self.output,.02);return f
    def run(self,t):
        for _ in range(round(t/.02)):self.step()
    def start(self):
        self.run(1.8);ok,reason=self.sup.request_start(self.now,self.ros);assert ok,reason
        self.run(.2);self.held=True;f=self.step();assert f.command==ZERO and self.sup.state=='ACTIVE'

@pytest.mark.parametrize('sign',[-1.,1.])
def test_90_degree_rotation_combined_xy_and_return(sign):
    h=Harness();h.start();hand=h.hand
    for i in range(1,101):
        h.hand=replace(hand,x=hand.x+.2*i/100,yaw=wrap_angle(hand.yaw+sign*math.pi/2*i/100));h.step()
    h.run(2.)
    assert h.sup.state=='ACTIVE' and not h.gate.fault,(h.sup.reason,h.gate.fault)
    assert abs(h.sup.follower.car_continuous-h.origin.yaw-sign*math.pi/2)<.04
    for i in range(1,101):
        h.hand=replace(hand,x=hand.x+.2*(1-i/100),yaw=wrap_angle(hand.yaw+sign*math.pi/2*(1-i/100)));h.step()
    h.run(2.);assert h.sup.state=='ACTIVE' and not h.gate.fault
    assert abs(h.sup.follower.car_continuous-h.origin.yaw)<.04
    assert (h.plant.pose.x,h.plant.pose.y)==pytest.approx((h.origin.x,h.origin.y),abs=.02)

@pytest.mark.parametrize('fault',['wand_loss','car_loss','feedback_loss','lease_loss','release','angle_flip','lift','tilt',
    'source','mode','actuator','command_loss','overspeed','reference_spoof','estop','session_limit'])
def test_faults_revoke_output_without_automatic_resume(fault):
    h=Harness();h.start()
    for _ in range(30):h.hand=replace(h.hand,x=h.hand.x+.001,yaw=wrap_angle(h.hand.yaw+.005));h.step()
    assert abs(h.output.wz)>.1
    if fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='lease_loss':h.lease_on=False
    elif fault=='release':h.held=False
    elif fault=='angle_flip':h.hand=replace(h.hand,yaw=wrap_angle(h.hand.yaw+math.pi))
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.1)
    elif fault=='tilt':h.roll=.3
    elif fault=='source':
        for i in (h.sup.inputs,h.gate.inputs):i.graph_ok=False
    elif fault=='mode':
        for i in (h.sup.inputs,h.gate.inputs):i.mode='rectangle'
    elif fault=='actuator':h.fault_bits=0x20
    elif fault=='command_loss':h.command_on=False
    elif fault=='overspeed':h.modify=lambda f:replace(f,command=replace(f.command,wz=2.01))
    elif fault=='reference_spoof':h.modify=lambda f:replace(f,goal=replace(f.goal,yaw=f.goal.yaw+.001)) if f.goal else f
    elif fault=='estop':h.gate.trip('ESTOP',hard=True)
    else:h.sup.active_started=h.now-120.
    h.run(.4);assert h.output==ZERO and h.sup.state!='ACTIVE',(fault,h.sup.state,h.gate.fault)
    h.pose_on={'car':True,'wand':True};h.feedback_on=h.lease_on=h.command_on=True;h.modify=lambda f:f;h.held=False
    h.run(.6);assert h.output==ZERO and not h.sup.request_start(h.now,h.ros)[0]

def comparison():
    # Import the preserved low-speed host in its ORIGINAL package namespace.
    # No source changed and no synthetic time-alignment or model-lag tuning.
    os.environ['ROTATION_HW_OUTPUT']=str(ROOT/'docs/wand_rotation_hardware_2026-10-26/prepared')
    p=ROOT/'candidates/wand_rotation_hardware/test_migration.py'
    spec=importlib.util.spec_from_file_location('low_rotation_reference',p);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    results=[]
    for label,h in (('existing_4ws8_yaw010',old.Harness()),('candidate_4ws9_tier6',Harness())):
        h.start();initial=h.hand;xy=[];yaw=[];peak=0.
        for i in range(1600):
            t=i*.02;dx=.15*math.sin(1.4*t);dy=.08*math.sin(.7*t);angle=.65*math.sin(1.6*t)
            h.hand=replace(initial,x=initial.x+dx,y=initial.y+dy,yaw=wrap_angle(initial.yaw+angle));h.step()
            assert h.sup.state=='ACTIVE' and not h.gate.fault,(label,h.sup.reason,h.gate.fault)
            xy.append(math.hypot(h.plant.pose.x-(h.origin.x+dx),h.plant.pose.y-(h.origin.y+dy)))
            yaw.append(h.sup.follower.car_continuous-h.origin.yaw-angle);peak=max(peak,abs(h.output.wz))
        results.append(dict(label=label,xy_rms_m=math.sqrt(sum(v*v for v in xy)/len(xy)),
            yaw_rms_deg=math.degrees(math.sqrt(sum(v*v for v in yaw)/len(yaw))),max_command_yaw_radps=peak))
    return dict(software_only=True,same_time_errors=True,time_alignment_applied=False,
        identical_default_plant=True,results=results,
        yaw_rms_improvement_percent=100*(1-results[1]['yaw_rms_deg']/results[0]['yaw_rms_deg']))

def test_high_sync_improves_same_time_rotation_error_without_changing_plant():
    r=comparison();assert r['yaw_rms_improvement_percent']>50.,r

def test_source_binding_and_console_stop_implementations_were_not_relaxed():
    old=ROOT/'docs/wand_rotation_hardware_2026-10-26/prepared/runtime/src'
    for package,name in (('control','rotation_source.py'),('hardware','rotation_nodes.py'),('control','auto_prepare.py'),
                         ('control','toggle_keys.py'),('control','toggle_ui.py'),('control','pose_quality.py')):
        p=old/f'mecanum_wand_4ws8_{package}'/f'mecanum_wand_4ws8_{package}'/name
        q=OUT/f'runtime/src/mecanum_wand_4ws9_{package}'/f'mecanum_wand_4ws9_{package}'/name
        expected=p.read_text().replace('mecanum_wand_4ws8','mecanum_wand_4ws9').replace('WAND_4WS8','WAND_4WS9')
        expected=expected.replace('4WS8','4WS9').replace('wand_4ws8','wand_4ws9')
        assert q.read_text()==expected,name
