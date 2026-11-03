"""Geometry, timing, angle continuity and passive ROS/UI regression."""
import ast
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

from preview_core import IDS,PreviewModel,identify_t,rotation,CAR_YAW_OFFSET
from receiver_guard import source_ok

HERE=Path(__file__).resolve().parent
SNAPSHOT=HERE.parents[1]/'docs/wand_rotation_preview_2026-10-26/initial_geometry.json'
GEOMETRY={'Sign1':(-.08,0.,0.),'Sign2':(.16,0.,0.),'Sign5':(0.,0.,0.),
    'Sign3':(0.,-.12,0.),'Sign4':(0.,-.24,0.)}


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,rclpy
    def denied(*a,**k):pytest.fail('offline preview test attempted socket or ROS initialization')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(rclpy,'init',denied)


def quaternion(yaw=0.,pitch=0.,roll=0.):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


def feed(m,t,yaw=0.,pitch=0.,roll=0.,position=(1.,2.,1.),omit=(),negative_q=False):
    q=quaternion(yaw,pitch,roll)
    if negative_q:q=tuple(-v for v in q)
    r=rotation(q);p=np.array(position)
    m.receive('sign',p,q,t,t)
    m.receive('car',(0.,0.,.1),quaternion(.20-CAR_YAW_OFFSET),t,t)
    for k,v in GEOMETRY.items():
        if k not in omit:m.receive(k,p+r@np.array(v),None,t,t)
    m.update(t,t)


def ready(yaw=0.):
    m=PreviewModel();m.source_ok=True
    for i in range(26):feed(m,10.+i*.02,yaw=yaw)
    assert len(m.candidates)==1
    m.confirm_geometry(10.5,10.5);m.zero(10.5,10.5)
    return m,10.5


def test_real_snapshot_identifies_actual_bar_and_not_consecutive_marker_numbers():
    data=json.loads(SNAPSHOT.read_text())['latest'];p=np.array(data['sign']['xyz']);r=rotation(data['sign']['xyzw'])
    candidates=identify_t({k:r.T@(np.array(data[k]['xyz'])-p) for k in IDS})
    assert len(candidates)==1
    shape=candidates[0]
    assert set(shape.bar)=={'Sign1','Sign2','Sign5'} and shape.bar[1]=='Sign5'
    assert set(shape.stem)=={'Sign3','Sign4'} and shape.width==pytest.approx(.24036,abs=.001)
    forward=r@shape.forward
    assert forward[1]>.99 and abs(forward[0])<.05


def test_stem_not_misidentified_as_bar_even_when_both_have_three_collinear_points():
    candidates=identify_t(GEOMETRY)
    assert len(candidates)==1 and candidates[0].bar[1]=='Sign5'
    assert np.array(candidates[0].forward)==pytest.approx([0.,1.,0.])


@pytest.mark.parametrize('yaw',[0.,.4,1.5,-2.4,math.pi])
def test_geometry_in_body_coordinates_is_invariant_under_world_rotation(yaw):
    m,t=ready(yaw)
    assert m.valid and m.turn==0.
    assert np.array(m.definition.forward)==pytest.approx([0.,1.,0.],abs=1e-12)


@pytest.mark.parametrize('bad',['missing','coincident','nonfinite','not_t'])
def test_bad_shapes_are_rejected_instead_of_guessing_a_positive_direction(bad):
    p=dict(GEOMETRY)
    if bad=='missing':p.pop('Sign5')
    elif bad=='coincident':p['Sign3']=p['Sign4']
    elif bad=='nonfinite':p['Sign3']=(float('nan'),0.,0.)
    else:p={k:(i*.13,i*i*.012,i*.007) for i,k in enumerate(IDS)}
    try:assert not identify_t(p)
    except ValueError:pass


def test_geometry_cannot_be_confirmed_without_stable_samples_or_with_stale_points():
    m=PreviewModel();m.source_ok=True;feed(m,10.)
    with pytest.raises(ValueError):m.confirm_geometry(10.,10.)
    m,t=ready();m.clear_zero()
    with pytest.raises(ValueError):m.confirm_geometry(t+.21,t+.21)


@pytest.mark.parametrize('angle',[30.,-30.,90.,-90.])
def test_relative_one_to_one_rotation_return_and_arbitrary_initial_car_heading(angle):
    m,t=ready(yaw=.8)
    for i in range(1,61):feed(m,t+i*.02,yaw=.8+math.radians(angle)*i/60)
    assert m.report()['relative_deg']==pytest.approx(angle,abs=1e-9)
    assert m.report()['target_car_heading_deg']==pytest.approx(math.degrees(.2)+angle)
    for i in range(1,61):feed(m,t+1.2+i*.02,yaw=.8+math.radians(angle)*(1-i/60))
    assert m.report()['relative_deg']==pytest.approx(0.,abs=1e-8)
    assert not m.report()['actuator_outputs'] and not m.report()['car_commanded']


def test_crossing_plus_minus_180_and_multiple_turns_never_reverses_the_angle():
    m,t=ready(yaw=math.radians(89.))
    for i in range(1,401):feed(m,t+i*.02,yaw=math.radians(89.+2*i))
    assert m.valid and m.report()['relative_deg']==pytest.approx(800.,abs=1e-8)


def test_quaternion_sign_change_and_vertical_translation_do_not_make_yaw_commands():
    m,t=ready()
    for i in range(1,31):
        feed(m,t+i*.02,position=(1.,2.,1.+i*.02),negative_q=bool(i%2))
    assert m.report()['relative_deg']==pytest.approx(0.,abs=1e-10)
    feed(m,t+.62,pitch=.4)
    assert m.report()['relative_deg']==pytest.approx(0.,abs=1e-10)


def test_vertical_reference_degeneracy_and_sudden_180_flip_invalidate_zero():
    m,t=ready();feed(m,t+.02,roll=math.pi/2)
    assert not m.valid and m.reference is None and '竖直' in m.reason
    m,t=ready();feed(m,t+.02,yaw=math.pi)
    assert not m.valid and m.reference is None and '跳变' in m.reason


def test_rigid_body_quaternion_flip_without_marker_motion_is_not_treated_as_real_turn():
    m,t=ready();t+=.02
    m.receive('sign',(1.,2.,1.),quaternion(math.pi),t,t)
    for k,p in GEOMETRY.items():m.receive(k,np.array(p)+[1.,2.,1.],None,t,t)
    m.update(t,t)
    assert not m.valid and m.reference is None and '不一致' in m.reason


@pytest.mark.parametrize('lost',IDS)
def test_occlusion_of_any_one_point_stops_preview_and_never_auto_restores_zero(lost):
    m,t=ready()
    for i in range(1,13):feed(m,t+i*.02,omit=(lost,))
    assert not m.valid and m.reference is None and m.report()['relative_deg'] is None
    for i in range(13,24):feed(m,t+i*.02,yaw=.1)
    assert m.valid and m.reference is None and m.report()['relative_deg'] is None
    m.zero(t+.46,t+.46)
    assert m.report()['relative_deg']==0. and m.reference_count==2


def test_repeated_or_out_of_order_source_stamp_cannot_keep_old_pose_fresh():
    m,t=ready()
    for i in range(1,31):
        assert not m.receive('sign',(1.,2.,1.),quaternion(),t,t+i*.02)
    m.update(t+.21,t+.21)
    assert not m.valid and m.reference is None


def test_different_frame_marker_samples_are_never_combined():
    m=PreviewModel();m.source_ok=True
    m.receive('sign',(0.,0.,0.),quaternion(),10.,10.)
    for i,k in enumerate(IDS):m.receive(k,GEOMETRY[k],None,10.001+i*.001,10.01)
    m.update(10.01,10.01)
    assert not m.valid and not m.candidates and '同一源时间戳' in m.reason


@pytest.mark.parametrize('fault',['source','future','clock_back','invalid_q','frame'])
def test_source_clock_frame_and_numeric_faults_invalidate_reference(fault):
    m,t=ready()
    if fault=='source':m.source_ok=False;m.update(t+.01,t+.01)
    elif fault=='future':m.update(t+.01,t-.01)
    elif fault=='clock_back':m.update(t+.01,t-1.)
    elif fault=='invalid_q':m.receive('sign',(1.,2.,1.),(0.,0.,0.,0.),t+.01,t+.01)
    else:m.receive('sign',(1.,2.,1.),quaternion(),t+.01,t+.01,frame='other')
    assert not m.valid and m.reference is None and m.report()['relative_deg'] is None


def test_zero_requires_a_fresh_car_pose_but_geometry_detection_does_not():
    m,t=ready();m.clear_zero();m.latest.pop('car');m.update(t,t)
    assert m.valid
    with pytest.raises(ValueError,match='car01'):m.zero(t,t)


@pytest.mark.parametrize('infos',[[],[NS(node_name='other',node_namespace='/')],
    [NS(node_name='vicon_client',node_namespace='/other')],
    [NS(node_name='vicon_client',node_namespace='/')]*2])
def test_missing_renamed_or_duplicate_publishers_are_not_accepted(infos):assert not source_ok(infos)


def test_only_expected_graph_source_is_accepted():
    assert source_ok([NS(node_name='vicon_client',node_namespace='/')])


def test_live_reader_creates_seven_subscriptions_and_no_motor_interfaces(monkeypatch):
    import rclpy,rclpy.node
    import live_input
    events=[]
    class PassiveNode:
        def __init__(self,*a,**kw):assert kw==dict(enable_rosout=False,start_parameter_services=False)
        def create_subscription(self,msg,topic,callback,qos):events.append(topic)
        def get_publishers_info_by_topic(self,t):return [NS(node_name='vicon_client',node_namespace='/')]
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=10_000_000_000))
        def destroy_node(self):pass
    monkeypatch.setattr(live_input,'verify_existing',lambda:dict(verified=True))
    monkeypatch.setattr(rclpy.node,'Node',PassiveNode)
    monkeypatch.setattr(rclpy,'init',lambda **k:None);monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(rclpy,'spin_once',lambda *a,**k:None)
    m=PreviewModel();reader=live_input.LiveInput(m);reader.pump();reader.close()
    assert set(events)==set(live_input.TOPICS.values()) and len(events)==7
    assert m.source_ok and not m.valid


def test_runtime_has_no_command_publisher_service_client_udp_or_controller_import():
    for name in ('preview_core.py','preview_ui.py','live_input.py','main.py','snapshot.py'):
        source=(HERE/name).read_text();tree=ast.parse(source)
        for n in ast.walk(tree):
            if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):
                assert n.func.attr not in ('create_publisher','create_client','create_service','sendto','send','cmd_vel')
            if isinstance(n,(ast.Import,ast.ImportFrom)):
                modules=([v.name for v in n.names] if isinstance(n,ast.Import) else [n.module or ''])
                assert not any('hardware' in x or 'commissioning' in x or x=='socket' for x in modules)


def test_real_tk_window_is_1280x720_and_does_not_enable_even_after_confirm_and_zero(tmp_path,monkeypatch):
    import tkinter as tk
    import preview_ui
    m,t=ready();m.clear_zero();m.definition=None;m.update(t,t)
    monkeypatch.setattr(preview_ui,'time',NS(monotonic=lambda:t))
    reader=NS(ros_now=lambda:t,pump=lambda:None,close=lambda:None,raw={},flags={})
    root=tk.Tk();root.withdraw();geometry=[];original=root.geometry
    monkeypatch.setattr(root,'geometry',lambda value:(geometry.append(value),original(value))[1])
    window=preview_ui.PreviewWindow(root,m,reader,tmp_path)
    try:
        window.draw();root.update_idletasks()
        assert geometry==['1280x720'] and tuple(root.minsize())==(1000,640)
        assert not window.ack.get() and m.reference is None
        window.confirm_geometry();assert m.definition is None
        window.ack.set(True);window.confirm_geometry();assert m.definition is not None
        window.zero_angle();window.draw();root.update_idletasks()
        assert m.reference is not None and m.report()['relative_deg']==0.
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert not m.report()['actuator_outputs'] and not m.report()['car_commanded']
    finally:window.close()
