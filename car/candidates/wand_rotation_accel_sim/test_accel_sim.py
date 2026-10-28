"""High-acceleration software envelope, input provenance and stop regressions."""
from dataclasses import fields,replace
import math
import os
from pathlib import Path
import pytest

from accel_core import (AccelerationLimits,AccelerationEngine,XY_ACCELS,YAW_ACCELS,
    BASE,ROOT,Clicks,ZERO,load_definition,preview_model)
from test_live_sim import Fixture
import test_rotation_sim as original_fault_tests


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket
    def forbidden(*args,**kwargs):pytest.fail('Offline acceleration test attempted external IO')
    monkeypatch.setattr(socket,'socket',forbidden)
    try:
        import rclpy
        monkeypatch.setattr(rclpy,'init',forbidden)
    except ImportError:pass


class FastFixture(Fixture):
    def __init__(self,limits=None):
        super().__init__()
        self.e=AccelerationEngine(self.d,self.now,self.ros,limits or AccelerationLimits(2.,12.))
        self.k=Clicks(self.e);self.k.focus(True)


@pytest.mark.parametrize('a',XY_ACCELS)
@pytest.mark.parametrize('aa',YAW_ACCELS)
def test_enumerated_pairs_keep_all_other_guards(a,aa):
    c=AccelerationLimits(a,aa)
    for field in fields(BASE.follower_config()):
        if field.name not in ('max_accel_mps2','max_yaw_accel_radps2'):
            assert getattr(c.follower_config(),field.name)==getattr(BASE.follower_config(),field.name)
    for name in ('max_speed_mps','max_yaw_rate_radps','pose_timeout_s','lease_timeout_s',
        'command_timeout_s','max_control_gap_s','max_session_s','max_tracking_error_m',
        'ground_z_m','tilt_limit_rad','boundary_margin_m','half_length_m','half_width_m'):
        assert getattr(c,name)==getattr(BASE,name)


@pytest.mark.parametrize('a,aa',[(0.,12.),(2.,0.),(3.,12.),(2.,20.),(True,12.),(2.,math.nan)])
def test_unsupported_or_nonfinite_configuration_rejected(a,aa):
    with pytest.raises(ValueError):AccelerationLimits(a,aa)


def test_acceleration_and_coupled_wheel_envelope_continuous_reversal():
    f=FastFixture();f.ready();f.click();old=ZERO;max_a=max_aa=0.
    for i in range(1,601):
        t=i*.02;angle=f.e.origin.yaw
        x=.2*math.sin(2.5*t);y=.1*(1-math.cos(2.5*t))
        f.displacement=(math.cos(angle)*x-math.sin(angle)*y,math.sin(angle)*x+math.cos(angle)*y,0.)
        f.turn=.5*math.sin(2.5*t);out=f.step()
        assert f.e.sup.state=='ACTIVE',(f.e.sup.reason,f.e.gate.fault)
        a=math.hypot(out.vx-old.vx,out.vy-old.vy)/.02;aa=abs(out.wz-old.wz)/.02
        max_a=max(a,max_a);max_aa=max(aa,max_aa);old=out
        assert a<=2.+1e-6 and aa<=12.+1e-6
        assert math.hypot(out.vx,out.vy)<=.7+1e-8 and abs(out.wz)<=2.+1e-8
        assert max(abs(v) for v in f.e.report()['wheel_rpm'])<=400.+1e-6
        assert f.e.c.inside(f.e.plant.pose)
    assert max_a>.8 # The raised envelope is actually exercised.


def test_large_prepare_adjustment_neutral_first_frame():
    f=FastFixture();f.ready()
    for i in range(1,51):
        f.displacement=(i*.002,0.,i*.005);f.turn=i*.005;f.step()
    assert f.e.output==ZERO and f.e.sup.follower.mapping is None
    f.click();f.run(10);assert f.e.output==ZERO


@pytest.mark.parametrize('cause',['esc','space','focus','source','stale','gap','clock'])
def test_high_accel_fail_stop_paths(cause):
    f=FastFixture();f.moving()
    if cause=='esc':f.k.escape()
    if cause=='space':f.k.press(f.now)
    if cause=='focus':f.k.focus(False)
    if cause=='source':f.step(live=False)
    if cause=='stale':f.run(12,update=False)
    if cause=='gap':f.step(dt=.11)
    if cause=='clock':f.e.step(f.now+.02,f.ros-.1,f.s)
    if cause in ('esc','space','focus'):f.step()
    assert f.e.output==ZERO and not f.e.requested
    if cause in ('gap','clock'):
        f.run(100);assert not f.e.prepare()


def test_high_accel_occlusion_no_resume_explicit_recovery():
    f=FastFixture();f.moving();f.step(live=False);assert f.e.output==ZERO
    f.run(150);assert f.e.output==ZERO and f.e.sup.stop_confirmed
    old_pose=f.e.plant.pose;assert f.e.prepare();assert f.e.plant.pose==old_pose
    assert f.e.c.max_accel_mps2==2. and f.e.c.max_yaw_accel_radps2==12.
    f.run(100);assert f.e.armed and f.e.output==ZERO
    f.click();f.turn+=.02;f.step();assert f.e.output.wz>0


def test_higher_accel_selected_source_tamper_still_rejected():
    f=FastFixture();f.ready();f.click()
    f.e.modify_bound=lambda b:replace(b,source_stamp=b.source_stamp-.001)
    f.step();assert f.e.output==ZERO
    assert f.e.gate.fault=='SELECTED_SOURCE_IDENTITY_MISMATCH'


def test_120_seconds_and_arena_unchanged():
    f=FastFixture();f.ready();f.click();f.run(6001)
    assert f.e.output==ZERO and not f.e.requested
    assert f.e.c.arena_forward_min_m==-1.5 and f.e.c.arena_forward_max_m==.5
    assert f.e.c.arena_left_min_m==-.4 and f.e.c.arena_left_max_m==.5


def test_high_accel_real_tk_label_and_key_binding(tmp_path,monkeypatch):
    import tkinter as tk
    from ui_accel import AccelerationWindow
    f=FastFixture();f.ready()
    class Key:
        value=False
        def down(self):return self.value
        def close(self):pass
    class Reader:
        flags={}
        def close(self):pass
    root=tk.Tk();root.withdraw();key=Key()
    w=AccelerationWindow(root,preview_model(f.d),Reader(),f.e,tmp_path,key_state=key,schedule=False)
    w.keys=f.k;monkeypatch.setattr(w,'advance',lambda:f.now)
    try:
        root.update_idletasks();w.draw();assert root.winfo_reqheight()<=720
        assert w.prepare_button.bindtags()[0]=='RotationSimKeys'
        assert 'A2' in root.title()
        def texts(p):
            result=[]
            for v in p.winfo_children():
                if 'text' in v.keys():result.append(str(v.cget('text')))
                result+=texts(v)
            return result
        assert any('2.00 m/s²' in t and '12.00 rad/s²' in t for t in texts(root))
        key.value=True;w.press();w.release();assert not f.e.requested
        key.value=False;w.release();assert f.e.requested
        w.stop();assert not f.e.requested
    finally:w.close()


def test_new_entry_has_no_motor_or_network_mutation_api():
    for name in ('main.py','accel_core.py','ui_accel.py'):
        text=(Path(__file__).parent/name).read_text()
        for bad in ('create_publisher(', 'create_client(', 'sendto(', 'serial.Serial(', 'nmcli', 'openocd', 'esptool'):
            assert bad not in text


@pytest.mark.parametrize('fault',original_fault_tests.FAULTS)
def test_original_full_fault_injection_at_higher_acceleration(fault,monkeypatch):
    from harness import RotationHarness
    def moving():
        h=RotationHarness(AccelerationLimits(2.,12.));h.start()
        for i in range(1,61):h.set_gesture(.001*i,.0008*i,.004*i);h.step()
        assert h.sup.state=='ACTIVE' and abs(h.output.wz)>.1
        return h
    monkeypatch.setattr(original_fault_tests,'moving',moving)
    original_fault_tests.test_full_supervisor_and_independent_gate_fail_stop_without_automatic_restart(fault)
