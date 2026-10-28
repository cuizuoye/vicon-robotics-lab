"""Offline full core + source packet + actual Tk event routing regressions."""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys
import pytest

from engine import (Engine, Clicks, Raw, Selected, load_definition, preview_model,
    select_source, ZERO, Pose, rotation, vector, ROOT)


def packet(definition, now, ros, turn=0., displacement=(0.,0.,0.)):
    # Vary raw rigid origin so Sign5, NOT raw Sign origin, is fixed under yaw.
    a = math.pi/2 + turn
    q = (0.,0.,math.sin(a/2),math.cos(a/2)); r = rotation(q)
    base = vector((1.,2.,1.)) + vector(displacement)
    origin = base-r@vector(definition['local_points']['Sign5'])
    rows = [Raw('sign',tuple(origin),q,ros,now)]
    for name in ('Sign1','Sign2','Sign3','Sign4','Sign5'):
        p = origin+r@vector(definition['local_points'][name])
        rows.append(Raw(name,tuple(p),None,ros,now))
    return Selected(tuple(rows))


class Fixture:
    def __init__(self):
        self.d = load_definition(); self.now=10.; self.ros=1e9
        self.e = Engine(self.d,self.now,self.ros); self.k=Clicks(self.e)
        self.k.focus(True); self.s=None; self.turn=0.; self.displacement=(0.,0.,0.)
    def step(self, *, live=True, dt=.02, update=True):
        self.now+=dt; self.ros+=dt
        if update: self.s=packet(self.d,self.now,self.ros,self.turn,self.displacement)
        return self.e.step(self.now,self.ros,self.s if live else None)
    def run(self,n=100,**kw):
        for _ in range(n): self.step(**kw)
    def ready(self):
        self.run(); assert self.e.can_prepare()
        assert self.e.prepare(); self.run(90); assert self.e.armed
    def click(self):
        self.k.press(self.now); assert not self.e.requested
        self.k.release(self.now+.001); self.step(); assert self.e.sup.state=='ACTIVE'
        assert self.e.output==ZERO
    def moving(self):
        self.ready(); self.click()
        for i in range(1,51): self.turn=i*.01; self.step()
        assert abs(self.e.output.wz)>.1


def test_geometry_reuses_human_definition_and_sign5():
    f=Fixture(); p0=packet(f.d,10.,1e9).pose(f.d,10.,1e9)
    p1=packet(f.d,10.,1e9,math.pi/2).pose(f.d,10.,1e9)
    assert math.dist((p0.x,p0.y,p0.z),(p1.x,p1.y,p1.z))<1e-12
    assert math.isclose(p1.yaw-p0.yaw,math.pi/2)


@pytest.mark.parametrize('fault', ['mixed','stale','future','missing','flip','nan'])
def test_raw_packet_rejects_faults(fault):
    f=Fixture(); p=packet(f.d,10.,1e9); rows=list(p.samples)
    if fault=='mixed': rows[2]=replace(rows[2],stamp=rows[2].stamp-.01)
    if fault=='stale': rows[2]=replace(rows[2],received=9.79)
    if fault=='future': rows=[replace(s,stamp=s.stamp+.02) for s in rows]
    if fault=='missing': rows.pop()
    if fault=='flip': rows[0]=replace(rows[0],quaternion=(0.,0.,0.,1.))
    if fault=='nan': rows[1]=replace(rows[1],position=(math.nan,0.,1.))
    with pytest.raises(ValueError): Selected(tuple(rows)).pose(f.d,10.,1e9)


def test_selects_only_complete_valid_same_frame():
    f=Fixture(); m=preview_model(f.d); m.source_ok=True
    p=packet(f.d,10.,1e9)
    for s in p.samples:
        m.receive(s.name,s.position,s.quaternion,s.stamp,s.received)
    m.update(10.,1e9)
    s=select_source(m,f.d,10.,1e9); assert s==p
    m.source_ok=False; assert select_source(m,f.d,10.,1e9) is None


def test_prepare_then_pick_up_move_and_rotate_before_engage_is_neutral():
    f=Fixture(); f.ready()
    for i in range(1,51):
        f.turn=.005*i; f.displacement=(.001*i,0.,.003*i); f.step()
        assert f.e.output==ZERO and f.e.sup.follower.mapping is None
    f.click(); f.run(10)
    assert f.e.output==ZERO
    origin=f.e.sup.follower.mapping.wand_origin
    expected=f.s.pose(f.d,f.now,f.ros)
    assert math.dist((origin.x,origin.y),(expected.x,expected.y))<1e-9


def test_real_xy_yaw_direction_and_combination():
    f=Fixture(); f.ready(); f.click()
    # World-axis motion, not hand-heading-relative XY. Height is ignored.
    a=f.e.origin.yaw
    for i in range(1,151):
        f.displacement=(.001*i*math.cos(a),.001*i*math.sin(a),.1*math.sin(i/20))
        f.turn=.005*i; f.step()
        assert f.e.sup.state=='ACTIVE', (f.e.sup.reason,f.e.gate.fault)
        assert max(abs(v) for v in f.e.report()['wheel_rpm'])<=400.+1e-6
    assert f.e.sup.follower.mapping.turn==pytest.approx(.75)
    assert f.e.plant.pose.yaw>f.e.origin.yaw+.5
    x,y=f.e.c.geometry().local(f.e.plant.pose.x,f.e.plant.pose.y)
    assert x>-.42 and abs(y-.05)<.04


def test_duplicate_does_not_restamp_or_refresh():
    f=Fixture(); f.moving(); old=f.e.sup.inputs.poses['wand'].copy()
    f.run(5,update=False)
    assert f.e.sup.inputs.poses['wand']==old
    f.run(7,update=False)
    assert f.e.output==ZERO and f.e.sup.state!='ACTIVE'


def test_loss_return_keeps_stopped_and_explicit_new_session_preserves_car():
    f=Fixture(); f.moving(); f.step(live=False)
    assert f.e.output==ZERO and not f.e.requested
    f.run(150)
    assert f.e.sup.state=='FAULT' and f.e.sup.stop_confirmed
    assert f.e.output==ZERO
    before=f.e.plant.pose; velocity=f.e.plant.velocity
    assert f.e.prepare(); assert f.e.plant.pose==before and f.e.plant.velocity==velocity
    f.run(100); assert f.e.armed and f.e.output==ZERO
    assert f.e.sup.follower.mapping is None
    f.click(); f.run(10); assert f.e.output==ZERO
    f.turn+=.01; f.step(); assert f.e.output.wz>0


@pytest.mark.parametrize('kind',['digest','stamp','gate_pose'])
def test_final_gate_selected_source_binding(kind):
    f=Fixture(); f.ready(); f.click()
    def tamper(b):
        if kind=='digest': return replace(b,source_digest='0'*64)
        if kind=='stamp': return replace(b,source_stamp=b.source_stamp-.02)
        f.e.gate.inputs.poses['wand']['pose']=Pose(99.,99.,1.,0.)
        return b
    f.e.modify_bound=tamper; f.turn=.01; f.step()
    assert f.e.output==ZERO
    assert f.e.gate.fault in ('SELECTED_SOURCE_IDENTITY_MISMATCH','GATE_SELECTED_POSE_MISMATCH')


def test_first_click_release_only_engages_and_autorepeat_does_not_toggle():
    f=Fixture(); f.ready(); f.k.press(f.now)
    for _ in range(5): f.k.press(f.now); f.step()
    assert not f.e.requested and f.e.output==ZERO
    assert f.k.release(f.now+.001); f.step(); assert f.e.sup.state=='ACTIVE'
    f.k.release(f.now+.001); assert f.e.requested # release is not a stop
    f.k.press(f.now+.002); f.step(); assert f.e.output==ZERO and not f.e.requested


def test_polling_release_and_expired_click_cannot_start():
    f=Fixture(); f.ready(); f.k.press(f.now); f.k.release(f.now+.01,commit=False)
    assert not f.e.requested
    f.k.press(f.now); f.k.release(f.now+1.1)
    assert not f.e.requested


@pytest.mark.parametrize('kind',['focus','escape','gap','clock','timeout'])
def test_stop_paths(kind):
    f=Fixture(); f.moving()
    if kind=='focus': f.k.focus(False)
    if kind=='escape': f.k.escape()
    if kind=='gap': f.step(dt=.11)
    if kind=='clock': f.e.step(f.now+.02,f.ros-.1, f.s)
    if kind=='timeout':
        # Advance real-like small steps, not one giant control timer gap.
        f.run(6000)
    if kind not in ('gap','clock'): f.step()
    assert f.e.output==ZERO and not f.e.requested
    if kind in ('gap','clock'): assert f.e.sup.hard_fault or f.e.gate.hard_fault


def test_tk_layout_keys_button_focus_and_repeat(tmp_path,monkeypatch):
    import tkinter as tk
    import ui
    f=Fixture(); f.ready()
    class FakeKeys:
        down_value=False
        def down(self): return self.down_value
        def close(self): pass
    class Reader:
        flags={}
        def close(self): pass
    key=FakeKeys(); root=tk.Tk(); root.withdraw()
    w=ui.Window(root,preview_model(f.d),Reader(),f.e,tmp_path,key_state=key,schedule=False)
    w.keys=f.k
    monkeypatch.setattr(w,'advance',lambda:f.now)
    try:
        root.update_idletasks(); w.draw()
        assert root.winfo_reqheight()<=720
        assert w.prepare_button.bindtags()[0]=='RotationSimKeys'
        # Execute the actual Tk bindtag script with a button target. Space
        # must not invoke ttk's default button activation or prepare callback.
        script=root.bind_class('RotationSimKeys','<KeyPress-space>')
        assert script and 'break' in script
        key.down_value=True; w.press(); w.press()
        assert not f.e.requested
        w.release(); assert not f.e.requested # synthetic autorepeat release
        key.down_value=False; w.release(); assert f.e.requested
        w.press(); assert not f.e.requested
        w.keys.focus(False); assert not w.keys.focused
    finally: w.close()


def test_no_actuation_api_in_live_boundary():
    here=Path(__file__).parent
    for name in ('main.py','ui.py','engine.py'):
        text=(here/name).read_text()
        for forbidden in ('create_publisher(', 'create_client(', 'sendto(', 'serial.Serial(', 'esptool', 'openocd'):
            assert forbidden not in text


def test_current_region_and_sim_only_limits():
    f=Fixture(); c=f.e.c
    assert c.max_speed_mps==.7 and c.max_accel_mps2==.8
    assert c.max_yaw_rate_radps==2. and c.max_yaw_accel_radps2==6.
    assert c.max_session_s==120.
    assert c.pose_timeout_s==.2 and c.max_control_gap_s==.1
    assert c.half_length_m==.095 and c.half_width_m==.105


def test_tilted_projection_invalid_and_height_only_stays_neutral():
    f=Fixture(); f.ready(); f.click()
    for i in range(1,41): f.displacement=(0.,0.,i*.004); f.step()
    assert f.e.output==ZERO
    from rotation_core import reference_pose
    # Accepted local forward approximately -Y; rotate about X by pi/2.
    with pytest.raises(ValueError): reference_pose((1.,2.,1.),(math.sin(math.pi/4),0.,0.,math.cos(math.pi/4)),f.d)


def test_source_change_during_pending_click_does_not_enable():
    f=Fixture(); f.ready(); f.k.press(f.now)
    f.step(live=False)
    assert not f.k.release(f.now+.001)
    f.run(100)
    assert not f.e.requested and f.e.output==ZERO


def test_bound_source_mismatch_also_blocks_disabled_arm_handshake():
    f=Fixture(); f.run(); assert f.e.prepare()
    f.e.modify_bound=lambda b: replace(b,source_digest='0'*64)
    f.run(90)
    assert not f.e.armed and f.e.output==ZERO
    assert not f.e.engage()


def test_hard_timer_fault_cannot_be_cleared_by_reprepare():
    f=Fixture(); f.moving(); f.step(dt=.11); f.run(100)
    assert not f.e.prepare() and f.e.output==ZERO


def test_boundary_governor_does_not_expand_area():
    f=Fixture(); f.ready(); f.click(); a=f.e.origin.yaw
    for i in range(1,701):
        f.displacement=(i*.004*math.cos(a),i*.004*math.sin(a),0.)
        f.step()
        assert f.e.c.inside(f.e.plant.pose)
        if f.e.sup.follower.goal is not None: assert f.e.c.inside(f.e.sup.follower.goal)


def test_private_display_real_tk_events(tmp_path,monkeypatch):
    """Real mapped widgets, events and focus; NEVER touches the user's display."""
    import os
    if os.environ.get('ROTATION_SIM_PRIVATE_TEST_DISPLAY') != os.environ.get('DISPLAY'):
        pytest.skip('requires explicit private nested X server')
    import tkinter as tk
    import ui
    f=Fixture(); f.ready()
    class Key:
        value=False
        def down(self): return self.value
        def close(self): pass
    class Reader:
        flags={}
        def close(self): pass
    root=tk.Tk(); key=Key()
    w=ui.Window(root,preview_model(f.d),Reader(),f.e,tmp_path,key_state=key,schedule=False)
    w.keys=f.k; monkeypatch.setattr(w,'advance',lambda:f.now)
    calls=[]; w.prepare_button.configure(command=lambda:calls.append('UNEXPECTED_PREPARE'))
    try:
        root.update(); w.prepare_button.focus_force(); root.update(); w.check_focus()
        key.value=True
        w.prepare_button.event_generate('<KeyPress-space>'); root.update()
        assert not f.e.requested and w.keys.pending is not None
        for _ in range(8):
            w.prepare_button.event_generate('<KeyRelease-space>')
            w.prepare_button.event_generate('<KeyPress-space>'); root.update()
        assert not f.e.requested and not calls
        key.value=False; w.prepare_button.event_generate('<KeyRelease-space>'); root.update()
        assert f.e.requested and not calls
        key.value=True; w.prepare_button.event_generate('<KeyPress-space>'); root.update()
        assert not f.e.requested and not calls
        key.value=False; w.prepare_button.event_generate('<KeyRelease-space>'); root.update()
        other=tk.Toplevel(root); other.update(); other.focus_force(); root.update(); w.check_focus()
        assert not w.keys.focused
        other.destroy()
    finally: w.close()
