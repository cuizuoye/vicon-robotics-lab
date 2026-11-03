"""Launcher transitions and explicit default provenance; no robot/device I/O."""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import tkinter as tk
import pytest

OUT=Path(os.environ['AUTOSTART_OUTPUT']);ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(OUT))
for package in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(OUT/'runtime/src'/package))
spec=importlib.util.spec_from_file_location('autostart_app_test',OUT/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)
from mecanum_wand_4ws8_hardware.standing_policy import preconditions_record,verify_preconditions
from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile
from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT
from mecanum_wand_4ws8_control.core import Pose


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket,subprocess,rclpy
    def denied(*a,**kw):pytest.fail('offline test attempted external process/socket/ROS I/O')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(subprocess,'Popen',denied)
    monkeypatch.setattr(rclpy,'init',denied)


def clean(root):
    for job in root.tk.call('after','info'):root.after_cancel(job)
    root.destroy()


def widgets(root):
    yield root
    for child in root.winfo_children():yield from widgets(child)


def launcher(monkeypatch,tmp_path,fail=None):
    calls=[]
    def step(name,result=None):
        def f(*a):
            calls.append(name)
            if name==fail:raise ValueError('TEST '+name+' failed')
            return result
        return f
    monkeypatch.setattr(app,'verify_files',step('files',json.loads((OUT/'build_manifest.json').read_text())))
    monkeypatch.setattr(app,'check_port',step('port'))
    monkeypatch.setattr(app,'ensure_tenda',step('tenda'))
    monkeypatch.setattr(app,'ensure_receiver',step('receiver'))
    monkeypatch.setattr(app,'check_poses',step('poses',dict(ready=True,car_world_m=[12.,-8.,.108],car_heading_deg=0.)))
    root=tk.Tk();root.withdraw();w=app.App(root,preview=False)
    # A pure record of the launch request, never a child process or ROS node.
    def opened():
        calls.append('open_disabled_console')
        w.trial=tmp_path;w.backend=NS(poll=lambda:None,returncode=None)
    w.open_console=opened
    return root,w,calls


def spin(root,seconds=.55):
    end=time.monotonic()+seconds
    while time.monotonic()<end:root.update();time.sleep(.005)


def test_no_checkboxes_and_no_manual_open_step_in_1280_720_preview():
    root=tk.Tk();root.withdraw();w=app.App(root,preview=True)
    try:
        root.update_idletasks()
        assert not any(x.winfo_class()=='TCheckbutton' for x in widgets(root))
        assert not hasattr(w,'checks') and not w.open_button.winfo_manager()
        assert '并非机器检测' in w.preconditions_text.get()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert not w.auto_open_allowed() and w.backend is None
        assert str(w.check_button['state'])=='disabled'
    finally:clean(root)


def test_successful_preflight_automatically_opens_exactly_once_without_motion(monkeypatch,tmp_path):
    root,w,calls=launcher(monkeypatch,tmp_path)
    try:
        spin(root)
        assert calls==['files','port','tenda','receiver','poses','open_disabled_console']
        assert w.auto_open_attempted and w.backend is not None
        spin(root,.15);assert calls.count('open_disabled_console')==1
        assert not hasattr(w,'model') and not hasattr(w,'node')
        assert '单击空格' in w.message.get()
    finally:clean(root)


@pytest.mark.parametrize('failure',['files','port','tenda','receiver','poses'])
def test_any_preflight_failure_keeps_console_closed_and_does_not_retry(monkeypatch,tmp_path,failure):
    root,w,calls=launcher(monkeypatch,tmp_path,fail=failure)
    try:
        spin(root)
        assert 'open_disabled_console' not in calls and w.backend is None
        assert w.good_at<0 and '检查未通过' in w.message.get()
        before=calls.copy();spin(root,.2);assert calls==before
    finally:clean(root)


@pytest.mark.parametrize('bad',['busy','closed','attempted','backend','stale','future','preview'])
def test_automatic_open_needs_fresh_success_and_valid_local_state(bad):
    root=tk.Tk();root.withdraw();w=app.App(root,preview=True)
    try:
        w.preview=False;w.good_at=time.monotonic()
        assert w.auto_open_allowed()
        if bad=='busy':w.busy=True
        elif bad=='closed':w.closed=True
        elif bad=='attempted':w.auto_open_attempted=True
        elif bad=='backend':w.backend=object()
        elif bad=='stale':w.good_at-=121.
        elif bad=='future':w.good_at+=1.
        else:w.preview=True
        assert not w.auto_open_allowed()
        called=[];w.open_console=lambda:called.append('opened');w.auto_open_if_ready()
        assert not called
    finally:clean(root)


def test_console_exit_does_not_restart_automatically_but_explicit_recheck_can(monkeypatch,tmp_path):
    root,w,calls=launcher(monkeypatch,tmp_path)
    try:
        spin(root);assert calls.count('open_disabled_console')==1
        w.backend=NS(poll=lambda:0,returncode=0);spin(root,.2)
        assert w.backend is None and w.good_at<0
        spin(root,.15);assert calls.count('open_disabled_console')==1
        w.preflight();spin(root,.3);assert calls.count('open_disabled_console')==2
    finally:clean(root)


def test_default_record_does_not_claim_machine_or_fresh_human_verification():
    r=preconditions_record()
    assert r['basis']=='EXPLICIT_USER_STANDING_INSTRUCTION'
    assert r['machine_verified'] is False and r['fresh_human_confirmation_recorded'] is False
    assert verify_preconditions(dict(site_preconditions=r))


@pytest.mark.parametrize('bad',['missing','verified','human_confirmed','wrong_request','wrong_hash','legacy_checks','legacy_approval'])
def test_declaration_cannot_be_replaced_by_fake_fresh_confirmations(bad):
    p=dict(site_preconditions=preconditions_record())
    if bad=='missing':p.pop('site_preconditions')
    elif bad=='verified':p['site_preconditions']['machine_verified']=True
    elif bad=='human_confirmed':p['site_preconditions']['fresh_human_confirmation_recorded']=True
    elif bad=='wrong_request':p['site_preconditions']['user_request']='/tmp/nope'
    elif bad=='wrong_hash':p['site_preconditions']['user_request_sha256']='0'*64
    elif bad=='legacy_checks':p['tracking_confirmations']={'immediate_power_cut_ready_now':True}
    else:p['staged_runtime_approval']={'clear_and_power_cut_ready_now':True}
    with pytest.raises(ValueError):verify_preconditions(p)


def valid_profile():
    m=json.loads((OUT/'build_manifest.json').read_text())
    p=json.loads(Path(m['draft_profiles'][4]).read_text());p['trial_review']=app.make_review(m)
    p['site_preconditions']=preconditions_record()
    p['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        automatic_preflight_launch_requested=True,preflight_completed_steady_s=time.monotonic(),
        flash_record=str(app.FLASH),build_manifest=str(OUT/'build_manifest.json'))
    return p


def test_runtime_accepts_declared_defaults_without_changing_motion_guards(tmp_path):
    import hashlib
    p=valid_profile();assert verify_runtime_profile(p,'sign')
    path=tmp_path/'SOFTWARE_ONLY_profile.json';path.write_text(json.dumps(p))
    r=Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    c=r.config
    assert (c.max_speed_mps,c.max_accel_mps2,c.max_session_s,c.pose_timeout_s)==(.7,.8,120.,.2)
    assert c.inside(Pose(12.,-8.,c.ground_z_m,0.)) and not c.follower_config().workspace_radius_enabled
    assert not p.get('tracking_confirmations')


@pytest.mark.parametrize('bad',['expired_utc','expired_steady','future_steady','missing_preflight','nan_steady',
    'wrong_firmware','loopback','wrong_stage','wrong_scale','loose_timeout'])
def test_automatic_profile_cannot_skip_fresh_preflight_or_existing_guards(bad):
    p=valid_profile();a=p['staged_runtime_approval']
    if bad=='expired_utc':a['issued_utc_s']-=601.
    elif bad=='expired_steady':a['preflight_completed_steady_s']-=121.
    elif bad=='future_steady':a['preflight_completed_steady_s']+=10.
    elif bad=='missing_preflight':a.pop('automatic_preflight_launch_requested')
    elif bad=='nan_steady':a['preflight_completed_steady_s']=math.nan
    elif bad=='wrong_firmware':p['hardware']['required_motion_tag']='4WS9'
    elif bad=='loopback':p['hardware']['esp_ip']='127.0.0.1'
    elif bad=='wrong_stage':p['stage']=6
    elif bad=='wrong_scale':p['mapping_scale']=2.
    else:p['safety']['pose_timeout_s']=.3
    with pytest.raises(ValueError):verify_runtime_profile(p,'sign')


def test_session_builder_records_default_provenance_and_preserves_ssh_environment(monkeypatch,tmp_path):
    m=json.loads((OUT/'build_manifest.json').read_text())
    monkeypatch.setattr(app,'verify_files',lambda:m);monkeypatch.setattr(app,'check_port',lambda:None)
    old_mkdtemp=app.tempfile.mkdtemp
    monkeypatch.setattr(app.tempfile,'mkdtemp',lambda **kw:old_mkdtemp(prefix='SOFTWARE_ONLY_',dir=tmp_path))
    monkeypatch.setenv('DISPLAY','localhost:10.0');monkeypatch.setenv('XAUTHORITY','/tmp/ssh-user-auth')
    folder,command,env=app.create_session(time.monotonic())
    p=json.loads((folder/'automatic_preflight_session.json').read_text())
    assert p['site_preconditions']['machine_verified'] is False
    assert p['site_preconditions']['fresh_human_confirmation_recorded'] is False
    assert 'clear_and_power_cut_ready_now' not in p['staged_runtime_approval']
    assert env['DISPLAY']=='localhost:10.0' and env['XAUTHORITY']=='/tmp/ssh-user-auth'
    assert env['WAND_AUTOPREP_FLOW']=='AUTO_PREPARE_SINGLE_SPACE_V1'
    assert 'gui:=true' in command and not any('enable' in x or 'start:=true' in x for x in command)
    with pytest.raises(ValueError):app.create_session(time.monotonic()-121.)


def test_all_controller_bridge_and_operator_console_sources_are_byte_identical():
    m=json.loads((OUT/'build_manifest.json').read_text());parent=json.loads((ROOT/'docs/wand_tracking_workspace_2026-10-27/prepared/build_manifest.json').read_text())
    for group in (m['control_package'],m['hardware_package']):
        for p in (Path(parent['runtime_src'])/group/group).glob('*.py'):
            if group==m['hardware_package'] and p.name in ('authorization.py','commissioning.py'):continue
            assert p.read_bytes()==(Path(m['runtime_src'])/group/group/p.name).read_bytes(),p


@pytest.mark.parametrize('reply',[b'AUTO_DEFAULT_V1',b'',b'OLD_MANUAL_UI'])
def test_running_old_console_is_not_mistaken_for_this_frontend(monkeypatch,tmp_path,reply):
    sent=[]
    class Client:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def settimeout(self,*a):pass
        def connect(self,*a):pass
        def sendall(self,p):sent.append(p)
        def recv(self,*a):return reply
    def locked(*a):raise BlockingIOError()
    monkeypatch.setenv('XDG_RUNTIME_DIR',str(tmp_path))
    monkeypatch.setattr(app.fcntl,'flock',locked)
    monkeypatch.setattr(app.socket,'socket',lambda *a:Client())
    if reply==b'AUTO_DEFAULT_V1':assert app.single_instance() is None
    else:
        with pytest.raises(ValueError,match='旧版/其他操作台'):app.single_instance()
    assert sent==[b'SHOW_AUTO_DEFAULT_V1']
