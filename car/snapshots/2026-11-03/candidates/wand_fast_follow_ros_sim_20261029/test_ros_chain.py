"""Actual Jazzy DDS, clocks and signed output; synthetic input, software car."""
import argparse
import json
import math
import os
from pathlib import Path
import time

from bootstrap import protected_hashes, input_hashes, save, digest, ZERO
from ros_chain import Chain, init_private, synthetic_packet
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mecanum_wand_4ws8_control.safety_nodes import LATEST


CASES=('fast_reverse_stop','occlusion_recover','bound_digest','bound_stamp','bound_car_stamp',
    'unknown_source_fifo','command_accel','signed_tamper','unsigned_bypass',
    'feedback_loss','car_pose_loss','ui_loss','graph_conflict','prepare_ui_flow','prepare_cancel_loss')


def run(output,cases=CASES):
    before=protected_hashes();save(output/'protected_before.json',before)
    inputs=input_hashes();results=[]
    here=Path(__file__).resolve().parent
    implementation={str(here/name):digest(here/name) for name in ('ros_chain.py','app.py','live_reader.py','bootstrap.py')}
    save(output/'IMPLEMENTATION_BEFORE.json',implementation)
    init_private()
    try:
        for case in cases:
            folder=output/case;folder.mkdir()
            state=dict(x=0.,y=0.,turn=0.,freeze=False,packet=None)
            def provider(now,ros):
                if not state['freeze']:state['packet']=synthetic_packet(state['x'],state['y'],state['turn'],now,ros)
                return state['packet']
            c=Chain(provider,folder);extra=[]
            def spin(seconds,touch=True,action=None):
                started=time.monotonic()
                while time.monotonic()-started<seconds:
                    if touch:c.touch()
                    if action:action(time.monotonic()-started)
                    c.step()
            def wait(pred,timeout=5.):
                started=time.monotonic()
                while time.monotonic()-started<timeout:
                    c.touch();c.step()
                    if pred():return
                raise AssertionError(c.snapshot())
            try:
                wait(lambda:c.sup.core.stop_confirmed and not c.sup.core.inputs.health(*c.sup.clocks()))
                if case=='prepare_ui_flow':
                    import tkinter as tk
                    import threading
                    from app import Window,Worker
                    class Adapter:
                        snapshot={};ui_stamp=0.
                        def __init__(self):self.done=threading.Event()
                        def post(self,action):Worker.dispatch(c,action,time.monotonic())
                    root=tk.Tk();root.withdraw();adapter=Adapter();window=Window(root,adapter)
                    try:
                        c.focus_lost();assert c.sup.core.state=='STANDBY' and c.sup.core.stop_confirmed
                        c.source.on=False;spin(.25)
                        old_sessions=c.sessions;assert not c.prepare();assert c.sessions==old_sessions
                        assert 'Sign' in c.message
                        c.source.on=True;wait(lambda:c.snapshot()['can_prepare'])
                        window.controls(c.snapshot());window.prepare_button.invoke()
                        assert c.prepare_pending and not c.requested
                        seen_gate_wait=False;began=time.monotonic()
                        while time.monotonic()-began<5.:
                            c.touch();c.step()
                            seen_gate_wait|=c.sup.core.state=='ARMED' and not c.ready()
                            if not c.ready():assert 'READY' not in c.message
                            window.controls(c.snapshot())
                            if c.ready() and not c.prepare_pending:break
                        assert c.ready() and not c.prepare_pending and seen_gate_wait,c.snapshot()
                        assert 'READY' in c.message and c.plant.command==ZERO
                        sessions=c.sessions;assert c.prepare();assert c.sessions==sessions
                        window.controls(c.snapshot());window.toggle_button.invoke()
                        wait(lambda:c.sup.core.state=='ACTIVE')
                        spin(.5,action=lambda t:state.update(x=.2*t))
                        assert c.plant.command.vx>0
                        c.focus_lost();wait(lambda:c.plant.command==ZERO)
                        assert '失去焦点' in c.message
                        wait(lambda:c.sup.core.stop_confirmed)
                        c.focus_lost();assert c.sup.core.stop_confirmed
                        result=dict(case=case,passed=True,actual_ros=True,actual_tk_button_to_dispatch=True,
                            no_false_ready=True,missing_sign_rejected_before_reset=True,
                            idle_focus_no_stop=True,armed_active_focus_still_revokes=True,
                            repeat_prepare_did_not_reset=True,final=c.snapshot())
                        save(folder/'RESULT.json',result);results.append(result);print(case,'PASS',flush=True)
                        continue
                    finally:window.close()
                assert c.prepare();wait(c.ready)
                assert c.plant.command==ZERO and c.gate.core.output==ZERO
                assert c.engage();wait(lambda:c.sup.core.state=='ACTIVE')
                # Continuous forward then reverse, <=0.60m/s genuine input.
                samples=[]
                def move(t):
                    state['x']=.3*(1-math.cos(2.*t));state['y']=.05*math.sin(t)
                    samples.append(c.snapshot())
                spin(3.,action=move)
                assert c.sup.core.state=='ACTIVE' and not c.gate.core.fault,c.snapshot()
                assert c.plant.accepted>100 and not c.plant.rejection
                positive=any(s['output']['vx']>.05 for s in samples)
                negative=any(s['output']['vx']<-.05 for s in samples)
                assert positive and negative
                assert all(math.hypot(s['output']['vx'],s['output']['vy'])<=.7+1e-8 for s in samples)
                start=time.monotonic();result=dict(case=case,passed=False,actual_ros=True,signed_commands=c.plant.accepted)
                if case=='fast_reverse_stop':
                    c.stop('TEST Esc');wait(lambda:c.plant.command==ZERO,.15)
                    result['stop_to_signed_zero_s']=time.monotonic()-start
                    wait(lambda:c.sup.core.stop_confirmed)
                    spin(.3);assert not c.requested and c.plant.command==ZERO
                elif case=='occlusion_recover':
                    old_pose=c.plant.plant.pose;c.source.on=False
                    wait(lambda:c.sup.core.state!='ACTIVE' and c.plant.command==ZERO,.35)
                    result['loss_to_signed_zero_s']=time.monotonic()-start
                    c.source.on=True;spin(1.5)
                    assert c.plant.command==ZERO and not c.requested
                    assert c.sup.core.state=='FAULT'
                    wait(lambda:c.sup.core.stop_confirmed)
                    before_rearm=c.plant.plant.pose
                    assert c.prepare();assert c.plant.plant.pose==before_rearm
                    wait(c.ready);spin(.3)
                    assert c.plant.command==ZERO and not c.requested
                    assert c.engage();wait(lambda:c.sup.core.state=='ACTIVE');spin(.15)
                    assert c.plant.command==ZERO
                    base=state['x'];spin(.5,action=lambda t:state.update(x=base+.2*t))
                    assert c.plant.command.vx>0 and not c.gate.core.fault,c.snapshot()
                    c.stop('TEST recovered stop');wait(lambda:c.plant.command==ZERO)
                    result.update(return_did_not_resume=True,explicit_reprepare_no_motion=True,
                        new_click_restored=True,simulated_pose_preserved=True)
                elif case in ('bound_digest','bound_stamp','bound_car_stamp','unknown_source_fifo','command_accel'):
                    injected=False
                    def tamper(value):
                        nonlocal injected
                        if value['frame']['enabled']:
                            if case=='unknown_source_fifo' and injected:return value
                            if case=='bound_stamp':value['sign_stamp']-=.01
                            elif case=='bound_car_stamp':value['car_stamp']-=.003
                            elif case=='command_accel':value['frame']['command']['vx']=.5
                            else:value['sign_digest']='0'*64
                            injected=True
                        return value
                    c.sup.modify_bound=tamper
                    wait(lambda:bool(c.gate.core.fault) and c.plant.command==ZERO,.20)
                    if case=='command_accel':
                        assert c.gate.core.fault=='FINAL_ACCELERATION_LIMIT',c.snapshot()
                    else:
                        result['selected_source_failure']=c.gate.selection_failure
                        assert c.gate.selection_failure
                        if case=='unknown_source_fifo':assert c.gate.selection_failure['queued']>=2
                elif case=='signed_tamper':
                    def tamper(text):
                        v=json.loads(text);v['signature']='0'*64;return json.dumps(v)
                    c.gate.modify_wire=tamper
                    wait(lambda:bool(c.plant.rejection) and c.plant.command==ZERO,.15)
                    assert not c.prepare()
                    result['signature_rejection']=c.plant.rejection
                elif case=='unsigned_bypass':
                    c.stop('TEST pause');wait(lambda:c.sup.core.stop_confirmed)
                    pub=c.operator.create_publisher(Twist,'cmd_vel',LATEST)
                    for _ in range(12):
                        msg=Twist();msg.linear.x=.7;pub.publish(msg);spin(.01)
                    assert c.plant.command==ZERO
                    result['raw_twist_never_drives_plant']=True
                elif case=='feedback_loss':
                    c.plant.feedback_on=False
                    wait(lambda:c.plant.command==ZERO and c.sup.core.state!='ACTIVE',.45)
                elif case=='car_pose_loss':
                    c.plant.pose_on=False
                    wait(lambda:c.plant.command==ZERO and c.sup.core.state!='ACTIVE',.35)
                elif case=='ui_loss':
                    spin(.20,touch=False)
                    assert c.plant.command==ZERO and not c.requested
                elif case=='prepare_cancel_loss':
                    c.stop('TEST stop before prepare');wait(lambda:c.sup.core.stop_confirmed)
                    assert c.prepare();c.source.on=False
                    wait(lambda:not c.prepare_pending,.35)
                    c.source.on=True;spin(1.5)
                    assert c.sup.core.state not in ('ARMED','ACTIVE') and c.plant.command==ZERO
                    wait(lambda:c.sup.core.stop_confirmed)
                    assert c.prepare();wait(lambda:c.ready() and not c.prepare_pending)
                    assert c.plant.command==ZERO and not c.requested
                    result['loss_cancels_preparation_no_automatic_retry']=True
                else:
                    n=Node('conflicting_source',namespace=c.namespace,enable_rosout=False)
                    n.create_publisher(String,'rotation_packet',LATEST);c.executor.add_node(n);extra.append(n)
                    wait(lambda:c.plant.command==ZERO and c.sup.core.state!='ACTIVE',.4)
                result.update(passed=True,final=c.snapshot(),max_pending=c.max_pending)
                save(folder/'RESULT.json',result);results.append(result)
                print(case,'PASS',flush=True)
            except Exception as error:
                save(folder/'FAILURE.json',dict(error=repr(error),snapshot=c.snapshot(),events=c.events))
                raise
            finally:
                for node in extra:c.executor.remove_node(node);node.destroy_node()
                c.close()
    finally:rclpy.shutdown()
    after=protected_hashes();save(output/'protected_after.json',after)
    report=dict(passed=all(r['passed'] for r in results),cases=len(results),results=results,
        original_sources_unchanged=before==after,inputs_unchanged=inputs==input_hashes(),
        domain=189,actual_ros_chain=True,real_sign_accepted=False,physical_car_started=False,
        hardware_bridge_constructed=False,firmware_changed=False,network_settings_changed=False,
        implementation_sha256=implementation)
    save(output/'RESULT.json',report)
    assert report['original_sources_unchanged'] and report['inputs_unchanged']
    assert all(digest(Path(p))==sha for p,sha in implementation.items())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    p.add_argument('--cases',nargs='+',choices=CASES);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);run(a.output,tuple(a.cases) if a.cases else CASES)
