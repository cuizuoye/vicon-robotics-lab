"""Deployment-shaped SOFTWARE experiment: supervisor, gate, UI each separate.

Only the synthetic source relay and native C actuator share the fixture loop.
All graph, raw source, signed output, timing and one-writer guards stay active.
"""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
from bootstrap import ROOT,config,synthetic_packet
from prepare import save,HERE
from split_runtime import runtime,stop_acknowledged,hand_gesture
from trace_spool import TraceSpool


def run(out,stage,duration,ui_mode=None,gesture='normal'):
    namespace='/host4wsb_'+secrets.token_hex(5);rt=runtime(namespace,stage)
    out.mkdir(parents=True,exist_ok=False);os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    key=secrets.token_hex(32);os.environ['WAND_4WSB_CHANNEL_KEY']=key
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from mecanum_wand_4wsb_control.safety_nodes import LATEST
    from mecanum_wand_4wsb_control.core import Pose
    from mecanum_wand_4wsb_control.operator_console import OperatorNode
    from mecanum_wand_4wsb_control.toggle_ui import ToggleOperatorModel
    from mecanum_wand_4wsb_control.auto_prepare import AutoPrepareOnce
    from mecanum_wand_4wsb_control.novice_ui import acknowledge_physical_release
    from mecanum_wand_4wsb_hardware.source_relay import SourceRelay
    from native_twin import NativeTwin
    rclpy.init(args=[],domain_id=191);executor=SingleThreadedExecutor()
    procs=[];logs=[];nodes=[];spool=None;ui=None
    source_status={};gate_status={};active=stopped=requested=confirmed=None
    model=ToggleOperatorModel();model.focused=True;model.release(time.monotonic())
    errors=[];ever_nonzero_before_click=False;unexpected=None;last_state='';policy=AutoPrepareOnce()
    try:
        twin=NativeTwin(rt,ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready/native/libstage0_oracle.so',key,publish_car=False)
        monitor=Node('readonly_split_monitor',namespace=namespace,enable_rosout=False)
        monitor.create_subscription(String,'supervisor_status',lambda m:source_status.update(json.loads(m.data)),LATEST)
        monitor.create_subscription(String,'gate_status',lambda m:gate_status.update(json.loads(m.data)),LATEST)
        def packet_value():
            now=time.monotonic();ros=monitor.get_clock().now().nanoseconds*1e-9
            t=0. if active is None else now-active
            x,y,turn=hand_gesture(t,stage,gesture)
            if turn is None:turn=min(.25,rt.config.max_yaw_rate_radps*.3)*math.sin(t*.7)
            p=synthetic_packet(10.1+x,10.+y,.3+turn,now,ros,z=1.3)
            pose=Pose(*twin.controller.pose[:2],rt.config.ground_z_m,twin.controller.pose[2])
            return dict(kind='data',issued=now,packet=p.payload(),
                car=dict(pose=asdict(pose),stamp=ros,roll=0.,pitch=0.),conflicts=[],rejected={})
        class Reader:
            def __init__(self):self.value=None;self.last=-math.inf
            def start(self):pass
            def close(self):pass
            def poll(self):
                now=time.monotonic()
                if now-self.last>=.0099:self.value=packet_value();self.last=now
                return self.value
        source=SourceRelay(rt,Reader());nodes=[source,twin,monitor]
        operator=None
        if ui_mode is None:
            operator=OperatorNode(model,rt);nodes.append(operator)
        for node in nodes:executor.add_node(node)
        env=dict(os.environ,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')
        for role in ('supervisor','gate'):
            log=(out/(role+'.log')).open('x');logs.append(log)
            p=subprocess.Popen([sys.executable,str(HERE/'split_node.py'),'--namespace',namespace,
                '--stage',str(stage),'--role',role,'--output',str(out/(role+'_TIMING.json'))],
                env=env,stdout=log,stderr=subprocess.STDOUT);procs.append(p)
        if ui_mode:
            log=(out/'observer_process.log').open('x');logs.append(log)
            observer=subprocess.Popen([sys.executable,str(HERE/'observer_probe.py'),'--namespace',namespace,
                '--stage',str(stage),'--output',str(out/'observer')],
                env=env,stdout=log,stderr=subprocess.STDOUT);procs.append(observer)
            log=(out/'ui_process.log').open('x');logs.append(log)
            ui=subprocess.Popen([sys.executable,str(HERE/'ui_probe.py'),'--namespace',namespace,
                '--stage',str(stage),'--duration',str(duration),'--mode',ui_mode,'--output',str(out/'ui')],
                env=env,stdout=log,stderr=subprocess.STDOUT);procs.append(ui)
        spool=TraceSpool(out/'normal_stop_TRACE.jsonl')
        started=time.monotonic();last_op=last_trace=0.;clicked=False
        while time.monotonic()-started<duration+14.:
            executor.spin_once(timeout_sec=.001);now=time.monotonic()
            state=source_status.get('state','STANDBY')
            if state!=last_state:
                print(json.dumps(dict(t=now,state=state,reason=source_status.get('reason'),gate=gate_status.get('fault'))),flush=True)
                last_state=state
            if operator and now-last_op>.009:
                model.tick(now);operator.heartbeat();last_op=now
                acknowledge_physical_release(model,False,now)
                if policy.poll(model,now,space_down=False,service_ready=operator.service_clients['start'].service_is_ready(),input_issue=operator.input_hint()):operator.request('start')
                if not clicked and policy.phase=='READY' and model.start_ready(now):
                    model.press(now);model.release(now+.0001);clicked=True
            if state=='ACTIVE' and active is None:active=now
            if active is None and twin.output_nonzero:ever_nonzero_before_click=True
            if active is not None:
                elapsed=now-active
                if elapsed<duration-.15 and state not in ('ACTIVE','ARMED'):
                    unexpected=dict(elapsed_s=elapsed,state=state,reason=source_status.get('reason'),gate=gate_status.get('fault'))
                    errors.append('unexpected interruption');break
                if elapsed>=duration and requested is None:
                    requested=now
                    if operator:model.end_trial(now,'SOFTWARE Esc');operator.request('stop')
                if requested is not None:
                    if not twin.enabled_on_wire and not twin.output_nonzero and stopped is None:stopped=now
                    if stop_acknowledged(source_status,requested) and confirmed is None:confirmed=now
                    if state=='STOP_UNCONFIRMED':errors.append('software stop confirmation deadline expired');break
                    if now-requested>2.3 and stopped is not None and confirmed is not None:break
            critical=procs[:3] if ui_mode else procs[:2]
            if any(p.poll() is not None for p in critical):errors.append('critical child exited');break
            if now-last_trace>.009:
                spool.emit(dict(t=now,state=state,reason=source_status.get('reason'),gate=gate_status.get('fault'),
                    command=asdict(twin.policy.command),body=twin.controller.status['body_si'],
                    car=twin.controller.pose.copy(),duty=twin.controller.status['raw_duty'],
                    native_fault=twin.controller.wheels.fault_bits));last_trace=now
        twin.close()
        if active is None or requested is None:errors.append('active period not completed')
        if stopped is None or confirmed is None:errors.append('zero output/stop acknowledgement missing')
        if ever_nonzero_before_click:errors.append('output before new click')
        if twin.first_error or twin.controller.wheels.fault_bits:errors.append('signature/native fault')
        if ui is not None:
            try:ui.wait(timeout=4.)
            except subprocess.TimeoutExpired:errors.append('UI shutdown not completed')
            u=out/'ui/RESULT.json'
            if not u.exists() or not json.loads(u.read_text())['passed']:errors.append('UI acceptance incomplete')
        for p in (procs[:3] if ui_mode else procs[:2]):
            p.terminate();p.wait(timeout=4.)
        timing={role:json.loads((out/(role+'_TIMING.json')).read_text()) for role in ('supervisor','gate')}
        logged=spool.close();spool=None
        if logged['dropped'] or logged['returncode']:errors.append('logging incomplete')
        result=dict(passed=not errors,errors=errors,stage=stage,requested_s=duration,
            active_duration_s=requested-active if requested and active else None,
            stop_output_after_injection_s=stopped-requested if stopped and requested else None,
            stop_confirmed=confirmed is not None,final_state=source_status.get('state'),
            unexpected=unexpected,timer_intervals=timing,logging=logged,native_metrics=twin.metrics,
            hardware_output=False,actual_ros=True,actual_c=True,deployment_shaped_processes=True,
            gesture_recipe=gesture,
            actual_production_observer=bool(ui_mode),
            hardware_bridge_constructed=False,firmware_flashed=False,network_changed=False)
        save(out/'normal_stop_RESULT.json',result);save(out/'RESULT.json',dict(passed=result['passed'],cases=[result],
            actual_ros_c=True,deployment_shaped_processes=True,hardware_output=False))
        print(json.dumps(result),flush=True);return result
    finally:
        if 'twin' in locals():twin.close()
        if spool is not None:spool.close()
        for p in procs:
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=2.)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2.)
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        for log in logs:log.close()
        rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',type=int,choices=range(8),default=7);p.add_argument('--duration',type=float,default=600.)
    p.add_argument('--ui-mode',choices=('normal','full_geometry'));p.add_argument('--case',choices=('normal_stop',))
    p.add_argument('--gesture',choices=('normal','demo'),default='normal')
    a=p.parse_args();raise SystemExit(0 if run(a.output.resolve(),a.stage,a.duration,a.ui_mode,a.gesture)['passed'] else 1)
