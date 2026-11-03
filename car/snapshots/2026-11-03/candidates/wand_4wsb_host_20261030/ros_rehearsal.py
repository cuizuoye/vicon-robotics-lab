"""Actual 4WSB ROS adapters + HMAC + MC-Link + C controller; NO hardware IO."""
import argparse
from collections import deque
from dataclasses import asdict
import gc
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
from types import SimpleNamespace as NS
from bootstrap import OUT,ROOT,config,synthetic_packet
from prepare import save
from trace_spool import TraceSpool

DOMAIN = 191
CASES = ('normal_stop','occlusion','car_lost','feedback_lost','source_digest','yaw_intent',
         'source_geometry','actuator_fault','source_conflict','signed_tamper',
         'control_timer_stall','gate_timer_stall','operator_lease_lost','candidate_gap')


def run(output,stage=7,duration=5.,cases=CASES,trace_output=None,ui_config=None,relay=True):
    if os.environ.get('ROS_DOMAIN_ID') != str(DOMAIN) or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise ValueError('private local-only domain191 required')
    output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from mecanum_wand_4wsb_control.safety_nodes import LATEST,SENSOR
    from mecanum_wand_4wsb_control.bound_history import HISTORY_QOS
    from mecanum_wand_4wsb_control.rotation_source import canonical
    from mecanum_wand_4wsb_control.operator_console import OperatorNode
    from mecanum_wand_4wsb_control.toggle_ui import ToggleOperatorModel
    from mecanum_wand_4wsb_control.auto_prepare import AutoPrepareOnce
    from mecanum_wand_4wsb_hardware.optimized_nodes import Supervisor,Gate
    from native_twin import NativeTwin
    key=secrets.token_hex(32);os.environ['WAND_4WSB_CHANNEL_KEY']=key
    c=config(stage)
    library=ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready/native/libstage0_oracle.so'
    results=[];rclpy.init(args=[],domain_id=DOMAIN)
    try:
        for case in cases:
            namespace='/host4wsb_'+secrets.token_hex(5)
            def guard(node):
                if node.get_namespace()!=namespace or node.context.get_domain_id()!=DOMAIN:
                    raise ValueError('isolated test escaped namespace/domain')
                if node.get_parameter('use_sim_time').value: raise ValueError('real deadlines required')
            runtime=NS(namespace=namespace,world='map',config=c,mode='hardware',software_only=True,
                guard=guard,yaw_offset=0.,wand_reference_margin_m=0.,
                expected=dict(car_pose='safe_native_twin',wand_pose='wand_sign_source',
                    rotation_packet='wand_sign_source',plant_feedback='safe_native_twin',
                    operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
                    cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate'))
            if relay:runtime.expected['car_pose']='wand_sign_source'
            executor=SingleThreadedExecutor();nodes=[]
            state=NS(ready=None,clicked=None,active=None,injected=None,restored=None,
                first_stopped=None,wire_fault=False,wire_used=False,source_on=True,
                geometry_fault=False,ui_on=True,stop_requested=False,stop_confirmed_at=None,
                stop_unconfirmed_ever=False)
            intervals={'control':[],'gate':[]};previous={}
            trace=deque(maxlen=200);errors=[];window=None
            gc_events=[];gc_started={}
            def gc_probe(phase,info):
                generation=info['generation']
                if phase=='start':gc_started[generation]=time.monotonic()
                else:
                    elapsed=time.monotonic()-gc_started.pop(generation,time.monotonic())
                    if elapsed>.005:
                        gc_events.append(dict(t=time.monotonic(),generation=generation,duration_ms=elapsed*1000))
            gc.callbacks.append(gc_probe)
            spool=TraceSpool(output/(case+'_TRACE.jsonl'))
            try:
                model=ToggleOperatorModel();model.focused=True;model.release(time.monotonic())
                operator=(OperatorNode(model,runtime) if ui_config is None else
                          Node('readonly_test_monitor',namespace=namespace,enable_rosout=False))
                source=Node('synthetic_vicon_fixture' if relay else 'wand_sign_source',namespace=namespace,enable_rosout=False)
                if not relay:
                    packet_pub=source.create_publisher(String,'rotation_packet',HISTORY_QOS)
                    wand_pub=source.create_publisher(PoseStamped,'wand_pose',SENSOR)
                    info_pub=source.create_publisher(String,'source_status',LATEST)
                    mode_pub=source.create_publisher(String,'mode_claim',LATEST)
                original=Node.create_publisher
                def intercepted(node,msg_type,topic,*a,**kw):
                    pub=original(node,msg_type,topic,*a,**kw)
                    if topic!='candidate' or node.get_name()!='safe_wand_supervisor': return pub
                    class FaultWire:
                        def publish(self,msg):
                            if case=='candidate_gap' and state.wire_fault and state.restored is None:
                                value=json.loads(msg.data)
                                if value['frame']['enabled']:
                                    state.wire_used=True
                                    return  # Private fault injection ONLY; control timer keeps running.
                            if state.wire_fault and not state.wire_used:
                                value=json.loads(msg.data)
                                if value['frame']['enabled']:
                                    if case=='source_digest':value['sign_digest']='f'*64
                                    else:value['frame']['goal']['yaw']+=.0005
                                    msg=String(data=canonical(value));state.wire_used=True
                            pub.publish(msg)
                    return FaultWire()
                Node.create_publisher=intercepted
                class TimedSupervisor(Supervisor):
                    def tick(self):
                        now=time.monotonic()
                        if 'control' in previous:intervals['control'].append(now-previous['control'])
                        previous['control']=now
                        return super().tick()
                try: sup=TimedSupervisor(runtime)
                finally: Node.create_publisher=original
                class TimedGate(Gate):
                    def create_timer(self,period,callback,*a,**kw):
                        timer=super().create_timer(period,callback,*a,**kw)
                        if period==.01 and getattr(callback,'__name__','')=='tick':self.output_timer=timer
                        return timer
                    def tick(self):
                        now=time.monotonic()
                        if 'gate' in previous:intervals['gate'].append(now-previous['gate'])
                        previous['gate']=now
                        return super().tick()
                gate=TimedGate(runtime)
                twin=NativeTwin(runtime,library,key,publish_car=not relay)
                nodes=[source,operator,sup,gate,twin]
                if relay:
                    from mecanum_wand_4wsb_hardware.source_relay import SourceRelay
                    class SyntheticReader:
                        def __init__(self):self.value=None
                        def start(self):pass
                        def poll(self):return self.value
                        def close(self):pass
                    reader=SyntheticReader()
                    relay_node=SourceRelay(runtime,reader)
                    nodes.append(relay_node)
                for node in nodes:executor.add_node(node)
                ui_process=None;ui_log=None
                if ui_config is not None:
                    ui_log=(output/'ui_process.log').open('w')
                    ui_process=subprocess.Popen([sys.executable,str(Path(__file__).with_name('ui_probe.py')),
                        '--namespace',namespace,'--stage',str(stage),'--duration',str(duration),
                        '--mode',ui_config['mode'],'--output',str(output/'ui')],
                        env=dict(os.environ,DISPLAY=ui_config['display'],WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1'),
                        stdout=ui_log,stderr=subprocess.STDOUT)
                policy=AutoPrepareOnce();last_fixture=0.
                def fixture():
                    from mecanum_wand_4wsb_control.safety_nodes import pose_message
                    now=time.monotonic();ros=source.get_clock().now().nanoseconds*1e-9
                    t=0. if state.active is None else now-state.active
                    if stage==0:
                        x,y,turn=.035*math.sin(t*.8),.025*math.sin(t*.7),.05*math.sin(t*.7)
                    else:
                        # Finite 1:1 hand trajectory. Do not call an unrealizable
                        # discontinuous position step a motor tracking test.
                        x,y,turn=.23*math.sin(t*1.4),.16*math.sin(t*1.1),min(.25,c.max_yaw_rate_radps*.3)*math.sin(t*.7)
                    # Deliberate pickup/reorientation while ARMED: origins must
                    # be captured at the first active neutral frame, not prepare.
                    pickup=.1 if state.ready is not None else 0.
                    p=synthetic_packet(10.+pickup+x,10.+y,.3+turn,now,ros,z=1.3)
                    value=None
                    if state.source_on:
                        value=p.payload()
                        if state.geometry_fault:value['samples'][1]['position']=[0.,0.,0.]
                        if not relay:
                            packet_pub.publish(String(data=canonical(value)))
                            wand_pub.publish(pose_message(p.pose(now,ros),source.get_clock().now().to_msg(),world='map'))
                    if relay:
                        from mecanum_wand_4wsb_control.core import Pose
                        car=Pose(*twin.controller.pose[:2],c.ground_z_m,twin.controller.pose[2])
                        reader.value=dict(kind='data',issued=now,packet=value,
                            car=dict(pose=asdict(car),stamp=ros,roll=0.,pitch=0.) if twin.pose_on else None,
                            conflicts=[],rejected={})
                    else:
                        info_pub.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,rejected={},conflicts=[]))))
                        mode_pub.publish(String(data='wand'))
                source.create_timer(.01,fixture)
                def operation():
                    now=time.monotonic()
                    if ui_config is None and state.ui_on:
                        model.tick(now);operator.heartbeat()
                    if ui_config is None and policy.poll(model,now,space_down=False,
                            service_ready=operator.service_clients['start'].service_is_ready(),input_issue=operator.input_hint()):
                        if not operator.request('start'):errors.append('start service request rejected')
                    if (policy.phase=='READY' if ui_config is None else sup.core.state=='ARMED') and state.ready is None:state.ready=now
                    if ui_config is None and state.ready and state.clicked is None and now-state.ready>.45 and model.start_ready(now):
                        model.press(now);model.release(now+.0001);state.clicked=now
                    if sup.core.state=='ACTIVE' and state.active is None:state.active=now
                    if state.active is not None and state.injected is None:
                        if ((gate.core.fault or gate.core.hard_fault or sup.core.state not in ('ACTIVE','ARMED'))
                                and not (ui_config is not None and now-state.active>duration-.10)):
                            errors.append('unexpected active interruption: '+sup.core.reason+' / '+gate.core.fault)
                        if now-state.active>=duration:
                            state.injected=now
                            if case=='normal_stop':
                                if ui_config is None:
                                    model.end_trial(now,'SOFTWARE Esc');operator.request('stop')
                                state.stop_requested=True
                            elif case=='occlusion':state.source_on=False
                            elif case=='car_lost':twin.pose_on=False
                            elif case=='feedback_lost':twin.feedback_on=False
                            elif case in ('source_digest','yaw_intent','candidate_gap'):state.wire_fault=True
                            elif case=='source_geometry':state.geometry_fault=True
                            elif case=='actuator_fault':twin.actuator_fault=0x20
                            elif case=='source_conflict':
                                dupe=Node('conflicting_source',namespace=namespace,enable_rosout=False)
                                dupe.create_publisher(String,'rotation_packet',HISTORY_QOS)
                                nodes.append(dupe);executor.add_node(dupe)
                            elif case=='signed_tamper':twin.on_signed(String(data='{"payload":{},"signature":"bad"}'))
                            elif case=='control_timer_stall':
                                for timer in sup.timers:
                                    if getattr(timer.callback,'__name__','')=='tick':timer.cancel()
                            elif case=='gate_timer_stall':gate.output_timer.cancel()
                            elif case=='operator_lease_lost':state.ui_on=False
                    if state.injected is not None:
                        stopped=(not twin.controller.status['permitted'] and not twin.output_nonzero)
                        if stopped and state.first_stopped is None:state.first_stopped=now
                        if sup.core.state=='STOP_UNCONFIRMED':state.stop_unconfirmed_ever=True
                        if stopped and sup.core.stop_confirmed and state.stop_confirmed_at is None:
                            state.stop_confirmed_at=now
                        if state.restored is None and now-state.injected>.55:
                            state.restored=now;state.source_on=True;state.geometry_fault=False
                            twin.pose_on=twin.feedback_on=True
                            if case=='control_timer_stall':
                                for timer in sup.timers:
                                    if getattr(timer.callback,'__name__','')=='tick':timer.reset()
                            if case=='gate_timer_stall':gate.output_timer.reset()
                        if state.restored is not None and (twin.enabled_on_wire or twin.output_nonzero):
                            errors.append('automatically resumed after injection')
                    row=dict(t=now,state=sup.core.state,reason=sup.core.reason,
                            gate=gate.core.fault,hard=gate.core.hard_fault,epoch=sup.core.epoch,
                            sign=asdict(sup.selected.pose(now,source.get_clock().now().nanoseconds*1e-9))
                                if sup.selected and now-sup.selected.received<.19 else None,
                            command=asdict(twin.policy.command),body=twin.controller.status['body_si'],
                            car=twin.controller.pose.copy(),duty=twin.controller.status['raw_duty'],
                            native_fault=twin.controller.wheels.fault_bits)
                    trace.append(row);spool.emit(row)
                operator.create_timer(.01,operation)
                started=time.monotonic();limit=duration+12.
                while time.monotonic()-started<limit:
                    executor.spin_once(timeout_sec=.001)
                    if state.restored is not None and time.monotonic()-state.restored>1.2:break
                    if errors: break
                if state.active is None:errors.append('never ACTIVE: '+sup.core.reason+' / '+gate.core.fault+' / '+repr(model.status))
                if state.injected is None:errors.append('test action never reached')
                if state.first_stopped is None:errors.append('no disabled zero output after test action')
                if gate.commit_count==0:errors.append('validated event commit was not exercised')
                if twin.first_error and case!='signed_tamper':errors.append('unexpected HMAC rejection '+twin.first_error)
                if twin.controller.wheels.fault_bits:errors.append('native controller fault '+str(twin.controller.wheels.fault_bits))
                if case=='gate_timer_stall' and not gate.core.hard_fault:errors.append('stalled independent gate timer did not hard latch')
                if not twin.metrics['stage_specific_guard']:errors.append('wrong stage wheel guard')
                if twin.metrics['peak_duty']==0 or twin.metrics['peak_measured_mps']<=.005:
                    errors.append('fixture never exercised actual C motor output and encoder motion')
                stop_qualification_issue=state.stop_confirmed_at is None or state.stop_unconfirmed_ever
                # Loss/tamper of sensing may correctly make physical stopping
                # unconfirmable. Functional success is ZERO + revoked + no
                # auto-resume, NOT invented mechanical stopping qualification.
                if case=='normal_stop' and stop_qualification_issue:
                    errors.append('software encoder stop confirmation missing or timed out')
                if case=='candidate_gap' and (not state.wire_used or 'COMMAND_TIMEOUT' not in gate.core.fault):
                    errors.append('candidate gap injection did not exercise original100ms timeout')
                time_limit_expected=(duration>=599.8 and 'PILOT_TIME_LIMIT_600S' in gate.core.fault)
                if case=='normal_stop' and sup.core.state!='PAUSED' and not time_limit_expected:
                    errors.append('normal stop did not finish in PAUSED')
                if ui_config is not None:
                    try:ui_process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:errors.append('UI failed to close after test stop')
                    resultfile=output/'ui/RESULT.json'
                    if not resultfile.exists() or not json.loads(resultfile.read_text())['passed']:
                        errors.append('private UI acceptance did not pass')
                def summary(values):
                    values=sorted(values)
                    return dict(samples=len(values),p95_ms=values[int((len(values)-1)*.95)]*1000,
                        max_ms=max(values)*1000) if values else None
                result=dict(case=case,stage=stage,passed=not errors,errors=errors,active_duration_s=(state.injected-state.active)
                    if state.injected and state.active else None,stop_output_after_injection_s=(state.first_stopped-state.injected)
                    if state.first_stopped and state.injected else None,final_state=sup.core.state,
                    final_reason=sup.core.reason,gate_fault=gate.core.fault,gate_hard_fault=gate.core.hard_fault,
                    validated_event_commits=gate.commit_count,signed_accepted=twin.accepted,
                    native_metrics=twin.metrics,software_stop_confirmed_ever=state.stop_confirmed_at is not None,
                    software_stop_confirmed_now=sup.core.stop_confirmed,
                    stop_confirmation_timed_out=state.stop_unconfirmed_ever,
                    stopping_qualified=not stop_qualification_issue,
                    timer_intervals={k:summary(v) for k,v in intervals.items()},
                    hardware_output=False,actual_ros=True,actual_c=True,
                    encoder_plant='historical uncalibrated PWM model, not hardware qualification')
                logging=spool.close();spool=None
                result['logging']=logging;result['gc_pauses_over5ms']=gc_events
                if logging['dropped'] or not logging['writer_stopped'] or logging['returncode']:
                    result['errors'].append('test logging incomplete');result['passed']=False
                results.append(result);save(output/(case+'_RESULT.json'),result)
                save(output/(case+'_LAST200.json'),list(trace))
                print(json.dumps(result,ensure_ascii=False),flush=True)
                if errors:break
            finally:
                if spool is not None:spool.close()
                gc.callbacks.remove(gc_probe)
                if 'twin' in locals():twin.close()
                if 'ui_process' in locals() and ui_process is not None and ui_process.poll() is None:
                    ui_process.terminate()
                    try:ui_process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:ui_process.kill();ui_process.wait(timeout=3.)
                if 'ui_log' in locals() and ui_log is not None:ui_log.close()
                executor.shutdown()
                for node in reversed(nodes):node.destroy_node()
        report=dict(passed=bool(results) and len(results)==len(cases) and all(r['passed'] for r in results),
            stage=stage,cases=results,private_domain=DOMAIN,hardware_socket_opened=False,
            hardware_bridge_constructed=False,firmware_flashed=False,network_changed=False,
            original_entry_changed=False,real_sign_user_acceptance=False)
        report['production_source_relay_exercised']=relay
        save(output/'RESULT.json',report)
        return report
    finally:rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',type=int,choices=range(8),default=7)
    p.add_argument('--duration',type=float,default=2.)
    p.add_argument('--case',choices=CASES,action='append')
    p.add_argument('--ui-mode',choices=('normal','full_geometry'))
    a=p.parse_args();ui=dict(mode=a.ui_mode,display=os.environ['DISPLAY']) if a.ui_mode else None
    if ui and a.case!=['normal_stop']:raise ValueError('UI comparison is normal_stop only')
    r=run(a.output.resolve(),a.stage,a.duration,tuple(a.case) if a.case else CASES,ui_config=ui)
    raise SystemExit(0 if r['passed'] else 1)
