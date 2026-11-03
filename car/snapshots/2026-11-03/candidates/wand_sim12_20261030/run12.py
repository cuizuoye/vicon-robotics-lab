"""Real Sign or explicit synthetic hand -> private signed nominal plant only."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
from profile12 import ROOT,runtime,protected,save,contract
from live_sign import SoftwarePassiveReader,SoftwareCarReader,qualify_reader,ensure_receiver
from reader12 import Reader
from geometry12 import install_geometry,synthetic_packet
from common import diagnostic as d

HERE=Path(__file__).resolve().parent

def run(out,synthetic=False,scenario='normal'):
    if scenario not in ('normal','occlusion','operator_gap') or (not synthetic and scenario!='normal'):
        raise ValueError('fault scenarios are explicit synthetic-only tests')
    install_geometry()
    namespace='/host4wsb_'+secrets.token_hex(5);rt=runtime(namespace)
    out.mkdir(parents=True,exist_ok=False);before=protected();receiver=reader=spool=None
    logs=[];processes=[];nodes=[];executor=None;initialized=False;finished=False
    key=secrets.token_hex(32);os.environ['WAND_4WSB_CHANNEL_KEY']=key;os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    def finish(*_):
        nonlocal finished
        finished=True
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,finish)
    try:
        if not synthetic:
            receiver=ensure_receiver(out);reader=Reader();reader.start()
            ready=qualify_reader(reader,6.);save(out/'LIVE_READINESS.json',ready)
            if not ready['ready']:raise ValueError('fresh coherent Sign not ready; no fake READY')
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.node import Node
        from std_msgs.msg import String
        from mecanum_wand_4wsb_control.core import Pose
        from mecanum_wand_4wsb_control.safety_nodes import LATEST
        from mecanum_wand_4wsb_control.operator_console import OperatorNode,OperatorModel
        from mecanum_wand_4wsb_control.auto_prepare import AutoPrepareOnce
        from mecanum_wand_4wsb_control.novice_ui import acknowledge_physical_release
        from mecanum_wand_4wsb_hardware.source_relay import SourceRelay
        from plant12 import SoftwarePlant
        from trace_spool import TraceSpool
        from split_runtime import stop_acknowledged
        from random_rotation import LargeGesture
        rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO);initialized=True
        executor=SingleThreadedExecutor()
        plant=SoftwarePlant(rt,key);nodes.append(plant)
        monitor=Node('sim12_observer',namespace=namespace,enable_rosout=False);nodes.append(monitor)
        status={};gate={};states=[];last_state=None
        monitor.create_subscription(String,'supervisor_status',lambda m:status.update(json.loads(m.data)),LATEST)
        monitor.create_subscription(String,'gate_status',lambda m:gate.update(json.loads(m.data)),LATEST)
        active=stopping=None;acknowledged=False;errors=[];gesture=LargeGesture(20261030,1.)
        revoked_at=None;resumed=False;injected=False
        duration=gesture.duration if scenario=='normal' else 18.
        if synthetic:
            class SyntheticReader:
                last=0.;value=None;provenance=dict(synthetic=True)
                def start(self):pass
                def close(self):pass
                def poll(self):
                    now=time.monotonic();ros=time.time()
                    if now-self.last>=.0099:
                        t=0. if active is None else now-active;x,y,turn=gesture(t,6,'normal')
                        self.value=dict(packet=synthetic_packet(10.1+x,10.+y,.3+turn,now,ros).payload(),
                            car=None,issued=now,conflicts=[],rejected={});self.last=now
                        if scenario=='occlusion' and 10.<=t<13.:self.value['packet']=None
                    return self.value
            reader=SyntheticReader()
        adapter=SoftwareCarReader(reader,lambda:asdict(Pose(*plant.model.pose[:2],rt.config.ground_z_m,plant.model.pose[2])),
            lambda:monitor.get_clock().now().nanoseconds*1e-9)
        source=SourceRelay(rt,adapter);nodes.append(source)
        model=operator=None
        if synthetic:
            model=OperatorModel();model.focused=True;model.release(time.monotonic());operator=OperatorNode(model,rt);nodes.append(operator)
            policy=AutoPrepareOnce();clicked=False
        for node in nodes:executor.add_node(node)
        env=dict(os.environ,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')
        for role in ('supervisor','gate'):
            log=(out/(role+'.log')).open('x');logs.append(log)
            processes.append(subprocess.Popen([sys.executable,str(HERE/'node12.py'),'--namespace',namespace,'--role',role],env=env,stdout=log,stderr=log))
        ui=None
        if not synthetic:
            log=(out/'console.log').open('x');logs.append(log)
            ui=subprocess.Popen([sys.executable,str(HERE/'console12.py'),'--namespace',namespace,'--output',str(out/'ui')],env=env,stdout=log,stderr=log)
            processes.append(ui)
        save(out/'CONTRACT.json',dict(**contract(),synthetic=synthetic,scenario=scenario,namespace=namespace))
        spool=TraceSpool(out/'TRACE.jsonl');started=time.monotonic();last_trace=last_op=0.
        while not finished and time.monotonic()-started<(duration+15. if synthetic else 720.):
            executor.spin_once(timeout_sec=.001);now=time.monotonic();state=status.get('state')
            if state!=last_state:
                last_state=state;states.append(dict(t=now,state=state,reason=status.get('reason')));print(json.dumps(states[-1]),flush=True)
            elapsed=0. if active is None else now-active
            if synthetic and scenario!='normal' and elapsed>=10.:injected=True
            input_gap=scenario=='operator_gap' and 10.<=elapsed<10.35
            if operator and now-last_op>.009 and not input_gap:
                model.tick(now);operator.heartbeat();acknowledge_physical_release(model,False,now);last_op=now
                if policy.poll(model,now,space_down=False,service_ready=operator.service_clients['start'].service_is_ready(),input_issue=operator.input_hint()):operator.request('start')
                if not clicked and policy.phase=='READY' and model.start_ready(now):model.press(now);model.release(now+.0001);clicked=True
            if state=='ACTIVE' and active is None:active=now
            if synthetic and active is not None:
                if scenario!='normal' and injected and state not in ('ACTIVE','ARMED') and revoked_at is None:
                    revoked_at=now;stopping=now
                if revoked_at is not None and state=='ACTIVE':resumed=True
                if scenario=='normal' and state not in ('ACTIVE','ARMED') and stopping is None:
                    errors.append('unexpected interruption: '+str(status.get('reason')));stopping=now
                if now-active>=duration and stopping is None:
                    stopping=now;model.end_trial(now,'SOFTWARE Esc');operator.request('stop')
                if stopping is not None:
                    acknowledged|=stop_acknowledged(status,stopping)
                    if scenario=='normal' and now-stopping>2.3:break
                if scenario!='normal' and now-active>=duration:break
            if now-last_trace>.02:
                spool.emit(dict(t=now,state=state,reason=status.get('reason'),supervisor=status.copy(),gate=gate.copy(),
                    car=plant.model.pose.copy(),velocity=asdict(plant.model.velocity),command=asdict(plant.policy.command),
                    reference=gesture.last.copy() if synthetic and gesture.last else None,car01_readonly=adapter.last_car_observed,
                    policy_fault=plant.policy.fault,hardware_output=False));last_trace=now
            if ui is not None and ui.poll() is not None:break
            if any(p.poll() is not None for p in processes[:2]):raise ValueError('critical simulation process exited')
        if synthetic:
            if active is None:errors.append('never ACTIVE')
            if not acknowledged:errors.append('no fresh standstill confirmation')
            if scenario!='normal' and (not injected or revoked_at is None):errors.append('expected revoke missing')
            if resumed:errors.append('automatic resume after fault')
        plant.close();logged=spool.close();spool=None
        if logged['dropped'] or logged['returncode']:errors.append('trace incomplete')
        if plant.policy.hard_fault:errors.append('signed authority hard fault')
        save(out/'RESULT.json',dict(**contract(),synthetic=synthetic,scenario=scenario,states=states,errors=errors,
            fault_injected=injected,revoke_after_injection_s=revoked_at-(active+10.) if revoked_at else None,
            automatically_resumed=resumed,
            automated_passed=not errors if synthetic else None,human_acceptance_complete=False,
            stop_confirmed=acknowledged if synthetic else status.get('stop_confirmed'),logging=logged,metrics=plant.metrics))
    except Exception as error:
        save(out/'ERROR.json',dict(error=repr(error),hardware_output=False));raise
    finally:
        if 'plant' in locals():plant.close()
        if spool is not None:spool.close()
        for process in processes:d.terminate_owned(process)
        if executor is not None:executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        if initialized:
            import rclpy
            rclpy.shutdown()
        if reader is not None:reader.close()
        if receiver is not None:d.terminate_owned(receiver)
        for log in logs:log.close()
        save(out/'PROTECTED.json',dict(unchanged=before==protected(),hardware_output=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--synthetic',action='store_true')
    p.add_argument('--scenario',choices=('normal','occlusion','operator_gap'),default='normal')
    a=p.parse_args();run(a.output,a.synthetic,a.scenario)
