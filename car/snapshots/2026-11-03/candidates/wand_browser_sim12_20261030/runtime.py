"""Same sealed SIM12 controller, source binding, signed gate and nominal plant."""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
SIM=ROOT/'candidates/wand_sim12_20261030'
sys.path.insert(0,str(SIM))
from profile12 import runtime,save,protected,contract


class Rig:
    def __init__(self,channel,session,synthetic=False,occlusion=False):
        self.channel=channel;self.session=session;self.synthetic=synthetic
        if occlusion and not synthetic:raise ValueError('fault injection requires synthetic software input')
        self.occlusion=occlusion
        self.reader=self.receiver=self.spool=self.executor=None
        self.nodes=[];self.processes=[];self.logs=[];self.initialized=False
        self.active_since=None;self.activation_ever=False;self.stopping=None;self.last_trace=0.;self.closing=False
        self.status={};self.gate={};self.transitions=[];self.last_state=None
        self.before=protected()
        try:
            self.setup();self.warmup()
        except BaseException:
            self.dispose();raise

    def setup(self):
        from entry12 import check
        check()
        from geometry12 import install_geometry,synthetic_packet
        install_geometry()
        if not self.synthetic:
            from reader12 import Reader
            from live_sign import ensure_receiver,qualify_reader
            self.receiver=ensure_receiver(self.session);self.reader=Reader();self.reader.start()
            ready=qualify_reader(self.reader,6.);save(self.session/'LIVE_READINESS.json',ready)
            if not ready['ready']:raise ValueError('Sign没有足够的新鲜完整帧；不会显示假READY')
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.node import Node
        from std_msgs.msg import String
        from mecanum_wand_4wsb_control.core import Pose
        from mecanum_wand_4wsb_control.safety_nodes import LATEST
        from mecanum_wand_4wsb_control.toggle_ui import ToggleOperatorModel
        from mecanum_wand_4wsb_hardware.source_relay import SourceRelay
        from live_sign import SoftwareCarReader
        from random_rotation import LargeGesture
        from plant12 import SoftwarePlant
        from browser_operator import BrowserOperator
        from trace_spool import TraceSpool
        self.rclpy=rclpy;self.rt=runtime('/host4wsb_'+secrets.token_hex(5))
        key=secrets.token_hex(32);os.environ['WAND_4WSB_CHANNEL_KEY']=key
        rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO);self.initialized=True
        self.executor=SingleThreadedExecutor()
        self.plant=SoftwarePlant(self.rt,key);self.nodes.append(self.plant)
        self.monitor=Node('browser_sim12_observer',namespace=self.rt.namespace,enable_rosout=False);self.nodes.append(self.monitor)
        self.monitor.create_subscription(String,'supervisor_status',lambda m:self.status.update(json.loads(m.data)),LATEST)
        self.monitor.create_subscription(String,'gate_status',lambda m:self.gate.update(json.loads(m.data)),LATEST)
        self.gesture=LargeGesture(20261030,1.)
        if self.synthetic:
            rig=self
            class SyntheticReader:
                last=0.;value=None;provenance=dict(synthetic=True)
                def poll(self):
                    now=time.monotonic();ros=time.time()
                    if now-self.last>=.0099:
                        elapsed=0. if rig.active_since is None else now-rig.active_since
                        x,y,yaw=rig.gesture(elapsed,6,'normal')
                        self.value=dict(packet=synthetic_packet(10.1+x,10.+y,.3+yaw,now,ros).payload(),
                            car=None,issued=now,conflicts=[],rejected={});self.last=now
                        if rig.occlusion and 2.5<=elapsed<5.5:self.value['packet']=None
                    return self.value
                def close(self):pass
            self.reader=SyntheticReader()
        self.adapter=SoftwareCarReader(self.reader,
            lambda:asdict(Pose(*self.plant.model.pose[:2],self.rt.config.ground_z_m,self.plant.model.pose[2])),
            lambda:self.monitor.get_clock().now().nanoseconds*1e-9)
        source=SourceRelay(self.rt,self.adapter);self.nodes.append(source)
        for node in self.nodes:self.executor.add_node(node)
        self.model=ToggleOperatorModel();self.operator=BrowserOperator(self.model,self.rt,self.channel)
        env=dict(os.environ,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')
        for role in ('supervisor','gate'):
            log=(self.session/(role+'.log')).open('x');self.logs.append(log)
            self.processes.append(subprocess.Popen([sys.executable,str(SIM/'node12.py'),'--namespace',self.rt.namespace,'--role',role],
                env=env,stdout=log,stderr=log))
        self.spool=TraceSpool(self.session/'TRACE.jsonl')
        save(self.session/'CONTRACT.json',dict(contract(),frontend='LOCAL_BROWSER_SIM12',synthetic=self.synthetic,
             domain=191,namespace=self.rt.namespace,no_fixed_fence=True,car01_feedback_used=False))

    def warmup(self):
        """Cold DDS setup occurs BEFORE browser input exists; zero authority."""
        from mecanum_wand_4wsb_control.auto_prepare import AutoPrepareOnce
        started=last=time.monotonic();stable=None;maximum=0.
        while time.monotonic()-started<20.:
            self.step(time.monotonic());now=time.monotonic();gap=now-last;last=now
            maximum=max(maximum,gap)
            if self.model.held or self.activation_ever:raise ValueError('warmup must never acquire authority')
            if AutoPrepareOnce.zero_baseline_qualified(self.model,now) and gap<.05:
                if stable is None:stable=now
                if now-stable>=.75:
                    save(self.session/'WARMUP.json',dict(ready=True,elapsed_s=now-started,
                        maximum_cold_step_s=maximum,stable_zero_window_s=now-stable,
                        browser_owner_exists=False,authority_acquired=False,hardware_output=False))
                    return
            else:stable=None
            time.sleep(.005)
        save(self.session/'WARMUP.json',dict(ready=False,elapsed_s=time.monotonic()-started,
            maximum_cold_step_s=maximum,status=self.status,gate=self.gate,hardware_output=False))
        raise ValueError('失能零速冷启动自检未就绪；不开放浏览器输入')

    def step(self,now):
        begin=time.monotonic()
        for _ in range(48):
            self.executor.spin_once(timeout_sec=0.)
            if time.monotonic()-begin>.004:break
        if self.closing:self.operator.pump()
        else:self.operator.step()
        now=time.monotonic();state=self.status.get('state')
        if state!=self.last_state:
            self.last_state=state;self.transitions.append(dict(t=now,state=state,reason=self.status.get('reason')))
        if state=='ACTIVE':
            if self.active_since is None:self.active_since=now
            self.activation_ever=True
        elif self.activation_ever and state in ('STOPPING','PAUSED','FAULT','STOP_UNCONFIRMED') and not self.closing:
            self.channel.stop(self.status.get('first_stop_reason') or self.status.get('reason') or 'BACKEND_STOPPED')
        if now-self.last_trace>.020:
            self.spool.emit(dict(t=now,state=state,reason=self.status.get('reason'),supervisor=self.status.copy(),gate=self.gate.copy(),
                car=self.plant.model.pose.copy(),velocity=asdict(self.plant.model.velocity),command=asdict(self.plant.policy.command),
                reference=self.gesture.last.copy() if self.synthetic else None,
                car01_readonly=self.adapter.last_car_observed,policy_fault=self.plant.policy.fault,
                browser=self.channel.summary(now),hardware_output=False))
            self.last_trace=now
        if any(p.poll() is not None for p in self.processes):raise RuntimeError('critical SIM12 process exited')

    def snapshot(self,now):return self.operator.snapshot(now)

    def close(self):
        from split_runtime import stop_acknowledged
        self.stopping=time.monotonic()
        self.closing=True
        self.model.end_trial(self.stopping,self.channel.fault or '软件会话结束')
        self.operator.request('stop')
        confirmed=False
        try:
            deadline=self.stopping+1.5
            while time.monotonic()<deadline:
                self.step(time.monotonic())
                if stop_acknowledged(self.status,self.stopping):confirmed=True;break
                time.sleep(.003)
            self.plant.close()
            logging=self.spool.close();self.spool=None
            result=dict(stop_confirmed=confirmed,stop_wait_s=time.monotonic()-self.stopping,
                state=self.status.get('state'),reason=self.status.get('first_stop_reason') or self.status.get('reason'),
                gate_output_zero=self.gate.get('output_zero'),logging=logging,
                metrics=self.plant.metrics,transitions=self.transitions,physical_stop_verified=False)
            save(self.session/'STOP.json',result)
            return result
        finally:self.dispose()

    def dispose(self):
        from common import diagnostic as d
        if hasattr(self,'plant'):self.plant.close()
        if self.spool is not None:
            self.spool.close();self.spool=None
        for process in self.processes:d.terminate_owned(process)
        if hasattr(self,'operator'):self.operator.close_node()
        if self.executor is not None:self.executor.shutdown()
        for node in reversed(self.nodes):node.destroy_node()
        if self.initialized:self.rclpy.shutdown();self.initialized=False
        if self.reader is not None:self.reader.close()
        if self.receiver is not None:d.terminate_owned(self.receiver)
        for log in self.logs:log.close()
        save(self.session/'PROTECTED.json',dict(unchanged=self.before==protected(),hardware_output=False))
