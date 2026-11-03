"""Browser lease -> real ROS bound gate -> signed exclusive 4WSC MC-Link bridge.

Rehearsal constructs the SAME bridge but may address only dedicated loopback
ports and an exact-C peer. Real mode never constructs a software car.
"""
from dataclasses import asdict
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
import bootstrap

READY=Path(__file__).resolve().parent
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def raw_child(channel,stop,ready):
    os.environ['ROS_DOMAIN_ID']='0'
    bootstrap.install()
    from mecanum_wand_4wsc_hardware.source_relay import passive_child
    passive_child(channel,stop,ready)

def real_reader():
    from mecanum_wand_4wsc_hardware.source_relay import PassiveReader
    reader=PassiveReader()
    reader.process=mp.get_context('spawn').Process(target=raw_child,args=(reader.channel,reader.stop,reader.ready),daemon=True,name='4wsc-passive-vicon-only')
    return reader

def Rig(channel,session,synthetic=False,occlusion=False):
    rt=bootstrap.runtime()
    if synthetic!=rt.software_only:raise ValueError('real Sign and software rehearsal must not be mixed')
    if rt.purpose=='suspended':
        from suspended_runtime import SuspendedRig
        return SuspendedRig(rt,channel,session)
    return FollowRig(rt,channel,session,occlusion)

class FollowRig:
    def __init__(self,rt,channel,session,occlusion=False,peer_oracle=None,keep_peer=False):
        if rt.purpose!='sign':raise ValueError('sign purpose required')
        self.rt,self.channel,self.session=rt,channel,session
        self.physical=not rt.software_only;self.bridge_constructed=False;self.ros_initialized=False;self.ros_initialized_ever=False
        self.active_since=None;self.activation_ever=False;self.stopping=None;self.closing=False
        self.reader=self.peer=self.executor=self.operator=None;self.nodes=[];self.processes=[];self.logs=[]
        self.status={};self.gate={};self.transport={};self.transitions=[];self.last_state=None;self.trace_at=0.
        self.occlusion=occlusion;self.occlude=False;self.peer_oracle=peer_oracle;self.keep_peer=keep_peer
        if peer_oracle is not None and self.physical:raise ValueError('software peer injection forbidden on real hardware')
        self.trace=(session/'TRACE.jsonl').open('x')
        try:self.setup();self.warmup()
        except BaseException:self.dispose();raise

    def setup(self):
        bootstrap.install()
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from mecanum_wand_4wsc_control.safety_nodes import LATEST
        from mecanum_wand_4wsc_control.toggle_ui import ToggleOperatorModel
        from mecanum_wand_4wsc_hardware.source_relay import SourceRelay
        from browser_operator import BrowserOperator
        self.rclpy=rclpy
        key=secrets.token_hex(32);os.environ['WAND_4WSC_CHANNEL_KEY']=key
        if self.rt.software_only:
            from native_peer import Peer
            from geometry import synthetic_packet
            self.peer=self.peer_oracle or Peer(self.rt.command_port)
            if self.peer.port!=self.rt.command_port:raise ValueError('reused software MCU endpoint mismatch')
            rig=self
            class SyntheticReader:
                provenance=dict(synthetic=True,real_car_feedback=False)
                def start(self):pass
                def close(self):pass
                def poll(self):
                    now=time.monotonic();ros=time.time();sample=rig.peer.sample()
                    if sample['peer_error']:raise ValueError(sample['peer_error'])
                    t=0. if rig.active_since is None else now-rig.active_since
                    scale=.035 if rig.rt.config.stage==0 else .15
                    x=scale*math.sin(.8*t);y=scale*.7*math.sin(1.1*t)
                    yaw=(.025 if rig.rt.config.stage==0 else .2)*math.sin(.7*t)
                    packet=synthetic_packet(1.+x,1.+y,yaw,now,ros).payload()
                    if rig.occlude or (rig.occlusion and 2.5<=t<5.5):packet=None
                    car=None
                    if now-sample['at']<=.20:
                        cx,cy,ca=sample['pose'];car=dict(pose=dict(x=cx,y=cy,z=rig.rt.config.ground_z_m,yaw=ca),stamp=ros-(now-sample['at']),roll=0.,pitch=0.)
                    return dict(issued=now,packet=packet,car=car,conflicts=[],rejected={})
            self.reader=SyntheticReader()
        else:self.reader=real_reader()
        rclpy.init(args=[],domain_id=self.rt.domain,signal_handler_options=SignalHandlerOptions.NO);self.ros_initialized=True;self.ros_initialized_ever=True
        self.executor=SingleThreadedExecutor()
        self.monitor=Node('browser_4wsc_observer',namespace=self.rt.namespace,enable_rosout=False);self.nodes.append(self.monitor)
        for topic,target in (('supervisor_status',self.status),('gate_status',self.gate),('transport_status',self.transport)):
            self.monitor.create_subscription(String,topic,lambda m,t=target:t.update(json.loads(m.data)),LATEST)
        source=SourceRelay(self.rt,self.reader);self.nodes.append(source)
        for node in self.nodes:self.executor.add_node(node)
        self.model=ToggleOperatorModel();self.operator=BrowserOperator(self.model,self.rt,self.channel)
        for role in ('bridge','supervisor','gate'):
            name={'bridge':'wand_hardware_bridge','supervisor':'safe_wand_supervisor','gate':'safe_wand_gate'}[role]
            args=[sys.executable,str(READY/'node.py'),'--role',role,'--ros-args','-r','__node:='+name,'-r','__ns:='+self.rt.namespace]
            if role=='bridge':
                params=dict(esp_ip=self.rt.ip,network_interface=self.rt.interface,
                    command_port=self.rt.command_port,telemetry_port=self.rt.telemetry_port,
                    bounded_motion_mode=True,required_motion_tag='4WSC',wire_timeout_ms=600,
                    max_vx_mps=self.rt.config.max_speed_mps,max_vy_mps=self.rt.config.max_speed_mps,
                    max_wz_radps=self.rt.config.max_yaw_rate_radps,command_timeout_s=.10,
                    link_receive_timeout_s=.30,invert_body_xy=False,udp_low_latency_qos=False)
                for k,v in params.items():args+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
            log=(self.session/(role+'.log')).open('x');self.logs.append(log)
            self.processes.append(subprocess.Popen(args,stdout=log,stderr=log,env=dict(os.environ,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')))
        self.bridge_constructed=True
        save(self.session/'CONTRACT.json',dict(identity='4WSC',stage=self.rt.config.stage,
            mode=self.rt.mode,purpose='sign',mapping='1:1 XY + relative yaw; Z ignored',
            direct_target=True,intermediate_reference=False,fixed_fence=False,car01_feedback_used=self.physical,
            limits=dict(speed=self.rt.config.max_speed_mps,accel=self.rt.config.max_accel_mps2,
                yaw=self.rt.config.max_yaw_rate_radps,yaw_accel=self.rt.config.max_yaw_accel_radps2),
            browser_timeout_s=.1,command_timeout_s=.1,feedback_timeout_s=.3,pose_timeout_s=.2,
            source_clock_not_renewed=True,automatic_enable=False,physical_qualified=False))

    def warmup(self):
        from mecanum_wand_4wsc_control.auto_prepare import AutoPrepareOnce
        start=time.monotonic();stable=None
        while time.monotonic()-start<22.:
            self.step(time.monotonic());now=time.monotonic()
            if self.model.held or self.activation_ever:raise ValueError('no authority before browser input')
            if AutoPrepareOnce.zero_baseline_qualified(self.model,now) and self.transport.get('startup_clock',{}).get('ready'):
                if stable is None:stable=now
                if now-stable>=.75:
                    save(self.session/'WARMUP.json',dict(ready=True,disabled=True,held=False,elapsed_s=now-start,transport=self.transport));return
            else:stable=None
            time.sleep(.003)
        save(self.session/'WARMUP.json',dict(ready=False,status=self.status,gate=self.gate,transport=self.transport))
        if self.peer is not None:save(self.session/'PEER_FAILURE.json',list(self.peer.wire_history))
        raise ValueError('4WSC失能握手/时钟/Sign/car01未就绪，不开放运动入口')

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
        if now-self.trace_at>=.02:
            self.trace.write(json.dumps(dict(t=now,status=self.status,gate=self.gate,transport=self.transport,physical_output=self.physical),ensure_ascii=False)+'\n');self.trace.flush();self.trace_at=now
        if any(p.poll() is not None for p in self.processes):raise RuntimeError('critical 4WSC process exited')

    def snapshot(self,now):
        value=self.operator.snapshot(now)
        value.update(stage=self.rt.config.stage,purpose='sign',limits=dict(speed=self.rt.config.max_speed_mps,
            accel=self.rt.config.max_accel_mps2,yaw=self.rt.config.max_yaw_rate_radps,yaw_accel=self.rt.config.max_yaw_accel_radps2))
        return value

    def close(self):
        self.stopping=time.monotonic();self.closing=True;confirmed=False
        if self.operator is None:self.dispose();return dict(stop_confirmed=False,gate_output_zero=False)
        self.model.end_trial(self.stopping,self.channel.fault or '会话结束');self.operator.request('stop')
        try:
            while time.monotonic()-self.stopping<1.5:
                self.step(time.monotonic());s=self.status
                if s.get('state') in ('PAUSED','FAULT') and s.get('issued_steady',0)>=self.stopping and s.get('stop_confirmed') is True and self.gate.get('output_zero') is True:
                    confirmed=True;break
                time.sleep(.003)
            result=dict(stop_confirmed=confirmed,gate_output_zero=self.gate.get('output_zero'),state=self.status.get('state'),
                stop_wait_s=time.monotonic()-self.stopping,transitions=self.transitions,
                physical_stop_distance_verified=False,actual_hardware_trial=self.physical,
                exact_C_peer=None if self.peer is None else dict(peak_duty=self.peer.peak_duty,peak_rpm=self.peer.peak_rpm,permit_ever=self.peer.permit_ever))
            save(self.session/'STOP.json',result);return result
        finally:self.dispose()

    def dispose(self):
        for p in reversed(self.processes):
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2)
        if self.operator is not None:self.operator.close_node();self.operator=None
        if self.executor is not None:self.executor.shutdown()
        for n in reversed(self.nodes):n.destroy_node()
        self.nodes=[]
        if self.ros_initialized:self.rclpy.shutdown();self.ros_initialized=False
        if self.peer is not None and not self.keep_peer:self.peer.close()
        for log in self.logs:log.close()
        if not self.trace.closed:self.trace.close()
