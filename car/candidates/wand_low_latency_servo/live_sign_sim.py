#!/usr/bin/env python3
"""Real Sign -> software car acceptance. NO MCU transport or firmware code.

Imports the verified servo unchanged. The only domain0 application interface
is a subscriber to /vicon/Sign/Sign, plus read-only graph inspection. All
commands, operator services and synthetic car feedback stay in domain114.
Opening this console NEVER prepares/enables motion automatically.
"""
import argparse
from collections import Counter
import copy
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from mecanum_wand_servo_hardware.commissioning import Runtime as OfflineRuntime
from mecanum_wand_servo_control.operator_console import Console,OperatorNode
from mecanum_wand_servo_control.operator_model import OperatorModel
from mecanum_wand_servo_control.core import Pose
from mecanum_wand_servo_control.safety_nodes import (
    SupervisorNode,GateNode,PlantNode,SENSOR,LATEST,decode_pose,SERVICES)

WORKSPACE=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
PROFILE=WORKSPACE/'docs/wand_low_latency_servo_2026-10-23/verified_runtime/src/mecanum_wand_servo_hardware/config/servo.3x3_1to1.OFFLINE_ONLY.json'
PROFILE_SHA='242f97b9b5bd6a47c5317af18a06d2206a21131bb102d70180a6dab5d25e5a99'
DOMAIN=114
NAMESPACE='/wand_servo_real_sign_sim'
RAW_TOPIC='/vicon/Sign/Sign'
LEGACY_MOTION=('/car/cmd_vel','/car/cmd_vel_raw','/car/enable','/car/control_enable',
               '/car/run_lease','/car/clear_faults','/car/trajectory_enable')
LEGACY_NODES=('cmd_vel_udp_bridge','continuous_loop_supervisor','rectangle_run_guard')


class LiveRuntime(OfflineRuntime):
    namespace=NAMESPACE

    def __init__(self,sim_start='corner'):
        if hashlib.sha256(PROFILE.read_bytes()).hexdigest()!=PROFILE_SHA:
            raise RuntimeError('verified offline profile drift')
        super().__init__('shadow',str(PROFILE))
        if sim_start not in ('corner','center'):raise ValueError('invalid software start')
        self.sim_start=sim_start
        if sim_start=='center':
            self.initial_pose=Pose(*self.config.geometry().world(1.5,1.5),
                                   self.config.ground_z_m,self.config.arena_heading_rad)
        self.source_domain=0  # User-authorized read-only real Sign input.
        if self.mode!='shadow' or self.ip or self.config.follower_config().scale!=1.:
            raise RuntimeError('software car only')

    def guard(self,node):
        if (os.environ.get('ROS_DOMAIN_ID')!=str(DOMAIN)
                or node.context.get_domain_id()!=DOMAIN or node.get_namespace()!=NAMESPACE
                or node.get_parameter('use_sim_time').value):
            raise RuntimeError('real Sign simulation requires domain114 / fixed namespace / real clocks')
        topics=(*self.expected,'supervisor_status','gate_status','mode_claim','source_status','inject')
        for name in topics:
            if node.resolve_topic_name(name)!=NAMESPACE+'/'+name:
                raise RuntimeError('simulation topic remapping forbidden')
        for name in SERVICES:
            if node.resolve_service_name(name)!=NAMESPACE+'/'+name:
                raise RuntimeError('simulation service remapping forbidden')


class SignReader(Node):
    """No publishers, no service clients; original raw headers stay intact."""
    def __init__(self,context):
        super().__init__('wand_servo_sign_readonly',context=context,use_global_arguments=False)
        self.sample=None
        self.last_stamp=-math.inf
        self.rejected=0
        self.received=0
        self.create_subscription(PoseStamped,RAW_TOPIC,self.receive,SENSOR)

    def receive(self,msg):
        try:
            _,stamp,_,_=decode_pose(msg,'map')
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not -.005<=age<=.20:
                raise ValueError('raw source age')
            if stamp<=self.last_stamp:
                return
            self.last_stamp=stamp
            self.sample=copy.deepcopy(msg)
            self.received+=1
        except (ValueError,TypeError,AttributeError):
            self.sample=None
            self.rejected+=1

    def audit(self):
        conflicts=[t for t in LEGACY_MOTION if self.count_publishers(t)]
        conflicts.extend(n for n,_ in self.get_node_names_and_namespaces() if n in LEGACY_NODES)
        pubs=self.get_publishers_info_by_topic(RAW_TOPIC)
        if len(pubs)!=1 or pubs[0].node_name!='vicon_client' or pubs[0].node_namespace!='/':
            conflicts.append('Sign source must be exactly vicon_client in domain0')
        return conflicts


class SignRelay(Node):
    def __init__(self,runtime):
        super().__init__('wand_sign_source',namespace=NAMESPACE)
        runtime.guard(self)
        self.source_context=Context()
        rclpy.init(context=self.source_context,domain_id=0,signal_handler_options=SignalHandlerOptions.NO)
        self.reader=SignReader(self.source_context)
        self.source_executor=SingleThreadedExecutor(context=self.source_context)
        self.source_executor.add_node(self.reader)
        self.pose_pub=self.create_publisher(PoseStamped,'wand_pose',SENSOR)
        self.mode_pub=self.create_publisher(String,'mode_claim',LATEST)
        self.status_pub=self.create_publisher(String,'source_status',LATEST)
        self.forwarded=-math.inf
        self.audit_at=-math.inf
        self.conflicts=['discovery pending']
        self.ready_at=time.monotonic()+2.
        self.create_timer(.01,self.tick)

    def tick(self):
        for _ in range(8):
            self.source_executor.spin_once(timeout_sec=0.)
        now=time.monotonic()
        if now-self.audit_at>=.10:
            self.conflicts=self.reader.audit()
            if now<self.ready_at:
                self.conflicts.append('discovery pending')
            self.audit_at=now
        if self.reader.sample is not None and self.reader.last_stamp>self.forwarded:
            self.pose_pub.publish(self.reader.sample)  # Never re-stamp/project input.
            self.forwarded=self.reader.last_stamp
        self.mode_pub.publish(String(data='conflict' if self.conflicts else 'wand'))
        self.status_pub.publish(String(data=json.dumps(dict(
            mode='shadow',source_domain=0,real_sign=True,software_car_only=True,
            raw_stamps={'wand':self.reader.last_stamp if math.isfinite(self.reader.last_stamp) else None},
            rejected={'wand':self.reader.rejected},raw_samples=self.reader.received,
            conflicts=self.conflicts,issued_steady=now,issued_ros=self.get_clock().now().nanoseconds*1e-9))))

    def destroy_node(self):
        self.source_executor.shutdown()
        self.reader.destroy_node()
        self.source_context.shutdown()
        return super().destroy_node()


class Observer(Node):
    def __init__(self,output):
        super().__init__('wand_servo_sim_observer',namespace=NAMESPACE)
        self.output=output
        self.log=(output/'events.jsonl').open('x')
        self.values={};self.counts=Counter();self.transitions=[];self.previous=None
        self.max_speed=0.
        for topic in ('supervisor_status','gate_status','source_status','candidate','plant_feedback','operator_lease','gate_timing'):
            self.create_subscription(String,topic,lambda m,t=topic:self.record(t,json.loads(m.data)),LATEST)
        for topic in ('wand_pose','car_pose'):
            self.create_subscription(PoseStamped,topic,lambda m,t=topic:self.pose(t,m),SENSOR)
        self.create_subscription(Twist,'cmd_vel',self.command,LATEST)
        self.create_timer(.5,self.snapshot)

    def record(self,topic,value):
        now=time.monotonic()
        self.values[topic]={'received_steady':now,'value':value}
        self.counts[topic]+=1
        self.log.write(json.dumps(dict(topic=topic,received_steady=now,value=value),allow_nan=False)+'\n')
        if topic=='supervisor_status':
            transition=(value['state'],value['reason'],value['epoch'])
            if transition!=self.previous:
                self.transitions.append(dict(steady=now,state=value['state'],reason=value['reason'],
                    epoch=value['epoch'],stop_confirmed=value['stop_confirmed'],hard_fault=value['hard_fault']))
                self.previous=transition

    def pose(self,topic,msg):
        try:
            pose,stamp,roll,pitch=decode_pose(msg,'map')
            self.record(topic,dict(**asdict(pose),source_ros=stamp,roll=roll,pitch=pitch))
        except (ValueError,TypeError):
            self.counts[topic+'_invalid']+=1

    def command(self,msg):
        speed=math.hypot(msg.linear.x,msg.linear.y)
        self.max_speed=max(self.max_speed,speed)
        self.record('cmd_vel',dict(vx=msg.linear.x,vy=msg.linear.y,wz=msg.angular.z,speed=speed))

    def snapshot(self):
        report=dict(label='REAL SIGN / SOFTWARE CAR ONLY; NO PHYSICAL LATENCY CLAIM',
            issued_steady=time.monotonic(),counts=dict(self.counts),max_command_speed_mps=self.max_speed,
            state_transitions=self.transitions,latest=self.values,
            nodes=[dict(name=n,namespace=ns) for n,ns in self.get_node_names_and_namespaces()],
            publishers={t:[dict(name=p.node_name,namespace=p.node_namespace) for p in self.get_publishers_info_by_topic(NAMESPACE+'/'+t)]
                        for t in ('wand_pose','car_pose','cmd_vel','operator_lease','actuator_command')})
        temp=self.output/'snapshot.tmp'
        temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        temp.replace(self.output/'snapshot.json')
        self.log.flush()

    def destroy_node(self):
        self.snapshot();self.log.close()
        return super().destroy_node()


class TraceGate(GateNode):
    """Same signed output and independent core; extra read-only timing topic."""
    def __init__(self,runtime):
        super().__init__(runtime)
        self.timing_pub=self.create_publisher(String,'gate_timing',LATEST)

    def publish_output(self,output):
        super().publish_output(output)
        if not hasattr(self,'timing_pub'):
            return
        steady,ros=self.clocks()
        f=self.core.frame
        self.timing_pub.publish(String(data=json.dumps(dict(
            issued_steady=steady,issued_ros=ros,command=asdict(output),
            candidate_sequence=f.sequence if f else None,
            candidate_issued_steady=f.issued_steady if f else None,
            candidate_issued_ros=f.issued_ros if f else None,
            state=f.state if f else None,epoch=f.epoch if f else '',
            gate_fault=self.core.fault,hard_fault=self.core.hard_fault))))


class ServoConsole(Console):
    def __init__(self,root,node,model):
        super().__init__(root,node,model)
        title='真实 Sign → 低延迟连续伺服仿真（实车断电）'
        root.title(title)
        def labels(widget):
            for child in widget.winfo_children():
                if isinstance(child,tk.Label):
                    text=child.cget('text')
                    if text=='真实 Sign → 安全仿真车（实车断电）':child.configure(text=title)
                    if text.startswith('域97 独立0.20首档'):
                        child.configure(text='域114 · 1:1 XY · 固定3×3m · ≤0.20m/s / 0.08m/s² · 仅软件执行器 · 起点：'
                                        +('区内中心，围栏未移动' if node.runtime.sim_start=='center' else '原起点一角'))
                labels(child)
        labels(root)

    def draw(self,now):
        super().draw(now)
        if self.model.status.get('state')=='ACTIVE' and self.model.held:
            self.step_text.set('低延迟连续跟随：橙色为1:1目标，蓝色为仿真车；轻缓移动和换向。松键停车；遮挡后不会自动恢复。')


def worker(role,output,sim_start='corner'):
    runtime=LiveRuntime(sim_start)
    rclpy.init(domain_id=DOMAIN,signal_handler_options=SignalHandlerOptions.NO)
    if role=='console':
        model=OperatorModel();node=OperatorNode(model,runtime)
        root=tk.Tk();app=ServoConsole(root,node,model)
        signal.signal(signal.SIGINT,lambda *_:root.after(0,app.close))
        signal.signal(signal.SIGTERM,lambda *_:root.after(0,app.close))
        try:root.mainloop()
        finally:
            node.close_node()
            if rclpy.ok():rclpy.shutdown()
        return
    factories=dict(source=lambda:SignRelay(runtime),supervisor=lambda:SupervisorNode(runtime),
        gate=lambda:TraceGate(runtime),plant=lambda:PlantNode(runtime),observer=lambda:Observer(output))
    node=factories[role]()
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while running:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def manager(output,sim_start='corner'):
    runtime=LiveRuntime(sim_start)  # Validate profile before constructing any node.
    output.mkdir(parents=True,exist_ok=False)
    lock=open('/tmp/wand_servo_live_sim.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    probe=subprocess.run(['pgrep','-af','[w]and_acceptance_transport|[c]md_vel_udp_bridge|[r]ectangle_run_guard'],
                          text=True,capture_output=True)
    if probe.returncode==0:raise RuntimeError('existing car transport present; not touching it: '+probe.stdout)
    env=dict(os.environ,ROS_DOMAIN_ID=str(DOMAIN),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
             WAND_SERVO_CHANNEL_KEY=secrets.token_hex(32))
    workers=[];files=[];running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    (output/'session.json').write_text(json.dumps(dict(domain=DOMAIN,namespace=NAMESPACE,
        real_sign_domain=0,raw_topic=RAW_TOPIC,profile_sha256=PROFILE_SHA,
        mapping_scale=1.,goal_area_m=[3,3],hardware_transport=False,auto_enable=False,
        software_initial_pose=asdict(runtime.initial_pose),software_start=sim_start,
        scope='REAL SIGN TO SOFTWARE CAR ONLY; physical car off; no firmware'),indent=2)+'\n')
    try:
        def launch(role):
            log=(output/(role+'.log')).open('x');files.append(log)
            child_env=dict(env,ROS_LOG_DIR=str(output/('ros_'+role)))
            Path(child_env['ROS_LOG_DIR']).mkdir(mode=0o700)
            p=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--role',role,
                                '--output',str(output),'--sim-start',sim_start],env=child_env,stdout=log,stderr=subprocess.STDOUT)
            workers.append((role,p))
        for role in ('source','supervisor','gate','plant','observer'):
            launch(role)
        # Do not flash a console then immediately close it if a backend's
        # real constructor fails. Wait for the source and observer first;
        # never send a start service or an operator lease here.
        deadline=time.monotonic()+10.
        while time.monotonic()<deadline:
            failed=[(r,p.returncode) for r,p in workers if p.poll() is not None]
            if failed:raise RuntimeError('simulation backend startup failed: '+str(failed))
            snapshot=output/'snapshot.json'
            if snapshot.exists():
                value=json.loads(snapshot.read_text())
                if value.get('latest',{}).get('source_status'):
                    break
            time.sleep(.05)
        else:raise RuntimeError('simulation source/observer did not become ready')
        launch('console')
        (output/'pids.json').write_text(json.dumps({role:p.pid for role,p in workers},indent=2)+'\n')
        print('REAL Sign / SIM ONLY; waiting for human operation. Evidence: '+str(output),flush=True)
        while running:
            for role,p in workers:
                if p.poll() is not None:
                    print('Stopping this simulation: '+role+' exited '+str(p.returncode),flush=True)
                    running=False
            time.sleep(.1)
    finally:
        # Close console first so it sends released leases/stop, then stop only
        # our own children. Never signal an existing receiver or old console.
        for role,p in reversed(workers):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    p.terminate();p.wait(timeout=3.)
        for log in files:log.close()
        lock.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=('manager','source','supervisor','gate','plant','observer','console'),default='manager')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sim-start',choices=('corner','center'),default='corner')
    args=p.parse_args()
    if args.role=='manager':manager(args.output.resolve(),args.sim_start)
    else:worker(args.role,args.output.resolve(),args.sim_start)


if __name__=='__main__':main()
