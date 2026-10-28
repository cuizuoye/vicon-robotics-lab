#!/usr/bin/env python3
"""Actual ROS SIM chain self-test, synthetic private-domain input, no domain0.

No GUI/keyboard events and no real Sign or hardware access. All owned nodes
are destroyed before the human-operated real-Sign test is opened.
"""
import argparse
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import secrets
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import matching_sim as m
from mecanum_wand_servo_control.safety_nodes import pose_message,LATEST,SENSOR

PATH=m.WORKSPACE/'docs/wand_servo_matching_sim_2026-10-23/live_runtime_v1/live_matching_sim.py'
spec=importlib.util.spec_from_file_location('matching_preflight_adapter',PATH)
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)


class SyntheticSource(Node):
    def __init__(self,runtime):
        super().__init__('wand_sign_source',namespace=runtime.namespace)
        runtime.guard(self)
        self.pose=type(runtime.initial_pose)(10.,10.,1.)
        self.pose_on=True
        self.pub=self.create_publisher(live.PoseStamped,'wand_pose',SENSOR)
        self.mode=self.create_publisher(String,'mode_claim',LATEST)
        self.status=self.create_publisher(String,'source_status',LATEST)
        self.create_timer(.01,self.tick)

    def tick(self):
        now=self.get_clock().now()
        if self.pose_on:self.pub.publish(pose_message(self.pose,now.to_msg(),world='map'))
        self.mode.publish(String(data='wand'))
        self.status.publish(String(data=json.dumps(dict(mode='shadow',source_domain=115,
            software_car_only=True,rejected={'wand':0},issued_steady=time.monotonic(),
            issued_ros=now.nanoseconds*1e-9))))


class OperatorProbe(Node):
    def __init__(self,runtime):
        super().__init__('wand_real_operator',namespace=runtime.namespace)
        runtime.guard(self)
        self.held=False;self.sequence=0;self.events=[];self.latest={}
        self.pub=self.create_publisher(String,'operator_lease',LATEST)
        for topic in ('supervisor_status','gate_status','gate_timing','plant_feedback'):
            self.create_subscription(String,topic,lambda msg,t=topic:self.receive(t,msg),LATEST)
        self.service_clients={name:self.create_client(Trigger,name) for name in ('start','recover')}
        self.create_timer(.01,self.tick)

    def receive(self,topic,msg):
        value=json.loads(msg.data)
        self.events.append(dict(received=time.monotonic(),topic=topic,value=value))
        self.latest[topic]=value

    def tick(self):
        self.sequence+=1
        self.pub.publish(String(data=json.dumps(dict(source='synthetic-preflight-operator',
            sequence=self.sequence,issued_steady=time.monotonic(),
            issued_ros=self.get_clock().now().nanoseconds*1e-9,held=self.held))))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    args=p.parse_args(argv)
    if args.output.exists():raise FileExistsError('new evidence directory required')
    args.output.mkdir(parents=True)
    os.environ['ROS_DOMAIN_ID']='115'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['WAND_SERVO_CHANNEL_KEY']=secrets.token_hex(32)
    rclpy.init(domain_id=115)
    runtime=live.LiveRuntime('center')
    nodes=[];executor=SingleThreadedExecutor()
    report=dict(software_only=True,domain0_access=False,hardware_access=False,passed=False)
    try:
        for factory in (live.SupervisorNode,live.TraceGate,live.MatchingPlantNode,SyntheticSource,OperatorProbe):
            node=factory(runtime);nodes.append(node);executor.add_node(node)
        sup,gate,plant,source,probe=nodes
        def spin(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:executor.spin_once(timeout_sec=.003)
        def wait(predicate,timeout=5.):
            until=time.monotonic()+timeout
            while time.monotonic()<until:
                executor.spin_once(timeout_sec=.003)
                if predicate():return
            raise AssertionError(dict(supervisor=probe.latest.get('supervisor_status'),gate=probe.latest.get('gate_status')))
        def service(name):
            client=probe.service_clients[name]
            assert client.service_is_ready()
            future=client.call_async(Trigger.Request())
            wait(future.done)
            response=future.result()
            assert response.success,response.message
        def moving(duration=1.6):
            begin=time.monotonic();hand=source.pose
            while time.monotonic()-begin<duration:
                t=time.monotonic()-begin
                c,s=math.cos(runtime.initial_pose.yaw),math.sin(runtime.initial_pose.yaw)
                source.pose=type(hand)(hand.x+c*.4*t,hand.y+s*.4*t,hand.z)
                executor.spin_once(timeout_sec=.003)
            assert sup.core.state=='ACTIVE',(sup.core.reason,gate.core.fault)
            assert math.hypot(gate.core.output.vx,gate.core.output.vy)>.30
        wait(lambda: bool(probe.latest.get('supervisor_status',{}).get('ready_to_start')),10.)
        service('start')
        wait(lambda: probe.latest.get('supervisor_status',{}).get('gate_ready'))
        probe.held=True
        wait(lambda:sup.core.state=='ACTIVE')
        moving()
        report['moving_command_speed_mps']=math.hypot(gate.core.output.vx,gate.core.output.vy)
        released=time.monotonic();probe.held=False;probe.tick()
        wait(lambda:gate.core.output==m.ZERO,.25)
        report['release_to_zero_s']=time.monotonic()-released
        assert report['release_to_zero_s']<.10
        wait(lambda:sup.core.state=='PAUSED' and sup.core.stop_confirmed)
        wait(lambda:bool(probe.latest.get('supervisor_status',{}).get('ready_to_start')))
        service('start');wait(lambda:probe.latest.get('supervisor_status',{}).get('gate_ready'))
        probe.held=True;wait(lambda:sup.core.state=='ACTIVE');moving(1.)
        lost=time.monotonic();source.pose_on=False
        wait(lambda:sup.core.fault_latched and gate.core.output==m.ZERO,.4)
        report['synthetic_occlusion_to_zero_s']=time.monotonic()-lost
        assert report['synthetic_occlusion_to_zero_s']<=.27
        source.pose_on=True;spin(2.)
        assert sup.core.state=='FAULT' and gate.core.output==m.ZERO
        assert not sup.core.request_recover(*sup.clocks())[0]
        probe.held=False;spin(1.2)
        service('recover')
        wait(lambda:probe.latest.get('supervisor_status',{}).get('gate_ready'))
        assert sup.core.state=='ARMED' and gate.core.output==m.ZERO
        probe.held=True;wait(lambda:sup.core.state=='ACTIVE');spin(.35)
        assert gate.core.output==m.ZERO
        moving(.5)
        probe.held=False;probe.tick();spin(.10)
        assert gate.core.output==m.ZERO
        report.update(passed=True,no_automatic_resume=True,explicit_reprepare_then_new_hold_verified=True,
                      signed_matching_verifier_used=type(plant.verifier).__name__,
                      model_type=type(plant.plant).__module__,preserved_fixed_arena=True)
    except Exception as exc:
        report['error']=repr(exc)
        raise
    finally:
        if nodes and isinstance(nodes[-1],OperatorProbe):nodes[-1].held=False;nodes[-1].tick()
        for node in reversed(nodes):
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
        if 'probe' in locals():
            (args.output/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in probe.events))
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
