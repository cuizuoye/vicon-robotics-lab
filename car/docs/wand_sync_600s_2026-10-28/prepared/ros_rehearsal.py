#!/usr/bin/env python3
"""Actual migrated ROS nodes, PRIVATE DDS domain187, SOFTWARE plant only."""
import argparse
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import secrets
import sys
import time
from types import SimpleNamespace as NS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--runtime',type=Path,required=True);args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='187' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise ValueError('Private local-only rehearsal domain required')
    os.environ['WAND_4WSA_CHANNEL_KEY']=secrets.token_hex(32)
    for name in ('mecanum_wand_4wsa_control','mecanum_wand_4wsa_hardware'):
        sys.path.insert(0,str(args.runtime/name))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped,Twist
    from std_msgs.msg import String
    from mecanum_wand_4wsa_control.core import Pose
    from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig
    from mecanum_wand_4wsa_control.rotation_source import Packet,Raw,definition,canonical
    from mecanum_wand_4wsa_control.rotation_geometry import REQUIRED,rotation
    from mecanum_wand_4wsa_control.safety_core import ZERO
    from mecanum_wand_4wsa_control.safety_nodes import pose_message,LATEST,SENSOR
    from mecanum_wand_4wsa_control.offline_suite import Plant
    from mecanum_wand_4wsa_control.operator_console import OperatorNode
    from mecanum_wand_4wsa_control.toggle_ui import ToggleOperatorModel
    from mecanum_wand_4wsa_control.auto_prepare import AutoPrepareOnce
    from mecanum_wand_4wsa_hardware.rotation_nodes import RotationSupervisorNode,RotationGateNode
    c=RotationSafetyConfig(**json.loads((args.runtime/'mecanum_wand_4wsa_hardware/config/stage6.NOT_APPROVED.json').read_text())['safety'])
    rclpy.init(args=[]);results=[]
    try:
        for case in ('normal_stop','occlusion','source_digest','yaw_intent','actuator_fault','source_conflict'):
            namespace='/rotation_migration_software_'+case
            def guard(node):
                if node.get_namespace()!=namespace or node.context.get_domain_id()!=187:raise ValueError('Rehearsal escaped')
            runtime=NS(namespace=namespace,world='map',config=c,mode='shadow',guard=guard,
                expected=dict(car_pose='software_rotation_source',wand_pose='software_rotation_source',
                    rotation_packet='software_rotation_source',plant_feedback='software_rotation_source',
                    operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
                    cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate'))
            executor=SingleThreadedExecutor();nodes=[]
            state=NS(seq=0,ready=None,active=None,injected=None,stopped=None,restored=None,
                source_on=True,fault_bits=0,wire_fault=False,wire_fault_used=False,cmd=ZERO,
                max_speed=0.,max_wz=0.,positive=False,negative=False,output_count=0,
                last_fixture=time.monotonic(),before_click_nonzero=False)
            phases=[];faults=[];calls=[];trace=[];unexpected_active_faults=[]
            try:
                source=Node('software_rotation_source',namespace=namespace,enable_rosout=False)
                model=ToggleOperatorModel();model.focused=True;model.release(time.monotonic())
                operator=OperatorNode(model,runtime)
                # Interpose ONLY the private test DDS publisher to inject a
                # bad bound digest or wrong-but-rate-valid angle. No hook is
                # installed in any production runtime source.
                original=Node.create_publisher
                def intercepted(node,msg_type,topic,*a,**kw):
                    pub=original(node,msg_type,topic,*a,**kw)
                    if topic!='candidate' or node.get_name()!='safe_wand_supervisor':return pub
                    class TestWire:
                        def publish(self,msg):
                            if state.wire_fault and not state.wire_fault_used:
                                v=json.loads(msg.data)
                                if v['frame']['enabled']:
                                    if case=='source_digest':v['sign_digest']='0'*64
                                    else:v['frame']['goal']['yaw']+=.0005
                                    msg=String(data=canonical(v));state.wire_fault_used=True
                            pub.publish(msg)
                    return TestWire()
                Node.create_publisher=intercepted
                try:sup=RotationSupervisorNode(runtime)
                finally:Node.create_publisher=original
                gate=RotationGateNode(runtime)
                nodes=[source,operator,sup,gate]
                for node in nodes:executor.add_node(node)
                posepub={k:source.create_publisher(PoseStamped,k+'_pose',SENSOR) for k in ('car','wand')}
                pubs={k:source.create_publisher(String,k,LATEST) for k in ('rotation_packet','plant_feedback','mode_claim','source_status','transport_status')}
                origin=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad)
                plant=Plant(origin);policy=AutoPrepareOnce();d=definition()
                def command(m):
                    from mecanum_wand_4wsa_control.core import Velocity
                    state.cmd=Velocity(m.linear.x,m.linear.y,m.angular.z);state.output_count+=1
                    speed=math.hypot(state.cmd.vx,state.cmd.vy);state.max_speed=max(state.max_speed,speed)
                    state.max_wz=max(state.max_wz,abs(state.cmd.wz))
                    state.positive|=state.cmd.wz>.01;state.negative|=state.cmd.wz<-.01
                    if state.active is None and state.cmd!=ZERO:state.before_click_nonzero=True
                source.create_subscription(Twist,'cmd_vel',command,LATEST)
                def fixture():
                    now=time.monotonic();ros=source.get_clock().now().nanoseconds*1e-9;state.seq+=1
                    dt=min(.03,max(0.,now-state.last_fixture));state.last_fixture=now;plant.step(state.cmd,dt)
                    p=replace(plant.pose,yaw=math.atan2(math.sin(plant.pose.yaw),math.cos(plant.pose.yaw)))
                    stamp=source.get_clock().now().to_msg()
                    posepub['car'].publish(pose_message(p,stamp,world='map'))
                    # Pickup and orientation adjustment WHILE ARMED, before
                    # the fresh click: first enabled reference must be neutral.
                    t=0. if state.active is None else now-state.active
                    gesture=.0 if state.active is None else .07*math.sin(t*1.6)
                    yaw=.5+(.0 if state.active is None else .55*math.sin(t*1.8))
                    q=(0.,0.,math.sin(yaw/2),math.cos(yaw/2));r=rotation(q)
                    import numpy as np
                    point=np.array([1.+gesture,2.,1.2]);raw=point-r@np.array(d['local_points']['Sign5'])
                    packet=Packet((Raw('sign',tuple(raw),q,ros,now),*
                        (Raw(k,tuple(raw+r@np.array(d['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))
                    if state.source_on:
                        pubs['rotation_packet'].publish(String(data=canonical(packet.payload())))
                        posepub['wand'].publish(pose_message(packet.pose(now,ros),stamp,world='map'))
                    def send(name,v):pubs[name].publish(String(data=json.dumps(v)))
                    send('source_status',dict(issued_steady=now,issued_ros=ros,rejected={},conflicts=[]))
                    pubs['mode_claim'].publish(String(data='wand'))
                    send('plant_feedback',dict(sequence=state.seq,issued_steady=now,issued_ros=ros,
                        output_zero=state.cmd==ZERO,enabled=state.cmd!=ZERO,
                        speed=math.hypot(plant.velocity.vx,plant.velocity.vy),yaw_rate=abs(plant.velocity.wz),fault_bits=state.fault_bits))
                    # Declared SOFTWARE transport stub. This is not MCU clock,
                    # enabled-permit or physical stop qualification.
                    send('transport_status',dict(issued_steady=now,issued_ros=ros,fresh=True,ready=True,fault='',local_stop=False,
                        baseline_current=True,safety_enabled=False,motion_gate_requested=False,command_progress_age_s=0.,
                        startup_clock=dict(ready=True,qualified=True,fault='')))
                source.create_timer(.01,fixture)
                def operation():
                    now=time.monotonic();model.tick(now);operator.heartbeat()
                    if policy.poll(model,now,space_down=False,service_ready=operator.service_clients['start'].service_is_ready(),input_issue=operator.input_hint()):
                        assert operator.request('start');calls.append('start')
                    if not phases or phases[-1]!=policy.phase:phases.append(policy.phase)
                    if policy.phase=='READY' and state.ready is None:state.ready=now
                    if state.ready is not None and state.active is None and now-state.ready>.4 and model.start_ready(now):
                        model.press(now);assert model.release(now+.001);state.active=now;calls.append('fresh_click')
                    if gate.core.fault and (not faults or faults[-1]!=gate.core.fault):faults.append(gate.core.fault)
                    if state.active is not None and state.injected is None and sup.core.state=='ACTIVE' and gate.core.fault:
                        unexpected_active_faults.append(gate.core.fault)
                    if state.active is not None and state.injected is None and now-state.active>(4.5 if case=='normal_stop' else 1.2):
                        state.injected=now
                        if case=='normal_stop':model.end_trial(now,'SOFTWARE Esc');operator.request('stop');calls.append('stop')
                        elif case=='occlusion':state.source_on=False
                        elif case in ('source_digest','yaw_intent'):state.wire_fault=True
                        elif case=='actuator_fault':state.fault_bits=0x20
                        else:
                            # Exact duplicate graph owner, not merely a status flag.
                            duplicate=Node('conflicting_source',namespace=namespace,enable_rosout=False)
                            duplicate.create_publisher(String,'rotation_packet',LATEST);nodes.append(duplicate);executor.add_node(duplicate)
                    if state.injected is not None and state.restored is None and now-state.injected>.45:
                        state.source_on=True;state.restored=now
                    if state.injected is not None and state.cmd==ZERO and sup.core.state!='ACTIVE' and state.stopped is None:
                        state.stopped=now
                    if len(trace)<2400:trace.append(dict(t=now,state=sup.core.state,gate=gate.core.fault,
                        command=[state.cmd.vx,state.cmd.vy,state.cmd.wz]))
                operator.create_timer(.01,operation)
                started=time.monotonic()
                while time.monotonic()-started<13.:
                    executor.spin_once(timeout_sec=.003)
                    if state.restored is not None and time.monotonic()-state.restored>.8:break
                assert state.active is not None,dict(case=case,phases=phases,reason=sup.core.reason,gate=gate.core.fault,model=model.status)
                assert state.injected is not None and state.stopped is not None,dict(case=case,reason=sup.core.reason,gate=gate.core.fault)
                assert not state.before_click_nonzero and state.output_count>0
                assert not unexpected_active_faults,unexpected_active_faults
                assert state.max_speed<=.7+1e-6 and .005<state.max_wz<=c.max_yaw_rate_radps+1e-6
                assert state.cmd==ZERO and sup.core.state!='ACTIVE'
                assert not sup.core.request_start(time.monotonic(),source.get_clock().now().nanoseconds*1e-9)[0]
                if case=='normal_stop':assert state.positive and state.negative
                if case in ('source_digest','yaw_intent'):assert state.wire_fault_used
                if case=='source_digest':assert 'ROTATION_SELECTED_SOURCE_INVALID' in faults,faults
                if case=='yaw_intent':assert 'FINAL_REFERENCE_STEP_LIMIT' in faults,faults
                result=dict(case=case,passed=True,phases=phases,service_and_click_calls=calls,gate_faults=faults,unexpected_active_faults=unexpected_active_faults,
                    max_speed_mps=state.max_speed,max_yaw_radps=state.max_wz,commands=state.output_count,
                    zero_before_click=True,stopped_after_injection_s=state.stopped-state.injected,
                    no_automatic_resume=True,new_start_rejected_after_consumed_trial=True)
                results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
                (args.output.parent/(case+'_trace.json')).write_text(json.dumps(trace,ensure_ascii=False)+'\n')
            finally:
                executor.shutdown()
                for node in reversed(nodes):node.destroy_node()
        report=dict(passed=True,ros_domain=187,discovery='LOCALHOST',cases=results,
            actual_ros_nodes=True,actuator_backend='SOFTWARE Plant only',hardware_bridge_constructed=False,
            physical_car_started=False,firmware_changed=False,network_settings_changed=False,
            real_sign_occlusion_acceptance_pending=True,physical_rotation_acceptance_pending=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print('ALL PRIVATE ROS CASES PASS',flush=True)
    finally:rclpy.shutdown()


if __name__=='__main__':main()
