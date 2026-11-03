#!/usr/bin/env python3
"""Real ROS callbacks/services, isolated software inputs, NEVER hardware IO."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import secrets
import sys
import time
from types import SimpleNamespace as NS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--runtime',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='187':raise ValueError('Private rehearsal domain187 required')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':raise ValueError('Local discovery required')
    # Ephemeral key only for this private software DDS graph; no hardware
    # bridge or production transport is imported/constructed.
    os.environ['WAND_4WS8_CHANNEL_KEY']=secrets.token_hex(32)
    for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
        sys.path.insert(0,str(args.runtime/name))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped,Twist
    from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
    from mecanum_wand_4ws8_control.core import Pose
    from mecanum_wand_4ws8_control.safety_nodes import SupervisorNode,GateNode,pose_message,LATEST,SENSOR
    from mecanum_wand_4ws8_control.operator_console import OperatorNode
    from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
    from mecanum_wand_4ws8_control.auto_prepare import AutoPrepareOnce
    profile=args.runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'
    c=StagedSafetyConfig(**json.loads(profile.read_text())['safety'])
    namespace='/startup_fixed_software_rehearsal'
    def guard(node):
        if node.get_namespace()!=namespace or node.context.get_domain_id()!=187:
            raise ValueError('Private rehearsal scope violation')
    runtime=NS(namespace=namespace,world='startup_software_map',config=c,mode='shadow',guard=guard,
        expected={'car_pose':'software_sources','wand_pose':'software_sources',
            'plant_feedback':'software_sources','operator_lease':'wand_real_operator',
            'candidate':'safe_wand_supervisor','cmd_vel':'safe_wand_gate','actuator_command':'safe_wand_gate'})
    rclpy.init(args=[]);executor=SingleThreadedExecutor();nodes=[];calls=[];outputs=[];leases=[];phases=[]
    result=None
    try:
        source=Node('software_sources',namespace=namespace,enable_rosout=False)
        model=ToggleOperatorModel();model.focused=True
        operator=OperatorNode(model,runtime);sup=SupervisorNode(runtime);gate=GateNode(runtime)
        nodes=[source,operator,sup,gate]
        for node in nodes:executor.add_node(node)
        posepub={key:source.create_publisher(PoseStamped,key+'_pose',SENSOR) for key in ('car','wand')}
        pubs={key:source.create_publisher(String,key,LATEST) for key in ('plant_feedback','mode_claim','source_status','transport_status')}
        source.create_subscription(Twist,'cmd_vel',lambda m:outputs.append((m.linear.x,m.linear.y,m.angular.z)),LATEST)
        source.create_subscription(String,'operator_lease',lambda m:leases.append(json.loads(m.data)),LATEST)
        # Actual FinalGate boot condition, not a fabricated status publisher.
        gate.core.trip('COMMAND_TIMEOUT')
        policy=AutoPrepareOnce();state=NS(injected=False,ready_at=None,blocked_at=None,sequence=0)
        car=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad)
        def fixture_tick():
            now=time.monotonic();ros=source.get_clock().now().nanoseconds*1e-9
            stamp=source.get_clock().now().to_msg();state.sequence+=1
            posepub['car'].publish(pose_message(car,stamp,world=runtime.world))
            posepub['wand'].publish(pose_message(Pose(1.,2.,1.),stamp,world=runtime.world))
            pubs['mode_claim'].publish(String(data='wand'))
            def send(name,value):pubs[name].publish(String(data=json.dumps(value)))
            send('source_status',dict(issued_steady=now,issued_ros=ros,rejected={},conflicts=[]))
            send('plant_feedback',dict(sequence=state.sequence,issued_steady=now,issued_ros=ros,
                output_zero=True,enabled=False,speed=0.,yaw_rate=0.,fault_bits=0))
            # Explicit SOFTWARE stub, not real MCU telemetry/clock qualification.
            send('transport_status',dict(issued_steady=now,issued_ros=ros,fresh=True,ready=True,
                local_stop=False,fault='',startup_clock=dict(ready=True,qualified=True,fault=''),
                baseline_current=True,command_progress_age_s=0.,safety_enabled=False,motion_gate_requested=False))
        source.create_timer(.01,fixture_tick)
        def operator_tick():
            now=time.monotonic();operator.heartbeat()
            if policy.poll(model,now,space_down=False,
                    service_ready=operator.service_clients['start'].service_is_ready() and 'start' not in operator.pending,
                    input_issue=operator.input_hint()):
                calls.append('start')
                if not operator.request('start'):policy.cancel('rehearsal service request rejected')
            if not phases or phases[-1]!=policy.phase:phases.append(policy.phase)
            if policy.phase=='READY' and state.ready_at is None:state.ready_at=now
            if policy.cancelled and state.blocked_at is None:
                state.blocked_at=now;model.end_trial(now,'software rehearsal stop')
                if policy.attempted:calls.append('stop');operator.request('stop')
        operator.create_timer(.01,operator_tick)
        start=time.monotonic()
        while time.monotonic()-start<10.:
            executor.spin_once(timeout_sec=.005)
            if state.ready_at is not None and not state.injected and time.monotonic()-state.ready_at>.35:
                # Once READY the same timeout is a real fault, NEVER startup wait.
                gate.core.trip('COMMAND_TIMEOUT');state.injected=True
            if state.blocked_at is not None and time.monotonic()-state.blocked_at>.5:break
        assert state.ready_at is not None,dict(phase=policy.phase,reason=policy.reason,status=model.status,gate=model.gate)
        assert state.injected and policy.cancelled and model.ended
        assert calls==['start','stop'],calls
        assert outputs and all(not any(v) for v in outputs)
        assert leases and not any(v['held'] for v in leases)
        assert not sup.core.pilot.used and not model.activation_used
        result=dict(passed=True,real_ros_nodes=[n.get_name() for n in nodes],ros_domain=187,
            namespace=namespace,phase_history=phases,service_calls=calls,
            initial_timeout_reached_ready=True,post_ready_timeout_locked=True,
            nonzero_outputs=0,enabled_leases=0,command_samples=len(outputs),lease_samples=len(leases),
            physical_hardware_connected=False,raw_vicon_used=False,feedback='declared software ZERO stub',
            firmware_changed=False,network_settings_changed=False)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)
    finally:
        executor.shutdown()
        for node in nodes:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
