#!/usr/bin/env python3
"""Replay recorded preparation poses with a synthetic stopped car, no ROS/UDP.

Raw wand samples are selected by their recorded order; no live subscription,
interpolation, timestamp repair or hardware action is used. The post-engage
5cm movement is explicitly synthetic and is not a physical acceptance result.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
EVIDENCE=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ui_fix_v2_ready/ground_stage4_single_nn_at6cn/observation.jsonl'


def run(runtime):
    for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
        sys.path.insert(0,str(runtime/name))
    from mecanum_wand_4ws8_control.pilot_core import PilotSupervisor,PilotFinalGate
    from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
    from mecanum_wand_4ws8_control.core import Pose
    from mecanum_wand_4ws8_control.safety_core import ZERO
    arm=stop=None;poses=[]
    with EVIDENCE.open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']=='wand_pose':poses.append(row)
            elif row['topic']=='supervisor_status':
                d=row['data']
                if arm is None and d['state']=='ARMED':arm=row
                if stop is None and d['reason']=='PILOT_HAND_LIMIT':stop=row
    if arm is None or stop is None:raise ValueError('recorded preparation/failed enable missing')
    start,end=arm['received_steady'],stop['received_steady']
    previous=[r for r in poses if r['received_steady']<=start][-1]
    sequence=[previous]+[r for r in poses if start<r['received_steady']<=end]
    cfg=StagedSafetyConfig(**json.loads((runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])
    spec=importlib.util.spec_from_file_location('mecanum_wand_4ws8_control.recorded_origin_harness',
        ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests/offline_harness.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.Supervisor,module.FinalGate=PilotSupervisor,PilotFinalGate
    actual_car=Pose(**stop['data']['anchors']['car_origin'])
    h=module.Harness(cfg,origin_local=cfg.geometry().local(actual_car.x,actual_car.y))
    h.plant.pose=h.origin=actual_car
    p=sequence[0]['data'];h.hand=Pose(p['x'],p['y'],p['z'])
    h.run(1.8);ok,reason=h.sup.request_start(h.now,h.ros)
    if not ok:raise ValueError('synthetic replay preparation failed: '+reason)
    h.run(.1);idx=0
    for step in range(1,math.ceil((end-start)/.02)+1):
        at=min(end,start+step*.02)
        while idx+1<len(sequence) and sequence[idx+1]['received_steady']<=at:idx+=1
        p=sequence[idx]['data'];h.hand=Pose(p['x'],p['y'],p['z'])
        h.step()
        if h.sup.state!='ARMED' or h.output!=ZERO:raise ValueError('replay moved or failed while preparing: '+h.sup.reason)
    hold=h.hand;initial=sequence[0]['data']
    h.held=True;out=h.step()
    result=dict(physical_hardware_measurement=False,recorded_prepare_s=end-start,
        recorded_pre_enable_xy_m=math.hypot(hold.x-initial['x'],hold.y-initial['y']),
        recorded_pre_enable_z_m=hold.z-initial['z'],enable_state=h.sup.state,
        enable_reason=h.sup.reason,enable_command_zero=out==ZERO,
        supervisor_used=h.sup.pilot.used,gate_used=h.gate.pilot.used,
        hardware_started=False,live_vicon_opened=False)
    if h.sup.state=='ACTIVE':
        assert out==ZERO
        h.run(.2);assert h.output==ZERO
        c,s=math.cos(h.origin.yaw),math.sin(h.origin.yaw)
        for i in range(1,16):
            delta=.05*i/15
            h.hand=Pose(hold.x+c*delta,hold.y+s*delta,hold.z)
            h.step()
        goal=h.sup.follower.mapped_goal
        result.update(post_enable_5cm_command_nonzero=h.output!=ZERO,
            mapped_goal_displacement_m=math.hypot(goal.x-actual_car.x,goal.y-actual_car.y),
            supervisor_origin_matches_mapping=h.sup.pilot.wand==h.sup.follower.mapping.wand_origin,
            shared_start_steady=h.sup.pilot.started==h.gate.pilot.started)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--runtime',type=Path,required=True)
    a=parser.parse_args()
    import socket
    def forbidden(*args,**kwargs):raise AssertionError('offline replay: sockets prohibited')
    socket.socket=forbidden
    import rclpy
    rclpy.init=forbidden
    print(json.dumps(run(a.runtime.resolve()),ensure_ascii=False))


if __name__=='__main__':main()
