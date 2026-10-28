#!/usr/bin/env python3
"""Isolated callback/core diagnosis. Never initialize ROS or open a socket.

Uses the unmodified accepted runtime's actual RotationGateNode callbacks and
RotationFinalGate core. Source delivery/drop schedules are SOFTWARE fixtures,
not claims about the DDS delivery order in the physical capture.
"""
import argparse
from collections import OrderedDict,deque
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace as NS

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_feedback_300ms_startup_2026-10-27/prepared'
sys.path.insert(0,str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'))
from launch_cli import check
for package in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(READY/'runtime/src'/package))
import rclpy
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4ws8_control.rotation_core import RotationFinalGate
from mecanum_wand_4ws8_control.rotation_source import Packet,Raw,PacketCache,canonical,definition
from mecanum_wand_4ws8_control.rotation_geometry import REQUIRED
from mecanum_wand_4ws8_control.safety_core import Frame,Lease,ZERO
from mecanum_wand_4ws8_control.safety_nodes import pose_message,decode_pose,SENSOR,LATEST
from mecanum_wand_4ws8_hardware.rotation_nodes import RotationGateNode


def forbidden(*args,**kwargs):raise AssertionError('offline diagnosis forbids physical socket or ROS init')
socket.socket=forbidden
rclpy.init=forbidden


class Harness:
    def __init__(self,c):
        self.now=10.;self.seq=0;self.feedback_seq=0;self.lease_seq=0
        self.logs=[];self.outputs=[];self.trips=[];self.snapshots=[]
        self.car=Pose(12.,-8.,c.ground_z_m,0.)
        self.g=RotationGateNode.__new__(RotationGateNode)
        g=self.g;g.world='map';g.core=RotationFinalGate(c)
        g.packet_cache=PacketCache();g.cars=OrderedDict();g.pending=deque()
        g.clocks=lambda:(self.now,self.ros)
        g.car_pose_quality=NS(observe=lambda *a:None,reject=lambda *a:None)
        g.get_logger=lambda:NS(warn=self.logs.append)
        g.publish_output=self.outputs.append
        original=g.core.trip
        def trip(reason,hard=False):
            self.trips.append(dict(at=self.now,reason=reason,hard=hard))
            original(reason,hard)
        g.core.trip=trip
        # Each unmodified input contract receives genuine coherent software
        # samples; neutral prepare/activate is validated by the real core.
        for _ in range(60):
            self.now+=.02;v=self.source();self.refresh(False);self.candidate(v,False);self.tick()
        assert g.core.primed_epoch=='offline-epoch' and not g.core.fault,self.snapshots[-1]
        self.now+=.02;v=self.source();self.refresh(True);self.candidate(v,True);self.tick()
        assert g.core.pilot.used and not g.core.fault and not g.core.hard_fault,self.snapshots[-1]
        self.started=self.now

    @property
    def ros(self):return 90.+self.now

    def source(self,*,deliver_sign=True,deliver_car=True):
        d=definition();origin=(30.,-20.,1.2);stamp=self.ros-.006;received=self.now-.004
        raw=Raw('sign',origin,(0.,0.,0.,1.),stamp,received)
        points=[Raw(name,tuple(origin[i]+d['local_points'][name][i] for i in range(3)),
                    None,stamp,received) for name in REQUIRED[1:]]
        p=Packet((raw,*points))
        if deliver_sign:self.g.on_packet(String(data=canonical(p.payload())))
        stamp_car=self.ros-.005;sec=int(stamp_car);ns=round((stamp_car-sec)*1e9)
        msg=pose_message(self.car,Time(sec=sec,nanosec=ns),world='map')
        car_stamp=decode_pose(msg,'map')[1]
        if deliver_car:self.g.on_pose('car',msg)
        return dict(sign_digest=p.digest,sign_stamp=p.stamp,car_stamp=car_stamp,
            packet=p,car_message=msg)

    def refresh(self,held=True):
        self.feedback_seq+=1;self.lease_seq+=1;i=self.g.core.inputs
        assert i.observe_feedback(self.feedback_seq,self.now,self.ros,self.now,self.ros,
            output_zero=True,enabled=False,speed=0.,yaw_rate=0.,fault_bits=0)
        assert i.observe_lease(Lease('offline-operator',self.lease_seq,self.now,self.ros,held),self.now,self.ros)

    def candidate(self,v,active=True):
        self.seq+=1
        f=Frame('offline-supervisor',self.seq,self.now,self.ros,'offline-epoch',
            'ACTIVE' if active else 'ARMED',active,ZERO,self.car if active else None,False)
        self.g.on_candidate(String(data=canonical(dict(frame=asdict(f),
            **{k:v[k] for k in ('sign_digest','sign_stamp','car_stamp')}))))

    def tick(self):
        self.g.process_pending();output=self.g.core.tick(self.now,self.ros)
        s=dict(at=self.now,fault=self.g.core.fault,hard_fault=self.g.core.hard_fault,
            pending=len(self.g.pending),accepted_sequence=self.g.core.last_sequence,
            output_zero=output==ZERO,input_health=self.g.core.inputs.health(self.now,self.ros))
        self.snapshots.append(s);return s


def case(c,kind):
    h=Harness(c);t0=h.started
    if kind=='healthy':
        for _ in range(10):
            h.now+=.02;v=h.source();h.refresh();h.candidate(v);h.tick()
        assert not h.g.core.fault
    elif kind=='reordered_within_40ms':
        h.now=t0+.02;v=h.source(deliver_car=False);h.refresh();h.candidate(v);h.tick()
        assert len(h.g.pending)==1 and not h.g.core.fault
        h.now=t0+.04;h.refresh();h.g.on_pose('car',v['car_message']);h.tick()
        assert not h.g.pending and not h.g.core.fault
    else:
        h.now=t0+.02
        lost=h.source(deliver_sign=kind!='missing_sign',deliver_car=kind!='missing_car')
        if kind=='fault_reason_overwrite':
            # Lose the exact car sample, but keep later Vicon samples fresh.
            h.g.cars.pop(lost['car_stamp'])
        h.refresh();h.candidate(lost);h.tick()
        h.now=t0+.04;new=h.source();h.refresh();h.candidate(new);h.tick()
        assert len(h.g.pending)==2 and h.g.core.last_sequence==h.seq-2
        # The newer command's exact sources are already cached and valid, yet
        # it cannot be consumed because only pending[0] is considered.
        assert h.g.packet_cache.exact(new['sign_digest'],new['sign_stamp'],h.now,h.ros)
        assert new['car_stamp'] in h.g.cars
        if kind=='fault_reason_overwrite':
            h.now=t0+.12  # 80ms since last tick, inside the 100ms timer guard.
        else:h.now=t0+.07
        h.source();h.refresh();h.tick()
        expected='COMMAND_TIMEOUT' if kind=='fault_reason_overwrite' else 'ROTATION_SELECTED_SOURCE_INVALID'
        assert h.g.core.fault==expected,h.snapshots[-1]
        assert h.outputs and h.outputs[-1]==ZERO
        assert any('SELECTED_SOURCE_MISSING' in text for text in h.logs)
        if kind=='fault_reason_overwrite':
            assert [v['reason'] for v in h.trips[-2:]]==['ROTATION_SELECTED_SOURCE_INVALID','COMMAND_TIMEOUT']
            assert not h.g.core.hard_fault and not h.snapshots[-1]['input_health']
    return dict(case=kind,passed=True,source_fixture='coherent software poses/markers',
        final_fault=h.g.core.fault,trip_history=h.trips,detail_logs=h.logs,
        snapshots=h.snapshots[-4:],raw_pose_deadline_ms=200,feedback_deadline_ms=300,
        control_deadline_ms=100,selected_source_wait_ms=40)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    m=check(READY/'build_manifest.json')
    c=TrackingSafetyConfig(**json.loads(Path(m['draft_profiles'][4]).read_text())['safety'])
    results=[case(c,k) for k in ('healthy','reordered_within_40ms','missing_car','missing_sign','fault_reason_overwrite')]
    check(READY/'build_manifest.json')
    report=dict(passed=True,cases=results,actual_callbacks_and_core=True,ros_initialized=False,
        hardware_socket_opened=False,existing_demo_files_changed=False,
        qualification='Pure callback reproduction, NOT proof of which DDS stream lost the physical selected frame',
        subscription_qos=dict(car=dict(depth=SENSOR.depth,reliability=SENSOR.reliability.name),
                              rotation_packet=dict(depth=LATEST.depth,reliability=LATEST.reliability.name)),
        analysed_manifest=str(READY/'build_manifest.json'),
        analysed_manifest_sha256=hashlib.sha256((READY/'build_manifest.json').read_bytes()).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
