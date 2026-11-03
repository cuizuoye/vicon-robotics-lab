"""Bounded real Vicon check with the qualified 4WSC packet geometry. No actuator IO."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_4wsc_browser_2026-11-02/prepared_v12'


def main(output,seconds):
    if output.exists() or not 2<=seconds<=15:raise ValueError('new output folder and 2..15s required')
    output.mkdir(parents=True)
    sys.path.insert(0,str(ROOT/'candidates/wand_4wsc_browser_20261102'))
    from entry import check
    check(READY)
    sys.path.insert(0,str(ROOT.parent/'vicon_receiver_ws/drone_sim/tools/gamepad_rc'))
    from vicon_receiver_provenance import verify_current,child_environment,recheck_before_spawn
    proof=verify_current()
    sys.path.insert(0,str(READY));import bootstrap
    bootstrap.install()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsc_control.rotation_receiver_guard import verify_existing
    from mecanum_wand_4wsc_hardware.rotation_nodes import RotationRawSource
    owned=None;node=None;executor=None;context=Context();rows=[];conflicts=['not discovered'];errors=[]
    existing=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal():continue
        try:
            if (p/'comm').read_text().strip()=='vicon_client':existing.append(p)
        except OSError:pass
    if len(existing)>1:raise ValueError('multiple receivers; do not alter external processes')
    try:
        if not existing:
            params=dict(hostname='10.10.10.2',buffer_size='200',namespace='vicon',world_frame='map',
                vicon_frame='vicon',map_xyz='[0.0,0.0,0.0]',map_rpy='[0.0,0.0,0.0]',
                map_rpy_in_degrees='false',stream_mode='ServerPush',enable_marker_ray_data='false',
                enable_debug_data='false',enable_device_data='false',publish_latency='true',capture_frame_samples='false')
            cmd=[proof['installed'],'--ros-args']
            for k,v in params.items():cmd+=['-p',k+':='+v]
            env=child_environment(os.environ,proof['policy']);env.update(ROS_DOMAIN_ID='0',
                ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(output/'receiver_ros_logs'))
            recheck_before_spawn(proof)
            with (output/'receiver.log').open('xb') as log:
                owned=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        os.environ['ROS_DOMAIN_ID']='0'
        provenance=verify_existing()
        rclpy.init(context=context,domain_id=0,signal_handler_options=SignalHandlerOptions.NO)
        node=RotationRawSource(context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        start=time.monotonic();packets=set();cars=set();audit=0.;latest_car=None;last=None
        while time.monotonic()-start<seconds:
            executor.spin_once(timeout_sec=.001)
            now=time.monotonic();ros=node.get_clock().now().nanoseconds*1e-9
            if now-audit>=.1:conflicts=node.audit();audit=now
            car=node.samples.get('car')
            if car is not None:
                from mecanum_wand_4wsc_control.safety_nodes import decode_pose
                pose,stamp,roll,pitch=decode_pose(car,'map')
                if -.005<=ros-stamp<=.2:
                    cars.add(stamp);latest_car=dict(x=pose.x,y=pose.y,z=pose.z,raw_yaw=pose.yaw,roll=roll,pitch=pitch,age_s=ros-stamp)
            if node.packet is not None and node.packet.stamp not in packets:
                try:
                    pose=node.packet.pose(now,ros);packets.add(node.packet.stamp);last=now
                    rows.append(dict(at=now,source_stamp=node.packet.stamp,age_s=ros-node.packet.stamp,
                        sign=dict(x=pose.x,y=pose.y,z=pose.z,yaw=pose.yaw),car=latest_car))
                except ValueError as e:errors=(errors+[str(e)])[-10:]
        result=dict(passed=bool(len(packets)>=100 and len(cars)>=100 and last and time.monotonic()-last<=.2 and not conflicts),
            duration_s=time.monotonic()-start,complete_Sign_frames=len(packets),fresh_car01_frames=len(cars),
            conflicts=conflicts,rejected=dict(node.rejected),errors=errors,last=rows[-1] if rows else None,
            receiver_provenance=provenance,receiver_started_by_probe=owned is not None,
            receiver_stopped_after_probe=owned is not None,motor_socket_opened=False,enable_sent=False,motion_started=False)
        (output/'SAMPLES.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
        (output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(dict(result,output=str(output)),ensure_ascii=False),flush=True)
        return 0 if result['passed'] else 1
    except Exception as e:
        (output/'RESULT.json').write_text(json.dumps(dict(passed=False,error=repr(e),motor_socket_opened=False,
            enable_sent=False,motion_started=False),ensure_ascii=False,indent=2)+'\n')
        raise
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()
        if owned is not None and owned.poll() is None:
            owned.terminate()
            try:owned.wait(timeout=3)
            except subprocess.TimeoutExpired:owned.kill();owned.wait(timeout=2)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=8.)
    a=p.parse_args();raise SystemExit(main(a.output.resolve(),a.seconds))
