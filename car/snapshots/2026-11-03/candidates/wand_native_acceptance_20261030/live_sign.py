"""Actual Sign passive process/IPC -> SOFTWARE C car only. No motor transport."""
import argparse
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
from common import ROOT,HOST,HERE,OUT,config,save,protected,diagnostic as d
from mecanum_wand_4wsb_hardware.source_relay import PassiveReader,passive_child


def domain0_reader_child(*args):
    # Explicit separate process: production124/source0 becomes test191/source0.
    # Source provenance checks remain EXACT; never make domain191 acceptable
    # inside the domain0 Vicon reader or modify the sealed guard.
    os.environ['ROS_DOMAIN_ID']='0'
    passive_child(*args)


class SoftwarePassiveReader(PassiveReader):
    def __init__(self):
        super().__init__()
        self.process=mp.get_context('spawn').Process(target=domain0_reader_child,
            args=(self.channel,self.stop,self.ready),name='4wsb-software-passive-vicon',daemon=True)


def ensure_receiver(out):
    sys.path.insert(0,str(ROOT.parent/'vicon_receiver_ws/drone_sim/tools/gamepad_rc'))
    from vicon_receiver_provenance import verify_current,child_environment,recheck_before_spawn
    p=verify_current()
    existing=[]
    for x in Path('/proc').iterdir():
        if not x.name.isdecimal():continue
        try:
            if (x/'comm').read_text().strip()=='vicon_client':existing.append(x)
        except (OSError,ProcessLookupError):pass
    if existing:
        if len(existing)!=1:raise ValueError('multiple receiver processes; never stop/replace an external one')
        # Reuse the sealed original domain/executable/coordinate guard.
        from mecanum_wand_4wsb_control.rotation_receiver_guard import verify_existing
        before=os.environ.get('ROS_DOMAIN_ID')
        try:os.environ['ROS_DOMAIN_ID']='0';proof=verify_existing()
        finally:
            if before is None:os.environ.pop('ROS_DOMAIN_ID',None)
            else:os.environ['ROS_DOMAIN_ID']=before
        save(out/'RECEIVER.json',dict(started=False,existing=True,provenance=proof))
        return None
    env=child_environment(os.environ,p['policy'])
    logdir=out/'receiver_ros_logs';logdir.mkdir()
    env.update(ROS_DOMAIN_ID='0',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logdir))
    params=dict(hostname='10.10.10.2',buffer_size='200',namespace='vicon',world_frame='map',
        vicon_frame='vicon',map_xyz='[0.0,0.0,0.0]',map_rpy='[0.0,0.0,0.0]',
        map_rpy_in_degrees='false',stream_mode='ServerPush',enable_marker_ray_data='false',
        enable_debug_data='false',enable_device_data='false',publish_latency='true',capture_frame_samples='false')
    command=[p['installed'],'--ros-args']
    for k,v in params.items():command.extend(['-p',k+':='+v])
    recheck_before_spawn(p)
    with (out/'receiver.log').open('xb') as log:
        child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    save(out/'RECEIVER.json',dict(started=True,pid=child.pid,provenance=p,
        purpose='pose/marker read-only receiver; no car/device control'))
    return child


def qualify_reader(reader,seconds=10.):
    from mecanum_wand_4wsb_control.cached_packet import decode_packet
    if not math.isfinite(seconds) or not 0<seconds<=60:raise ValueError('finite probe duration in (0,60] required')
    begin=time.monotonic();packets=set();last=None;errors=[];car_seen=False;value=None
    while time.monotonic()-begin<seconds:
        value=reader.poll();now=time.monotonic()
        if value and not value['conflicts'] and value['packet']:
            try:
                # Original source and monotonic receipt times, never refreshed.
                ros=time.time()
                p=decode_packet(value['packet'],now,ros);p.pose(now,ros)
                packets.add(p.stamp);last=now
                car_seen|=value.get('car') is not None
            except (ValueError,TypeError,KeyError) as e:errors=(errors+[str(e)])[-10:]
        time.sleep(.005)
    return dict(ready=bool(len(packets)>=100 and last is not None and time.monotonic()-last<.2
                          and value and not value.get('conflicts')),
        complete_fresh_packets=len(packets),last_valid_age_s=time.monotonic()-last if last else None,
        car01_observed=car_seen,errors=errors[-10:],
        conflicts=value.get('conflicts') if value else ['no data'],
        provenance=reader.provenance,hardware_output=False)


class SoftwareCarReader:
    """Replace only car feedback; preserve raw Sign timestamps and conflicts."""
    def __init__(self,reader,pose,clock):
        self.reader,self.pose,self.clock=reader,pose,clock
        self.last_car_observed=None
    def start(self):pass
    def close(self):pass  # The owning runner closes its passive process.
    def poll(self):
        value=self.reader.poll()
        if value is None:return None
        self.last_car_observed=value.get('car')
        return dict(value,car=dict(pose=self.pose(),stamp=self.clock(),roll=0.,pitch=0.))


def probe(out,seconds):
    out.mkdir(parents=True,exist_ok=False);before=protected();receiver=reader=None
    try:
        receiver=ensure_receiver(out);reader=SoftwarePassiveReader();reader.start()
        result=qualify_reader(reader,seconds);save(out/'RESULT.json',result)
        print(json.dumps(result,ensure_ascii=False),flush=True);return result
    finally:
        if reader is not None:reader.close()
        if receiver is not None:d.terminate_owned(receiver)
        save(out/'PROTECTED.json',dict(unchanged=before==protected(),hardware_opened=False))


def run(out,stage):
    if os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise ValueError('private domain191 required before ROS initialization')
    out.mkdir(parents=True,exist_ok=False);before=protected();receiver=reader=spool=None
    procs=[];logs=[];nodes=[];executor=None;initialized=False
    namespace='/host4wsb_'+secrets.token_hex(5)
    from split_runtime import runtime
    rt=runtime(namespace,stage);key=secrets.token_hex(32);os.environ['WAND_4WSB_CHANNEL_KEY']=key
    os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    try:
        receiver=ensure_receiver(out);reader=SoftwarePassiveReader();reader.start()
        q=qualify_reader(reader,5.);save(out/'LIVE_READINESS.json',q)
        if not q['ready']:raise ValueError('no qualified fresh Sign; do not open a misleading ready UI')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from mecanum_wand_4wsb_control.core import Pose
        from mecanum_wand_4wsb_control.safety_nodes import LATEST
        from mecanum_wand_4wsb_hardware.source_relay import SourceRelay
        from native_twin import NativeTwin
        from trace_spool import TraceSpool
        rclpy.init(args=[],domain_id=191);initialized=True;executor=SingleThreadedExecutor()
        twin=NativeTwin(rt,ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready/native/libstage0_oracle.so',key,publish_car=False);nodes.append(twin)
        monitor=Node('software_live_sign_observer',namespace=namespace,enable_rosout=False);nodes.append(monitor)
        status={};gate={}
        monitor.create_subscription(String,'supervisor_status',lambda m:status.update(json.loads(m.data)),LATEST)
        monitor.create_subscription(String,'gate_status',lambda m:gate.update(json.loads(m.data)),LATEST)
        adapter=SoftwareCarReader(reader,
            lambda:asdict(Pose(*twin.controller.pose[:2],rt.config.ground_z_m,twin.controller.pose[2])),
            lambda:monitor.get_clock().now().nanoseconds*1e-9)
        source=SourceRelay(rt,adapter);nodes.append(source)
        for node in nodes:executor.add_node(node)
        env=dict(os.environ,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')
        host_here=ROOT/'candidates/wand_4wsb_host_20261030'
        for role in ('supervisor','gate'):
            log=(out/(role+'.log')).open('x');logs.append(log)
            procs.append(subprocess.Popen([sys.executable,str(host_here/'split_node.py'),'--namespace',namespace,
                '--stage',str(stage),'--role',role,'--output',str(out/(role+'_TIMING.json'))],env=env,stdout=log,stderr=log))
        log=(out/'console.log').open('x');logs.append(log)
        ui=subprocess.Popen([sys.executable,str(HERE/'live_sign_console.py'),'--namespace',namespace,
            '--stage',str(stage),'--output',str(out/'ui')],env=env,stdout=log,stderr=log);procs.append(ui)
        spool=TraceSpool(out/'LIVE_TRACE.jsonl');last_trace=0.;started=time.monotonic();states=[];last_state=None
        while time.monotonic()-started<720.:
            executor.spin_once(timeout_sec=.001);now=time.monotonic()
            if status.get('state')!=last_state:
                last_state=status.get('state');states.append(dict(at=now,state=last_state,reason=status.get('reason')))
            if now-last_trace>.02:
                spool.emit(dict(t=now,supervisor=status.copy(),gate=gate.copy(),car_software=twin.controller.pose.copy(),
                    car01_readonly=adapter.last_car_observed,duty=twin.controller.status['raw_duty'],
                    signed_fault=twin.policy.fault,hardware_output=False));last_trace=now
            if ui.poll() is not None:break
            if any(p.poll() is not None for p in procs[:2]):raise ValueError('critical software node exited')
        # Revoke only software authority. This has no physical IO path.
        twin.close();logging=spool.close();spool=None
        save(out/'RESULT.json',dict(scope='real Sign -> production ROS -> native C software car',states=states,
            logging=logging,native_metrics=twin.metrics,signed_fault=twin.policy.fault,car01_feedback_used=False,
            hardware_bridge_constructed=False,hardware_output=False,firmware_flashed=False,
            real_sign_user_acceptance=False,note='requires operator actions and trace analysis; no automatic pass'))
    except Exception as error:
        save(out/'ERROR.json',dict(error=repr(error),hardware_output=False,firmware_flashed=False))
        raise
    finally:
        if spool is not None:spool.close()
        for p in procs:d.terminate_owned(p)
        if executor is not None:executor.shutdown()
        for n in reversed(nodes):n.destroy_node()
        if initialized:
            import rclpy
            rclpy.shutdown()
        if reader is not None:reader.close()
        if receiver is not None:d.terminate_owned(receiver)
        for f in logs:f.close()
        save(out/'PROTECTED.json',dict(unchanged=before==protected(),hardware_output=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('probe','run'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--stage',type=int,choices=range(8),default=7);a=p.parse_args()
    if a.action=='probe':raise SystemExit(0 if probe(a.output,a.seconds)['ready'] else 2)
    else:run(a.output,a.stage)
