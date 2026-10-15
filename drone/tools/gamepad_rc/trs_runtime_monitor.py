#!/usr/bin/env python3
"""Persistent single-TRS/Vicon/HOME integration foundation, NOT a flight runner.

Never sends poses, targets, mode changes, arm/unlock, parameter writes or rate
changes. HOME can ONLY lock an identified, freshly DISARMED PX4 1.14.3. It
continues observing after HOME release, unlike the one-shot benches. This
entry does not claim airborne emergency-stop or independent-PC safety.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import serial
from pymavlink.dialects.v20 import common as m
from estop_only_policy import (DurableStopLatch, FIRMWARE, LISTENER, LOCK_COMMAND,
    lock_message, parse_actuator_snapshot, shell_message, shell_message_allowed,
    stop_message_allowed)
from gamepad_estop_monitor import EVENT, HOME, find_pad
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence, readonly_message
from persistent_shell import reply_complete
from trs_battery_readiness import PARAMS, parameters_match
from trs_only_estop_bench import state_valid
from trs_uart_readonly_check import UID2
from rigid_body_revision import inspect_current_calibration, digest


def allowed(msg, *, identity_fresh=False, home_latched=False):
    """A live transport whitelist, not advisory mission intents."""
    kind = msg.get_type()
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(identity_fresh and name in PARAMS and
                    (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1))
    if kind == 'SERIAL_CONTROL':
        return bool(identity_fresh and shell_message_allowed(msg))
    if stop_message_allowed(msg, home_latched, True):
        return bool(identity_fresh)
    return readonly_message(msg)


def observation_age(now, last):
    return now-last if last is not None and now >= last else None


def observed_rate(times):
    return ((len(times)-1)/(times[-1]-times[0])
            if len(times) >= 2 and times[-1] > times[0] else None)


def readiness_fault(previous, *, baseline_seen, joint_seen, identity_fresh, source_fresh):
    """Do not silently resume readiness after a transient; keep diagnosis alive."""
    if previous is not None:return previous
    if baseline_seen and not identity_fresh:return 'fc_heartbeat_stale_after_ready'
    if joint_seen and not source_fresh:return 'vicon_source_lost_after_ready'
    return None


def check_physical(mode):
    if (mode == 'usb-only') != FC_USB.exists():
        raise RuntimeError('Physical mode differs from FC USB presence; no automatic reconnect/changeover')


def inspect_evidence(heading, board, proof_path):
    registry = inspect_current_calibration(heading, board)
    proof = json.loads(Path(proof_path).read_text())
    permit_path = Path(proof['vision_reference_permit_path'])
    permit = json.loads(permit_path.read_text())
    consumed = json.loads(Path(str(permit_path)+'.consumed.json').read_text())
    if (proof.get('rigid_body_revision_id') != registry['active_revision_id']
            or proof.get('rigid_revision_static_validation') is not True
            or proof.get('ekf_stationary_xyz_yaw_fusion_verified') is not True
            or proof.get('error') or proof.get('restore_errors')
            or proof.get('arming_mode_motion_or_unlock_commands_sent') is not False
            or proof.get('restored') != {'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
            or proof.get('actuator_after', {}).get('armed') is not False
            or proof.get('actuator_after', {}).get('lockdown') is not True
            or any(proof.get('identities',{}).get(route,{}).get('uid2') != UID2 for route in ('usb','trs'))
            or Path(proof['heading_candidate_path']).resolve() != Path(heading).resolve()
            or Path(proof['board_center_candidate_path']).resolve() != Path(board).resolve()
            or Path(permit['output_path']).resolve() != Path(proof_path).resolve()
            or permit.get('heading_sha256') != digest(heading)
            or permit.get('board_sha256') != digest(board)
            or consumed.get('permit_sha256') != digest(permit_path)):
        raise ValueError('Same-revision passing static proof and consumed candidate-bound permit required')
    return registry


def run(args):
    check_physical(args.physical_mode)
    if not args.props_installed_monitor_stop_only_confirmed:
        raise ValueError('Operator-confirmed stationary physical mode required; never a flight authorization')
    for path in (args.output, args.status_log, args.latch_file):
        if path.exists():
            raise ValueError('New paths required; retaining old reports and stop latches')
    registry = inspect_evidence(args.heading, args.board, args.static_proof)
    h, b = [json.loads(Path(p).read_text()) for p in (args.heading, args.board)]
    if subprocess.run(['fuser',str(Path(args.trs_port).resolve())],capture_output=True).returncode == 0:
        raise ValueError('TRS already occupied; no process killed')
    device = find_pad()
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
    from uav_coord_bridge.mavlink_vision import PoseGate, transform_rigid_pose
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('single_trs_runtime_monitor')
            self.gate = PoseGate(max_age_ms=150)
            self.subscription_kept = self.create_subscription(PoseStamped,
                '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))

        def receive(self,msg):
            p,q = msg.pose.position,msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds,time.monotonic_ns(),msg.header.frame_id,
                [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    rclpy.init(args=[])
    node = Source()
    ev,power = Evidence(250,202,expected_parameters=tuple(PARAMS)), PassivePowerEvidence()
    enc,dec = m.MAVLink(None,srcSystem=250,srcComponent=202),m.MAVLink(None)
    dec.robust_parsing=True
    latch=DurableStopLatch(args.latch_file,UID2)
    start=time.monotonic()
    tx,commands=Counter(),Counter()
    snapshots,acks,events,statustext,local_times=[],[],[],[],[]
    last_attitude=last_local=last_estimator=last_land=None
    buffer='';pending_at=None
    next_hb=next_version=next_params=next_shell=next_poll=next_status=0
    next_retry=0;attempts=0;request_at=release_at=None
    ready=joint_ready=lock_after_home=False
    pad_open=False
    error=None;failure_latched=None;last_status=None
    max_loop_gap=0.;previous_loop=None;bad_packets=[]

    def elapsed():return time.monotonic()-start

    def identity_fresh():
        return bool(ev.identity and ev.identity['uid2']==UID2 and ev.identity['firmware']==FIRMWARE
            and not ev.fatal and not power.reboot_observed and ev.heartbeats
            and not ev.heartbeats[-1]['armed'] and 0<=elapsed()-ev.heartbeats[-1]['elapsed_s']<2)

    def send(port,msg):
        if not allowed(msg,identity_fresh=identity_fresh(),home_latched=latch.latched):
            raise RuntimeError('Monitor-stop-only transport gate rejected '+msg.get_type())
        packet=msg.pack(enc)
        if port.write(packet)!=len(packet):raise OSError('Short serial write')
        enc.seq=(enc.seq+1)%256;tx[msg.get_type()]+=1
        if msg.get_type()=='COMMAND_LONG':commands[str(msg.command)]+=1

    def status():
        t=elapsed();fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
        position=None
        if fresh:
            sample=node.gate.last
            position,_=transform_rigid_pose(sample.position,sample.quaternion,
                b['board_center_in_rigid_m_candidate'],h['body_orientation_in_rigid_xyzw_candidate'])
        local_age=observation_age(t,last_local['at'] if last_local else None)
        return {'elapsed_s':round(t,3),'identity_fresh_disarmed':identity_fresh(),
            'vicon_fresh':fresh,'vicon_accepted':node.gate.accepted,'vicon_rejected':node.gate.rejected,
            'live_lockdown':snapshots[-1]['lockdown'] if snapshots else None,
            'actuator_observation_age_s':observation_age(t,snapshots[-1]['at'] if snapshots else None),
            'stop_latched':latch.latched,'home_press_observed':request_at is not None,
            'home_release_lock_verified':lock_after_home,
            'last_home_event_age_s':observation_age(t,events[-1]['at'] if events else None),
            'gamepad_fd_present_not_awake_proof':pad_open,'sleep_detection_present':False,
            'local_position_age_s':local_age,'local_position_rate_hz':observed_rate(local_times),
            'local_position_within_500ms':local_age is not None and local_age<=.5,
            'estimator_status_age_s':observation_age(t,last_estimator['at'] if last_estimator else None),
            'estimator_flags':last_estimator['flags'] if last_estimator else None,
            'landed_state':last_land['state'] if last_land else None,
            'parameters_match_maintenance_baseline':parameters_match(ev.parameters),
            'preview_approximate_board_position_frd_m':position,
            'preview_is_not_a_control_mapping':True,'monitor_fault_latched':failure_latched,
            'physical_mode':args.physical_mode,'fc_usb_opened_by_this_program':False,
            'pose_messages_sent':0,'motion_targets_sent':0,'parameter_writes_sent':0,
            'flight_readiness_approved':False}

    try:
        with ExitStack() as stack:
            port=stack.enter_context(serial.Serial(args.trs_port,115200,timeout=.002,write_timeout=.3,exclusive=True))
            log=stack.enter_context(args.status_log.open('x'))
            fd=os.open(device['path'],os.O_RDONLY|os.O_NONBLOCK);stack.callback(os.close,fd)
            pad_open=True
            inode=os.fstat(fd).st_rdev
            bitmap=fcntl.ioctl(fd,0x80000000|(96<<16)|(ord('E')<<8)|0x18,bytes(96))
            if bitmap[HOME//8]&(1<<(HOME%8)):raise RuntimeError('Release HOME before monitor starts')
            pending_input=b''
            port.reset_input_buffer()
            while elapsed()<args.seconds:
                t=elapsed();check_physical(args.physical_mode)
                if previous_loop is not None:max_loop_gap=max(max_loop_gap,t-previous_loop)
                previous_loop=t
                if os.stat(device['path']).st_rdev!=inode:raise RuntimeError('Gamepad device changed')
                rclpy.spin_once(node,timeout_sec=0)
                for msg in dec.parse_buffer(port.read(4096)) or []:
                    ev.ingest(msg,t);power.ingest(msg,t)
                    if msg.get_type()=='BAD_DATA' and len(bad_packets)<50:
                        bad_packets.append({'at':t,'reason':str(getattr(msg,'reason','')),
                            'data_hex':bytes(getattr(msg,'data',b''))[:96].hex()})
                    if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
                    kind=msg.get_type()
                    if kind=='ATTITUDE_QUATERNION':last_attitude={'at':t,'boot_ms':msg.time_boot_ms}
                    elif kind=='LOCAL_POSITION_NED':
                        values=[msg.x,msg.y,msg.z,msg.vx,msg.vy,msg.vz]
                        if not all(math.isfinite(x) for x in values):raise RuntimeError('Non-finite local telemetry')
                        last_local={'at':t,'boot_ms':msg.time_boot_ms,'position':values[:3],'velocity':values[3:]}
                        local_times.append(t)
                    elif kind=='ESTIMATOR_STATUS':last_estimator={'at':t,'flags':msg.flags}
                    elif kind=='EXTENDED_SYS_STATE':last_land={'at':t,'state':msg.landed_state}
                    elif kind=='STATUSTEXT':statustext.append({'at':t,'text':msg.text})
                    elif kind=='COMMAND_ACK' and msg.command==LOCK_COMMAND and request_at is not None:
                        if (msg.target_system,msg.target_component)==(250,202):
                            acks.append({'at':t,'result':msg.result})
                            if msg.result!=m.MAV_RESULT_ACCEPTED:raise RuntimeError('HOME output lock rejected')
                    elif kind=='SERIAL_CONTROL' and pending_at is not None:
                        buffer+=bytes(msg.data[:msg.count]).decode('utf8','replace')
                        snapshot=parse_actuator_snapshot(buffer)
                        if snapshot and reply_complete(buffer,LISTENER):
                            boot=last_attitude['boot_ms'] if last_attitude else None
                            if not state_valid(snapshot,boot):raise RuntimeError('Stale or unsafe actuator state')
                            snapshots.append({**snapshot,'at':t,'requested_at':pending_at})
                            buffer='';pending_at=None
                            if not ready:
                                ready=True
                                print('READY: 单TRS常驻监控已就绪；HOME可短按，仅锁定未解锁输出；松开后继续监控。',flush=True)
                            if (acks and release_at is not None and snapshot['requested_at']>=release_at+1.5
                                    and snapshot['lockdown'] and not lock_after_home):
                                lock_after_home=True
                                print('HOME PASS: TRS ACK接受，松开后内部lockdown=True；常驻监控继续运行。',flush=True)
                if ev.fatal or power.reboot_observed:raise RuntimeError(ev.fatal or 'FC reboot; session invalidated')
                if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!=FIRMWARE):
                    raise RuntimeError('Unexpected FC identity')
                # A monitor may continue receiving after a stale heartbeat,
                # but the send gate still requires <2s heartbeat age. The
                # readiness fault stays latched after recovery; never auto-arm.
                source_fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
                fault=readiness_fault(failure_latched,baseline_seen=ready,joint_seen=joint_ready,
                    identity_fresh=identity_fresh(),source_fresh=source_fresh)
                if fault is not None and failure_latched is None:
                    failure_latched=fault;latch.fail(fault)
                try:
                    chunk=os.read(fd,EVENT.size*64)
                    if not chunk:raise OSError('Gamepad disconnected')
                    pending_input+=chunk
                except BlockingIOError:pass
                while len(pending_input)>=EVENT.size:
                    _,_,kind,code,value=EVENT.unpack(pending_input[:EVENT.size]);pending_input=pending_input[EVENT.size:]
                    if kind==0 and code==3:raise RuntimeError('Gamepad event queue overflow')
                    if kind==1 and code==HOME:
                        if value in (1,2) and not ready:raise RuntimeError('HOME before identified actuator baseline')
                        latch.event(kind,code,value);events.append({'at':t,'value':value})
                        if value==1 and request_at is None:
                            request_at=t;next_retry=t
                        elif value==0 and request_at is not None:release_at=t
                # Real HOME stop is ahead of diagnostics; never clear the lock.
                if request_at is not None and not acks and t>=next_retry and attempts<3 and identity_fresh():
                    send(port,lock_message(enc,ev.identity,UID2,latch,True,identity_fresh(),confirmation=attempts))
                    attempts+=1;next_retry=t+.8
                if request_at is not None and not lock_after_home and t-request_at>10:
                    raise RuntimeError('HOME ACK/release-lock observation timeout')
                if t>=next_hb:
                    send(port,enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
                if ev.heartbeats and ev.identity is None and t>=next_version:
                    ev.version_requested_at=t
                    send(port,enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=t+3
                if identity_fresh() and t>=next_params:
                    for name in PARAMS:
                        ev.requests[name]=t;send(port,enc.param_request_read_encode(1,1,name.encode(),-1))
                    next_params=t+8
                if any(k in ev.parameters and ev.parameters[k]!=v for k,v in PARAMS.items()):
                    raise RuntimeError('Maintenance configuration changed; monitor will not correct it')
                if identity_fresh() and pending_at is None and t>=next_shell:
                    send(port,shell_message(enc));pending_at=t;buffer='';next_shell=t+1
                if pending_at is not None:
                    if t-pending_at>6 or len(buffer)>16384:raise RuntimeError('Fixed actuator reply incomplete')
                    if t>=next_poll and identity_fresh():send(port,shell_message(enc,poll=True));next_poll=t+.3
                fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
                if ready and parameters_match(ev.parameters) and fresh:joint_ready=True
                if t>=next_status:
                    last_status=status();log.write(json.dumps(last_status,ensure_ascii=False)+'\n');log.flush()
                    # Detailed telemetry is file-backed, not a potentially blocking
                    # terminal stream in the serial owner/control event loop.
                    next_status=t+5
                if not ready and t>15:raise RuntimeError('Initial TRS/actuator readiness timeout')
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc) or 'Operator interrupted monitor'
        latch.fail('monitor_ended: '+error)
    finally:
        pad_open=False
        last_status=status()
        node.destroy_node()
        # rclpy's SIGINT handler may already have shut the context down.
        # A second shutdown must not prevent the final evidence report.
        if rclpy.ok():rclpy.shutdown()
    result={'scope':__doc__,'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'physical_mode':args.physical_mode,'propeller_state':'installed','physical_conditions_source':'operator report',
        'rigid_body_revision_id':registry['active_revision_id'],'duration_s':elapsed(),
        'device':device,'identity':ev.identity,'last_status':last_status,
        'snapshots':snapshots,'home_events':events,'home_acks':acks,'statustext':statustext,
        'passive_power':power.result(),'parameters':ev.parameters,'tx_messages':dict(tx),
        'command_counts':dict(commands),'local_position_samples':len(local_times),
        'heartbeats':ev.heartbeats,'received_fc_message_counts':dict(ev.fc_counts),
        'max_reactor_loop_gap_s':max_loop_gap,'bad_packet_examples':bad_packets,
        'local_position_rate_hz':observed_rate(local_times),'home_release_lock_verified':lock_after_home,
        'fc_usb_opened_by_this_program':False,'pose_messages_sent':0,'motion_targets_sent':0,
        'parameter_writes_sent':0,'flight_adapter_enabled':False,'airborne_emergency_stop_verified':False,
        'continuous_monitor_inputs_healthy':bool(joint_ready and not error and not failure_latched),
        'error':error,'serial_port_released':True,'monitor_running_after_exit':False}
    with args.output.open('x') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    print('MONITOR EXIT: '+json.dumps({'error':error,'home_release_lock_verified':lock_after_home,
                                     'serial_port_released':True,'flight_adapter_enabled':False}),flush=True)
    return 0 if not error and not failure_latched else 2


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('trs-port','heading','board','static-proof','output','status-log','latch-file'):
        ap.add_argument('--'+name,type=str if name=='trs-port' else Path,required=True)
    ap.add_argument('--physical-mode',choices=('usb-only','battery-only'),required=True)
    ap.add_argument('--props-installed-monitor-stop-only-confirmed',action='store_true',required=True)
    ap.add_argument('--seconds',type=float,default=600)
    args=ap.parse_args()
    if not 10<=args.seconds<=3600:ap.error('Use a bounded 10..3600 second monitor session')
    try:return run(args)
    except (ValueError,OSError,RuntimeError,KeyError) as exc:ap.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
