#!/usr/bin/env python3
"""真实Vicon→TRS→飞控接收台架：融合关闭，20Hz/5秒；USB仅内部只读观察。"""

import argparse
from collections import Counter
import datetime
import json
import math
from pathlib import Path
import secrets
import subprocess
import sys
import time

import numpy as np
import serial
from persistent_shell import reply_complete
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import (PoseGate, checked_quaternion, finite_vector,
                                          odometry_message, transform_rigid_pose)
from mavlink_readonly_probe import Evidence, decode_parameter, readonly_message
from estop_only_policy import parse_actuator_snapshot
from usb_vision_policy import (PARAMS, fusion_disabled, send_allowed, shell_message,
                              parse_visual_snapshot, match_snapshot, parse_timesync_snapshot, ClockReadiness)
from trs_uart_readonly_check import UID2, USB
import compact_vision as compact
from incident_guard import require_active_bench_clear
from rigid_body_revision import require_current_calibration

TRS = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'


def allowed(message, *, route, safe=False, bench_confirmed=False, compact_timing=False):
    """旧USB/融合策略不放宽：新的例外仅允许TRS、融合关闭、短时接收诊断。"""
    if route not in ('usb', 'trs'):
        return False
    if compact_timing and route == 'usb' and message.get_type() == 'SERIAL_CONTROL':
        p=compact.SERIES_COMMAND
        if ((message.device,message.flags,message.timeout,message.baudrate)==(10,2,0,0)
                and message.count==len(p) and bytes(message.data)==p.ljust(70,b'\0')):
            return bool(safe)
    if message.get_type() == 'VISION_POSITION_ESTIMATE':
        return bool(route == 'trs' and compact_timing and bench_confirmed and safe
                    and compact.pose_allowed(message))
    if message.get_type() == 'ODOMETRY':
        return not compact_timing and route == 'trs' and bench_confirmed and send_allowed(
            message, safe=safe, bench_authorized=True)
    if message.get_type() in ('PARAM_SET', 'RC_CHANNELS_OVERRIDE', 'SET_POSITION_TARGET_LOCAL_NED'):
        return False
    if route == 'trs':
        return readonly_message(message) or (message.get_type() == 'TIMESYNC'
            and safe and message.tc1 > 0 and message.ts1 > 0)
    if message.get_type() == 'TIMESYNC':
        return bool(compact_timing and safe and message.tc1 > 0 and message.ts1 > 0)
    return send_allowed(message, safe=safe, bench_authorized=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--heading-candidate', type=Path, required=True)
    ap.add_argument('--board-center-candidate', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--bench-confirmed', action='store_true')
    ap.add_argument('--compact-timing', action='store_true',
                    help='Fusion-off single-packet timing and 32Hz compact reception; USB time reference is bench-only')
    args = ap.parse_args()
    require_active_bench_clear(ap)
    if not args.bench_confirmed or args.output.exists():
        ap.error('Physical unpowered/props-off confirmation and new output required')
    try:
        require_current_calibration(args.heading_candidate, args.board_center_candidate)
    except (OSError, ValueError, KeyError) as exc:
        ap.error(str(exc))
    for name in (USB, TRS):
        if subprocess.run(['fuser', str(Path(name).resolve())], capture_output=True).returncode == 0:
            ap.error('Occupied port; no process will be killed: ' + name)
    heading = json.loads(args.heading_candidate.read_text())
    board = json.loads(args.board_center_candidate.read_text())
    if (heading.get('manual_forward_horizontal_confirmed') is not True
            or board.get('marker_name') != 'MircoAir4' or board.get('imu_offset_verified') is not False):
        ap.error('Known approximate board/heading references required; not an IMU calibration')
    mount = checked_quaternion(heading['body_orientation_in_rigid_xyzw_candidate'])
    offset = finite_vector(board['board_center_in_rigid_m_candidate'], 3)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('trs_vicon_receive_bench_source')
            self.gate = PoseGate(max_age_ms=150)
            self.sub = self.create_subscription(PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1 if args.compact_timing else 10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
            p, q = msg.pose.position, msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds, time.monotonic_ns(), msg.header.frame_id,
                [p.x,p.y,p.z], [q.x,q.y,q.z,q.w])

    rclpy.init(args=[])
    node = Source()
    ports = {}
    ev = {r: Evidence(source_component=197 if r == 'trs' else 190, expected_parameters=PARAMS)
          for r in ('usb', 'trs')}
    enc = {r: m.MAVLink(None, srcSystem=250, srcComponent=197 if r == 'trs' else 190) for r in ev}
    dec = {r: m.MAVLink(None) for r in ev}
    for parser in dec.values():
        parser.robust_parsing = True
    tx = {r: Counter() for r in ev}
    parameter_times = {}
    next_hb = {r: 0 for r in ev}
    next_info = {r: 0 for r in ev}
    next_params = 0
    buffer = ''
    rows, snapshots, listener_records = [], [], []
    start = time.monotonic()
    active_start = None
    next_emit = 0
    ping_sent = False
    latest_attitude = None
    alignment_yaw = None
    reference_rows, timing_probes = [], []
    receive_series = []
    clock_ready = ClockReadiness()
    stationary_sample = None
    arm_before = arm_after = visual_before = None
    error = None

    def identity(route):
        e = ev[route]
        now = time.monotonic() - start
        return bool(e.identity and e.identity['uid2'] == UID2 and e.identity['firmware'] == '1.14.3'
            and not e.fatal and e.heartbeats and not e.heartbeats[-1]['armed']
            and now - e.heartbeats[-1]['elapsed_s'] < 2)

    def safe():
        now = time.monotonic() - start
        return (all(identity(r) for r in ev) and fusion_disabled(ev['usb'].parameters)
            and all(now - parameter_times.get(n, -100) < 4 for n in PARAMS))

    def send(route, msg):
        if not allowed(msg, route=route, safe=bool(safe()), bench_confirmed=args.bench_confirmed,
                       compact_timing=args.compact_timing):
            raise ValueError('Receive-only send gate rejected: ' + route + '/' + msg.get_type())
        packet = msg.pack(enc[route])
        if ports[route].write(packet) != len(packet):
            raise OSError('Short serial write')
        enc[route].seq = (enc[route].seq + 1) % 256
        tx[route][msg.get_type()] += 1

    def send_sample(sample):
        if not safe() or not node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns()):
            raise RuntimeError('Fresh source or fusion-off safety gate lost; position transmission stopped')
        if len(rows) >= (170 if args.compact_timing else 100):
            raise RuntimeError('Position hard packet limit reached')
        ros_now = node.get_clock().now().nanoseconds
        mono = time.monotonic_ns()
        if not (0 <= ros_now-sample.stamp_ns <= node.gate.max_age_ns
                and 0 <= mono-sample.received_monotonic_ns <= node.gate.max_age_ns):
            raise RuntimeError('Held source expired; no pose sent')
        stamp = (mono - (ros_now - sample.stamp_ns)) // 1000
        p, q = transform_rigid_pose(sample.position, sample.quaternion, offset, mount)
        if args.compact_timing:
            if alignment_yaw is None or clock_ready.count < clock_ready.required:
                raise RuntimeError('Explicit NED candidate/reference clock not established')
            base_p,base_q=transform_rigid_pose(stationary_sample.position,stationary_sample.quaternion,offset,mount)
            angle=2*math.acos(float(np.clip(abs(np.asarray(q)@np.asarray(base_q)),0,1)))
            if np.linalg.norm(np.asarray(p)-base_p)>.01 or angle>math.radians(2):
                raise RuntimeError('Compact reception/timing requires stationary body; no fusion or motion trial')
            msg,p,q=compact.message(enc['trs'],stamp,p,q,alignment_yaw)
        else:
            msg=odometry_message(enc['trs'],stamp,p,q)
        pc_sent_us=time.monotonic_ns()//1000
        send('trs',msg)
        row={'elapsed_s':time.monotonic()-start,'source_stamp_ns':sample.stamp_ns,
             'wire_time_usec':stamp,'pc_sent_us':pc_sent_us,'position':p,'q':q,
             'phase':'continuous' if active_start is not None else 'single_packet_probe'}
        rows.append(row)
        return row

    def emit():
        nonlocal next_emit
        now=time.monotonic()
        if active_start is None or now-active_start>=5 or now<next_emit:
            return
        sample=node.gate.take_new(node.get_clock().now().nanoseconds,time.monotonic_ns())
        if sample is None:
            return
        if len(rows)>=(170 if args.compact_timing else 100):
            return
        send_sample(sample)
        if args.compact_timing:
            # 固定相位，只取最新数据；不补发积压帧、不重复缓存，目标32Hz。
            next_emit += 1/32
            if next_emit < now:
                next_emit=now+1/32
        else:
            next_emit=now+.05

    def pump():
        nonlocal next_params, buffer, ping_sent, latest_attitude
        now = time.monotonic() - start
        if now > (120 if args.compact_timing else 30):
            raise RuntimeError('Bounded receive-only session expired')
        rclpy.spin_once(node, timeout_sec=0)
        for route, port in ports.items():
            for msg in dec[route].parse_buffer(port.read(min(port.in_waiting, 65536))) or []:
                ev[route].ingest(msg, now)
                if ev[route].fatal:
                    raise RuntimeError(ev[route].fatal)
                if ev[route].identity and not identity(route):
                    raise RuntimeError('Wrong aircraft or stale disarmed identity on ' + route)
                if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                    continue
                if route == 'usb' and msg.get_type() == 'PARAM_VALUE' and str(msg.param_id) in PARAMS:
                    name = str(msg.param_id)
                    if name in ev['usb'].requests:
                        parameter_times[name] = now
                        if decode_parameter(msg) != 0:
                            raise RuntimeError('Fusion/offset must remain disabled: ' + name)
                if route == 'usb' and msg.get_type() == 'SERIAL_CONTROL':
                    buffer += bytes(msg.data[:msg.count]).decode('utf-8','replace')
                    if len(buffer) > 16384:
                        raise RuntimeError('USB observer overflow')
                if route == 'usb' and msg.get_type() == 'ATTITUDE_QUATERNION':
                    latest_attitude={'q':checked_quaternion([msg.q1,msg.q2,msg.q3,msg.q4]).tolist(),
                                     'elapsed_s':now,'time_boot_ms':msg.time_boot_ms}
                if (msg.get_type() == 'TIMESYNC' and msg.tc1 == 0 and msg.ts1 > 0 and safe()
                        and (route == 'trs' or args.compact_timing)):
                    send(route, enc[route].timesync_encode(time.monotonic_ns(), msg.ts1))
            if now >= next_hb[route]:
                send(route, enc[route].heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0))
                next_hb[route] = now + 1
            if ev[route].heartbeats and ev[route].identity is None and now >= next_info[route]:
                ev[route].version_requested_at = now
                send(route, enc[route].command_long_encode(1,1,512,0,148,0,0,0,0,0,0))
                next_info[route] = now + 2
        if identity('usb') and now >= next_params:
            for name in PARAMS:
                ev['usb'].requests[name] = now
                send('usb', enc['usb'].param_request_read_encode(1,1,name.encode(),-1))
            next_params = now + 2
        if identity('trs') and not ping_sent:
            usec, seq = time.monotonic_ns() // 1000, secrets.randbits(31)
            ev['trs'].pings[(usec,seq)] = now
            send('trs',enc['trs'].ping_encode(usec,seq,0,0))
            ping_sent = True
        emit()
        time.sleep(.002)

    def listener(topic, parse, optional=False):
        nonlocal buffer
        buffer = ''
        requested = time.monotonic()
        msg=(enc['usb'].serial_control_encode(10,2,0,0,len(compact.SERIES_COMMAND),
              compact.SERIES_COMMAND.ljust(70,b'\0')) if topic=='compact_visual_series'
              else shell_message(enc['usb'],topic))
        send('usb',msg)
        next_poll = requested + .25
        while time.monotonic() - requested < 4:
            pump()
            if not safe():
                raise RuntimeError('Receive-only baseline lost during USB observation')
            parsed = parse(buffer)
            if (parsed or (optional and 'never published' in buffer)) and reply_complete(buffer, bytes(msg.data[:msg.count])):
                listener_records.append({'topic':topic,'text':buffer,'parsed':parsed})
                return parsed
            if time.monotonic() >= next_poll:
                send('usb',shell_message(enc['usb']))
                next_poll = time.monotonic() + .25
        listener_records.append({'topic':topic,'text':buffer,'incomplete':True})
        raise RuntimeError('Incomplete USB observer reply: ' + topic)

    try:
        for route, path in (('usb',USB),('trs',TRS)):
            ports[route] = serial.Serial(path,115200,timeout=0,write_timeout=.5,exclusive=True)
        while not safe() or not ev['trs'].ping_rtt_ms or not node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):
            pump()
            if time.monotonic()-start > 12:
                raise RuntimeError('Identity/fusion-off/source/TRS PING preflight timeout')
        arm_before = listener('actuator_armed',parse_actuator_snapshot)
        if arm_before['armed']:
            raise RuntimeError('Internal actuator state armed')
        visual_before = listener('vehicle_visual_odometry',parse_visual_snapshot,True)
        if args.compact_timing:
            if not arm_before['lockdown']:
                raise RuntimeError('Output lockdown required for compact timing bench')
            if latest_attitude is None or time.monotonic()-start-latest_attitude['elapsed_s']>.25:
                raise RuntimeError('Fresh FC attitude required for explicit NED candidate')
            stationary_sample=node.gate.last
            _,q=transform_rigid_pose(stationary_sample.position,stationary_sample.quaternion,offset,mount)
            alignment_yaw=compact.yaw_alignment(q,latest_attitude['q'])
            deadline=time.monotonic()+85
            while clock_ready.count<clock_ready.required:
                ref=listener('timesync_status',parse_timesync_snapshot)
                ready=clock_ready.update(ref,time.monotonic_ns()//1000)
                reference_rows.append(ref)
                print(f'USB REFERENCE: ready={ready}, residual={ref["observed_offset"]-ref["estimated_offset"]}us',flush=True)
                if time.monotonic()>deadline:
                    raise RuntimeError('Original USB timing gate not ready; no compact pose sent')
                until=time.monotonic()+2
                while not ready and time.monotonic()<until:
                    pump()
            # 两个独立单包：无连续发送干扰，0/50ms暂存真实样本，测量发布到达时间。
            for hold_ms in (0,50):
                ref_before=listener('timesync_status',parse_timesync_snapshot)
                if not compact.reference_usable(ref_before,time.monotonic_ns()//1000):
                    raise RuntimeError('USB reference lost before timing probe')
                floor=listener('vehicle_visual_odometry',parse_visual_snapshot,True)
                pump()
                sample=node.gate.take_new(node.get_clock().now().nanoseconds,time.monotonic_ns())
                if sample is None:
                    raise RuntimeError('No fresh source for single compact probe')
                until=time.monotonic()+hold_ms/1000
                while time.monotonic()<until:
                    pump()
                row=send_sample(sample)
                until=time.monotonic()+.18
                while time.monotonic()<until:
                    pump()
                snap=listener('vehicle_visual_odometry',parse_visual_snapshot)
                if (not compact.match(snap,[row]) or (floor and snap['timestamp']<=floor['timestamp'])):
                    raise RuntimeError('No fresh single compact packet internally observed')
                ref_after=listener('timesync_status',parse_timesync_snapshot)
                if not compact.reference_usable(ref_after,time.monotonic_ns()//1000):
                    raise RuntimeError('USB reference lost after single-packet probe')
                measurement=compact.arrival_measurement(row,snap,ref_before,ref_after)
                timing_probes.append({'requested_hold_ms':hold_ms,'sent':row,'snapshot':snap,
                                      'clock_before':ref_before,'clock_after':ref_after,'measurement':measurement})
                print(f'COMPACT TIMING: hold={hold_ms}ms, send-to-FC={measurement["send_to_fc_publication_ms"]:.2f}ms',flush=True)
        active_start = next_emit = time.monotonic()
        print('TRS RECEIVE-ONLY: '+('32Hz compact NED candidate' if args.compact_timing else '20Hz real LOCAL_FRD')+
              '/5s; EV_CTRL remains0; USB observer only',flush=True)
        for elapsed in (1,3,4.5):
            while time.monotonic()-active_start < elapsed:
                pump()
            if args.compact_timing:
                group=listener('compact_visual_series',compact.visual_series)
                receive_series.append({'snapshots':group,'timing':compact.series_timing(group)})
                snap=group[-1]
            else:
                snap = listener('vehicle_visual_odometry',parse_visual_snapshot)
            matched = (compact.match if args.compact_timing else match_snapshot)(snap,rows)
            snapshots.append({'snapshot':snap,'matching_sent_pose':matched})
            if matched is None:
                raise RuntimeError('Internal visual odometry does not match transmitted poses')
        while time.monotonic()-active_start < 5:
            pump()
        active_start = None
        arm_after = listener('actuator_armed',parse_actuator_snapshot)
        if arm_after['armed']:
            raise RuntimeError('Final internal state armed')
        if args.compact_timing and not arm_after['lockdown']:
            raise RuntimeError('Output lockdown lost')
        if not safe():
            raise RuntimeError('Final restored fusion-off baseline not fresh')
        stamps = [s['snapshot']['timestamp'] for s in snapshots]
        floor = visual_before['timestamp'] if visual_before else arm_before['timestamp_us']
        if not (len(stamps) >= 2 and stamps[0] > floor and all(b>a for a,b in zip(stamps,stamps[1:]))):
            raise RuntimeError('Visual publication timestamps not fresh/increasing')
    except (OSError, ValueError, RuntimeError, serial.SerialException, KeyboardInterrupt) as exc:
        error = str(exc)
    finally:
        for port in ports.values():
            port.close()
        node.destroy_node()
        rclpy.shutdown()
    continuous=[r for r in rows if r['phase']=='continuous']
    gaps = [(b['elapsed_s']-a['elapsed_s'])*1000 for a,b in zip(continuous,continuous[1:])]
    result = {'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope':'Unpowered, fusion-off wireless packet reception bench; not EKF or flight validation',
              'identities':{r:ev[r].identity for r in ev},'tx_messages':{r:dict(tx[r]) for r in tx},
              'parameters_after':ev['usb'].parameters,'actual_pose_packets_sent':len(rows),
              'transmit_rate_hz':(len(continuous)-1)/(continuous[-1]['elapsed_s']-continuous[0]['elapsed_s']) if len(continuous)>1 else None,
              'maximum_transmit_gap_ms':max(gaps) if gaps else None,
              'continuous_pose_packets_sent':len(continuous),'internal_receive_series':receive_series,
              'sent_poses':rows,'visual_before':visual_before,'visual_snapshots':snapshots,
              'listener_records':listener_records,'actuator_before':arm_before,'actuator_after':arm_after,
              'trs_ping_rtt_ms':ev['trs'].ping_rtt_ms,'source_accepted':node.gate.accepted,
              'source_rejected':node.gate.rejected,'fusion_enable_sent':False,'parameter_writes':0,
              'fc_visual_reception_verified':error is None and len(snapshots)>=2,
              'clock_mapping_verified':False,'ekf_fusion_verified':False,'flight_ready':False,
              'compact_timing_requested':args.compact_timing,'alignment_yaw_candidate_rad':alignment_yaw,
              'frame_alignment_scope':'FC yaw-aligned local NED candidate; true North not calibrated; not a persisted control mapping'
                  if args.compact_timing else 'Existing arbitrary LOCAL_FRD',
              'usb_reference_clock_rows':reference_rows,'single_packet_timing_probes':timing_probes,
              'imu_reference_verified':False,'reference':'Approximate board center only',
              'serial_ports_released':True,'error':error,'duration_s':time.monotonic()-start}
    with args.output.open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('sent_poses','listener_records')},ensure_ascii=False,indent=2))
    return 0 if result['fc_visual_reception_verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
