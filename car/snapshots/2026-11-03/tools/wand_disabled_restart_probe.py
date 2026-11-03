#!/usr/bin/env python3
"""Bounded diagnostic: HELLO + DISABLED ZERO ONLY, capture before host filter.

No ROS publishers, control-enable, clear-faults, ESTOP, firmware or network
configuration writes. Uses the reviewed endpoint and exclusive MC-Link lock.
Anomalous identity/output/fault is fail-stop; never retries a motion action.
"""
import argparse
import fcntl
import hashlib
import json
import math
from pathlib import Path
import socket
import time

from mecanum_vicon_control.mc_link_session import McLinkSession
from mecanum_vicon_control.transport_protocol import MessageType, decode_frame, unpack_wheel_telemetry
from mecanum_wand_hardware.commissioning import Runtime, CONSENT, McuSampleClock
from mecanum_wand_hardware.continuous_status import decode_motion_status


def trial(runtime, duration, log):
    session, clock = McLinkSession(), McuSampleClock()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, runtime.interface.encode()+b'\0')
    sock.bind(('0.0.0.0', runtime.telemetry_port))
    sock.settimeout(.005)
    destination = runtime.ip, runtime.command_port
    started, last_hello, last_send = time.monotonic(), -math.inf, -math.inf
    counts, rejections = {}, {}
    last_received, last_accepted, gaps = {}, {}, {}
    result = dict(all_sent_commands_disabled_zero=True, received={}, rejected={}, max_receive_gap_s={}, max_accepted_gap_s={})
    try:
        while time.monotonic()-started < duration:
            now = time.monotonic()
            if not session.connected and now-last_hello >= .30:
                sock.sendto(session.new_hello(int(now*1000)), destination)
                last_hello = now
            elif session.connected and now-last_send >= .02:
                sock.sendto(session.cmd_vel(0., 0., 0., 600, False, int(now*1000)), destination)
                last_send = now
            try: data, sender = sock.recvfrom(512)
            except socket.timeout: continue
            received = time.monotonic()
            if sender[0] != runtime.ip: continue
            frame = decode_frame(data)
            if frame.session_id != session.session_id: continue
            session.accept(data)
            if frame.message_type not in (MessageType.WHEEL_TELEMETRY, MessageType.LINK_STATUS): continue
            kind, tick = MessageType(frame.message_type).name, frame.sender_time_ms
            if frame.message_type == MessageType.LINK_STATUS:
                d = decode_motion_status(frame.payload)
                if d['tag'] != runtime.motion_tag or d['permitted'] or d['fault_wheel'] or any(
                        d[k] for k in ('compares', 'raw_duty', 'target_cps', 'body_milli') if isinstance(d[k], (int, float))):
                    raise RuntimeError('unexpected diagnostic identity, fault or permit')
                if any(any(d[k]) for k in ('compares', 'raw_duty', 'target_cps', 'body_milli')):
                    raise RuntimeError('nonzero real MCU output during zero-only probe')
            else:
                telemetry = unpack_wheel_telemetry(frame.payload)
                if telemetry.sample_time_ms != tick or telemetry.fault_bits:
                    raise RuntimeError('invalid telemetry or MCU fault; no retry')
            counts[kind] = counts.get(kind, 0)+1
            if kind in last_received:
                gaps[('received', kind)] = max(gaps.get(('received', kind), 0.), received-last_received[kind])
            last_received[kind] = received
            previous, latest = clock.by_kind.get(frame.message_type), clock.latest_tick
            delta = (tick-latest) & 0xffffffff if latest is not None else 0
            mapped_age = received-(clock.local_baseline+(clock.elapsed_ms+delta)/1000.) if latest is not None else 0.
            accepted = clock.observe(frame.message_type, tick, received)
            reason = ''
            if accepted is None:
                reason = ('KIND_REPLAY_OR_REORDER' if previous is not None and not 0 < ((tick-previous)&0xffffffff) < 0x80000000 else
                          'CROSS_KIND_REORDER' if delta >= 0x80000000 else
                          'MAPPED_FUTURE' if mapped_age < -.02 else 'MAPPED_STALE')
                rejections[reason] = rejections.get(reason, 0)+1
            else:
                if kind in last_accepted:
                    gaps[('accepted', kind)] = max(gaps.get(('accepted', kind), 0.), received-last_accepted[kind])
                last_accepted[kind] = received
            log.write(json.dumps(dict(received_steady=received,kind=kind,tick=tick,sequence=frame.sequence,
                session=frame.session_id,payload_hex=frame.payload.hex(),accepted_sample_steady=accepted,
                mapped_age_before_filter_s=mapped_age,rejection=reason))+'\n')
        result.update(received=counts,rejected=rejections,connected=session.connected,
            max_receive_gap_s={k:gaps.get(('received',k)) for k in counts},
            max_accepted_gap_s={k:gaps.get(('accepted',k)) for k in counts})
        return result
    finally:
        if session.connected:
            for _ in range(3):
                sock.sendto(session.cmd_vel(0.,0.,0.,600,False,int(time.monotonic()*1000)),destination)
        sock.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--approved-sha256',required=True)
    p.add_argument('--hardware-consent',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=8.)
    a = p.parse_args()
    if not 3 <= a.seconds <= 15: raise ValueError('bounded capture duration is 3..15s')
    r = Runtime('hardware',str(a.profile),a.approved_sha256,a.hardware_consent)
    if r.motion_tag != '4WGC': raise ValueError('probe is restricted to reviewed 4WGC')
    a.output.mkdir(exist_ok=False)
    with open('/tmp/wand_mc_link_'+str(r.telemetry_port)+'.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with (a.output/'raw_frames.jsonl').open('x') as log:
            result = trial(r,a.seconds,log)
    report = dict(verification='REAL RAW UDP / HELLO + DISABLED ZEROS ONLY / BEFORE EXISTING FILTER',
        profile_sha256=hashlib.sha256(a.profile.read_bytes()).hexdigest(),result=result)
    with (a.output/'summary.json').open('x') as stream: json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
