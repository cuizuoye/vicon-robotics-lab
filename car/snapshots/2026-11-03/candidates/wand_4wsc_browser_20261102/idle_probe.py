"""Explicitly opt-in, disabled-zero-only probe. No enable/select/clear API."""
import fcntl
import json
from pathlib import Path
import socket
import time
import bootstrap
from mecanum_wand_4wsc_hardware.authorization import endpoint,verify_flash_record
from mecanum_wand_4wsc_hardware.stage0_session import Stage0Session
from mecanum_wand_4wsc_hardware.stage0_contract import decode_motion_status,disabled_motion_status_ready
from mecanum_wand_4wsc_hardware.startup_clock import StartupProgressSocket
from mecanum_vicon_control.transport_protocol import MessageType,FrameFlags,decode_frame,unpack_cmd_vel,unpack_wheel_telemetry

CONSENT='4WSC_DISABLED_ZERO_ONLY_NO_ENABLE_NO_CLEAR'

def zero_only(raw):
    f=decode_frame(raw)
    if f.message_type==MessageType.HELLO:
        if f.flags or f.payload:raise ValueError('HELLO schema')
    elif f.message_type==MessageType.CMD_VEL:
        v=unpack_cmd_vel(f.payload)
        if f.flags or any((v.vx_mps,v.vy_mps,v.wz_radps)) or v.timeout_ms!=600:raise ValueError('disabled zero only')
    else:raise ValueError('probe forbids enable, clear, select, ESTOP and all other TX types')
    return f

def run(profile,record,folder,consent,seconds=8.,rehearsal=False):
    if consent!=CONSENT or not 1<=seconds<=60:raise ValueError('explicit zero-only consent and 1..60s limit required')
    verify_flash_record(record)
    ip,iface,command_port,telemetry_port=endpoint(profile,rehearsal=rehearsal)
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    owner=open('/tmp/wand_mc_link_'+str(telemetry_port)+'.lock','a')
    fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
    session=Stage0Session(stage=0);sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    wrapped=None;log=(folder/'wire.jsonl').open('x');result=None
    start=time.monotonic();last_hello=last_tx=-float('inf');status=wheel=None;progress=None;progress_at=None;stable=None
    def send(raw):
        f=zero_only(raw);wrapped.sendto(raw,(ip,command_port))
        log.write(json.dumps(dict(direction='TX',at=time.monotonic(),kind=int(f.message_type),enabled=False,nonzero=False))+'\n');log.flush()
    try:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,iface.encode()+b'\0')
        sock.bind(('0.0.0.0',telemetry_port));sock.setblocking(False)
        wrapped=StartupProgressSocket(sock,ip,lambda:session.session_id,stage=0)
        while time.monotonic()-start<seconds:
            now=time.monotonic()
            if not session.connected and now-last_hello>=.65:
                send(session.new_hello(int(now*1000)));last_hello=now
            if session.connected and now-last_tx>=.02:
                send(session.cmd_vel(0.,0.,0.,600,False,int(now*1000)));last_tx=now
            for _ in range(32):
                try:raw,sender=wrapped.recvfrom(512)
                except BlockingIOError:break
                f=session.accept(raw)
                if f.message_type==MessageType.LINK_STATUS:status=decode_motion_status(f.payload)
                elif f.message_type==MessageType.WHEEL_TELEMETRY:
                    wheel=unpack_wheel_telemetry(f.payload)
                    if progress!=wheel.last_command_sequence:progress=wheel.last_command_sequence;progress_at=now
                    if wheel.fault_bits or f.flags&int(FrameFlags.ENABLED):raise ValueError('native fault or enabled state observed; preserve latch')
                log.write(json.dumps(dict(direction='RX',at=now,kind=int(f.message_type),sequence=f.sequence,sender_ms=f.sender_time_ms))+'\n')
            # recvfrom samples its own later clock. Judge freshness AFTER the
            # bounded ingress drain, never against an earlier loop timestamp.
            now=time.monotonic()
            if wrapped.clock.fault:raise ValueError(wrapped.clock.fault)
            ready=bool(wrapped.clock.ready(now) and status and disabled_motion_status_ready(status)
                and wheel and not wheel.fault_bits and not any(wheel.wheel_rpm)
                and progress_at is not None and 0<=now-progress_at<=.20)
            if ready:
                if stable is None:stable=now
            else:stable=None
            if (session.connected and wrapped.clock.calibrated and any(
                    now-stamp>.30 for stamp in wrapped.clock.accepted_at.values())):
                raise ValueError('feedback exceeds existing 300ms bound')
            time.sleep(.003)
        now=time.monotonic()
        result=dict(passed=bool(stable is not None and now-stable>=.60),mode='DISABLED_ZERO_ONLY',
            elapsed_s=now-start,identity='4WSC',clock=wrapped.clock.snapshot(now),
            fault_bits=None if wheel is None else wheel.fault_bits,status=status,
            command_progress_age_s=None if progress_at is None else now-progress_at,
            hardware_socket_opened=not rehearsal,software_loopback_only=rehearsal,
            enable_sent=False,select_sent=False,clear_fault_sent=False,motion_started=False,
            physical_motion_authority=False)
        return result
    except Exception as e:
        result=dict(passed=False,reason=type(e).__name__+': '+str(e),enable_sent=False,clear_fault_sent=False,motion_started=False)
        raise
    finally:
        if session.connected and wrapped is not None and wrapped.clock.qualified and not wrapped.clock.fault:
            try:send(session.cmd_vel(0.,0.,0.,600,False,int(time.monotonic()*1000)))
            except (OSError,ValueError):pass
        if wrapped is not None:wrapped.close()
        else:sock.close()
        owner.close();log.close()
        (folder/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
