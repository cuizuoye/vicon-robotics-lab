#!/usr/bin/env python3
"""固定F50无线悬空验收：默认断动力只读；车载供电检查不使能电机。"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import secrets
import time

from four_link_ro_probe import Transport, MessageType, FrameFlags, encode_frame, pack_cmd_vel, unpack_wheel_telemetry
from mecanum_vicon_control.transport_protocol import pack_control_enable
from mecanum_vicon_control.bench_link_status import decode_bench_status


class TransitionGuard:
    """只容许切换后200ms内、尚未确认新状态时的旧反馈，不放宽固件期限。"""
    def __init__(self, moving, grace_s=0.0):
        if not 0.0 <= grace_s <= 0.2:
            raise ValueError('feedback transition grace must be 0..200ms')
        self.moving = moving
        self.grace_s = grace_s
        self.confirmed = set()

    def accept(self, kind, active, elapsed_s):
        if active == self.moving:
            self.confirmed.add(kind)
            return True
        if kind not in self.confirmed and elapsed_s < self.grace_s:
            return False
        raise ValueError('unexpected drive state after feedback transition')


class Probe:
    def __init__(self,args,transport=None):
        self.args=args; self.transport=transport or Transport(args)
        self.sid=secrets.randbits(32) or 1; self.sequence=0; self.start=time.monotonic()
        self.motion_attempted=False
        self.feedback_sequence={}
        self.record=dict(mode=args.mode,phases=[],commands=[],motion_attempted=False,
            battery_disconnected_confirmed=bool(getattr(args,'confirm_battery_disconnected',False)),
            powered_suspended_stationary_confirmed=bool(
                getattr(args,'confirm_powered_suspended_stationary',False)),
            battery_powered_motors_disconnected_confirmed=bool(
                getattr(args,'confirm_battery_powered_motors_disconnected',False)))

    def send(self,kind,payload=b'',flags=0):
        if flags&FrameFlags.ENABLED and self.args.mode!='forward50':
            raise ValueError('motion is forbidden outside the single forward50 mode')
        if kind==MessageType.CONTROL_ENABLE and payload!=pack_control_enable(False):
            if self.args.mode!='forward50' or payload!=pack_control_enable(True):
                raise ValueError('clear faults or enable outside forward50 is forbidden')
        if kind==MessageType.CMD_VEL:
            expected=pack_cmd_vel(0.05 if flags&FrameFlags.ENABLED else 0,0,0,150)
            if payload!=expected: raise ValueError('not exact fixed F50 or disabled zero')
        self.sequence=(self.sequence+1)&0xffffffff
        sent_s=time.monotonic()-self.start
        self.transport.send(encode_frame(kind,self.sequence,self.sid,
            int(sent_s*1000)&0xffffffff,payload,flags))
        self.record['commands'].append(dict(kind=int(kind),sequence=self.sequence,flags=int(flags),
                                            host_time_s=sent_s))
        return self.sequence

    def zero(self): self.send(MessageType.CMD_VEL,pack_cmd_vel(0,0,0,150))
    def forward(self): self.send(MessageType.CMD_VEL,pack_cmd_vel(0.05,0,0,150),FrameFlags.ENABLED)

    def collect(self,name,seconds,moving=False,transition_s=0.0):
        phase=dict(name=name,telemetry=[],diagnostics=[],ack=[],transition_frames=[])
        self.record['phases'].append(phase)
        started=time.monotonic(); end=started+seconds; next_send=0; last_telemetry=started
        guard=TransitionGuard(moving,transition_s)
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_send:
                (self.forward if moving else self.zero)(); next_send=now+0.02
            for frame in self.transport.receive():
                if frame.session_id!=self.sid: continue
                if frame.message_type==MessageType.HELLO_ACK:
                    if frame.flags!=FrameFlags.RESPONSE or frame.payload: raise ValueError('bad ACK')
                    phase['ack'].append(frame.sequence)
                elif frame.message_type==MessageType.WHEEL_TELEMETRY:
                    if not self.new_feedback(frame): continue
                    t=unpack_wheel_telemetry(frame.payload)
                    if t.fault_bits or frame.flags&FrameFlags.FAULT:
                        self.record['rejected_feedback']=dict(kind='telemetry',data=asdict(t),
                            sender_time_ms=frame.sender_time_ms,host_time_s=now-self.start)
                        raise ValueError(f'MCU fault {t.fault_bits:#x}; no motion retry')
                    if t.state_flags&0xc5!=0xc5 or t.state_flags&32:
                        raise ValueError('not calibrated four-wheel bench')
                    enabled=bool(t.state_flags&2)
                    if enabled!=bool(frame.flags&FrameFlags.ENABLED):
                        raise ValueError('enabled header/payload mismatch')
                    if t.state_flags&16: raise ValueError('command lease expired')
                    row=asdict(t); row['host_time_s']=now-self.start
                    if guard.accept('telemetry',enabled,now-started):
                        phase['telemetry'].append(row); last_telemetry=now
                    else:
                        phase['transition_frames'].append(dict(kind='telemetry',data=row))
                elif frame.message_type==MessageType.LINK_STATUS:
                    if not self.new_feedback(frame): continue
                    if frame.flags&FrameFlags.FAULT: raise ValueError('diagnostics report MCU fault')
                    d=decode_bench_status(frame.payload)
                    if d['active']!=bool(frame.flags&FrameFlags.ENABLED):
                        raise ValueError('actual drive header/payload mismatch')
                    d['sample_time_ms']=frame.sender_time_ms; d['host_time_s']=now-self.start
                    if guard.accept('diagnostics',d['active'],now-started):
                        phase['diagnostics'].append(d)
                    else:
                        phase['transition_frames'].append(dict(kind='diagnostics',data=d))
            if moving and now-last_telemetry>0.35: raise ValueError('telemetry missing; stopping motion')
            time.sleep(0.001)
        return phase

    def new_feedback(self,frame):
        # UDP重复/乱序包不计入验收，也不能把上次状态当成本轮已停轮证明。
        previous=self.feedback_sequence.get(frame.message_type)
        if previous is not None and not 0 < (frame.sequence-previous)&0xffffffff < 0x80000000:
            return False
        self.feedback_sequence[frame.message_type]=frame.sequence
        return True

    @staticmethod
    def validate(phase,used=None):
        t=phase['telemetry']; d=phase['diagnostics']
        if len(t)<5 or len(d)<5: raise ValueError('insufficient actual four-wheel feedback')
        if used is not None and any(bool(r['state_flags']&512)!=used for r in t):
            raise ValueError('single-run quota does not match requested stage')
        if len({r['last_command_sequence'] for r in t})<3:
            raise ValueError('command sequence not progressing')
        dt=((d[-1]['sample_time_ms']-d[0]['sample_time_ms'])&0xffffffff)/1000
        if dt<=0: raise ValueError('MCU clock did not progress')
        phase['sample_hz']=(d[-1]['ticks']-d[0]['ticks'])/dt
        if not 95<=phase['sample_hz']<=105 or d[-1]['ticks']-d[0]['ticks']!=d[-1]['updates']-d[0]['updates']:
            raise ValueError('sampling/update cadence failed')

    def run(self):
        try:
            hello=self.send(MessageType.HELLO,flags=FrameFlags.ACK_REQUIRED)
            baseline=self.collect('baseline_zero',getattr(self.args,'observe_seconds',5.0))
            self.validate(baseline,used=self.args.mode=='post-run')
            if hello not in baseline['ack']: raise ValueError('STM32 ACK not received')
            if any(d['permitted'] for d in baseline['diagnostics']): raise ValueError('already permitted')
            if self.args.mode=='forward50':
                # 本工具只发送一次使能门控请求；未收到门控确认就退出，不重发/重试。
                self.send(MessageType.CONTROL_ENABLE,pack_control_enable(True))
                gate=self.collect('enable_gate_zero',0.13)
                if not gate['diagnostics'] or not gate['diagnostics'][-1]['permitted']:
                    raise ValueError('enable gate not confirmed; motion not sent')
                self.motion_attempted=True; self.record['motion_attempted']=True
                moving=self.collect('forward50',9.6,True,transition_s=0.2)
                self.validate(moving,used=True)
                for i in range(4):
                    raw_delta=(moving['telemetry'][-1]['encoder_counts'][i]-moving['telemetry'][0]['encoder_counts'][i])&0xffffffff
                    signed_delta=raw_delta if raw_delta<0x80000000 else raw_delta-0x100000000
                    delta=signed_delta*(1,-1,1,-1)[i]
                    if delta<500: raise ValueError('wheel did not make forward progress')
                self.zero()
                stopped=self.collect('stopped_zero',1.0,transition_s=0.2)
                self.validate(stopped,used=True)
                if any(d['permitted'] for d in stopped['diagnostics']): raise ValueError('stop did not revoke gate')
            self.record['success']=True
            print(json.dumps(dict(success=True,mode=self.args.mode,motion_attempted=self.motion_attempted,
                phases=[dict(name=p['name'],frames=len(p['telemetry']),diagnostics=len(p['diagnostics']),
                             sample_hz=p.get('sample_hz'),last=p['diagnostics'][-1] if p['diagnostics'] else None)
                        for p in self.record['phases']]),ensure_ascii=False))
            return self.record
        except BaseException as error:
            self.record['success']=False; self.record['error']=str(error)
            self.record['failure_host_time_s']=time.monotonic()-self.start
            raise
        finally:
            # 退出/异常路径始终发未使能零速；固件另有150ms超时和10秒上限。
            try:
                for _ in range(3): self.zero()
            except Exception as error:
                self.record['stop_send_error']=str(error)
            self.transport.close()
            if self.args.output:
                Path(self.args.output).write_text(json.dumps(self.record,ensure_ascii=False,indent=2)+'\n')


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--esp-ip',required=True); p.add_argument('--bind-ip',required=True)
    p.add_argument('--interface',required=True); p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['no-power','logic-power','powered-stationary','forward50','post-run'],
                   default='no-power')
    p.add_argument('--observe-seconds',type=float,default=5.0,
                   help='5..30 seconds of disabled-zero observation; motion mode stays fixed at 5')
    confirms=p.add_mutually_exclusive_group(required=True)
    confirms.add_argument('--confirm-battery-disconnected',action='store_true')
    confirms.add_argument('--confirm-battery-powered-motors-disconnected',action='store_true',
                          help='logic-power only: all four motor six-pin plugs physically removed')
    confirms.add_argument('--confirm-powered-suspended-stationary',action='store_true')
    a=p.parse_args(argv); a.serial=None
    if not 5 <= a.observe_seconds <= 30 or (a.mode=='forward50' and a.observe_seconds!=5):
        p.error('zero observation must be 5..30 seconds; forward50 remains exactly 5')
    if a.mode in ('no-power','post-run') and not a.confirm_battery_disconnected:
        p.error('this mode requires battery disconnected')
    if a.mode=='logic-power' and not a.confirm_battery_powered_motors_disconnected:
        p.error('logic-power requires battery-powered controllers and all four motor plugs removed')
    if a.mode in ('powered-stationary','forward50') and not a.confirm_powered_suspended_stationary:
        p.error('this mode requires new onsite power/suspension confirmation')
    return a


def main():
    Probe(parse_args()).run()


if __name__=='__main__': main()
