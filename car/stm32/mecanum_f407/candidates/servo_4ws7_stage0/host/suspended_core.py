"""Pure five-second suspended controller. No sockets, ROS, GUI or devices.

The caller supplies fresh, validated MCU samples and physical-key observations.
No position outer loop, fake Vicon, automatic rearm or retry of enable/select.
"""
import math
from mecanum_vicon_control.mc_link_session import SessionMismatchError
from .stage0_session import Stage0Session
from .stage0_contract import decode_motion_status,disabled_motion_status_ready
from .commissioning import McuSampleClock
from mecanum_vicon_control.transport_protocol import MessageType,FrameFlags,ProtocolError,unpack_wheel_telemetry

DIRECTIONS={'前进':(.20,0.),'后退':(-.20,0.),'左移':(0.,.20),'右移':(0.,-.20),
    '左前':(.141,.141),'右前':(.141,-.141),'左后':(-.141,.141),'右后':(-.141,-.141)}
PREPARING=('HELLO','BASELINE','WAIT_CLEAR','WAIT_SELECT')
MOVING=('WAIT_ENABLE','ACTIVE')

class SuspendedController:
    def __init__(self,session_factory=None):
        self.session=Stage0Session(session_factory,action_mode=2)
        self.state='IDLE';self.reason='先轻按并松开空格解锁，再准备；准备不转轮'
        self.unlocked=False;self.unlock_press=False;self.release_required=False;self.focused=False;self.down=False
        self.ui_at=-math.inf;self.last_step=None;self.pending=[];self.status=None;self.wheel=None
        self.status_at=self.wheel_at=self.progress_at=-math.inf
        self.last_progress=None;self.clock=McuSampleClock();self.rx_sequences={}
        self.baseline_since=None;self.state_at=0.;self.action_at=None;self.direction='前进'
        self.stop_target='COMPLETE';self.events=[];self.last_tx=-math.inf
        self.first_fault_reason=''

    def event(self,now,reason):
        self.reason=reason;self.events.append(dict(at=now,state=self.state,reason=reason))

    @staticmethod
    def stamp(now):return int(now*1000)&0xffffffff

    def _zero(self,now):
        if self.session.connected:
            self.pending.append(self.session.cmd_vel(0,0,0,600,False,self.stamp(now)))

    def stop(self,now,reason='人工停止',fault=False,estop=False):
        self.unlocked=False;self.unlock_press=False
        self.release_required=True
        fault=fault or self.state=='FAULT'
        if fault and not self.first_fault_reason:self.first_fault_reason=reason
        if self.state=='STOPPING':
            if fault:self.stop_target='FAULT'
            return
        self.pending=[] # Never drain an old HELLO/enable after a stop edge.
        if self.session.connected:
            self.pending.append(self.session.estop(self.stamp(now)) if estop else
                self.session.control_enable(False,False,self.stamp(now)))
            self._zero(now)
        self.state='STOPPING';self.state_at=now;self.stop_target='FAULT' if fault else 'COMPLETE'
        self.event(now,reason)

    def operator(self,now,focused,down):
        old_down=self.down
        self.ui_at=now;self.focused=bool(focused);self.down=bool(down)
        if not focused:
            if self.state in (*PREPARING,*MOVING,'READY'):self.stop(now,'窗口失焦：撤能，不自动恢复')
            self.unlocked=False;self.unlock_press=False
            self.release_required=True
            return
        if self.release_required:
            if not down:self.release_required=False
            return
        if not self.unlocked:
            if down:self.unlock_press=True
            elif self.unlock_press:self.unlocked=True;self.unlock_press=False
            return
        if self.state in PREPARING and down:
            self.stop(now,'准备过程中按键：请松开后重新准备');return
        if self.state=='READY' and down and not old_down:
            self.state='WAIT_ENABLE';self.state_at=now;self.action_at=now
            self.pending.append(self.session.control_enable(True,False,self.stamp(now)))
            self.event(now,'等待本会话MCU许可；尚不发送非零速度')
        elif self.state in MOVING and not down:
            self.stop(now,'松键：立即撤能')

    def fresh(self,now):
        return bool(self.status and self.wheel and all(0<=now-t<=.20+1e-9
            for t in (self.status_at,self.wheel_at,self.progress_at)))

    def stationary(self,now):
        return self.fresh(now) and disabled_motion_status_ready(self.status) and all(abs(v)<2 for v in self.wheel.wheel_rpm)

    def prepare(self,now,direction):
        if (self.state not in ('IDLE','COMPLETE','FAULT') or not self.focused or self.down or
            not self.unlocked or now-self.ui_at>.15 or direction not in DIRECTIONS):return False
        # A true MCU fault is never cleared from this UI.
        if self.wheel and self.wheel.fault_bits:return False
        self.direction=direction;self.status=self.wheel=None
        self.status_at=self.wheel_at=self.progress_at=-math.inf
        self.clock=McuSampleClock();self.rx_sequences={};self.last_progress=None
        self.pending=[self.session.new_hello(self.stamp(now))]
        self.state='HELLO';self.state_at=now;self.baseline_since=None;self.action_at=None
        self.stop_target='COMPLETE';self.first_fault_reason=''
        self.event(now,'建立新会话；只发送失能零速');return True

    def receive(self,data,now):
        try:
            f=self.session.accept(data)
            if f.message_type==MessageType.HELLO_ACK:
                if (f.flags!=FrameFlags.RESPONSE or f.payload or
                    f.sequence!=self.session.pending_hello_sequence):
                    raise ProtocolError('invalid HELLO_ACK flags/payload/sequence')
                if self.state=='HELLO' and self.session.connected:
                    self.state='BASELINE';self.state_at=now
                return
            # Production publish() streams status, not command responses.
            # Only HELLO_ACK carries RESPONSE; telemetry carries FAULT/ENABLED.
            if (f.message_type not in (MessageType.WHEEL_TELEMETRY,MessageType.LINK_STATUS) or
                int(f.flags)&~int(FrameFlags.FAULT|FrameFlags.ENABLED)):
                raise ProtocolError('invalid MCU stream type/flags')
            if not self.session.connected:return # No freshness before a valid HELLO_ACK.
            old=self.rx_sequences.get(f.message_type)
            if old is not None and not 0<((f.sequence-old)&0xffffffff)<0x80000000:return
            accepted=self.clock.observe(f.message_type,f.sender_time_ms,now)
            if accepted is None:return
            self.rx_sequences[f.message_type]=f.sequence
            if f.message_type==MessageType.LINK_STATUS:
                self.status=decode_motion_status(f.payload);self.status_at=accepted
                if self.status['action_mode'] not in (0,2):raise ProtocolError('not suspended mode')
            else:
                w=unpack_wheel_telemetry(f.payload)
                if (bool(f.flags&FrameFlags.FAULT)!=bool(w.fault_bits) or
                    bool(f.flags&FrameFlags.ENABLED)!=bool(w.state_flags&2)):
                    raise ProtocolError('wheel header flags disagree with payload')
                if w.sample_time_ms!=f.sender_time_ms or any(abs(v)>120 for v in w.wheel_rpm):
                    raise ProtocolError('wheel timestamp/RPM mismatch')
                self.wheel=w;self.wheel_at=accepted
                if w.last_command_sequence and 0<((w.last_command_sequence-self.session.sequence)&0xffffffff)<0x80000000:
                    raise ProtocolError('future command acknowledgement')
                if self.last_progress is not None and w.last_command_sequence!=self.last_progress and not 0<((w.last_command_sequence-self.last_progress)&0xffffffff)<0x80000000:
                    raise ProtocolError('command acknowledgement regressed')
                if w.last_command_sequence!=self.last_progress:
                    self.last_progress=w.last_command_sequence;self.progress_at=accepted
                if w.fault_bits and self.state!='FAULT':self.stop(now,'MCU真实故障，保留锁存',fault=True)
        except SessionMismatchError:
            return
        except ProtocolError as error:
            # Foreign/old sessions cannot refresh any evidence. CRC/schema
            # errors in an owned active session fail closed.
            if self.state in (*PREPARING,*MOVING,'READY'):
                self.stop(now,'协议校验失败：'+str(error),fault=True)

    def step(self,now):
        if not math.isfinite(now):raise ValueError('finite monotonic clock required')
        if self.last_step is not None and (now<self.last_step or now-self.last_step>.10+1e-9):
            if self.state in (*PREPARING,*MOVING,'READY'):self.stop(now,'控制线程超时/时钟倒退',fault=True)
        self.last_step=now
        if self.state in (*PREPARING,*MOVING,'READY') and (now-self.ui_at>.15 or not self.focused):
            self.stop(now,'操作台租约失效',fault=True)
        if self.state in (*MOVING,'READY') and not self.fresh(now):
            self.stop(now,'MCU反馈/命令进展超时',fault=True)
        if self.state in MOVING and self.action_at is not None and now-self.action_at>=5.-1e-9:
            self.stop(now,'5秒单次测试结束；不会自动重跑')
        if self.state in PREPARING and now-self.state_at>3.:
            self.stop(now,'准备握手超时，不自动重试',fault=True)
        if self.state=='BASELINE':
            if self.stationary(now):
                if self.baseline_since is None:self.baseline_since=now
                if now-self.baseline_since>=.60:
                    if self.status['started_ms']:
                        self.pending.append(self.session.control_enable(False,True,self.stamp(now)))
                        self.state='WAIT_CLEAR'
                    else:
                        self.pending.append(self.session.select_mode(self.stamp(now)))
                        self.state='WAIT_SELECT'
                    self.state_at=now
            else:self.baseline_since=None
        elif self.state=='WAIT_CLEAR' and self.fresh(now) and self.status_at>self.state_at and not self.status['started_ms']:
            self.pending.append(self.session.select_mode(self.stamp(now)))
            self.state='WAIT_SELECT';self.state_at=now
        elif self.state=='WAIT_SELECT' and self.fresh(now) and self.status_at>self.state_at and self.status['selected']:
            if self.status['action_mode']!=2 or self.status['permitted']:
                self.stop(now,'选档回执不是失能悬空模式',fault=True)
            else:self.state='READY';self.state_at=now;self.event(now,'就绪：现在按住空格才转轮，最长5秒')
        elif self.state=='WAIT_ENABLE':
            if self.fresh(now) and self.status_at>self.state_at and self.status['permitted']:
                self.state='ACTIVE';self.event(now,'悬空测试运行中，松键立即撤能')
            elif now-self.state_at>1.:self.stop(now,'使能回执超时',fault=True)
        elif self.state=='ACTIVE' and self.fresh(now) and not self.status['permitted']:
            self.stop(now,'MCU许可提前丢失',fault=True)
        elif self.state=='STOPPING':
            if self.stationary(now):self.state=self.stop_target;self.event(now,self.reason+'；反馈确认停稳')
            elif now-self.state_at>1.5:
                self.state='FAULT';self.event(now,'停车未确认：请准备手动断电，不宣称已停稳')
        if self.session.connected and now-self.last_tx>=.019:
            if self.state=='ACTIVE' and self.down and self.focused and self.fresh(now):
                x,y=DIRECTIONS[self.direction]
                self.pending.append(self.session.cmd_vel(x,y,0,600,True,self.stamp(now)))
            elif self.state=='WAIT_ENABLE':
                pass # Unflagged zero would revoke an enable before its status ACK.
            else:self._zero(now)
            self.last_tx=now
        output,self.pending=self.pending,[]
        return output
