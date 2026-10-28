"""Bounded virgin zero-baseline wait; prepare once, never enable or recover."""
import math
import os
import time

from .novice_ui import acknowledge_physical_release

TOKEN='AUTO_PREPARE_SINGLE_SPACE_V1'


class AutoPrepareOnce:
    def __init__(self):
        self.started=self.stable_since=self.requested_at=None
        self.attempted=self.cancelled=False
        self.phase='WAITING';self.reason=''
        self.boot_timeout_prepare_pending=False
        self.qualified_zero_baseline_seen=False

    def cancel(self,reason):
        self.cancelled=True;self.phase='BLOCKED';self.reason=reason

    def startup_timeout_context(self,model,now):
        """Only the initial, never-enabled, visibly ZERO COMMAND_TIMEOUT latch.

        This does not clear or alter the gate's fault. A fresh disabled arm
        epoch must still pass the unchanged independent gate and MCU bridge.
        No timeout after successful preparation/enable is eligible.
        """
        s,g=model.status,model.gate
        if (g.get('fault')!='COMMAND_TIMEOUT' or g.get('hard_fault')
                or g.get('output_zero') is not True or g.get('speed_mps')!=0.
                or g.get('primed_epoch') not in (None,'')
                or model.session_requested or model.activation_used or model.pilot_used
                or model.ended or model.closing or s.get('first_stop_reason') not in (None,'')
                or s.get('hard_fault') or s.get('recovery_pending')):
            return False
        t=getattr(model,'transport_status',{})
        if t.get('safety_enabled') or t.get('motion_gate_requested'):return False
        if not self.attempted:
            return s.get('state') in (None,'STANDBY') and s.get('epoch') in (None,'')
        return bool(self.boot_timeout_prepare_pending and self.requested_at is not None
            and 0<=now-self.requested_at<=5.
            and s.get('state') in ('STANDBY','ARMED'))

    @staticmethod
    def zero_baseline_qualified(model,now):
        s,g=model.status,model.gate;t=getattr(model,'transport_status',{})
        progress=t.get('command_progress_age_s')
        return bool(model.fresh(now) and s.get('graph_ok') is True and g.get('graph_ok') is True
            and g.get('output_zero') is True and g.get('speed_mps')==0.
            and g.get('primed_epoch') in (None,'') and s.get('epoch') in (None,'')
            and not s.get('input_health') and s.get('ready_to_start') is True
            and s.get('stop_confirmed') is True
            and 0<=now-getattr(model,'transport_at',-math.inf)<=.25
            and t.get('fresh') is True
            and (t.get('ready') is True or t.get('startup_reset_ready') is True)
            and t.get('baseline_current') is True
            and t.get('safety_enabled') is False and t.get('motion_gate_requested') is False
            and t.get('startup_clock',{}).get('ready') is True
            and t.get('startup_clock',{}).get('qualified') is True
            and type(progress) in (int,float) and math.isfinite(progress) and 0<=progress<=.10)

    def poll(self,model,now,*,space_down,service_ready,input_issue=''):
        if self.started is None:self.started=now
        if self.cancelled:return False
        if model.session_requested or model.activation_used:
            self.phase='MANUAL_ACTIVE';return False
        s,g=model.status,model.gate;t=getattr(model,'transport_status',{})
        boot_wait=self.startup_timeout_context(model,now)
        if self.attempted and not g.get('fault'):
            self.boot_timeout_prepare_pending=False
        if (model.closing or model.ended or model.pilot_used or s.get('hard_fault') or
            (g.get('fault') and not boot_wait) or g.get('hard_fault') or
            t.get('local_stop') or t.get('fault') or t.get('startup_clock',{}).get('fault') or
            s.get('recovery_pending') or s.get('state') in ('FAULT','PAUSED','STOPPING','STOP_UNCONFIRMED')):
            self.cancel('状态已撤销或存在故障；不自动确认、恢复或重试');return False
        if s.get('state')=='ACTIVE':
            self.cancel('未观察到本窗口新的人工空格启动');return False
        if self.attempted:
            if s.get('state')=='ARMED' and s.get('gate_ready') and model.start_ready(now):
                self.phase='READY'
            elif self.requested_at is not None and now-self.requested_at>5.:
                self.cancel('自动准备未在5秒内确认；请关闭并重新检查，不会补启动')
            else:self.phase='PREPARING'
            return False
        if s.get('state')=='ARMED':
            self.cancel('出现非本流程发起的准备状态');return False
        if now-self.started>30.:
            self.cancel('等待自动准备条件超过30秒；请重新检查定位和通信');return False
        released=not space_down and not model.key_down and not model.session_requested
        gate_baseline_ok=not boot_wait or self.zero_baseline_qualified(model,now)
        self.phase='WAITING_ZERO_BASELINE' if boot_wait else 'WAITING'
        safe=(released and model.focused and not input_issue and model.fresh(now)
            and s.get('state')=='STANDBY' and not s.get('input_health')
            and s.get('stop_confirmed') is True and gate_baseline_ok
            and 0<=now-getattr(model,'transport_at',-math.inf)<=.25
            and t.get('fresh') is True and t.get('startup_clock',{}).get('ready') is True
            and not t.get('safety_enabled') and not t.get('motion_gate_requested')
            and service_ready and model.can_start(now))
        if not safe:self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=now
        if now-self.stable_since<.30-1e-9:return False
        self.attempted=True;self.requested_at=now;self.phase='PREPARING'
        self.boot_timeout_prepare_pending=boot_wait
        self.qualified_zero_baseline_seen=boot_wait
        model.record(now,'automatic_prepare_request_only_no_enable')
        return True

    def instruction(self):
        return dict(WAITING='自动检查/准备中：拿稳Sign、保持空格松开；无需点准备',
            WAITING_ZERO_BASELINE='启动零输出建基准中：尚未准备/使能；等待定位、唯一来源、时钟及停稳确认',
            PREPARING='自动准备中：等待安全门与实车确认；请暂不要按空格',
            READY='准备就绪：轻按并松开空格一次开始演示；再次空格或Esc停止',
            BLOCKED='自动准备已停止：'+self.reason+'；不会自动运动或重试').get(self.phase,'')


class AutoPrepareConsoleMixin:
    def __init__(self,root,node,model):
        self.auto_prepare=None;self.auto_prepare_stop_sent=False;self.last_auto_phase=None
        super().__init__(root,node,model)
        if node.input_kind=='hardware' and os.environ.get('WAND_AUTOPREP_FLOW')==TOKEN:
            self.auto_prepare=AutoPrepareOnce();self.buttons['start'].pack_forget()

    def loop(self):
        super().loop()
        policy=getattr(self,'auto_prepare',None)
        if policy is None or self.closing:return
        now=time.monotonic();down=self.server_space_down()
        if self.closing:return
        acknowledge_physical_release(self.model,down,now)
        ready=self.node.service_clients['start'].service_is_ready() and 'start' not in self.node.pending
        if policy.poll(self.model,now,space_down=down,service_ready=ready,input_issue=self.node.input_hint()):
            if not self.node.request('start'):policy.cancel('准备请求未被接受；不自动重试')
        phase=(policy.phase,policy.reason)
        if phase!=getattr(self,'last_auto_phase',None):
            self.last_auto_phase=phase
            if hasattr(self.node,'get_logger'):
                self.node.get_logger().info('AUTO_PREPARE_PHASE '+str(dict(phase=phase[0],reason=phase[1],
                    attempted=policy.attempted,zero_baseline=policy.qualified_zero_baseline_seen)))
        if policy.cancelled and not self.auto_prepare_stop_sent:
            self.auto_prepare_stop_sent=True;self.model.end_trial(now,policy.instruction());self.node.heartbeat()
            if policy.attempted or self.model.status.get('state') in ('ARMED','ACTIVE'):self.node.request('stop')

    def draw(self,now):
        super().draw(now)
        policy=getattr(self,'auto_prepare',None)
        if policy is not None and self.node.input_kind=='hardware':
            lines=self.hand_text.get().splitlines()
            if len(lines)>=6:
                origin=('等待空格记录原点；准备不运动' if '尚未记录本轮原点' in lines[2] else '本轮原点已记录')
                self.hand_text.set('\n'.join(lines[:2]+['1:1 · 2.0×0.9m · 四边缓冲0.5m',origin,lines[-1]]))
        if (policy is not None and not self.model.session_requested and not self.model.activation_used
                and not self.model.ended):self.step_text.set(self.node.input_hint() or policy.instruction())
