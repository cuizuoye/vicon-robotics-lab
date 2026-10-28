"""One released-key automatic PREPARE, never a synthetic start or recovery."""
import os
import time

from .novice_ui import acknowledge_physical_release

TOKEN='AUTO_PREPARE_SINGLE_SPACE_V1'


class AutoPrepareOnce:
    def __init__(self):
        self.started=self.stable_since=self.requested_at=None
        self.attempted=self.cancelled=False
        self.phase='WAITING'
        self.reason=''

    def cancel(self,reason):
        self.cancelled=True;self.phase='BLOCKED';self.reason=reason

    def poll(self,model,now,*,space_down,service_ready,input_issue=''):
        if self.started is None:self.started=now
        if self.cancelled:return False
        if model.session_requested or model.activation_used:
            # From the first real Space activation onward the unchanged
            # toggle model/backend own all stopping. Never auto-prepare again.
            self.phase='MANUAL_ACTIVE';return False
        s,g=model.status,model.gate
        t=getattr(model,'transport_status',{})
        if (model.closing or model.ended or model.pilot_used or s.get('hard_fault') or
            g.get('fault') or g.get('hard_fault') or t.get('local_stop') or t.get('fault') or
            t.get('startup_clock',{}).get('fault') or s.get('recovery_pending') or
            s.get('state') in ('FAULT','PAUSED','STOPPING','STOP_UNCONFIRMED')):
            self.cancel('状态已撤销或存在故障；不自动确认、恢复或重试')
            return False
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
        released=(not space_down and not model.key_down and not model.session_requested)
        safe=(released and model.focused and not input_issue and model.fresh(now)
            and s.get('state')=='STANDBY' and not s.get('input_health')
            and s.get('stop_confirmed') is True
            and 0<=now-getattr(model,'transport_at',float('-inf'))<=.25
            and t.get('fresh') is True and t.get('startup_clock',{}).get('ready') is True
            and not t.get('safety_enabled') and not t.get('motion_gate_requested')
            and service_ready and model.can_start(now))
        if not safe:self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=now
        if now-self.stable_since<.30-1e-9:return False
        self.attempted=True;self.requested_at=now;self.phase='PREPARING'
        model.record(now,'automatic_prepare_request_only_no_enable')
        return True

    def instruction(self):
        return dict(WAITING='自动检查/准备中：拿稳Sign、保持空格松开；无需点准备',
            PREPARING='自动准备中：等待安全门与实车确认；请暂不要按空格',
            READY='准备就绪：轻按并松开空格一次开始演示；再次空格或Esc停止',
            BLOCKED='自动准备已停止：'+self.reason+'；不会自动运动或重试').get(self.phase,'')


class AutoPrepareConsoleMixin:
    def __init__(self,root,node,model):
        self.auto_prepare=None;self.auto_prepare_stop_sent=False
        super().__init__(root,node,model)
        if node.input_kind=='hardware' and os.environ.get('WAND_AUTOPREP_FLOW')==TOKEN:
            self.auto_prepare=AutoPrepareOnce()
            self.buttons['start'].pack_forget()

    def loop(self):
        super().loop()
        policy=getattr(self,'auto_prepare',None)
        if policy is None or self.closing:return
        now=time.monotonic()
        down=self.server_space_down()
        if self.closing:return
        acknowledge_physical_release(self.model,down,now) # A real key-up only; cannot enable.
        ready=self.node.service_clients['start'].service_is_ready() and 'start' not in self.node.pending
        if policy.poll(self.model,now,space_down=down,service_ready=ready,input_issue=self.node.input_hint()):
            if not self.node.request('start'):
                policy.cancel('准备请求未被接受；不自动重试')
        if policy.cancelled and not self.auto_prepare_stop_sent:
            self.auto_prepare_stop_sent=True
            self.model.end_trial(now,policy.instruction())
            self.node.heartbeat()
            if policy.attempted or self.model.status.get('state') in ('ARMED','ACTIVE'):
                self.node.request('stop')

    def draw(self,now):
        super().draw(now)
        policy=getattr(self,'auto_prepare',None)
        if policy is not None and self.node.input_kind=='hardware':
            # Display only: retain coordinates, height, origin state and clamp
            # result, without the repeated explanatory paragraphs that push
            # the stop buttons outside a 720-pixel high-DPI window.
            lines=self.hand_text.get().splitlines()
            if len(lines)>=6:
                origin=('等待空格记录原点；准备不运动' if '尚未记录本轮原点' in lines[2]
                    else '本轮原点已记录')
                self.hand_text.set('\n'.join(lines[:2]+[
                    '1:1 · 2.0×0.9m · 四边缓冲0.5m',origin,lines[-1]]))
        if (policy is not None and not self.model.session_requested and not self.model.activation_used
                and not self.model.ended):
            self.step_text.set(self.node.input_hint() or policy.instruction())

