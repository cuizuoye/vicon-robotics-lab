"""Explicit one-click session intent, NOT a synthetic physical held key.

The legacy Lease.held field carries this mode's logical session request. The
physical key_down flag is independent. Backend120s and short leases are intact.
"""
from .demo_policy import INPUT_MODE
from .pilot_ui import PilotOperatorModel
from .novice_ui import backend_hint
from .pose_quality import stop_hint


class ToggleOperatorModel(PilotOperatorModel):
    input_mode = INPUT_MODE
    click_timeout_s = 1.
    activation_timeout_s = 1.

    def __init__(self):
        super().__init__()
        self.click_pending = False
        self.click_started = self.request_started = None
        self.activation_used = self.ended = False
        self.notice = '单击模式：准备就绪后轻按并松开空格开始；再次空格或Esc停止；松键不停车'

    @property
    def session_requested(self):return self.held

    def pilot_valid(self):
        return (super().pilot_valid() and
            self.status.get('pilot',{}).get('operator_input_mode') == INPUT_MODE)

    def start_ready(self, now):
        return bool(not self.ended and not self.activation_used and not self.pilot_used
            and not self.blocked and not self.closing and self.focused and self.fresh(now)
            and self.pilot_valid() and self.status.get('state') == 'ARMED'
            and self.status.get('gate_ready') and not self.status.get('hard_fault')
            and not self.status.get('input_health') and not self.gate.get('fault')
            and not self.gate.get('hard_fault'))

    def press(self, now):
        if self.key_down:return False  # Real autorepeat is not a second click.
        self.key_down = True
        if self.held:
            # Stop first, regardless of feedback freshness, focus or gate state.
            self.end_trial(now,'再次空格：已请求停止，本轮不能再次启动')
            return False
        if not self.start_ready(now):
            self.click_pending = False
            self.notice = '单击启动未接受：'+self.instruction(now)
            return False
        self.click_pending,self.click_started = True,now
        self.notice = '请松开空格完成本次单击；此时尚未使能'
        self.record(now,'toggle_click_pending_no_enable')
        return False

    def release(self, now, *, commit=True):
        was_down,pending = self.key_down,self.click_pending
        started = self.click_started
        self.key_down,self.click_pending,self.click_started = False,False,None
        if (was_down and pending and commit and started is not None
                and 0<=now-started<=self.click_timeout_s and self.start_ready(now)):
            self.held,self.activation_used,self.request_started = True,True,now
            self.notice = '单击启动已发送；松键继续跟随，再次空格或Esc停止'
            self.record(now,'toggle_session_requested_physical_key_up')
            return True
        if not self.ended and not self.held:
            self.blocked = False  # Release can unlock, but never enable here.
        if pending:
            self.notice = '本次单击未启动：已取消、超时或条件改变；不会自动补启动'
        # Releasing a key during an already requested session is NOT a stop.
        return False

    def observe_physical_release(self, now):
        """Polling may clear key state/cancel a click; never create authority."""
        return self.release(now,commit=False)

    def revoke(self, now, reason):
        if (self.held or self.click_pending or self.activation_used
                or self.status.get('state') in ('ARMED','ACTIVE')):
            self.ended = True
        self.click_pending,self.click_started = False,None
        super().revoke(now,reason)

    def end_trial(self, now, reason='Esc/停止：本轮已结束，不会自动恢复'):
        self.ended = True
        self.revoke(now,reason)

    def tick(self, now):
        super().tick(now)
        if self.click_pending and (self.click_started is None
                or not 0<=now-self.click_started<=self.click_timeout_s
                or not self.start_ready(now)):
            self.click_pending,self.click_started = False,None
            self.notice = '启动单击已取消：请保持停止；不会在条件恢复后自动启动'
        if self.held:
            if (not self.pilot_valid() or self.status.get('hard_fault')
                    or self.status.get('input_health') or self.gate.get('hard_fault')
                    or self.gate.get('fault')):
                self.revoke(now,'后端/安全门异常：本轮已撤能，不能自动恢复')
            elif (self.status.get('state') == 'ARMED' and self.request_started is not None
                    and now-self.request_started>=self.activation_timeout_s):
                self.revoke(now,'启动确认超时：本轮已撤能，不会延迟补启动')
        return self.held

    def can_start(self, now):
        return not self.ended and not self.activation_used and super().can_start(now)

    def can_recover(self, now):
        return not self.ended and not self.activation_used and super().can_recover(now)

    def instruction(self, now):
        if self.closing:return '关闭中：本轮已撤能，等待停车确认'
        if not self.fresh(now):return '等待新鲜后端反馈；不能启动；Esc/停止仍可撤能'
        if not self.pilot_valid():return '等待专用单击模式状态；不能用长按版状态启动'
        if self.ended or self.pilot_done:
            return stop_hint(self.status,self.gate) or '本轮已结束/取消：等待停车确认；本窗口不能再次启动'
        hint=backend_hint(self,now)
        if hint:return hint
        if not self.focused:return '点击本窗口；窗口失焦会停车，不会自动恢复'
        if self.held and self.status.get('state')=='ACTIVE':
            remaining=self.status['pilot'].get('remaining_s',0.)
            return f'单击跟随中，剩余≤{remaining:.1f}秒；松键继续；再次空格、Esc或停止按钮结束'
        if self.held:return '启动请求已发送，等待实车反馈；不用长按；再次空格或Esc可停止'
        if self.click_pending:return '松开空格完成单击启动；按键自动重复不会反复启停'
        if self.status.get('state')=='ARMED':
            return ('准备就绪：轻按并松开空格一次开始120秒；松键不停车；再次空格/Esc停止'
                if self.status.get('gate_ready') else '准备中：等待安全门握手，暂不要按空格')
        return '单击模式：点击准备，等就绪后轻按并松开空格一次；打开窗口不会运动'
