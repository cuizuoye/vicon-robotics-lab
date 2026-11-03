"""Backend command-intent model. No document-focus or physical-key synthesis."""
from mecanum_wand_4wsc_control.toggle_ui import ToggleOperatorModel
from mecanum_wand_4wsc_control.novice_ui import prepare_connection_ok,backend_hint


class SessionOperatorModel(ToggleOperatorModel):
    def __init__(self):
        super().__init__()
        self.backend_available=False
        # 'focused' stays False: this mode never pretends an invisible UI is focused.

    def start_ready(self,now):
        return bool(self.backend_available and not self.ended and not self.activation_used and not self.pilot_used
            and not self.blocked and not self.closing and self.fresh(now) and self.pilot_valid()
            and self.status.get('state')=='ARMED' and self.status.get('gate_ready')
            and not self.status.get('hard_fault') and not self.status.get('input_health')
            and not self.gate.get('fault') and not self.gate.get('hard_fault'))

    def can_start(self,now):
        return bool(self.backend_available and not self.ended and not self.activation_used and not self.pilot_used
            and not self.held and not self.blocked and not self.closing and self.fresh(now)
            and self.pilot_valid() and prepare_connection_ok(self,now)
            and self.status.get('ready_to_start') and self.status.get('state')=='STANDBY'
            and not self.status.get('recovery_pending') and not self.status.get('hard_fault')
            and not self.status.get('pilot',{}).get('prepare_blocked') and not self.gate.get('hard_fault'))

    def can_recover(self,now):return False

    def request_session(self,now):
        if not self.start_ready(now):return False
        self.held=self.activation_used=True;self.request_started=now
        self.record(now,'explicit_backend_session_request_no_key_synthesis')
        return True

    def tick(self,now):
        if (self.last_tick is not None and not 0<=now-self.last_tick<=.10
                and (self.held or self.activation_used)):
            self.end_trial(now,'后台控制循环超过100ms；本轮结束')
        self.last_tick=now
        if self.held:
            if (not self.backend_available or self.closing or not self.fresh(now)
                or self.status.get('state') not in ('ARMED','ACTIVE') or not self.pilot_valid()
                or self.status.get('hard_fault') or self.status.get('input_health')
                or self.gate.get('fault') or self.gate.get('hard_fault')):
                self.end_trial(now,'后端/定位/安全门异常；本轮撤权，不自动恢复')
            elif (self.status.get('state')=='ARMED' and self.request_started is not None
                  and now-self.request_started>=self.activation_timeout_s):
                self.end_trial(now,'启动确认超时；不延迟补启动')
        return self.held

    def instruction(self,now):
        if self.ended:return '本轮已结束；等待停车确认，不会自动恢复'
        if not self.fresh(now):return '等待新鲜后端状态；尚未使能'
        hint=backend_hint(self,now)
        if hint:return hint
        if self.held:return '后台限时跟随中；切换窗口、最小化继续。返回网页停车，或使用终端 --stop。'
        if self.start_ready(now):return 'READY：单击空格开始；切换窗口、最小化不会停车'
        return '正在检查定位、通信和安全门；尚未使能'
