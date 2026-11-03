"""Private finite-pilot UI model. No motion, ROS or keyboard synthesis."""
from .operator_model import OperatorModel
from .operator_guidance import reason_text as original_reason_text


class PilotOperatorModel(OperatorModel):
    def __init__(self):
        super().__init__()
        self.pilot_used = False

    @property
    def pilot_done(self):
        return self.pilot_used and self.status.get('state') != 'ACTIVE'

    def receive_status(self,value,now):
        super().receive_status(value,now)
        self.pilot_used |= value.get('pilot',{}).get('used') is True

    def pilot_valid(self):
        p=self.status.get('pilot',{})
        return p.get('max_active_s') == 5.0 and type(p.get('used')) is bool

    def can_start(self,now):
        return (self.pilot_valid() and not self.pilot_used
                and not self.status.get('hard_fault') and not self.gate.get('hard_fault')
                and not self.status['pilot'].get('prepare_blocked') and super().can_start(now))

    def can_recover(self,now):
        return self.pilot_valid() and not self.pilot_used and super().can_recover(now)

    def instruction(self,now):
        if self.closing:return '关闭中：使能已释放，等待链路结束'
        if not self.fresh(now):return '等待新鲜后端反馈；不可使能'
        if not self.pilot_valid():return '等待本入口专用单次测试状态；不可准备'
        state=self.status.get('state')
        if self.status.get('hard_fault') or self.gate.get('hard_fault'):
            return '实车保护已锁存：'+reason_text(self.status.get('reason') or self.gate.get('fault') or '硬故障')+'；保持松键停车，不能通过确认按钮解除'
        if self.pilot_done:
            if not self.status.get('stop_confirmed'):
                return '本轮已结束：请松开空格，等待停车确认；不可重新准备'
            return '本轮已结束并停稳：关闭窗口、报告结果；本窗口不能再次准备'
        if state=='ACTIVE':
            if not self.held or not self.focused:return '使能已释放：等待停车反馈，不会自动恢复'
            remaining=self.status['pilot'].get('remaining_s',0.)
            return f'本轮跟随中，剩余≤{remaining:.1f}秒；杆仅前移约5cm，松键立即停车'
        blocked=self.status['pilot'].get('prepare_blocked')
        if blocked:return blocked
        if state=='ARMED':
            if not self.status.get('gate_ready'):return '准备中：等待安全门与实车握手；暂不要按空格'
            if self.blocked or self.key_down:return '已准备：先松开一次空格，再重新按住；不用再点准备'
            return '准备就绪：按住空格并立即前移Sign约5cm；本轮最长5秒、仅一次'
        return super().instruction(now).replace('拖动虚拟杆','移动真实Sign').replace('拖动','移动真实Sign')


def real_instruction(model,now,hardware=False):
    return model.instruction(now)


def reason_text(reason):
    if 'PILOT_TIME_LIMIT_5S' in (reason or ''):return '本轮5秒已结束：正常限时撤能，不可重复准备'
    if 'PILOT_CONSUMED' in (reason or ''):return '本轮已使用：关闭窗口并报告结果，不可重复准备'
    if 'PILOT_START_REQUIRES_INTERIOR' in (reason or ''):return '小车起点在绿色目标区外：需人工向区域内部挪几厘米，软件不会自动挪车'
    return original_reason_text(reason)
