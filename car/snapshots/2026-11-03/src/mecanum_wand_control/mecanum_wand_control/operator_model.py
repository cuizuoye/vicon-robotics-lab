"""Pure input-state logic for the offline console; no ROS or GUI dependencies."""

import math


class OperatorModel:
    def __init__(self):
        self.x = self.y = 0.
        self.held = self.key_down = self.focused = self.blocked = self.closing = False
        self.status, self.gate = {}, {}
        self.status_at = self.gate_at = -math.inf
        self.last_tick = None
        self.notice = '仅仿真：等待后端和窗口焦点'
        self.events = []

    def record(self, now, kind):
        self.events.append(dict(steady=now, event=kind, held=self.held, x=self.x, y=self.y))

    def fresh(self, now):
        return 0 <= now-self.status_at <= .25 and 0 <= now-self.gate_at <= .25

    def receive_status(self, value, now):
        self.status, self.status_at = value, now

    def receive_gate(self, value, now):
        self.gate, self.gate_at = value, now

    def focus(self, value, now):
        self.focused = bool(value)
        if not value:
            self.revoke(now, '窗口失焦：已释放使能，需松开空格并重新准备')

    def revoke(self, now, reason):
        if self.held or not self.blocked: self.record(now, reason)
        self.held, self.blocked = False, True
        self.notice = reason

    def press(self, now):
        if self.key_down: return False  # OS autorepeat is not a new human edge.
        self.key_down = True
        if (self.blocked or self.closing or not self.focused or not self.fresh(now)
                or self.status.get('state') not in ('ARMED', 'ACTIVE')
                or not self.status.get('gate_ready') or self.gate.get('fault')
                or self.gate.get('hard_fault')):
            self.revoke(now, '空格使能未接受：'+self.instruction(now))
            return False
        self.held = True
        self.notice = '空格按住：虚拟车跟随；松开或切出窗口即停止'
        self.record(now, 'enable_pressed')
        return True

    def release(self, now):
        if self.held: self.record(now, 'enable_released')
        self.held = self.key_down = self.blocked = False
        self.notice = '使能已松开；等待停车反馈，不自动恢复'

    def tick(self, now):
        if self.last_tick is not None and not 0 <= now-self.last_tick <= .10:
            self.revoke(now, '操作台调度中断：使能已锁止，需重新松开空格')
        self.last_tick = now
        if not self.focused or self.closing or not self.fresh(now):
            if self.held: self.revoke(now, '后端失联/窗口失焦：使能已释放')
        if self.status.get('state') not in ('ARMED', 'ACTIVE') and self.held:
            self.revoke(now, '后端已退出运动状态：需重新准备')
        return self.held

    def can_start(self, now):
        return (self.focused and not self.held and not self.key_down and not self.blocked and not self.closing
                and self.fresh(now) and self.status.get('ready_to_start')
                and self.status.get('state') in ('STANDBY', 'PAUSED')
                and not self.status.get('recovery_pending'))

    def can_recover(self, now):
        return (self.focused and not self.held and not self.key_down and not self.blocked and not self.closing
                and self.fresh(now) and self.status.get('state') in ('FAULT', 'PAUSED')
                and self.status.get('stop_confirmed') and not self.status.get('hard_fault')
                and not self.gate.get('hard_fault')
                and not self.status.get('input_health') and not self.status.get('recovery_pending'))

    def instruction(self, now):
        """Explain the current interlock, rather than only the last callback."""
        if self.closing: return '关闭中：使能已释放，等待软件链路结束'
        if not self.focused: return '点击本窗口；失焦后须先轻按并松开空格，再重新准备'
        if not self.fresh(now): return '等待新鲜后端反馈；不可使能'
        state = self.status.get('state')
        if state == 'ACTIVE':
            return ('正在跟随：持续按住空格并拖动；松开即暂停' if self.held else
                    '使能已释放：等待后端停车确认，不会自动恢复')
        if state in ('STOPPING', 'STOP_UNCONFIRMED'):
            return '等待停车反馈确认；请松开空格，不可重新使能'
        if self.status.get('recovery_pending'):
            return '已请求重新准备：保持空格松开，等待反馈稳定；就绪后还须重新按住才会运动'
        if state == 'FAULT':
            return ('急停/硬故障已锁存：退出重启此仿真' if self.status.get('hard_fault') else
                    '故障停车：松开空格，待停车确认后点击确认可恢复故障')
        if state == 'ARMED' and self.held:
            return '使能已发送：等待后端进入ACTIVE，请继续按住空格'
        if self.blocked or self.key_down:
            return ('已准备，但输入仍锁止：松开一次空格，再重新按住（不用再点准备）' if state == 'ARMED' else
                    '先轻按并松开一次空格，确认松键；然后点击准备')
        if state == 'ARMED':
            return ('准备就绪：现在按住空格，并保持按住拖动虚拟杆' if self.status.get('gate_ready') else
                    '准备中：等待独立安全门握手，请暂不要按空格')
        if self.can_start(now): return '输入已解锁：点击准备，等就绪后持续按住空格拖动'
        return '等待位姿稳定及停车确认；保持空格松开'

    def move(self, x, y):
        if not all(math.isfinite(v) for v in (x, y)): return False
        # This is a bounded virtual input device, not clipping the 2x mapping.
        radius = math.hypot(x, y)
        factor = min(1., .245/radius) if radius else 1.
        x, y = x*factor, y*factor
        anchors = self.status.get('anchors')
        if self.status.get('state') == 'ACTIVE' and anchors:
            origin = anchors['wand_origin']
            ax, ay = origin['x']+.4, origin['y']+.4
            dx, dy = x-ax, y-ay
            distance = math.hypot(dx, dy)
            fraction = min(1., .245/distance) if distance else 1.
            x, y = ax+fraction*dx, ay+fraction*dy
        self.x, self.y = x, y
        return True
