"""急停节点中与 ROS 2 无关、可独立测试的安全逻辑。"""

from dataclasses import dataclass
from typing import Optional, Tuple


ACK_RESULT_NAMES = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


def ack_result_name(result: int) -> str:
    """把 VehicleCommandAck.result 转成便于记录的稳定文本。"""
    return ACK_RESULT_NAMES.get(int(result), f'UNKNOWN({int(result)})')


@dataclass
class TerminationGate:
    """飞行终止的双步骤确认门。

    ``prepare`` 只在显式启用时生成一个短时有效的确认窗口；``confirm``
    无论成功、过期还是失败都会消费该窗口，避免重复调用触发第二次动作。
    """

    enabled: bool = False
    window_s: float = 5.0
    deadline: Optional[float] = None

    def prepare(self, now: float) -> Tuple[bool, str]:
        """创建短时确认窗口。"""
        if not self.enabled:
            self.deadline = None
            return False, '飞行终止功能未启用'
        if self.window_s <= 0.0:
            self.deadline = None
            return False, '确认窗口必须大于 0 秒'
        self.deadline = float(now) + self.window_s
        return True, f'已准备，需在 {self.window_s:.1f} 秒内确认'

    def confirm(self, now: float) -> Tuple[bool, str]:
        """消费并校验当前确认窗口。"""
        deadline = self.deadline
        self.deadline = None
        if deadline is None:
            return False, '没有待确认的飞行终止请求'
        if float(now) > deadline:
            return False, '确认窗口已过期，请重新准备'
        return True, '确认通过'

    def cancel(self) -> Tuple[bool, str]:
        """取消当前确认窗口。"""
        had_pending = self.deadline is not None
        self.deadline = None
        if had_pending:
            return True, '已取消待确认的飞行终止请求'
        return True, '当前没有待确认请求'
