"""联合辅助的本机输入中断演练状态；不改网络、不控制电机。"""

import math
from usb_vision_policy import SourceLossLatch


class CombinedInputLoss:
    def __init__(self):
        self.latch=SourceLossLatch()
        self.masked_at_s=None
        self.blocked_at_s=None
        self.tx_count_when_blocked=None

    def mask(self,now_s,*,fusion_verified):
        if not math.isfinite(now_s) or now_s<0 or fusion_verified is not True or self.masked_at_s is not None:
            raise ValueError('Loss test needs verified fusion and one explicit input suppression')
        self.masked_at_s=now_s

    def permits(self,fresh,now_s,tx_count):
        if self.masked_at_s is None:
            raise ValueError('Planned suppression not started')
        if not math.isfinite(now_s) or now_s<self.masked_at_s or type(tx_count) is not int or tx_count<0:
            raise ValueError('Invalid loss observation')
        permitted=self.latch.permits(fresh)
        if not permitted and self.blocked_at_s is None:
            self.blocked_at_s=now_s
            self.tx_count_when_blocked=tx_count
        return permitted

    def result(self,*,last_tx_s,tx_count_after_recovery,suppressed,fresh_restored,rollback_verified):
        delay=(last_tx_s-self.masked_at_s)*1000 if self.masked_at_s is not None else None
        detection=(self.blocked_at_s-self.masked_at_s)*1000 if self.blocked_at_s is not None else None
        checks={
            'input_loss_latched':self.latch.blocked and self.blocked_at_s is not None,
            'last_position_write_within_150ms_of_mask':delay is not None and math.isfinite(delay) and delay<=150,
            # 150ms新鲜度门限后允许最多25ms调度观察延迟；不延长位姿有效期。
            'stale_detected_within_175ms_of_mask':detection is not None and 0<=detection<=175,
            'no_position_writes_after_latch_or_recovery':self.tx_count_when_blocked is not None
                and tx_count_after_recovery==self.tx_count_when_blocked,
            'real_ros_messages_suppressed':suppressed>=10,
            'fresh_source_restored':fresh_restored is True,
            'fusion_zero_restored':rollback_verified is True,
        }
        return {'scope':'Local subscriber suppression only; NOT optical occlusion or airborne failsafe',
                'masked_at_s':self.masked_at_s,'blocked_at_s':self.blocked_at_s,
                'last_tx_relative_to_mask_ms':delay,'stale_detection_delay_ms':detection,
                'tx_count_when_blocked':self.tx_count_when_blocked,
                'tx_count_after_recovery':tx_count_after_recovery,'suppressed_ros_messages':suppressed,
                'checks':checks,'verified':all(checks.values()),
                'automatic_restart_allowed':False,'airborne_stop_or_land_verified':False}
