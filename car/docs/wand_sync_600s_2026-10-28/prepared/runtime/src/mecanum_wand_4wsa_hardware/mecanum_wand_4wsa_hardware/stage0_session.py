"""Explicit mode selection. Existing frame/sequence/CRC builder is reused."""
import math
import struct
from mecanum_vicon_control.mc_link_session import McLinkSession
from mecanum_vicon_control.transport_protocol import MessageType,FrameFlags,encode_frame,ProtocolError

class Stage0Session(McLinkSession):
    def __init__(self,*args,action_mode=1,stage=0,**kwargs):
        from .stages import get_stage
        self.stage=stage;self.limits=get_stage(stage)
        if action_mode not in (1,2):raise ValueError('fixed Sign or suspended mode required')
        super().__init__(*args,**kwargs)
        self.action_mode=action_mode

    def _control(self,enabled,clear,reserved,sender_time_ms):
        if not self.connected:raise ProtocolError('HELLO_ACK required')
        self.sequence=(self.sequence+1)&0xffffffff
        return encode_frame(MessageType.CONTROL_ENABLE,sequence=self.sequence,
            session_id=self.session_id,sender_time_ms=sender_time_ms&0xffffffff,
            flags=FrameFlags.ACK_REQUIRED,payload=struct.pack('<BBH',enabled,clear,reserved))

    def select_mode(self,sender_time_ms):
        return self._control(0,0,(0x7A00 if self.action_mode==1 else 0x7A80)+self.stage,sender_time_ms)

    def control_enable(self,enabled,clear_latched_faults,sender_time_ms):
        if enabled and clear_latched_faults:raise ProtocolError('enable cannot clear faults')
        return self._control(int(enabled),int(clear_latched_faults),
            ((0x7A00 if self.action_mode==1 else 0x7A80)+self.stage) if enabled else 0,sender_time_ms)

    def cmd_vel(self,vx_mps,vy_mps,wz_radps,timeout_ms,enabled,sender_time_ms):
        if (timeout_ms!=600 or not all(math.isfinite(v) for v in (vx_mps,vy_mps,wz_radps)) or
            math.hypot(vx_mps,vy_mps)>self.limits.speed+1e-9 or abs(wz_radps)>self.limits.yaw_rate+1e-9):
            raise ProtocolError('stage0 vector/yaw/lease cap')
        return super().cmd_vel(*(math.trunc(v*1000)/1000 for v in (vx_mps,vy_mps,wz_radps)),
            timeout_ms,enabled,sender_time_ms)
