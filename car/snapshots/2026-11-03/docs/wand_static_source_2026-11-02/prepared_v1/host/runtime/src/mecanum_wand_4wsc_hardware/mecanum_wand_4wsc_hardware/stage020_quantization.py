"""Inward SI-to-mm/s quantisation at the private first-stage wire boundary."""
import math
from mecanum_vicon_control.mc_link_session import McLinkSession
from mecanum_vicon_control.transport_protocol import ProtocolError


def inward_velocity(vx,vy,wz):
    if (not all(math.isfinite(v) for v in (vx,vy,wz))
            or math.hypot(vx,vy)>.20+1e-9 or abs(wz)>.10+1e-9):
        raise ProtocolError('4W20 vector/yaw envelope')
    # Rounding axes independently can make a valid 0.20m/s vector exceed
    # 200mm/s and fault the MCU. Truncate towards zero; never widen its cap.
    return tuple(math.trunc(v*1000)/1000 for v in (vx,vy,wz))


class FirstStageSession(McLinkSession):
    def cmd_vel(self,vx_mps,vy_mps,wz_radps,timeout_ms,enabled,sender_time_ms):
        vx,vy,wz=inward_velocity(vx_mps,vy_mps,wz_radps)
        return super().cmd_vel(vx,vy,wz,timeout_ms,enabled,sender_time_ms)
