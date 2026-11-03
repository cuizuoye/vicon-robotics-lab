"""四轮通信只读固件的60字节LINK_STATUS扩展，不改变MC-Link v1基础布局。"""
import struct
from .transport_protocol import ProtocolError

STATUS = struct.Struct('<6I8I2H')
TAG = 0x314f5234  # little endian: 4RO1


def decode_readonly_status(payload: bytes) -> dict:
    if len(payload) != STATUS.size:
        raise ProtocolError('LINK_STATUS is not the 60-byte four-wheel readonly schema')
    v = STATUS.unpack(payload)
    if v[0] != TAG or any(v[6:14]) or v[14] != 1 or v[15] != 0:
        raise ProtocolError('readonly identity, GPIO, CCR or watchdog check failed')
    if not 1 <= v[3] <= v[4] <= 30 or v[5] > 1000:
        raise ProtocolError('100Hz sampling deadline or ISR time violated')
    return dict(tag='4RO1', ticks=v[1], updates=v[2], dt_min=v[3], dt_max=v[4],
                irq_us_max=v[5], compares=list(v[6:14]), gpio_low=v[14], watchdog_reset=v[15])
