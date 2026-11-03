"""Immutable geometry cache, not a freshness or cross-consumer approval cache."""
import math
from types import FunctionType
from .rotation_source import Packet, Raw, decode_packet as original_decode


class CachedPacket(Packet):
    def __init__(self, samples):
        if type(samples) is not tuple or any(type(s) is not Raw or type(s.position) is not tuple
                or (s.quaternion is not None and type(s.quaternion) is not tuple) for s in samples):
            raise ValueError('IMMUTABLE_PACKET_REQUIRED')
        super().__init__(samples)
        object.__setattr__(self, '_pose', None)
        object.__setattr__(self, '_digest', None)

    @property
    def digest(self):
        if self._digest is None:
            object.__setattr__(self, '_digest', Packet.digest.fget(self))
        return self._digest

    def pose(self, now, ros):
        for s in self.samples:
            if (s.stamp != self.stamp or type(s.stamp) not in (int,float)
                    or type(s.received) not in (int,float) or not math.isfinite(s.stamp)
                    or not math.isfinite(s.received) or s.stamp <= 0
                    or not 0 <= now-s.received <= .20 or not -.005 <= ros-s.stamp <= .20):
                raise ValueError('SIGN_PACKET_AGE_OR_MIXED_FRAME')
        if self._pose is None:
            object.__setattr__(self, '_pose', Packet.pose(self, now, ros))
        return self._pose


decode_packet = FunctionType(original_decode.__code__, dict(original_decode.__globals__, Packet=CachedPacket),
                            original_decode.__name__, original_decode.__defaults__, original_decode.__closure__)
