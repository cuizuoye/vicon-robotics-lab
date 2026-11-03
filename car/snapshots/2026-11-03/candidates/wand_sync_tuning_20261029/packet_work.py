"""Per-consumer memo of immutable raw geometry/digest, NEVER a freshness memo.

Each decoder creates its own packet, independently validates the original six
samples and digest, then reuses only time-invariant results for that object.
Every pose access still checks ALL original source/receipt timestamps.
"""
import math
from types import FunctionType

from model import ROOT
from mecanum_wand_4ws8_control.rotation_source import Packet, Raw, decode_packet as original_decode


class CachedPacket(Packet):
    def __init__(self, samples):
        if type(samples) is not tuple:
            raise ValueError('IMMUTABLE_PACKET_REQUIRED')
        for sample in samples:
            if (type(sample) is not Raw or type(sample.position) is not tuple
                    or (sample.quaternion is not None and type(sample.quaternion) is not tuple)
                    or any(not isinstance(v, (str, int, float, bool, type(None)))
                           for v in sample.position+(sample.quaternion or ()))):
                raise ValueError('IMMUTABLE_RAW_REQUIRED')
        super().__init__(samples)
        object.__setattr__(self, '_pose', None)
        object.__setattr__(self, '_digest', None)

    @property
    def digest(self):
        if self._digest is None:
            object.__setattr__(self, '_digest', Packet.digest.fget(self))
        return self._digest

    def pose(self, now, ros):
        # These are the unchanged original age/mixed-frame/type checks, on
        # EVERY access, including repeated references and after IPC silence.
        for sample in self.samples:
            if (sample.stamp != self.stamp or type(sample.stamp) not in (int, float)
                    or type(sample.received) not in (int, float)
                    or not math.isfinite(sample.stamp) or not math.isfinite(sample.received)
                    or sample.stamp <= 0 or not 0 <= now-sample.received <= .20
                    or not -.005 <= ros-sample.stamp <= .20):
                raise ValueError('SIGN_PACKET_AGE_OR_MIXED_FRAME')
        if self._pose is None:
            # Original implementation checks names, finite vectors, normalized
            # quaternion, all 5 marker residuals, forward degeneracy and pose.
            object.__setattr__(self, '_pose', Packet.pose(self, now, ros))
        return self._pose


decode_packet = FunctionType(original_decode.__code__,
    dict(original_decode.__globals__, Packet=CachedPacket), original_decode.__name__,
    original_decode.__defaults__, original_decode.__closure__)


_synthetic = None


def install_packets(module, enabled=True):
    global _synthetic
    import bound_rotation
    from mecanum_wand_4ws8_hardware import rotation_nodes
    # Process-local decoding factories only. Two subscribers still decode and
    # validate separately; no shared cross-consumer geometry-approval cache.
    bound_rotation.decode_packet = decode_packet if enabled else original_decode
    rotation_nodes.decode_packet = decode_packet if enabled else original_decode
    if _synthetic is None: _synthetic = module.synthetic_packet
    module.synthetic_packet = FunctionType(_synthetic.__code__,
        dict(_synthetic.__globals__, Packet=CachedPacket if enabled else Packet), _synthetic.__name__,
        _synthetic.__defaults__, _synthetic.__closure__)
