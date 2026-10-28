"""Bounded INTERNAL relay history; raw Vicon input QoS/deadlines unchanged."""
from collections import OrderedDict
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from .rotation_source import PacketCache

HISTORY_QOS=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)


class RetainedPackets(PacketCache):
    """Accept late exact references without rewinding latest or refreshing age."""
    def prune(self,now,ros):
        for digest,p in list(self.packets.items()):
            if not 0<=now-p.received<=.20 or not -.005<=ros-p.stamp<=.20:
                del self.packets[digest]

    def add(self,p,now,ros):
        p.pose(now,ros);self.prune(now,ros)
        for old in self.packets.values():
            if old.stamp==p.stamp and old.digest!=p.digest:
                raise ValueError('SIGN_SAME_STAMP_CONFLICT')
        if p.digest in self.packets:return False
        self.packets[p.digest]=p;self.last_stamp=max(self.last_stamp,p.stamp)
        self.packets=OrderedDict(sorted(self.packets.items(),key=lambda v:v[1].stamp))
        while len(self.packets)>64:self.packets.popitem(last=False)
        return True

    def exact(self,key,stamp,now,ros):
        # Inherited exact validates original source time and geometry. Pruning
        # here would hide the distinction between missing and stale references.
        return super().exact(key,stamp,now,ros)
