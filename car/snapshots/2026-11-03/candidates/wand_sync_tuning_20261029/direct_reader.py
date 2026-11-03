"""Passive domain0 Sign reader plus car01 reference, never a command publisher."""
import sys
import time
from model import ROOT
sys.path.insert(0, str(ROOT/'candidates/wand_fast_follow_ros_sim_20261029'))
from live_reader import RealReader


class DirectReader(RealReader):
    def __init__(self):
        self.car01 = None
        self.car01_identity = False
        self.rejection_history = []
        super().__init__()
        from geometry_msgs.msg import PoseStamped
        from mecanum_wand_4ws8_control.safety_nodes import SENSOR
        self.node.create_subscription(PoseStamped, '/vicon/car01/car01', self.car_receive, SENSOR)

    def car_receive(self, msg):
        from mecanum_wand_4ws8_control.safety_nodes import decode_pose
        try:
            p, stamp, roll, pitch = decode_pose(msg, 'map')
            with self.lock:
                self.car01 = dict(pose=dict(x=p.x, y=p.y, z=p.z, yaw=p.yaw), stamp=stamp)
        except (ValueError, TypeError):
            with self.lock:
                self.car01 = None

    def audit(self):
        previous = self.rejected
        super().audit()
        if self.rejected != previous:
            self.rejection_history.append(dict(at=time.monotonic(), reason=self.error,
                                               raw_rejected=self.rejected))
            self.rejection_history = self.rejection_history[-16:]
        from receiver_guard import source_ok
        self.car01_identity = source_ok(self.node.get_publishers_info_by_topic('/vicon/car01/car01'))

    def receive(self, name, msg):
        previous = self.rejected
        super().receive(name, msg)
        if self.rejected != previous:
            self.rejection_history.append(dict(at=time.monotonic(), reason=self.error,
                                               raw_rejected=self.rejected))
            self.rejection_history = self.rejection_history[-16:]

    def status(self):
        value = super().status()
        value['raw_rejection_history'] = self.rejection_history.copy()
        with self.lock:
            car = self.car01
        ros = self.node.get_clock().now().nanoseconds*1e-9
        value['car01_reference_only'] = dict(
            valid=bool(car and self.car01_identity and -.005 <= ros-car['stamp'] <= .2),
            pose=car['pose'] if car else None, stamp=car['stamp'] if car else None,
            used_as_motion_feedback=False)
        return value
