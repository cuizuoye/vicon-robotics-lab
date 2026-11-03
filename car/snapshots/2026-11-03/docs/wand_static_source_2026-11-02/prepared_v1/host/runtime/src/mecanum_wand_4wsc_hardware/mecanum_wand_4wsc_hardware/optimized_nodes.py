"""4WSC-only 100Hz control, bounded exact-source commit, independent timer."""
import time
from .rotation_nodes import RotationSupervisorNode, RotationGateNode
from .bound_rotation import HistoryQosMixin, BoundGateMixin
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from .commissioning import AuthoritySigner, runtime_from_environment
from mecanum_wand_4wsc_control.cached_packet import decode_packet
from mecanum_wand_4wsc_control.bound_history import RetainedPackets
from mecanum_wand_4wsc_control.safety_core import ZERO
from mecanum_wand_4wsc_control.safety_core import fresh_envelope
from mecanum_wand_4wsc_control.safety_nodes import decode_frame
from mecanum_wand_4wsc_control.stop_feedback import StopFeedbackInputs
from mecanum_wand_4wsc_control.ros_nodes import run_node
from .source_relay import SourceRelay
import os
from .source_pipeline import AsyncGraphMixin

COMMAND_QOS=QoSProfile(depth=4,reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)


def stale_virgin_zero(frame,core,now,ros):
    """Ignore only delayed cold-discovery ZERO, never renew a frame/lease.

    The first fresh ARMED/ACTIVE still needs normal exact-source validation.
    After ANY primed/blocked epoch, owner change or true fault this is closed.
    """
    previous=core.frame
    virgin=(not core.hard_fault and core.fault in ('','COMMAND_TIMEOUT')
        and not core.primed_epoch and not core.blocked_epoch
        and (previous is None or (previous.state=='STANDBY' and not previous.enabled and not previous.epoch))
        and frame.state=='STANDBY' and not frame.enabled and not frame.epoch
        and not frame.hard_fault and frame.command==ZERO
        and core.owner in (None,frame.source) and frame.sequence>core.last_sequence)
    return bool(virgin and now-frame.issued_steady>.10 and ros-frame.issued_ros>.10
        and not fresh_envelope(frame.issued_steady,frame.issued_ros,now,ros,.10))


class Supervisor(AsyncGraphMixin, RotationSupervisorNode):
    def __init__(self, runtime=None):
        super().__init__(runtime)
        self.core.inputs=StopFeedbackInputs(self.core.c)
        self.core.inputs.graph_ok=False  # normal graph audit must qualify it
        self.packet_cache = RetainedPackets()

    def create_publisher(self,msg_type,topic,qos,*args,**kw):
        return super().create_publisher(msg_type,topic,COMMAND_QOS if topic=='candidate' else qos,*args,**kw)

    def create_timer(self, period, callback, *args, **kw):
        if period == .02 and getattr(callback, '__name__', '') == 'tick':
            period = .01
        return super().create_timer(period, callback, *args, **kw)

    def on_packet(self, msg):
        # The parent's method uses a module decoder; this private adapter
        # explicitly owns its independent decoding instead of mutating globals.
        import json
        now, ros = self.clocks()
        try:
            p = decode_packet(json.loads(msg.data), now, ros)
            if self.packet_cache.add(p, now, ros):
                if self.selected is None or p.stamp > self.selected.stamp:
                    self.core.inputs.observe_pose('wand', p.pose(now, ros), p.stamp, p.received, ros)
                    self.selected = p
        except (ValueError, TypeError, KeyError, AttributeError):
            self.selected = None
            self.core.inputs.poses.pop('wand', None)
            self.core.inputs.event('ROTATION_RAW_SOURCE_INVALID')


class Gate(AsyncGraphMixin, BoundGateMixin, RotationGateNode):
    def __init__(self, runtime=None):
        runtime = runtime or runtime_from_environment()
        super().__init__(runtime)
        # Explicit runtime construction is used by both software and the real
        # launch path. Never fall back to the old unsigned software signer.
        self.authority_signer = AuthoritySigner(os.environ['WAND_4WSC_CHANNEL_KEY'], runtime.config.stage)
        self.commit_count = 0
        self.last_commit_timer = None

    def create_subscription(self,msg_type,topic,callback,qos,*args,**kw):
        if topic=='candidate':
            # Do not let the raw-source history mixin turn the continuous
            # command queue into32 old controls (~320ms at100Hz). In-order
            # neutral/epoch/reference checks still reject ANY skipped unsafe
            # transition; this is not an unchecked latest-command shortcut.
            return Node.create_subscription(self,msg_type,topic,callback,COMMAND_QOS,*args,**kw)
        return super().create_subscription(msg_type,topic,callback,qos,*args,**kw)

    def on_candidate(self,msg):
        import json
        try:
            value=json.loads(msg.data)
            if set(value)=={'frame','sign_digest','sign_stamp','car_stamp'}:
                from mecanum_wand_4wsc_control.rotation_source import canonical
                f=decode_frame(canonical(value['frame']))
                if stale_virgin_zero(f,self.core,*self.clocks()):
                    self.publish_output(ZERO)
                    return  # No core.receive, counter, timestamp or clock update.
        except (ValueError,TypeError,KeyError,AttributeError):
            pass  # Original strict decoder owns rejection and revocation.
        return super().on_candidate(msg)

    def on_packet(self, msg):
        import json
        now, ros = self.clocks()
        try:
            self.packet_cache.add(decode_packet(json.loads(msg.data), now, ros), now, ros)
            self.process_pending()
        except (ValueError, TypeError, KeyError, AttributeError):
            self.core.inputs.poses.pop('wand', None)
            self.core.inputs.event('ROTATION_RAW_SOURCE_INVALID')

    def process_pending(self):
        previous_sequence = self.core.last_sequence
        super().process_pending()
        f = self.core.frame
        if (self.core.last_sequence > previous_sequence and f and f.state == 'ACTIVE'
                and not self.core.fault and not self.core.hard_fault):
            baseline = self.core.clock.previous
            self.publish_output(self.core.output)
            if self.core.clock.previous != baseline:
                raise RuntimeError('event commit must not refresh watchdog timer')
            self.last_commit_timer = baseline
            self.commit_count += 1

    def publish_output(self, output):
        # Coalesce only a duplicate already-zero ACTIVE permit for the SAME
        # validated frame. Every transition to zero, new frame, revocation,
        # STOP/ESTOP and fault bypasses this optimization. Watchdogs still tick.
        f = self.core.frame
        cid = (f.source, f.sequence, f.epoch) if f else None
        now = time.monotonic()
        duplicate_active_zero = bool(output == ZERO and getattr(self, '_last_output', None) == ZERO
            and f and f.state == 'ACTIVE' and f.enabled and f.epoch
            and f.epoch == self.core.primed_epoch)
        if ((output != ZERO or duplicate_active_zero) and not self.core.fault and not self.core.hard_fault
                and cid == getattr(self, '_last_cid', None)
                and now-getattr(self, '_last_emitted', -float('inf')) < (.020 if duplicate_active_zero else .008)):
            return
        super().publish_output(output)
        self._last_cid, self._last_emitted = cid, now
        self._last_output = output


def source_main(): run_node(SourceRelay)
def supervisor_main(): run_node(Supervisor)
def gate_main(): run_node(Gate)
