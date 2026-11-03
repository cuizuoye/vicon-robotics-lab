"""Private domain190 SIM: scoped authority + bounded, validated fast commit.

Original entrypoints never import this module. The independent 10ms gate
watchdog still owns ClockCheck.previous; event commits NEVER renew it.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
from types import FunctionType, SimpleNamespace

from model import ROOT, Limits, Supervisor as ModelSupervisor, Gate as ModelGate, Session, ZERO, wheel_rpm
TRACE = ROOT/'candidates/wand_software_latency_trace_20261029'
sys.path.insert(0, str(TRACE))
import instrument
from trace_log import TraceLog, PublishTap, control_id
from mecanum_wand_4ws8_hardware import commissioning

SIM_SCOPE = 'PRIVATE_SIM_SYNC_TUNING_V1_'
_encode = commissioning.AuthoritySigner.encode
_decode = commissioning.AuthorityVerifier.decode
_sim_encode = FunctionType(_encode.__code__, dict(_encode.__globals__,SCOPE=SIM_SCOPE),
                           _encode.__name__, _encode.__defaults__, _encode.__closure__)
_sim_encode.__kwdefaults__ = _encode.__kwdefaults__
_sim_decode = FunctionType(_decode.__code__, dict(_decode.__globals__,SCOPE=SIM_SCOPE),
                           _decode.__name__, _decode.__defaults__, _decode.__closure__)


class Signer(commissioning.AuthoritySigner):
    encode = _sim_encode

    def __init__(self, key, profile):
        super().__init__(key,4)
        self.stage = profile.name


class Verifier(commissioning.AuthorityVerifier):
    def __init__(self, key, profile):
        super().__init__(key,4)
        self.stage = profile.name
        self.profile = profile
        self.limits = SimpleNamespace(speed=profile.max_speed_mps)

    def decode(self, text, steady, ros):
        payload, command = _sim_decode(self,text,steady,ros)
        if max(abs(rpm) for rpm in wheel_rpm(command)) > self.profile.sim_max_rpm+1e-6:
            raise ValueError('SIM ONLY wheel envelope')
        return payload,command


def install(log, profile=Limits(), optimized=True):
    module = instrument.install(log)
    from packet_work import install_packets
    install_packets(module, optimized)
    module.DOMAIN = 190
    module.config = lambda:profile
    # All substitutions live in this standalone interpreter, never source files.
    from mecanum_wand_4ws8_control import safety_nodes
    import bound_rotation
    safety_nodes.Supervisor = ModelSupervisor
    safety_nodes.FinalGate = ModelGate
    bound_rotation.SourceFaultGate = ModelGate
    module.RotationSupervisor = ModelSupervisor
    module.SourceFaultGate = ModelGate
    module.AuthoritySigner = lambda key, stage:Signer(key,profile)
    module.AuthorityVerifier = lambda key, stage:Verifier(key,profile)

    class Supervisor(instrument.Supervisor):
        def tick(self):
            begin = time.monotonic_ns()
            cpu = time.thread_time_ns()
            report = self.core.gate_report
            previous = getattr(self, '_last_timer_ns', None)
            self._last_timer_ns = begin
            try:
                return super().tick()
            finally:
                log.emit('control_timer', begin_ns=begin, end_ns=time.monotonic_ns(),
                    cpu_ns=time.thread_time_ns()-cpu,
                    gap_ns=begin-previous if previous is not None else None,
                    gate_report_age_s=begin/1e9-report['steady'] if report else None,
                    state=self.core.state, reason=self.core.reason)

        def on_gate(self, msg):
            super().on_gate(msg)
            report=self.core.gate_report
            if report:
                log.emit('gate_report_receive', issued_ns=round(report['steady']*1e9),
                         epoch=report['epoch'],fault=report['fault'])

        def use_candidate(self):
            c = self.core.c
            self.core.follower = Session(c.follower_config(), c.project_goal, c.project_reference)

        def create_timer(self, period, callback, *args, **kwargs):
            if optimized and period == .02 and getattr(callback,'__name__','') == 'tick':
                period = .01
            return super().create_timer(period,callback,*args,**kwargs)

        def create_publisher(self, msg_type, topic, *args, **kwargs):
            pub = super().create_publisher(msg_type,topic,*args,**kwargs)
            if topic != 'candidate':return pub
            def publish(msg):
                pub.publish(msg)
                f = self.core.follower
                car = self.core.inputs.poses.get('car',{}).get('pose')
                goal = f.mapped_goal
                error = math.hypot(goal.x-car.x,goal.y-car.y) if goal and car else None
                log.emit('tracking',cid=self.last_control,state=self.core.state,
                    goal=asdict(goal) if goal else None,reference=asdict(f.goal) if f.goal else None,
                    car=asdict(car) if car else None,error_m=error,
                    reference_limited=f.reference_limited,catchup=f.catchup_active,
                    fast_feedforward_ticks=f.moving_feedforward_ticks)
            return PublishTap(pub,publish)

    class Gate(instrument.Gate):
        def tick(self):
            begin = time.monotonic_ns()
            cpu = time.thread_time_ns()
            previous = self.core.clock.previous
            try:
                return super().tick()
            finally:
                log.emit('gate_timer', begin_ns=begin, end_ns=time.monotonic_ns(),
                    cpu_ns=time.thread_time_ns()-cpu, previous=previous,
                    fault=self.core.fault, hard_fault=self.core.hard_fault)

        def publish_output(self,output):
            now=time.monotonic()
            cid=control_id(self.core.frame)
            if (optimized and output!=ZERO and not self.core.fault and not self.core.hard_fault
                    and cid==getattr(self,'last_emitted_cid',None)
                    and now-getattr(self,'last_emitted_at',-math.inf)<.008):
                # Avoid timer re-signing an identical already-committed frame
                # a few milliseconds later. NEW IDs and every ZERO/revocation
                # are always delivered; the independent timer STILL validates
                # and publishes its health report on every timer tick.
                return
            super().publish_output(output)
            self.last_emitted_cid=cid;self.last_emitted_at=now

        def on_candidate(self,msg):
            if optimized:
                self.process_pending()
                if len(self.pending) >= 4:
                    # At 100Hz, four queued commands can reach capacity just
                    # before the 40ms missing-source deadline. Do NOT enlarge
                    # the queue/deadline or skip its unknown head. Fail closed
                    # and retain the selected-source cause instead of masking
                    # it with the parent's generic envelope-overflow message.
                    value,frame,arrived = self.pending[0]
                    now,ros = self.clocks()
                    self.selection_failure = dict(reason='BOUND_CAPACITY_WAITING_FOR_SELECTED_SOURCE',
                        sequence=frame.sequence,sign_digest=value.get('sign_digest'),
                        sign_stamp=value.get('sign_stamp'),car_stamp=value.get('car_stamp'),
                        queued=len(self.pending),wait_s=now-arrived)
                    self.pending.clear()
                    self.core.trip('ROTATION_SELECTED_SOURCE_INVALID')
                    self.publish_output(ZERO)
                    return
            return super().on_candidate(msg)

        def process_pending(self):
            before = self.core.last_sequence
            super().process_pending()
            f = self.core.frame
            if (optimized and self.core.last_sequence > before and f and f.state == 'ACTIVE'
                    and not self.core.fault and not self.core.hard_fault):
                # The unchanged bounded FIFO already ran independent raw-source,
                # reference/accel/lease/clock validation for every drained frame.
                # Publish its final VALIDATED output, not the candidate directly.
                # ClockCheck.previous is still the original TIMER baseline.
                previous = self.core.clock.previous
                self.publish_output(self.core.output)
                assert self.core.clock.previous == previous
                log.emit('validated_event_commit',cid=control_id(f),timer_previous=previous)

    class Source(instrument.Source):
        def tick(self):
            poll=getattr(self.provider, 'poll', None)
            if poll:
                poll(lambda:(time.monotonic(), self.get_clock().now().nanoseconds*1e-9))
            # The original tick obtains its now/ROS pair only AFTER reception.
            # A freshly received packet can no longer appear to be from the
            # future solely because we sampled the tick clock before IPC.
            return super().tick()

        def create_timer(self,period,callback,*args,**kwargs):
            if optimized and period == .01 and getattr(callback,'__name__','') == 'tick':
                period = .005
            return super().create_timer(period,callback,*args,**kwargs)

        def create_publisher(self,msg_type,topic,*args,**kwargs):
            pub = super().create_publisher(msg_type,topic,*args,**kwargs)
            if optimized and topic in ('mode_claim','source_status'):
                previous=[-math.inf,None]
                def status(msg):
                    now=time.monotonic()
                    state=(tuple(sorted(json.loads(msg.data)['rejected'].items()))
                           if topic=='source_status' else msg.data)
                    # Fresh 20Hz housekeeping is sufficient for its original
                    # 200ms envelope. Changed mode/rejection counters publish
                    # immediately; raw source packets are NEVER throttled.
                    if state!=previous[1] or now-previous[0]>=.05:
                        pub.publish(msg);previous[:]=[now,state]
                return PublishTap(pub,status)
            if topic != 'rotation_packet':return pub
            def publish(msg):
                pub.publish(msg)
                log.emit('source_packet',sign=self.last_packet.digest,wire=msg.data)
            return PublishTap(pub,publish)

    class Chain(instrument.Chain):
        def __init__(self, provider, output=None):
            super().__init__(provider,output)
            if optimized:
                # These five are logging observers ONLY, not graph/lease/safety
                # subscriptions. Sidecar already records the exact data path.
                # Removing their duplicate JSON work leaves both source-binding
                # consumers and all independent watchdogs untouched.
                expected = {self.namespace+'/'+topic for topic in (
                    'candidate','rotation_packet','plant_feedback','supervisor_status','gate_status')}
                observed = list(self.operator.subscriptions)
                if {sub.topic_name for sub in observed} != expected:
                    raise ValueError('unexpected operator subscriptions; refuse observer removal')
                for subscription in observed:
                    self.operator.destroy_subscription(subscription)
                for node in (self.sup,self.gate):
                    # Both rotation adapters explicitly ignore legacy raw
                    # wand_pose callbacks; their validated input is the exact
                    # rotation_packet. Keep its publisher/graph identity, but
                    # remove only those two demonstrably no-op subscriptions.
                    ignored=[s for s in node.subscriptions if s.topic_name==self.namespace+'/wand_pose']
                    if len(ignored)!=1:raise ValueError('unexpected legacy wand subscription')
                    node.destroy_subscription(ignored[0])

        def ready(self):
            if not super().ready():return False
            # Seeing gate.core primed is not enough: the supervisor must have
            # consumed that SAME epoch's gate report before a fresh start edge.
            # Faster scheduling exposed this pre-existing readiness race.
            report = self.sup.core.gate_report
            return bool(report and report['epoch'] == self.sup.core.epoch and not report['fault']
                        and not report['hard_fault'])

        def snapshot(self):
            value = super().snapshot()
            goal, car = value['goal'],value['car']
            value.update(profile=profile.name,optimized_scheduler=optimized,
                error_m=math.hypot(goal['x']-car['x'],goal['y']-car['y']) if goal else None,
                speed_limit=profile.max_speed_mps,accel_limit=profile.max_accel_mps2,
                wheel_cap_rpm=profile.sim_max_rpm,
                reference_limited=self.sup.core.follower.reference_limited)
            return value

        def step(self):
            begin=time.monotonic_ns();cpu=time.thread_time_ns()
            super().step()
            end=time.monotonic_ns()
            if end-begin > 5_000_000:
                log.emit('slow_executor_step', begin_ns=begin, end_ns=end,
                         cpu_ns=time.thread_time_ns()-cpu)

    module.Supervisor,module.Gate,module.Source,module.Chain = Supervisor,Gate,Source,Chain
    return module
