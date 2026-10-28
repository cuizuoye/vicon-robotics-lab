"""Exact-reference FIFO processing. No fallback, skipped frame or recovery."""
import json
import math
import time
from mecanum_wand_4ws8_control.bound_history import HISTORY_QOS,RetainedPackets
from mecanum_wand_4ws8_control.rotation_core import RotationFinalGate
from mecanum_wand_4ws8_control.rotation_source import canonical,decode_packet
from mecanum_wand_4ws8_control.safety_nodes import decode_pose
from mecanum_wand_4ws8_control.safety_core import ZERO


def diagnostic_value(value):
    # Untrusted bound identifiers can contain JSON NaN/arrays. Diagnostics
    # must not throw before the gate has revoked a rejected command.
    if value is None or type(value) in (int,bool):return value
    if type(value) is str:return value[:160]
    if type(value) is float and math.isfinite(value):return value
    return repr(value)[:160]


class HistoryQosMixin:
    def create_publisher(self,msg_type,topic,qos,*args,**kwargs):
        return super().create_publisher(msg_type,topic,
            HISTORY_QOS if topic in ('car_pose','rotation_packet','candidate') else qos,*args,**kwargs)

    def create_subscription(self,msg_type,topic,callback,qos,*args,**kwargs):
        return super().create_subscription(msg_type,topic,callback,
            HISTORY_QOS if topic in ('car_pose','rotation_packet','candidate') else qos,*args,**kwargs)


class SourceFaultGate(RotationFinalGate):
    def trip(self,reason,hard=False):
        # Preserve the more specific FIRST source-rejection cause against a
        # later generic command timeout. Never mask a new hard/native fault.
        if self.fault=='ROTATION_SELECTED_SOURCE_INVALID' and not hard:
            reason=self.fault
        return super().trip(reason,hard)


class BoundGateMixin(HistoryQosMixin):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.core=SourceFaultGate(self.core.c);self.core.inputs.graph_ok=False
        self.packet_cache=RetainedPackets();self.selection_failure=None
        self.processing_bound=False;self.bound_drained=0

    def on_pose(self,source,msg):
        if source!='car':return
        now,ros=self.clocks()
        try:
            pose,stamp,roll,pitch=decode_pose(msg,self.world)
            if not -.005<=ros-stamp<=.20:raise ValueError('CAR_TIMESTAMP')
            old=self.cars.get(stamp)
            if old is not None:
                if (old[0],old[3],old[4])!=(pose,roll,pitch):raise ValueError('CAR_SAME_STAMP_CONFLICT')
                return  # Duplicates cannot refresh receipt time.
            latest=max(self.cars,default=float('-inf'))
            self.cars[stamp]=(pose,stamp,now,roll,pitch)
            for key,value in list(self.cars.items()):
                if not 0<=now-value[2]<=.20 or not -.005<=ros-key<=.20:del self.cars[key]
            while len(self.cars)>64:del self.cars[min(self.cars)]
            if stamp>latest:self.car_pose_quality.observe(pose,stamp,now,ros,roll,pitch)
            self.process_pending()
        except (ValueError,TypeError,AttributeError):
            self.core.inputs.poses.pop('car',None);self.core.inputs.event('CAR_DECODE_INVALID')
            self.car_pose_quality.reject('CAR_DECODE_INVALID',now)

    def on_packet(self,msg):
        now,ros=self.clocks()
        try:
            self.packet_cache.add(decode_packet(json.loads(msg.data),now,ros),now,ros)
            self.process_pending()
        except (ValueError,TypeError,KeyError,AttributeError):
            self.core.inputs.poses.pop('wand',None);self.core.inputs.event('ROTATION_RAW_SOURCE_INVALID')

    def on_candidate(self,msg):
        # Parent schema/owner/sequence validation, max4 queue and STOP priority
        # remain. Processing is also woken by source-arrival callbacks.
        super().on_candidate(msg)
        self.process_pending()

    def process_pending(self):
        if self.processing_bound:return
        self.processing_bound=True;started=time.monotonic();drained=0
        try:
            # Drain the COMPLETE FIFO prefix, never skip an unknown/tampered
            # head or the neutral/ARMED frame. Work is bounded by4 / 3ms.
            while self.pending and drained<4 and time.monotonic()-started<.003:
                v,f,arrived=self.pending[0];now,ros=self.clocks()
                missing_sign=missing_car=False
                try:
                    p=self.packet_cache.exact(v['sign_digest'],v['sign_stamp'],now,ros)
                    car=self.cars.get(v['car_stamp'])
                    missing_sign,missing_car=p is None,car is None
                    if missing_sign or missing_car:
                        if now-arrived>.040:raise ValueError('SELECTED_SOURCE_MISSING')
                        break
                    pose,stamp,received,roll,pitch=car
                    if not 0<=now-received<=.20 or not -.005<=ros-stamp<=.20:raise ValueError('SELECTED_CAR_STALE')
                    if f.issued_steady-p.received<0 or f.issued_ros-p.stamp<-.005:raise ValueError('SOURCE_AFTER_COMMAND')
                    self.core.inputs.observe_pose('car',pose,stamp,received,ros,roll,pitch)
                    sign=p.pose(now,ros)
                    self.core.inputs.observe_pose('wand',sign,p.stamp,p.received,ros)
                    stored=self.core.inputs.poses.get('car')
                    if not stored or stored['source_ros']!=min(stamp,ros) or stored['pose']!=pose:
                        raise ValueError('SELECTED_CAR_NOT_APPLIED')
                    stored=self.core.inputs.poses.get('wand')
                    if not stored or stored['source_ros']!=min(p.stamp,ros) or stored['pose']!=sign:
                        raise ValueError('SELECTED_POSE_NOT_APPLIED')
                    accepted=self.core.receive(f,now,ros);self.pending.popleft();drained+=1
                    # Validate EACH frame (including neutral first ACTIVE)
                    # before accepting the next. Intermediate commands never
                    # publish a physical permit/output; timer publishes latest.
                    # Validation must NOT renew the publication timer's clock
                    # baseline. Only the unchanged timer tick may advance it;
                    # otherwise busy source callbacks could hide a stalled
                    # output timer from the original 100ms timing protection.
                    timer_previous=self.core.clock.previous
                    try:self.core.tick(now,ros)
                    finally:self.core.clock.previous=timer_previous
                    if not accepted or self.core.fault or self.core.hard_fault:
                        self.pending.clear();self.publish_output(ZERO);break
                except (ValueError,TypeError,KeyError) as error:
                    first=self.selection_failure is None
                    if first:
                        detail=dict(reason=str(error),sequence=f.sequence,
                            sign_digest=v.get('sign_digest'),sign_stamp=v.get('sign_stamp'),
                            car_stamp=v.get('car_stamp'),missing_sign=missing_sign,missing_car=missing_car,
                            wait_s=now-arrived,queued=len(self.pending),issued_age_s=now-f.issued_steady)
                        self.selection_failure={k:diagnostic_value(v) for k,v in detail.items()}
                    self.pending.clear();self.core.trip('ROTATION_SELECTED_SOURCE_INVALID');self.publish_output(ZERO)
                    if first:self.get_logger().warn('ROTATION_SELECTED_SOURCE_REJECTED '+canonical(self.selection_failure))
                    break
        finally:
            self.bound_drained+=drained;self.processing_bound=False
