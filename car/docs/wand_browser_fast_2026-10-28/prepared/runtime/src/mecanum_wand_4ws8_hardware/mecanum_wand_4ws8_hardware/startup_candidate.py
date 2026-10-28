"""Discard ONLY bounded stale virgin STANDBY-zero history, never a motion frame.

No fault clearing, deadline relaxation, skipped ARMED/neutral ACTIVE frame, or
startup exception after the first fresh frame / any arm or native fault.
"""
import json
import time
from mecanum_wand_4ws8_control.rotation_source import canonical
from mecanum_wand_4ws8_control.safety_core import ZERO,fresh_envelope
from mecanum_wand_4ws8_control.safety_nodes import decode_frame
from .bound_rotation import diagnostic_value


class StartupCandidateMixin:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.startup_frame_window=time.monotonic();self.startup_fresh_frame=False
        self.startup_stale_zero_dropped=0;self.startup_envelope_error_logged=False

    def stale_virgin_zero(self,f,now,ros):
        if not hasattr(self,'startup_frame_window'):return False
        core=self.core;old=core.frame
        ds,dr=now-f.issued_steady,ros-f.issued_ros
        return bool(not self.startup_fresh_frame and 0<=now-self.startup_frame_window<=2.
            and self.startup_stale_zero_dropped<32 and not self.pending
            and f.state=='STANDBY' and not f.enabled and not f.epoch and f.command==ZERO
            and f.goal is None and not f.hard_fault
            and not core.hard_fault and core.fault in ('','COMMAND_TIMEOUT')
            and not core.primed_epoch and not core.blocked_epoch and not core.pilot.used
            and not core.angle_epoch and (core.owner is None or core.owner==f.source)
            and (old is None or (old.state=='STANDBY' and not old.enabled and not old.epoch and old.command==ZERO))
            and ds>core.c.command_timeout_s and dr>core.c.command_timeout_s
            and abs(ds-dr)<=core.c.max_clock_skew_s)

    def on_candidate(self,msg):
        # Preserve the parent's full decoder/fault path for malformed messages.
        if not hasattr(self,'startup_frame_window'):return super().on_candidate(msg)
        f=None;now,ros=self.clocks()
        try:
            value=json.loads(msg.data)
            if set(value)=={'frame','sign_digest','sign_stamp','car_stamp'}:
                f=decode_frame(canonical(value['frame']))
        except (ValueError,TypeError,KeyError,AttributeError):pass
        if f is not None:
            if self.stale_virgin_zero(f,now,ros):
                self.startup_stale_zero_dropped+=1
                self.publish_output(ZERO)
                if self.startup_stale_zero_dropped==1:
                    self.get_logger().warn('STARTUP_STALE_ZERO_DISCARDED '+canonical(dict(
                        sequence=f.sequence,steady_age_s=now-f.issued_steady,ros_age_s=ros-f.issued_ros,
                        reason='expired virgin STANDBY ZERO only; no source/owner/epoch/permit accepted')))
                return
            if (f.sequence>self.core.last_sequence and not self.startup_envelope_error_logged
                    and not fresh_envelope(f.issued_steady,f.issued_ros,now,ros,
                        self.core.c.command_timeout_s,self.core.c.future_tolerance_s)):
                self.startup_envelope_error_logged=True
                detail=dict(sequence=f.sequence,source=f.source,state=f.state,enabled=f.enabled,
                    epoch=f.epoch,steady_age_s=now-f.issued_steady,ros_age_s=ros-f.issued_ros,
                    startup_dropped=self.startup_stale_zero_dropped,startup_fresh_seen=self.startup_fresh_frame)
                self.get_logger().warn('CANDIDATE_ENVELOPE_REJECT_DIAGNOSTIC '+canonical(
                    {k:diagnostic_value(v) for k,v in detail.items()}))
        super().on_candidate(msg)
        if (f is not None and self.core.last_sequence==f.sequence
                and fresh_envelope(f.issued_steady,f.issued_ros,now,ros,
                    self.core.c.command_timeout_s,self.core.c.future_tolerance_s)):
            self.startup_fresh_frame=True
