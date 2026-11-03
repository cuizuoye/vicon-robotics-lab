"""Same direct ROS controller and gate; only independent low-tier trial logging."""
import json
import math
import time
from pathlib import Path
from hardware_runtime import FollowRig
from measurements import analyze,read_rows
import bootstrap


def Rig(channel,session,synthetic=False,occlusion=False):
    rt=bootstrap.runtime()
    if rt.purpose!='sign' or rt.config.stage!=0 or synthetic!=rt.software_only:
        raise ValueError('stage0 ground Sign contract only; synthetic cannot address hardware')
    return GroundRig(rt,channel,session,occlusion)


class GroundRig(FollowRig):
    def __init__(self,rt,channel,session,occlusion=False,**kw):
        self.pose_path=Path(session)/'POSE_OBSERVATIONS.jsonl'
        self.pose_log=self.pose_path.open('x');self.pose_last=-math.inf;self.pose_flush=0.
        self.stop_request=None;self.cached_close=None
        super().__init__(rt,channel,session,occlusion,**kw)
        contract=Path(session)/'CONTRACT.json'
        v=json.loads(contract.read_text());v.update(trial_kind=rt.profile['trial_kind'],max_active_s=60.,
            physical_qualified=False,prior_measurements='UNKNOWN; explicitly commissioning, not a qualified long demo')
        contract.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')

    def before_operator_step(self,now):
        # Called after draining status and BEFORE the model sees a gate-timeout
        # withdrawal. Preserve the actual 60s cause rather than relabelling the
        # ensuing logical lease release as an operator key release. Never mask
        # an earlier operator/source/link/loop fault.
        if self.closing or self.channel.stopped:return
        started=self.status.get('pilot',{}).get('capture_started_steady')
        if type(started) in (int,float) and math.isfinite(started) and now-started>=60.:
            self.channel.stop('GROUND60_TIME_COMPLETE')

    def step(self,now):
        super().step(now)
        now=time.monotonic();o=self.operator
        if self.stop_request is None and (self.channel.stopped or self.status.get('state') in ('STOPPING','STOP_UNCONFIRMED','PAUSED','FAULT')) and self.activation_ever:
            self.stop_request=now
        if o is not None and o.car is not None and o.car_stamp>self.pose_last:
            ros=o.get_clock().now().nanoseconds*1e-9
            row=dict(at=now,source_ros=o.car_stamp,source_age_s=ros-o.car_stamp,
                x=o.car.x,y=o.car.y,z=o.car.z,yaw=o.car.yaw,state=self.status.get('state'),
                goal=self.status.get('goal'),gate_speed_mps=self.gate.get('speed_mps'),
                native=self.transport.get('commissioning_observation'),physical_output=self.physical)
            self.pose_log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');self.pose_last=o.car_stamp
        if now-self.pose_flush>=.1:self.pose_log.flush();self.pose_flush=now

    def close(self):
        if self.cached_close is not None:return self.cached_close
        if self.stop_request is None:self.stop_request=time.monotonic()
        result=super().close()
        measured=analyze(read_rows(self.pose_path),self.stop_request,result['stop_confirmed'],self.physical)
        (self.session/'MEASUREMENTS.json').write_text(json.dumps(measured,ensure_ascii=False,indent=2)+'\n')
        self.cached_close=dict(result,measurements=str(self.session/'MEASUREMENTS.json'),physical_qualified=False)
        return self.cached_close

    def dispose(self):
        try:super().dispose()
        finally:
            if not self.pose_log.closed:self.pose_log.close()
