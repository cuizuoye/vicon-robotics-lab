"""Seeded random XY+yaw gestures through sealed production ROS + C, no motors.

Imports/reuses the accepted split-process harness; never edits its source,
limits or guards. Deliberately applies the SAME gestures to stages6 and7.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import time
from common import save,protected,HERE,diagnostic as d


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


@dataclass(frozen=True)
class Segment:
    start:float
    duration:float
    a:float
    b:float
    kind:str


class Gesture:
    def __init__(self,seed,combined=True):
        self.seed,self.combined=seed,combined;self.segments=[];self.last=None;self.origin=None
        rng=random.Random(seed);position=0.;clock=0.
        targets=[rng.choice((-1.,1.))*rng.uniform(.55,2.0) for _ in range(8)]
        # Include both directions, return to zero, and a continuous crossing
        # of the quaternion +/-pi boundary (raw Sign starts at +0.3rad).
        targets += [0.,3.3,2.7,0.]
        for index,target in enumerate(targets):
            duration=max(.7,1.875*abs(target-position)/1.2,
                         math.sqrt(5.774*abs(target-position)/3.5))
            self.segments.append(Segment(clock,duration,position,target,'move'))
            clock+=duration
            hold=.8 if index>=8 else rng.uniform(.65,1.0)
            self.segments.append(Segment(clock,hold,target,target,'hold'));clock+=hold
            position=target
        self.end=clock
        self.xy_until=self.segments[16].start

    def xy(self,t):
        if not self.combined:return 0.,0.
        # End the small translation smoothly, then test PURE yaw as well.
        s=max(0.,min(1.,self.xy_until-t))
        envelope=s*s*(3.-2.*s)
        return envelope*.08*math.sin(1.1*t),envelope*.06*math.sin(.8*t)

    def sample(self,t):
        t=max(0.,t)
        for segment in self.segments:
            if t<=segment.start+segment.duration:
                s=max(0.,min(1.,(t-segment.start)/segment.duration))
                q=s*s*s*(10.+s*(-15.+6.*s))
                dq=30*s*s*(1-s)*(1-s)/segment.duration
                return dict(yaw=segment.a+(segment.b-segment.a)*q,
                    yaw_rate=(segment.b-segment.a)*dq,kind=segment.kind,
                    phase_s=t-segment.start,segment_end_s=segment.start+segment.duration)
        return dict(yaw=0.,yaw_rate=0.,kind='final_hold',phase_s=t-self.end,segment_end_s=self.end)

    def __call__(self,t,stage,recipe):
        now=time.monotonic();value=self.sample(t)
        if t>0 and self.origin is None:self.origin=now-t
        x,y=self.xy(t);self.last=dict(value,t=t,steady=now,x=x,y=y)
        # Small smooth simultaneous translation, zero at clutch capture.
        return x,y,value['yaw']

    @property
    def duration(self):return self.end+2.


class LargeGesture(Gesture):
    """Real-scale 6m x 5m route, diagonal return, and repeated random yaw."""
    def __init__(self,seed,hand_speed=.55):
        if not math.isfinite(hand_speed) or not .1<=hand_speed<=1.5:raise ValueError('explicit bounded hand speed required')
        super().__init__(seed,combined=True)
        self.hand_speed=hand_speed;self.route=[];clock=1.;previous=(0.,0.)
        for target in ((6.,0.),(6.,5.),(0.,5.),(0.,0.),(4.2,4.2),(0.,0.)):
            distance=math.dist(previous,target)
            duration=max(1.,1.875*distance/hand_speed,math.sqrt(5.774*distance/2.))
            self.route.append(dict(start=clock,duration=duration,a=previous,b=target))
            clock+=duration+1.;previous=target
        self.path_end=clock

    def xy(self,t):
        previous=(0.,0.)
        for segment in self.route:
            if t<segment['start']:return previous
            if t<=segment['start']+segment['duration']:
                s=max(0.,min(1.,(t-segment['start'])/segment['duration']))
                q=s*s*s*(10.+s*(-15.+6.*s))
                return tuple(a+(b-a)*q for a,b in zip(segment['a'],segment['b']))
            previous=segment['b']
        return previous

    def sample(self,t):
        value=super().sample(max(0.,t)%self.end)
        return dict(value,cycle=int(max(0.,t)//self.end))

    @property
    def duration(self):return self.path_end+2.


def analyze(path,gesture):
    import numpy as np
    rows=[]
    with path.open() as file:
        for line in file:
            row=json.loads(line)
            if row.get('state')=='ACTIVE' and row.get('reference'):
                rows.append(row)
    if not rows:raise ValueError('no genuine ACTIVE trace samples')
    t=np.array([r['reference']['t'] for r in rows]);target=np.array([r['reference']['yaw'] for r in rows])
    actual=np.array([r['car'][2] for r in rows]);rate=np.array([r['reference']['yaw_rate'] for r in rows])
    # Both reference and C-integrated car yaw are continuous/unwrapped.
    # Do not hide an unintended extra 360-degree revolution with wrapping.
    errors=target-actual
    dynamic=(t>1.) & (abs(rate)>.15)
    held=np.array([r['reference']['kind']!='move' and r['reference']['phase_s']>.4 for r in rows])
    candidates=[]
    for lag in np.arange(0.,.801,.005):
        delayed=np.interp(t-lag,t,target)
        err=delayed-actual
        candidates.append((float(np.sqrt(np.mean(err[dynamic]**2))),float(lag)))
    rmse,lag=min(candidates)
    deg=180/math.pi;absolute=np.abs(errors)*deg
    p95=float(np.percentile(absolute,95));maximum=float(absolute.max())
    hold_p95=float(np.percentile(np.abs(errors[held])*deg,95)) if held.any() else None
    # Explicit PROVISIONAL demo acceptance; not an inferred hardware spec.
    criteria=dict(angle_p95_deg=10.,angle_max_deg=20.,lag_s=.15,held_p95_deg=3.)
    passed=bool(dynamic.any() and held.any() and p95<=10. and maximum<=20. and lag<=.15 and hold_p95<=3.)
    spatial={}
    if all('x' in r['reference'] and 'y' in r['reference'] for r in rows):
        reference=np.array([[r['reference']['x'],r['reference']['y']] for r in rows])
        car=np.array([r['car'][:2] for r in rows]);xy=np.linalg.norm(reference-car,axis=1)
        spatial=dict(xy_error_p95_m=float(np.percentile(xy,95)),xy_error_max_m=float(xy.max()),
            reference_span_xy_m=np.ptp(reference,axis=0).tolist(),actual_span_xy_m=np.ptp(car,axis=0).tolist())
    return dict(synchronization_passed=passed,criteria=criteria,criteria_status='provisional engineering demo criteria',
        samples=len(rows),angle_p95_deg=p95,angle_max_deg=maximum,
        held_p95_deg=hold_p95,estimated_follow_lag_s=lag,lag_aligned_rmse_deg=rmse*deg,
        reference_angle_span_deg=float(np.ptp(target))*deg,
        actual_angle_span_deg=float(np.ptp(actual))*deg,
        peak_reference_rate_radps=float(np.max(abs(rate))),seed=gesture.seed,combined=gesture.combined,
        software_plant_only=True,physical_latency_measured=False,**spatial)


def run(out,stage,seed,combined=True,large_area=False,hand_speed=.55,preview=False):
    import split_rehearsal as harness
    from trace_spool import TraceSpool
    gesture=LargeGesture(seed,hand_speed) if large_area else Gesture(seed,combined);before=protected()
    duration=gesture.duration
    viewer=None;viewer_log=None
    old_gesture,old_spool=harness.hand_gesture,harness.TraceSpool
    class GroundTruthSpool(TraceSpool):
        last_preview=0.
        def emit(self,row):
            value=dict(row,reference=gesture.last.copy() if gesture.last else None)
            if preview and row['t']-self.last_preview>.04:
                pending=out/'LATEST_PREVIEW.pending.json'
                save(pending,value);pending.replace(out/'LATEST_PREVIEW.json');self.last_preview=row['t']
            return super().emit(value)
    try:
        if preview:
            # Harness creates this directory; retain its exist_ok=False guard.
            viewer_log=out.parent/(out.name+'_preview.log')
            viewer_log.parent.mkdir(parents=True,exist_ok=True)
            viewer_log=viewer_log.open('x')
            viewer=subprocess.Popen([sys.executable,str(HERE/'trace_preview.py'),'--path',str(out/'LATEST_PREVIEW.json'),
                '--stage',str(stage)],stdout=viewer_log,stderr=viewer_log)
        harness.hand_gesture=gesture;harness.TraceSpool=GroundTruthSpool
        result=harness.run(out,stage,duration)
        save(out/'GESTURE.json',dict(seed=seed,combined=combined,duration_s=duration,
            segments=[s.__dict__ for s in gesture.segments],route=getattr(gesture,'route',None),
            hand_speed=hand_speed if large_area else None,hardware_output=False))
        metrics=analyze(out/'normal_stop_TRACE.jsonl',gesture)
        save(out/'ROTATION_RESULT.json',dict(stability_passed=result['passed'],**metrics,
            stage=stage,protected_unchanged=before==protected(),hardware_output=False))
        print(json.dumps(dict(stage=stage,**metrics)),flush=True)
        return metrics
    finally:
        if viewer is not None:d.terminate_owned(viewer)
        if viewer_log is not None:viewer_log.close()
        harness.hand_gesture,harness.TraceSpool=old_gesture,old_spool


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',type=int,choices=(6,7),required=True);p.add_argument('--seed',type=int,default=20261030)
    p.add_argument('--pure-rotation',action='store_true')
    p.add_argument('--large-area',action='store_true');p.add_argument('--hand-speed',type=float,default=.55)
    p.add_argument('--preview',action='store_true')
    a=p.parse_args();run(a.output,a.stage,a.seed,not a.pure_rotation,a.large_area,a.hand_speed,a.preview)
