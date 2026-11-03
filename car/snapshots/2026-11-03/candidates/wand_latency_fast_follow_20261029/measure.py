"""Read-only offline timing analysis; no ROS/SDK/network/device startup.

Two DIFFERENT quantities: recorded trajectory-equivalent lag, and manually
annotated common-camera physical onset brackets. Never label the first as a
camera-exposure-to-wheel timing measurement. No interpolation across source gaps.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from common import digest,fresh_output,save


def rows(path):
    tail=False
    with Path(path).open() as stream:
        for line in stream:
            if not line.strip('\x00\r\n\t '):tail=True;continue
            if tail:raise ValueError('corrupt interior record')
            r=json.loads(line);v=r['data']
            if isinstance(v,str):v=json.loads(v)
            yield r['topic'],v


def stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    if not np.all(np.isfinite(a)):raise ValueError('nonfinite timing values')
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(np.max(a)))


def velocity_alignment(t,goal,car,window_samples=10):
    """Matched derivative on BOTH signals; common analysis filter, no control."""
    vg=(goal[window_samples:]-goal[:-window_samples])/(.01*window_samples)
    vc=(car[window_samples:]-car[:-window_samples])/(.01*window_samples)
    if len(vg)<450:return dict(qualified=False,reason='insufficient_duration')
    if np.max(np.linalg.norm(vg,axis=1))<.02:return dict(qualified=False,reason='insufficient_excitation')
    n=len(vg)-150
    a=vg[:n].reshape(-1)
    scores=[]
    for lag in range(151):
        b=vc[lag:n+lag].reshape(-1)
        scores.append(float(np.corrcoef(a,b)[0,1]) if np.std(a)>1e-8 and np.std(b)>1e-8 else -1.)
    index=int(np.argmax(scores));score=scores[index]
    return dict(qualified=score>=.90 and index not in (0,150),best_lag_ms=index*10,
        correlation=score,derivative_window_ms=window_samples*10,
        search_range_ms=[0,1500],grid_resolution_ms=10,
        meaning='trajectory-equivalent lag, NOT first wheel motion or exposure latency')


def analyze_log(path,seconds=60.):
    if not math.isfinite(seconds) or seconds<=0:raise ValueError('positive finite analysis window required')
    candidates={};statuses=[];cars={};act=[];feedback=[]
    for topic,d in rows(path):
        if topic=='candidate' and d['frame']['state']=='ACTIVE':candidates[d['frame']['issued_steady']]=d
        elif topic=='supervisor_status' and d['state']=='ACTIVE':statuses.append(d)
        elif topic=='car_pose':cars[d['stamp']]=[d['x'],d['y']]
        elif topic=='actuator_command':act.append(d['payload'])
        elif topic=='transport_status':feedback.append(d)
    if not statuses or not candidates:raise ValueError('no ACTIVE episode')
    if len({s['epoch'] for s in statuses})!=1:raise ValueError('split epochs before analysis')
    start=min(s['issued_ros'] for s in statuses);end=min(start+seconds,max(s['issued_ros'] for s in statuses))
    chosen=[c for c in candidates.values() if start<=c['frame']['issued_ros']<=end]
    seen=set();age=[];goals={}
    for c in chosen:
        if c['sign_stamp'] in seen:continue
        seen.add(c['sign_stamp']);value=(c['frame']['issued_ros']-c['sign_stamp'])*1000
        if value<-.005:raise ValueError('source after command')
        age.append(value)
    for s in statuses:
        if not start<=s['issued_ros']<=end:continue
        c=candidates.get(s['issued_steady'])
        if c is None:raise ValueError('exact candidate/status join missing')
        if s.get('mapped_goal') is not None:
            goals[c['sign_stamp']]=[s['mapped_goal'][k] for k in ('x','y')]
    gt=np.array(sorted(goals));gp=np.array([goals[t] for t in gt])
    ct=np.array(sorted(cars));cp=np.array([cars[t] for t in ct])
    if len(gt)<2 or len(ct)<2:raise ValueError('missing source/vehicle trajectory')
    t0=max(start+2.,gt[0],ct[0]);t1=min(end-2.,gt[-1],ct[-1])
    # Split on >100ms gaps in EITHER stream, never interpolate over loss.
    bad=[]
    for stamps in (gt,ct):
        for lo,hi in zip(stamps,stamps[1:]):
            if hi-lo>.10 and hi>t0 and lo<t1:bad.append((max(t0,lo),min(t1,hi)))
    boundaries=sorted({t0,t1,*[v for pair in bad for v in pair]})
    blocks=[]
    for lo,hi in zip(boundaries,boundaries[1:]):
        if hi-lo<5. or any(a<hi and b>lo for a,b in bad):continue
        t=np.arange(lo,hi,.01)
        goal=np.column_stack([np.interp(t,gt,gp[:,i]) for i in range(2)])
        car=np.column_stack([np.interp(t,ct,cp[:,i]) for i in range(2)])
        err=np.linalg.norm(goal-car,axis=1)
        velocity=np.linalg.norm((goal[10:]-goal[:-10])/.1,axis=1)
        blocks.append(dict(start_after_active_s=lo-start,duration_s=hi-lo,
            alignment=[velocity_alignment(t,goal,car,w) for w in (5,10,20)],
            same_time_error_m=stats(err),hand_speed_mps=stats(velocity),
            hand_samples_over_070_percent=float(np.mean(velocity>.7)*100)))
    return dict(kind='RECORDED_TRAJECTORY_ANALYSIS',source=str(Path(path).resolve()),sha256=digest(path),
        active_window_s=end-start,pc_publication_to_first_candidate_ms=stats(age),
        control_intervals_ms=stats(np.diff([c['frame']['issued_steady'] for c in chosen])*1000),
        blocks=blocks,excluded_gaps_after_active_s=[[a-start,b-start] for a,b in bad],
        feedback_age_ms={k:stats([d['feedback_ages_s'][k]*1000 for d in feedback
            if start<=d['issued_ros']<=end and d.get('feedback_ages_s',{}).get(k) is not None])
            for k in ('wheel','diagnostic')},
        physical_onset_measured=False,exposure_time_known=False,
        caveats=['Local ROS publication stamp is NOT camera exposure time',
                 'Feedback age includes telemetry period; NOT forward motion latency',
                 'Do not convert low-correlation/boundary optimum into a fixed delay guarantee'])


def video_intervals(annotations,metadata):
    fps=metadata['capture_fps']
    if type(fps) not in (int,float) or not math.isfinite(fps) or not 24<=fps<=1000:
        raise ValueError('verified actual capture FPS required, not slow-motion playback FPS')
    for key in ('same_camera','constant_frame_spacing','no_missing_frames','capture_fps_verified',
                'initial_standstill_verified'):
        if metadata.get(key) is not True:raise ValueError('unverified video timing: '+key)
    if not metadata.get('onset_definition'):raise ValueError('visible movement criterion must be documented')
    result=[]
    for row in annotations:
        event=row['event']
        if event not in ('wheel','body'):raise ValueError('event must identify wheel or body onset')
        a,b,c,d=(int(row[k]) for k in ('sign_last_still','sign_first_moving','output_last_still','output_first_moving'))
        if min(a,b,c,d)<0 or b<=a or d<=c or d<a:raise ValueError('invalid/noncausal onset brackets')
        lower=(c-b)/fps;upper=(d-a)/fps
        result.append(dict(trial_id=row['trial_id'],event=event,
            delay_lower_ms=lower*1000,delay_upper_ms=upper*1000,
            interval_midpoint_ms=(lower+upper)*500,interval_width_ms=(upper-lower)*1000,
            includes_zero=lower<=0<=upper,
            low_frame_rate_warning=(fps<120),
            measurement='manually annotated common-camera visible onset interval',
            camera_exposure_to_motor_electrical_onset_measured=False))
    if not result:raise ValueError('empty annotations are a template, not a measurement')
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    logs=sub.add_parser('logs');logs.add_argument('--input',nargs='+',required=True);logs.add_argument('--seconds',type=float,default=60.)
    video=sub.add_parser('video');video.add_argument('--video',required=True);video.add_argument('--metadata',required=True);video.add_argument('--annotations',required=True)
    for p in (logs,video):p.add_argument('--output',required=True)
    a=ap.parse_args();out=fresh_output(a.output)
    if a.mode=='logs':result=[analyze_log(p,a.seconds) for p in a.input]
    else:
        meta=json.loads(Path(a.metadata).read_text());v=Path(a.video)
        if not v.is_file() or digest(v)!=meta.get('video_sha256'):raise ValueError('video file/hash missing or mismatched')
        with Path(a.annotations).open() as f:result=video_intervals(list(csv.DictReader(f)),meta)
        save(out/'input_metadata.json',meta)
    save(out/'results.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
