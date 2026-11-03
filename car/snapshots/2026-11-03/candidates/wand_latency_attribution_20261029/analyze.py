"""Recorded REAL Sign -> SOFTWARE car attribution, never a live controller.

Preserves original clocks/epochs/digests, reports gaps, and distinguishes
measured point-to-point ages, curve-equivalent lags and model constants.
"""
import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(v):
    a=np.asarray(v,dtype=float)
    if not len(a):return None
    if not np.isfinite(a).all():raise ValueError('nonfinite metric')
    return dict(n=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))


def load_lines(path):
    with Path(path).open() as f:
        for line in f:
            if not line.endswith('\n'):raise ValueError('open/partial session; close before attribution')
            if line.strip():yield json.loads(line)


def lag_search(t,a,b,derivative_ms=100,maximum_ms=1500,grid_s=.01):
    """Equal differencing in both signals; a(t) vs b(t+lag), no cherry shift."""
    a=np.asarray(a);b=np.asarray(b)
    if a.shape!=b.shape or len(a)!=len(t) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('matched finite trajectories required')
    if a.ndim==1:a=a[:,None];b=b[:,None]
    window=round(derivative_ms/1000/grid_s);lag_count=round(maximum_ms/1000/grid_s)
    va=(a[window:]-a[:-window])/(window*grid_s)
    vb=(b[window:]-b[:-window])/(window*grid_s)
    n=len(va)-lag_count
    if n<300:return dict(qualified=False,reason='insufficient_duration')
    aa=va[:n].reshape(-1)
    if np.std(aa)<.005:return dict(qualified=False,reason='insufficient_excitation')
    scores=[]
    for lag in range(lag_count+1):
        bb=vb[lag:lag+n].reshape(-1)
        scores.append(float(np.corrcoef(aa,bb)[0,1]) if np.std(bb)>1e-9 else -1.)
    k=int(np.argmax(scores));r=scores[k]
    return dict(qualified=r>=.9 and k not in (0,lag_count),best_lag_ms=k*grid_s*1000,
        correlation=r,derivative_window_ms=derivative_ms,grid_ms=grid_s*1000,
        search_max_ms=maximum_ms,meaning='trajectory-equivalent lag, not motion-onset latency')


def common_blocks(streams,start,end,max_gap=.10):
    if not math.isfinite(start+end) or start>=end:raise ValueError('invalid common analysis interval')
    gaps=[]
    for stamps in streams:
        for lo,hi in zip(stamps,stamps[1:]):
            if hi-lo>max_gap and hi>start and lo<end:gaps.append((max(start,lo),min(end,hi)))
    bounds=sorted({start,end,*[v for pair in gaps for v in pair]})
    return [(a,b) for a,b in zip(bounds,bounds[1:]) if b-a>=5 and not any(x<b and y>a for x,y in gaps)],gaps


def analyze(folder):
    folder=Path(folder).resolve()
    if not (folder/'SESSION_RESULT.json').exists():raise ValueError('closed session evidence required')
    packets={};candidates={};statuses=[];gates=[];feedback=[];packet_obs={}
    for e in load_lines(folder/'events.jsonl'):
        v=e['value'];topic=e['topic']
        if topic=='rotation_packet':
            raw_digest=hashlib.sha256(json.dumps(v['samples'],sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
            if raw_digest!=v['digest']:raise ValueError('raw packet digest mismatch')
            old=packets.get(v['digest'])
            if old is not None and old!=v:raise ValueError('same digest changed')
            packets[v['digest']]=v;packet_obs.setdefault(v['digest'],e['received'])
        elif topic=='candidate' and v['frame']['state']=='ACTIVE':
            f=v['frame'];key=(f['source'],f['sequence'])
            if key in candidates:raise ValueError('duplicate candidate sequence')
            candidates[key]=(e['received'],v)
        elif topic=='supervisor_status' and v['state']=='ACTIVE':statuses.append(v)
        elif topic=='gate_status':gates.append(v)
        elif topic=='plant_feedback':feedback.append(v)
    snapshots=list(load_lines(folder/'snapshots.jsonl'))
    if not statuses:raise ValueError('no ACTIVE episode')
    epochs=sorted({v['epoch'] for v in statuses})
    reports=[];series=[]
    for epoch in epochs:
        ss=sorted([s for s in statuses if s['epoch']==epoch],key=lambda s:s['issued_steady'])
        cc=sorted([(received,c) for received,c in candidates.values() if c['frame']['epoch']==epoch],key=lambda x:x[1]['frame']['issued_steady'])
        by_tick={c['frame']['issued_steady']:(received,c) for received,c in cc}
        if len(by_tick)!=len(ss) or any(s['issued_steady'] not in by_tick for s in ss):
            raise ValueError('exact status/selected candidate join incomplete')
        start,end=ss[0]['issued_steady'],ss[-1]['issued_steady']
        first_used=set();rows=[];a_age=[];b_age=[];total_age=[];skew=[];offsets=[];observer=[];selected=[]
        history=[];history_last=None;window_spans=[];velocity_centroid_ages=[]
        for s in ss:
            received,c=by_tick[s['issued_steady']];f=c['frame'];d=c['sign_digest']
            p=packets.get(d)
            if p is None:raise ValueError('selected packet missing; never fallback to nearest')
            stamp=c['sign_stamp'];samples=p['samples']
            if any(v['stamp']!=stamp for v in samples):raise ValueError('mixed raw frame')
            offset=f['issued_ros']-f['issued_steady'];offsets.append(offset)
            if d not in first_used:
                first_used.add(d);lo=min(v['received'] for v in samples);hi=max(v['received'] for v in samples)
                stage_a=(hi-(stamp-offset))*1000;stage_b=(f['issued_steady']-hi)*1000
                if min(stage_a,stage_b)<-.1:raise ValueError('clock/causality mismatch')
                a_age.append(stage_a);b_age.append(stage_b);total_age.append((f['issued_ros']-stamp)*1000)
                skew.append((hi-lo)*1000)
                selected.append(dict(sequence=f['sequence'],stamp=stamp,clock_offset=offset,
                    source_complete_steady=hi,control_issued_steady=f['issued_steady'],
                    publish_to_complete_ms=stage_a,complete_to_control_ms=stage_b,
                    publish_to_control_ms=(f['issued_ros']-stamp)*1000))
            observer.append((received-f['issued_steady'])*1000)
            goal=s['mapped_goal'];reference=s['goal'];command=f['command']
            if stamp!=history_last:
                if history and stamp-history[-1]>.060:history.clear()
                history.append(stamp)
                while len(history)>2 and history[0]<stamp-.040-1e-9:history.pop(0)
                while len(history)>16:history.pop(0)
                history_last=stamp
                if len(history)>=2 and history[-1]-history[0]>=.005:
                    window_spans.append((history[-1]-history[0])*1000)
                    velocity_centroid_ages.append((f['issued_ros']-history[0]-np.mean(np.array(history)-history[0]))*1000)
            rows.append(dict(t=f['issued_steady']-start,issued=f['issued_steady'],
                source_stamp=stamp,source_steady=stamp-offset,goal=[goal[k] for k in ('x','y','yaw')],
                reference=[reference[k] for k in ('x','y','yaw')],
                command=[command[k] for k in ('vx','vy','wz')],reference_limited=s['reference_limited'],
                sign_digest=d))
        car=[s for s in snapshots if start<=s['steady']<=end]
        if len(car)<5:raise ValueError('software car snapshots missing')
        ct=np.array([s['steady'] for s in car]);cp=np.array([[s['car'][k] for k in ('x','y','yaw')] for s in car])
        cp[:,2]=np.unwrap(cp[:,2])
        # Deduplicate source times using the FIRST selected target per stamp.
        goals={}
        for r in rows:goals.setdefault(r['source_stamp'],(r['source_steady'],r['goal']))
        unique=sorted(goals.values(),key=lambda value:value[0])
        gt=np.array([v[0] for v in unique]);gp=np.array([v[1] for v in unique])
        gp[:,2]=np.unwrap(gp[:,2])
        rt=np.array([r['issued'] for r in rows]);rp=np.array([r['reference'] for r in rows]);rp[:,2]=np.unwrap(rp[:,2])
        blocks,gaps=common_blocks((gt,ct,rt),max(start+2,gt[0],ct[0]),min(end-2,gt[-1],ct[-1]))
        curve=[];all_hand_speed=[];all_hand_yaw=[];all_error=[];all_yaw_error=[]
        for lo,hi in blocks:
            t=np.arange(lo,hi,.01)
            goal=np.column_stack([np.interp(t,gt,gp[:,k]) for k in range(3)])
            body=np.column_stack([np.interp(t,ct,cp[:,k]) for k in range(3)])
            ref=np.column_stack([np.interp(t,rt,rp[:,k]) for k in range(3)])
            v=(goal[10:,:2]-goal[:-10,:2])/.1;hand_speed=np.linalg.norm(v,axis=1)
            hand_w=(goal[10:,2]-goal[:-10,2])/.1
            error=np.linalg.norm(goal[:,:2]-body[:,:2],axis=1);yaw_error=goal[:,2]-body[:,2]
            all_hand_speed.extend(hand_speed);all_hand_yaw.extend(abs(hand_w));all_error.extend(error);all_yaw_error.extend(abs(yaw_error))
            block=dict(start_s=lo-start,duration_s=hi-lo,
                xy_lag=[lag_search(t,goal[:,:2],body[:,:2],w) for w in (50,100,200)],
                yaw_lag=[lag_search(t,goal[:,2],body[:,2],w,maximum_ms=5000) for w in (50,100,200)],
                reference_to_car_xy_lag=[lag_search(t,ref[:,:2],body[:,:2],w) for w in (50,100,200)],
                same_time_error_m=stats(error),same_time_yaw_error_rad=stats(abs(yaw_error)))
            # Ten-second, 50%-overlap windows expose nonstationarity. These
            # cannot be interpreted as independent confidence intervals.
            block['local_windows']=[]
            for base in np.arange(lo,hi-10.+1e-6,5.):
                choose=(t>=base)&(t<base+10.)
                block['local_windows'].append(dict(start_s=base-start,duration_s=10.,
                    xy=lag_search(t[choose],goal[choose,:2],body[choose,:2]),
                    yaw=lag_search(t[choose],goal[choose,2],body[choose,2],maximum_ms=5000)))
            curve.append(block)
            series.append(dict(epoch=epoch,t=(t-start).tolist(),goal=goal.tolist(),body=body.tolist(),reference=ref.tolist()))
        commands=np.array([r['command'] for r in rows]);dt=np.diff(rt)
        if np.any(dt<=0):raise ValueError('control time not strictly increasing')
        speed=np.linalg.norm(commands[:,:2],axis=1)
        accel=np.linalg.norm(np.diff(commands[:,:2],axis=0),axis=1)/dt
        yaw_accel=abs(np.diff(commands[:,2]))/dt
        def percent(mask,weights):return float(np.average(np.asarray(mask,dtype=float),weights=weights)*100)
        report=dict(epoch=epoch,active_s=end-start,control_samples=len(rows),unique_selected_packets=len(first_used),
            paired_point_to_point_ms=dict(pc_stamp_to_raw_complete=stats(a_age),raw_complete_to_control_tick=stats(b_age),
                pc_stamp_to_control_tick=stats(total_age),six_raw_callback_spread=stats(skew),
                control_tick_to_log_observer=stats(observer)),
            clock_offset_span_us=(max(offsets)-min(offsets))*1e6,
            clock_caveat='ROS->steady conversion uses each control tick clock pair; capture time unknown',
            control_interval_ms=stats(dt*1000),
            gate_interval_ms=stats(np.diff([g['issued_steady'] for g in gates if start<=g['issued_steady']<=end])*1000),
            feedback_interval_ms=stats(np.diff([f['issued_steady'] for f in feedback if start<=f['issued_steady']<=end])*1000),
            snapshot_interval_ms=stats(np.diff(ct)*1000),
            reconstructed_estimator=dict(source_window_span_ms=stats(window_spans),
                velocity_fit_centroid_age_at_tick_ms=stats(velocity_centroid_ages),
                meaning='source-history reconstruction; centroid age is NOT a measured pure delay'),
            bound_activity_percent=dict(xy_speed_at_070=percent(speed[1:]>=.7*.98,dt),
                xy_acceleration_at_080=percent(accel>=.8*.98,dt),
                yaw_speed_at_010=percent(abs(commands[1:,2])>=.1*.98,dt),
                yaw_accel_at_040=percent(yaw_accel>=.4*.98,dt),
                reference_lead_hold=percent([r['reference_limited'] for r in rows[1:]],dt)),
            hand_speed_100ms_mps=stats(all_hand_speed),hand_yaw_rate_100ms_radps=stats(all_hand_yaw),
            hand_speed_over_070_percent=float(np.mean(np.array(all_hand_speed)>.7)*100) if all_hand_speed else None,
            hand_yaw_over_010_percent=float(np.mean(np.array(all_hand_yaw)>.1)*100) if all_hand_yaw else None,
            command_speed_mps=stats(speed),command_accel_mps2=stats(accel),
            command_yaw_rate_radps=stats(abs(commands[:,2])),
            same_time_error_m=stats(all_error),same_time_yaw_error_rad=stats(all_yaw_error),
            curves=curve,excluded_gaps_after_start_s=[[a-start,b-start] for a,b in gaps],
            missing_downstream_timestamps=['actual candidate DDS publish','gate receive and selected validation',
                'signed actuator publish and software plant decode','GUI render/present'],
            selected_pair_rows=selected)
        reports.append(report)
    result=dict(kind='REAL_SIGN_TO_SOFTWARE_CAR_LATENCY_ATTRIBUTION',source=str(folder),
        source_hashes={name:sha(folder/name) for name in ('events.jsonl','snapshots.jsonl','SESSION_RESULT.json','CONTRACT.json')},
        reports=reports,session_end=json.loads((folder/'SESSION_RESULT.json').read_text())['reason'],
        exposure_to_wheel_measured=False,real_car_latency_measured=False,
        model_only=dict(first_order_tau_s=.12,velocity_gains=[.98,.88,.95],
            theoretical_10_90_rise_ms=1000*.12*math.log(9),
            not_identified_from_hardware=True),
        caution='Curve lag is not an additive pure delay. Percentile sums are not total percentiles.')
    return result,series


def plot(out,series):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for j,s in enumerate(series):
        t=np.array(s['t']);g=np.array(s['goal']);c=np.array(s['body']);r=np.array(s['reference'])
        fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True,constrained_layout=True)
        for k in range(2):
            axes[k].plot(t,g[:,k],label='Sign mapped target',lw=1.3)
            axes[k].plot(t,r[:,k],label='Governed reference',lw=1.1,alpha=.8)
            axes[k].plot(t,c[:,k],label='Software car',lw=1.3)
            axes[k].set_ylabel(('X','Y')[k]+' (m)');axes[k].grid(alpha=.2)
        axes[0].legend(ncol=3)
        axes[2].plot(t,np.rad2deg(g[:,2]),label='Sign target yaw')
        axes[2].plot(t,np.rad2deg(r[:,2]),label='Rate-limited reference')
        axes[2].plot(t,np.rad2deg(c[:,2]),label='Software car yaw')
        axes[2].set_ylabel('Yaw (deg)');axes[2].legend(ncol=3);axes[2].grid(alpha=.2)
        axes[3].plot(t,np.linalg.norm(g[:,:2]-c[:,:2],axis=1),label='Same-time XY error')
        axes[3].set_ylabel('Error (m)');axes[3].set_xlabel('Seconds after ACTIVE');axes[3].grid(alpha=.2)
        fig.suptitle('Recorded real Sign -> software car; no time shift, no physical latency claim')
        fig.savefig(out/f'trajectories_{j}.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--session',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    result,series=analyze(a.session)
    (a.output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (a.output/'series.json').write_text(json.dumps(series)+'\n');plot(a.output,series)
    compact=json.loads(json.dumps(result))
    for r in compact['reports']:r.pop('selected_pair_rows')
    print(json.dumps(compact,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
