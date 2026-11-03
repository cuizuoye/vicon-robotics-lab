#!/usr/bin/env python3
"""Plot recorded ROS simulation paths and progress-aware geometric errors."""
import argparse
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))
from mecanum_vicon_control.trajectory_metrics import tracking_metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir', required=True)
    p.add_argument('--output', required=True)
    a=p.parse_args()
    root=Path(a.input_dir)
    fig, axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    summaries=[]
    labels={'rectangle':'Rectangle', 'circle':'Circle (16 waypoint edges)',
            'figure_eight':'Figure eight (16 waypoint edges)'}
    for col, name in enumerate(labels):
        result=json.loads((root/f'{name}.json').read_text())
        if not result.get('success'):
            raise ValueError(f'{name} did not pass live ROS verification')
        trajectory=result['trajectory']; loops=trajectory['completed_loops']
        rows=result['samples']; poses=rows['poses']
        n=(len(trajectory['goals'])-1)//loops+1
        plan=trajectory['goals'][:n]
        ax=axes[0,col]
        ax.plot([r[1] for r in poses],[r[2] for r in poses],color='#16833b',lw=1.8,label='Simulated path')
        ax.plot([v[0] for v in plan],[v[1] for v in plan],color='#7e46b4',ls='--',lw=1.4,
                marker='o',ms=3,label='Waypoint reference')
        ax.scatter([poses[0][1]],[poses[0][2]],marker='s',s=30,color='#222222',label='Start')
        ax.set_title(f'{labels[name]} | {loops} loops')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal',adjustable='box')
        ax.grid(alpha=.25); ax.margins(.1)
        if col==0: ax.legend(fontsize=8,loc='upper left')
        metric,_=tracking_metrics(poses,plan,rows['active_segment_errors'])
        times=[r[0]-poses[0][0] for r in poses]
        active=[v*1000 for v in rows['active_segment_errors']]
        ax=axes[1,col]
        ax.plot(times,active,lw=.9,color='#bd4a33',label='Current-segment error')
        ax.axhline(metric['active_segment']['rmse_time_weighted_m']*1000,color='#333333',ls='--',lw=1,
                   label='Time-weighted RMSE')
        ax.set_xlabel('Elapsed trajectory time (s)'); ax.set_ylabel('Geometric error (mm)')
        ax.set_title(f'RMSE {metric["active_segment"]["rmse_time_weighted_m"]*1000:.1f} mm | '
                     f'P95 {metric["active_segment"]["p95_m"]*1000:.1f} mm')
        ax.set_ylim(bottom=0); ax.grid(alpha=.25)
        if col==0: ax.legend(fontsize=8)
        summaries.append({'shape':name,'loops':loops,
            'duration_s':trajectory['duration_s'], 'lap_reference_length_m':metric['reference_length_m'],
            'total_reference_length_m':metric['reference_length_m']*loops,
            'travelled_length_m':metric['travelled_length_m'],
            'closure_error_m':trajectory['closure_error_m'],
            'active_segment':metric['active_segment'], 'nearest_polyline':metric['nearest_polyline'],
            'loop_reports':trajectory['loop_reports'],
            'effective_pose_sample_hz':metric['effective_sample_hz'],
            'pose_sample_gap_max_s':metric['sample_interval_max_s'],
            'lost_pose_test':result['lost_pose_test'], 'ros_domain_id':result['ros_domain_id']})
    max_error=max(max(json.loads((root/f'{name}.json').read_text())['samples']['active_segment_errors'])
                  for name in labels)*1000
    for ax in axes[1]: ax.set_ylim(0,max_error*1.1)
    fig.suptitle('ROS2 simulation only: waypoint following and geometric deviation',fontsize=14)
    fig.savefig(a.output,dpi=180)
    plt.close(fig)
    summary={'simulation_only':True, 'hardware_commands_sent':False,
             'metric_caveat':'Polyline geometric error, not continuous-curve or time-aligned tracking accuracy',
             'cases':summaries}
    (root/'suite_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
