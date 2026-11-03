"""Read recorded real-Sign SOFTWARE sessions. Never controls a running node."""
import argparse
import json
import math
from pathlib import Path
from bootstrap import save


def rows(path):
    values=[]
    if not path.exists():return values
    for line in path.read_text().splitlines(keepends=True):
        if not line.endswith('\n'):break  # A live writer may have one partial tail.
        if line.strip():values.append(json.loads(line))
    return values


def analyze(folder):
    samples=rows(folder/'snapshots.jsonl');events=rows(folder/'events.jsonl')
    active=[s for s in samples if s['state']=='ACTIVE']
    transitions=[];last=None
    for s in samples:
        signature=(s['sessions'],s['state'],s['reason'],s['sign_valid'])
        if signature!=last:
            transitions.append(dict(steady=s['steady'],session=s['sessions'],state=s['state'],
                reason=s['reason'],sign_valid=s['sign_valid'],stop_confirmed=s['stop_confirmed']))
            last=signature
    grouped=[]
    for session in sorted({s['sessions'] for s in active}):
        data=[s for s in active if s['sessions']==session]
        errors=[math.hypot(s['goal']['x']-s['car']['x'],s['goal']['y']-s['car']['y']) for s in data if s['goal']]
        speeds=[math.hypot(s['output']['vx'],s['output']['vy']) for s in data]
        grouped.append(dict(session=session,active_duration_observed_s=data[-1]['steady']-data[0]['steady'],
            samples=len(data),peak_gate_speed_mps=max(speeds),
            same_time_model_position_rms_m=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            catchup_samples_max=max(s['catchup_samples'] for s in data),
            moving_feedforward_samples_max=max(s['feedforward_samples'] for s in data),
            x_positive=any(s['output']['vx']>.03 for s in data),
            x_negative=any(s['output']['vx']<-.03 for s in data),
            y_positive=any(s['output']['vy']>.03 for s in data),
            y_negative=any(s['output']['vy']<-.03 for s in data)))
    feedback=[e['value'] for e in events if e['topic']=='plant_feedback']
    report=dict(kind='REAL_SIGN_SOFTWARE_CAR_OBSERVATIONS',folder=str(folder),
        samples=len(samples),active_sessions=grouped,transitions=transitions,
        total_complete_raw_packets=max((s['reader']['complete_packets'] for s in samples),default=0),
        signed_output_faults=sorted({s['signature_fault'] for s in samples if s['signature_fault']}),
        hard_fault_observed=any(s['hard_fault'] for s in samples),
        any_sign_invalid_observed=any(not s['sign_valid'] for s in samples),
        model_feedback_messages=len(feedback),session_closed=(folder/'SESSION_RESULT.json').exists(),
        physical_car_started=False,physical_latency_measured=False,
        human_occlusion_and_manual_recovery_confirmation_required=True,
        passed=None,explanation='Observations only; no automatic claim that user completed every acceptance step')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--session',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();r=analyze(a.session);save(a.output,r)
    print(json.dumps(r,ensure_ascii=False,indent=2))
