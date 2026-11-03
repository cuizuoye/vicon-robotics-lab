"""Real Sign + actual ROS software-chain startup, DISABLED ZERO ONLY."""
import argparse
from pathlib import Path
import time
from bootstrap import save
from app import Worker


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);w=Worker(a.output);samples=[];began=time.monotonic()
    try:
        while time.monotonic()-began<10.:
            w.ui_stamp=time.monotonic();s=w.snapshot
            if 'error' in s:raise RuntimeError(s['error'])
            if s:
                samples.append(s.copy())
                if any(abs(v)>1e-12 for v in s['output'].values()):raise AssertionError('disabled output was nonzero')
                if s['state'] in ('ARMED','ACTIVE'):raise AssertionError('unexpected preparation/enable')
            time.sleep(.02)
        result=dict(passed=bool(samples) and max(s['accepted_signed'] for s in samples)>100
            and sum(s['sign_valid'] for s in samples)>50
            and samples[-1]['reader']['complete_packets']>=10,
            sign_valid_samples=sum(s['sign_valid'] for s in samples),
            accepted_signed_max=max((s['accepted_signed'] for s in samples),default=0),
            no_prepare_no_enable=True,all_output_zero=True,last=samples[-1] if samples else {},
            actual_real_sign=True,physical_car_started=False)
        save(a.output/'STARTUP_RESULT.json',result);print(result,flush=True)
        if not result['passed']:raise AssertionError('no actual ROS output observed')
    finally:w.done.set();w.thread.join(timeout=4.)
