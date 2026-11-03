"""Real Sign -> private software car READY handshake only; NEVER START."""
import argparse
from pathlib import Path
import time
from bootstrap import save
from app import Worker


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    w=Worker(a.output);samples=[];began=time.monotonic();requested=False;ready_since=None;reason=''
    try:
        while time.monotonic()-began<15.:
            now=time.monotonic();w.ui_stamp=now;s=w.snapshot
            if 'error' in s:raise RuntimeError(s['error'])
            if s:
                samples.append(s.copy())
                if any(abs(v)>1e-12 for v in s['output'].values()) or s['state']=='ACTIVE':
                    raise AssertionError('zero-only preparation probe must NEVER start motion')
                if not requested and s['can_prepare']:
                    w.post('prepare');requested=True
                if s['ready'] and not s['prepare_pending']:
                    if ready_since is None:ready_since=now
                    if now-ready_since>.3:break
                else:ready_since=None
                reason=s.get('prepare_blocker','')
            time.sleep(.02)
        passed=ready_since is not None
        result=dict(passed=passed,requested_prepare=requested,ready_held_s=(time.monotonic()-ready_since) if passed else 0.,
            sample_count=len(samples),last=samples[-1] if samples else {},
            real_sign=True,actual_private_ros=True,all_outputs_zero=True,
            start_action_sent=False,active_ever_observed=False,hardware_bridge_constructed=False,
            physical_car_started=False,reason_if_not_ready=reason if not passed else '')
        save(a.output/'PREPARE_RESULT.json',result);print(result,flush=True)
        if not passed:raise RuntimeError('No READY without motion: '+reason)
    finally:
        w.done.set();w.thread.join(timeout=4.)
