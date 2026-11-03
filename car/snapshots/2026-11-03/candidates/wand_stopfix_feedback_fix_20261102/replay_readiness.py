"""Offline replay of the probe's extra progress/continuous-ready checks; no IO."""
import json
from pathlib import Path
import sys
from capture import M,unpack_wheel_telemetry,save

def main(folder):
    report=json.loads((folder/'RESULT.json').read_text());sid=report['clock']['session_id']
    events=[json.loads(x) for x in (folder/'native_clock.jsonl').read_text().splitlines()]
    events=[x for x in events if x['session']==sid and x['accepted_sample_steady'] is not None]
    outgoing=json.loads((folder/'outgoing.json').read_text());end=outgoing[-1]['at'];start=outgoing[0]['at']
    result={}
    for progress_limit in (.2,.3,1.):
        i=0;source={};last_sequence=None;progress_at=None;calibrated=False;stable=None;longest=0.;ready_time=0.;progress_only_fail_time=0.
        now=start
        while now<=end:
            while i<len(events) and events[i]['received_steady']<=now:
                row=events[i];i+=1;source[row['kind']]=row['accepted_sample_steady']
                calibrated|=row['startup_clock']['ready']
                if row['kind']=='WHEEL_TELEMETRY':
                    seq=unpack_wheel_telemetry(bytes.fromhex(row['payload_hex'])).last_command_sequence
                    if seq!=last_sequence:last_sequence=seq;progress_at=row['received_steady']
            fresh=calibrated and all(k in source and 0<=now-source[k]<=.3 for k in ('WHEEL_TELEMETRY','LINK_STATUS'))
            progress=progress_at is not None and 0<=now-progress_at<=progress_limit
            if fresh and not progress:progress_only_fail_time+=.003
            if fresh and progress:
                if stable is None:stable=now
                longest=max(longest,now-stable);ready_time+=.003
            else:stable=None
            now+=.003
        result[str(progress_limit)]=dict(max_continuous_ready_s=longest,ready_fraction=ready_time/max(.001,end-start),
            source_fresh_but_progress_failed_s=progress_only_fail_time,terminal_continuous_ready_s=0. if stable is None else end-stable,
            would_pass_terminal_600ms_requirement=bool(stable is not None and end-stable>=.6))
    value=dict(offline_only=True,original_feedback_bound_s=.3,current_probe_progress_bound_s=.2,
        actual_bridge_default_progress_stall_bound_s=1.,cadence='Actual100ms MCU publish and observed arrivals',
        replay=result,caveat='3ms-grid approximate replay; not an authorization or a proposal to relax motion protection. Source/receipt times are retained, not restamped.')
    save(folder/'READINESS_REPLAY.json',value);print(json.dumps(value,indent=2))
if __name__=='__main__':main(Path(sys.argv[1]).resolve())
