"""Actual ROS + actual exclusive UDP bridge + loopback exact-C offline trial."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')

def run(ready,stage,seconds,case,peer_oracle=None,keep_peer=False):
    folder=Path(tempfile.mkdtemp(prefix=f'ros_c_stage{stage}_{case}_',dir=ready.parent))
    from entry import setup_rehearsal
    profile,consent=setup_rehearsal(stage,folder)
    if peer_oracle is not None:
        p=json.loads(profile.read_text());p['hardware']['command_port']=peer_oracle.port;profile.write_text(json.dumps(p,indent=2)+'\n')
    os.environ.update(WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(profile),
        WAND_PROFILE_APPROVAL=hashlib.sha256(profile.read_bytes()).hexdigest(),WAND_HARDWARE_CONSENT=consent,
        ROS_DOMAIN_ID='192',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    sys.path.insert(0,str(ready));import bootstrap
    from control_channel import Channel
    from hardware_runtime import FollowRig
    channel=Channel(readonly=True);rig=None;result={};stop=None;error=None
    try:
        rig=FollowRig(bootstrap.runtime(),channel,folder,peer_oracle=peer_oracle,keep_peer=keep_peer)
        channel.open('offline-protocol-peer',time.monotonic())
        seq=0;pressed=False;released=False;down=False;press_at=None;injected=None;max_active=0.
        started=time.monotonic();revoked_seen=False;resumed_after_revocation=False
        while time.monotonic()-started<seconds+30:
            now=time.monotonic();channel.begin_tick(now)
            events=[]
            if not pressed and rig.operator.prepare.phase=='READY':
                events=[dict(kind='press')];pressed=True;down=True;press_at=now
            elif pressed and not released and now-press_at>.04:
                events=[dict(kind='release')];released=True;down=False
            challenge=channel.challenge(now)
            if challenge:
                seq+=1;data=dict(type='pulse',nonce=challenge['nonce'],sequence=seq,
                    focused=True,visible=True,space=down,ui_gap_ms=10.,events=events)
                if channel.receive(channel.owner,data,now):channel.events.extend(events)
            channel.finish_tick(now);rig.step(now)
            active=0. if rig.active_since is None else now-rig.active_since
            max_active=max(max_active,active)
            native=rig.peer.sample()['status']
            if injected is not None and channel.stopped and native:
                if not native['permitted']:revoked_seen=True
                elif revoked_seen:resumed_after_revocation=True
            if rig.active_since is not None:
                if case!='normal' and active>=2. and injected is None:
                    injected=now
                    if case=='occlusion':rig.occlude=True
                    elif case=='feedback':rig.peer.feedback=False
                    elif case=='loop_gap':time.sleep(.15)
                if injected is not None and now-injected>=.8:
                    rig.occlude=False;rig.peer.feedback=True
                if case=='normal' and active>=seconds:
                    channel.stop('OFFLINE_OPERATOR_STOP');break
                if injected is not None and now-injected>=2.:break
            if channel.stopped and injected is None:break
            time.sleep(.003)
        result.update(active_ever=rig.activation_ever,active_elapsed_s=max_active,
            stop_reason=channel.fault,stage=stage,case=case,physical_output=False,
            ros_initialized=True,actual_exclusive_bridge_used=True,exact_C_peer_used=True,
            native_revocation_seen=revoked_seen,resumed_after_revocation=resumed_after_revocation)
    except BaseException as e:
        error=repr(e);result.update(passed=False,error=error,stage=stage,case=case,physical_output=False)
    finally:
        if rig is not None:
            try:stop=rig.close()
            except BaseException as e:
                result['close_error']=repr(e);rig.dispose()
        result['stop']=stop
        confirmation_ok=bool(stop and (stop['stop_confirmed'] or (case=='feedback' and stop['state']=='STOP_UNCONFIRMED')))
        result['feedback_loss_stop_confirmation_unavailable_expected']=bool(case=='feedback' and confirmation_ok and not stop['stop_confirmed'])
        result['passed']=bool(not error and result.get('active_ever') and stop and confirmation_ok
            and stop['gate_output_zero'] and ((case=='normal' and result['active_elapsed_s']>=seconds)
                or (case!='normal' and result['stop_reason'] and result['stop_reason']!='OFFLINE_OPERATOR_STOP'
                    and result.get('native_revocation_seen') and not result.get('resumed_after_revocation'))))
        (folder/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(result,folder=str(folder)),ensure_ascii=False),flush=True)
    if not result['passed']:raise SystemExit(1)
    return rig.peer

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ready',type=Path,required=True);p.add_argument('--stage',type=int,default=0)
    p.add_argument('--seconds',type=float,default=8.);p.add_argument('--case',choices=('normal','occlusion','feedback','loop_gap'),default='normal')
    p.add_argument('--reopen',action='store_true')
    a=p.parse_args();oracle=run(a.ready.resolve(),a.stage,a.seconds,a.case,keep_peer=a.reopen)
    if a.reopen:run(a.ready.resolve(),a.stage,a.seconds,a.case,peer_oracle=oracle)
