"""Disabled probe and suspended browser logic, exact C + loopback only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')

def main(ready,kind,release_early=False):
    from entry import setup_rehearsal,FLASH_RECORD
    folder=Path(tempfile.mkdtemp(prefix=kind+'_',dir=ready.parent))
    path,consent=setup_rehearsal(0,folder,'suspended')
    p=json.loads(path.read_text())
    os.environ.update(WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),
        WAND_PROFILE_APPROVAL=hashlib.sha256(path.read_bytes()).hexdigest(),WAND_HARDWARE_CONSENT=consent,
        ROS_DOMAIN_ID='192',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    sys.path.insert(0,str(ready));import bootstrap
    if kind=='probe':
        from native_peer import Peer
        from idle_probe import run,CONSENT
        peer=Peer(p['hardware']['command_port'])
        try:
            r=run(p,FLASH_RECORD,folder/'probe',CONSENT,3.,rehearsal=True)
            assert r['passed'] and not peer.permit_ever and peer.peak_duty==0
            r['exact_C_never_permitted']=True
        finally:peer.close()
    else:
        from control_channel import Channel
        from suspended_runtime import SuspendedRig
        channel=Channel(readonly=True);rt=bootstrap.runtime();rig=SuspendedRig(rt,channel,folder)
        start=time.monotonic();channel.open('suspended-offline-peer',start);seq=0;unlocked=0;down=False;pressed=False;at=None
        try:
            while time.monotonic()-start<15:
                now=time.monotonic();channel.begin_tick(now);events=[]
                if unlocked==0 and now-start>.10:
                    unlocked=1;down=True;events=[dict(kind='press')]
                elif unlocked==1 and now-start>.16:
                    unlocked=2;down=False;events=[dict(kind='release')]
                elif rig.core.state=='READY' and not pressed:
                    pressed=True;down=True;events=[dict(kind='press')];at=now
                elif release_early and pressed and down and now-at>1.:
                    down=False;events=[dict(kind='release')]
                challenge=channel.challenge(now)
                if challenge:
                    seq+=1;data=dict(type='pulse',nonce=challenge['nonce'],sequence=seq,focused=True,visible=True,space=down,ui_gap_ms=10.,events=events)
                    if channel.receive(channel.owner,data,now):channel.events.extend(events)
                channel.finish_tick(now);rig.step(now)
                if rig.core.state in ('COMPLETE','FAULT'):break
                time.sleep(.003)
            stop=rig.close()
            r=dict(passed=bool(rig.activation_ever and stop['stop_confirmed']),release_early=release_early,
                active_ever=rig.activation_ever,stop=stop,elapsed_s=time.monotonic()-start,
                peak_duty=rig.peer.peak_duty,physical_output=False)
        except BaseException:
            rig.close();raise
    (folder/'RESULT.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(r,folder=str(folder)),ensure_ascii=False));assert r['passed']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ready',required=True,type=Path);p.add_argument('--kind',choices=('probe','suspended'),required=True)
    p.add_argument('--release-early',action='store_true');a=p.parse_args();main(a.ready.resolve(),a.kind,a.release_early)
