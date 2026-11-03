"""Independent large-area random-yaw ROS+C acceptance batch, no device IO."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from common import HERE,save,protected


def run(out):
    if os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise ValueError('isolated software domain required')
    out.mkdir(parents=True,exist_ok=False);before=protected();results=[]
    cases=[dict(name=f'stage{s}_{kind}',stage=s,speed=speed,pure=False)
           for kind,speed in (('large_nominal',.55),('large_fast_hand',1.)) for s in (6,7)]
    cases += [dict(name=f'stage{s}_pure_from_rest',stage=s,speed=None,pure=True) for s in (6,7)]
    try:
        for case in cases:
            save(out/'PROGRESS.json',case);print(json.dumps(case),flush=True)
            command=[sys.executable,str(HERE/'random_rotation.py'),'--output',str(out/case['name']),
                '--stage',str(case['stage']),'--seed','20261030','--preview']
            if case['pure']:command.append('--pure-rotation')
            else:command.extend(['--large-area','--hand-speed',str(case['speed'])])
            with (out/(case['name']+'.log')).open('x') as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            path=out/case['name']/'ROTATION_RESULT.json'
            detail=json.loads(path.read_text()) if path.exists() else dict(execution_error=True)
            results.append(dict(case=case,returncode=result.returncode,result=detail))
            save(out/'RESULT.json',dict(complete=False,cases=results,hardware_output=False))
            print(json.dumps(results[-1]),flush=True)
    finally:
        save(out/'RESULT.json',dict(complete=len(results)==len(cases),cases=results,
            protected_unchanged=before==protected(),hardware_output=False,firmware_flashed=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args().output)
