"""Two independent600s software displays under the SAME concurrent load."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from prepare import save


def run(output,duration):
    output.mkdir(parents=True,exist_ok=False)
    jobs=[];logs=[]
    try:
        for mode in ('normal','full_geometry'):
            log=(output/(mode+'.log')).open('x');logs.append(log)
            folder=output/(mode+'_run')
            p=subprocess.Popen([sys.executable,str(Path(__file__).with_name('private_display.py')),
                '--output',str(folder),'--duration',str(duration),'--mode',mode],
                stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
            jobs.append((mode,folder,p))
        started=time.monotonic()
        while any(p.poll() is None for _,_,p in jobs):
            if time.monotonic()-started>duration+30.:raise TimeoutError('parallel display driver expired')
            time.sleep(.2)
        results=[]
        for mode,folder,p in jobs:
            record=json.loads((folder/'RESULT.json').read_text())
            if p.returncode or not record['passed']:raise ValueError('nested display failed: '+mode)
            results.extend(record['runs'])
        result=dict(passed=True,runs=results,requested_s=duration,concurrent_software_chains=2,
            comparison='concurrent equal ambient load; stability stress, NOT native driver causality',
            hardware_output=False,native_host_maximize_or_fullscreen_qualified=False,
            original_os_freeze_root_cause_known=False)
        save(output/'RESULT.json',result)
        return result
    finally:
        for _,_,p in jobs:
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=3.)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2.)
        for log in logs:log.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=600.)
    a=p.parse_args();run(a.output.resolve(),a.duration)
