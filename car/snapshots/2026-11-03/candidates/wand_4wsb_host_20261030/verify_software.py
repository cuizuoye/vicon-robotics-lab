"""Serial, fail-fast software qualification. No hardware/firmware/network IO.

Synthetic Sign -> production ROS nodes -> signed MC-Link -> actual C oracle.
All graphical actions occur only inside the hidden private Xephyr display.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from bootstrap import OUT
from prepare import HERE,save,hashes,sha


def run():
    lock=(OUT/'SOFTWARE_TEST.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=json.loads((OUT/'HOST_MANIFEST.json').read_text())
    assert hashes(OUT/'runtime/src')==manifest['runtime_hashes']
    assert all(sha(p)==v for p,v in manifest['protected'].items())
    env=dict(os.environ,ROS_DOMAIN_ID='191',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        PYTHONDONTWRITEBYTECODE='1',WAND_4WSB_HOST=str(OUT))
    jobs=[('stage_matrix','complete.py',['--prepared',str(OUT),'--stages'],160.),
        ('faults','ros_rehearsal.py',['--output',str(OUT/'ros_faults_final'),'--duration','2'],210.)]
    # Predeclared phases, never retry a failure until it passes. Different stop
    # phases exercise coast/encoder quantization at different actual velocities.
    for stage,duration in ((3,1.7),(3,3.1),(3,7.3),(7,1.7),(7,3.1),(7,7.3)):
        name='stop_phase_%d_%s'%(stage,str(duration).replace('.','_'))
        jobs.append((name,'split_rehearsal.py',['--output',str(OUT/name),'--stage',str(stage),
            '--case','normal_stop','--duration',str(duration)],duration+24.))
    jobs.append(('rotation_demo_60s','split_rehearsal.py',['--output',str(OUT/'rotation_demo_60s'),
        '--stage','6','--case','normal_stop','--duration','60','--gesture','demo'],86.))
    jobs.append(('ui_600s_actual_pixels','private_display.py',['--output',str(OUT/'ui_600s_actual_pixels'),
        '--duration','600'],1270.))
    result=dict(passed=False,hardware_output=False,firmware_flashed=False,
        network_changed=False,execution='serial single instance, no parallel compilation',jobs=[])
    try:
        for name,script,args,timeout in jobs:
            save(OUT/'SOFTWARE_PROGRESS.json',dict(current=name,completed=len(result['jobs']),total=len(jobs),
                hardware_output=False,started=time.monotonic()))
            began=time.monotonic()
            with (OUT/(name+'_verification.log')).open('x') as stream:
                task=subprocess.run([sys.executable,str(HERE/script),*args],env=env,
                    stdout=stream,stderr=subprocess.STDOUT,timeout=timeout)
            item=dict(name=name,passed=task.returncode==0,elapsed_s=time.monotonic()-began,returncode=task.returncode)
            result['jobs'].append(item);save(OUT/'SOFTWARE_VERIFICATION.json',result)
            print(json.dumps(item),flush=True)
            if task.returncode:raise ValueError('software qualification failed at '+name)
        result['passed']=True
    except Exception as error:
        result['error']=repr(error)
        raise
    finally:
        save(OUT/'SOFTWARE_VERIFICATION.json',result)
        fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
    return result


if __name__=='__main__':
    argparse.ArgumentParser(description=__doc__).parse_args()
    run()
