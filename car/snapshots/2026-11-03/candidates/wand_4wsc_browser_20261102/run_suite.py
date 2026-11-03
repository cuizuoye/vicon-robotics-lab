"""Serial loopback-only acceptance; preserve each failure, never retry automatically."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from prepare import HERE, save


def main(ready):
    output=Path(tempfile.mkdtemp(prefix='suite_',dir=ready.parent))
    records=[]
    def run(name,script,arguments):
        command=[sys.executable,'-B',str(HERE/script),'--ready',str(ready),*arguments]
        p=subprocess.run(command,text=True,capture_output=True,timeout=240)
        (output/(name+'.stdout.log')).write_text(p.stdout)
        (output/(name+'.stderr.log')).write_text(p.stderr)
        record=dict(name=name,command=command,exit_code=p.returncode,stdout=str(output/(name+'.stdout.log')),
            stderr=str(output/(name+'.stderr.log')))
        folders=[]
        for line in p.stdout.splitlines():
            if line.startswith('{'):
                r=json.loads(line)
                if 'folder' in r:folders.append(str(Path(r['folder'])/'RESULT.json'))
            elif line.startswith('BROWSER_PROTOCOL_REPORT '):
                folders.append(line.split(' ',1)[1])
        record['results']=folders;records.append(record)
        save(output/'INDEX.json',dict(prepared=str(ready),records=records,complete=False))
        print(json.dumps(record),flush=True)
        if p.returncode or not folders:raise SystemExit('failure preserved in '+str(output))
        return folders
    trials=[]
    trials+=run('reopen','trial.py',['--stage','0','--seconds','6','--reopen'])
    trials+=run('high_stage','trial.py',['--stage','9','--seconds','8'])
    for case in ('occlusion','feedback','loop_gap'):
        trials+=run(case,'trial.py',['--stage','0','--case',case])
    probe=run('zero_probe','other_trials.py',['--kind','probe'])[0]
    suspended=run('five_seconds','other_trials.py',['--kind','suspended'])[0]
    early=run('release_early','other_trials.py',['--kind','suspended','--release-early'])[0]
    browser=run('browser','browser_cases.py',['--normal-seconds','60'])[0]
    save(output/'INDEX.json',dict(prepared=str(ready),records=records,complete=True,trials=trials,
        probe=probe,suspended=suspended,early_release=early,browser=browser))
    print('COMPLETE_SUITE '+str(output/'INDEX.json'),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ready',type=Path,required=True)
    main(p.parse_args().ready.resolve())
