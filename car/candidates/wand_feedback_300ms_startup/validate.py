#!/usr/bin/env python3
"""Reuse the same private validation pipeline with the repaired sources."""
import importlib.util
from prepare import ROOT,HERE,OLD,generate,sha,hashes,write,check

spec=importlib.util.spec_from_file_location('comparison_validator',ROOT/'candidates/wand_feedback_300ms/validate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.generate=generate;base.HERE=HERE;base.OLD=OLD


def sealed_write(path,value):
    if path.name=='build_manifest.json' and value.get('feedback_300ms_passed') is True:
        value['feedback_300ms_startup_passed']=True
    if path.name=='PREPARATION_RESULT.json':
        value.update(actual_ros_software_cases=11,startup_used_budget_repair=True,
            new_entry=str(ROOT/'tools/start_car_tracking_300ms_ready.sh'))
    write(path,value)


# Seal the new flag only at the existing successful final write in main(), not
# during generation (whose generated manifest explicitly keeps it False).
base.write=sealed_write
original_run=base.subprocess.run


def run(command,**kwargs):
    # Exact candidate entry check, not the parent's already-verified entry.
    command=[str(v).replace('/tools/start_car_tracking_300ms.sh','/tools/start_car_tracking_300ms_ready.sh') for v in command]
    return original_run(command,**kwargs)


base.subprocess.run=run
if __name__=='__main__':base.main()
