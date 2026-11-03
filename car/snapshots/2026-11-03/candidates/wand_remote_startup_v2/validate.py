#!/usr/bin/env python3
"""Full independent private validation; never instantiate a hardware bridge."""
import importlib.util
from prepare import ROOT,HERE,OLD,generate,write

spec=importlib.util.spec_from_file_location('remote_v2_base_validator',ROOT/'candidates/wand_feedback_300ms/validate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.generate=generate;base.HERE=HERE;base.OLD=OLD
original_run=base.subprocess.run


def run(command,**kwargs):
    command=[str(v).replace('/tools/start_car_tracking_300ms.sh','/tools/start_car_tracking_remote_v2.sh') for v in command]
    if any(v.endswith('/test_autostart.py') for v in command):
        command.insert(command.index('-k'),str(ROOT/'candidates/wand_remote_x11/test_remote_tk.py'))
        command.insert(command.index('-k'),str(HERE/'test_v2_tk.py'))
    return original_run(command,**kwargs)


def sealed_write(path,value):
    if path.name=='build_manifest.json' and value.get('feedback_300ms_passed') is True:value['remote_startup_v2_passed']=True
    if path.name=='PREPARATION_RESULT.json':
        value.update(actual_ros_software_cases=15,remote_display_acceptance_pending=True,
            input_freshness_ms=80,query_deadline_ms=60,reply_progress_deadline_ms=80,
            ui_gap_guard_ms=100,startup_exception='<=32 virgin stale STANDBY ZERO only, <=2s, before first fresh frame',
            fault_clearing_added=False,lease_worker_added=False,
            new_entry=str(ROOT/'tools/start_car_tracking_remote_v2.sh'),
            readonly_remote_check='bash '+str(ROOT/'tools/start_car_tracking_remote_v2.sh')+' --x11-check')
    write(path,value)


base.subprocess.run=run;base.write=sealed_write
if __name__=='__main__':base.main()
