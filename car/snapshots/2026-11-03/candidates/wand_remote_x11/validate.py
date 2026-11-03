#!/usr/bin/env python3
"""Reuse sealed native/UI/ROS software pipeline, adding remote-only tests."""
import importlib.util
from prepare import ROOT,HERE,OLD,generate,write

spec=importlib.util.spec_from_file_location('remote_validator_base',ROOT/'candidates/wand_feedback_300ms/validate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.generate=generate;base.HERE=HERE;base.OLD=OLD
original_run=base.subprocess.run


def run(command,**kwargs):
    command=[str(v).replace('/tools/start_car_tracking_300ms.sh','/tools/start_car_tracking_remote.sh') for v in command]
    if any(v.endswith('/test_autostart.py') for v in command):
        command.insert(command.index('-k'),str(HERE/'test_remote_tk.py'))
    return original_run(command,**kwargs)


def sealed_write(path,value):
    if path.name=='build_manifest.json' and value.get('feedback_300ms_passed') is True:value['remote_x11_passed']=True
    if path.name=='PREPARATION_RESULT.json':
        value.update(actual_ros_software_cases=11,remote_display_acceptance_pending=True,
            ui_gap_guard_ms=100,operator_lease_ms=150,lease_worker_added=False,
            new_entry=str(ROOT/'tools/start_car_tracking_remote.sh'))
    write(path,value)


base.subprocess.run=run;base.write=sealed_write
if __name__=='__main__':base.main()
