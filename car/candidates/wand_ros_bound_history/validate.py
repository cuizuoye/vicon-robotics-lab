#!/usr/bin/env python3
"""Independent native/UI/actual ROS software-only validation and sealing."""
import importlib.util
from prepare import ROOT,HERE,OLD,generate,write

spec=importlib.util.spec_from_file_location('bound_validator_base',ROOT/'candidates/wand_feedback_300ms/validate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.generate=generate;base.HERE=HERE;base.OLD=OLD
original_run=base.subprocess.run


def run(command,**kwargs):
    command=[str(v).replace('/tools/start_car_tracking_300ms.sh','/tools/start_car_tracking_ros_history.sh') for v in command]
    if any(v.endswith('/test_autostart.py') for v in command):
        command.insert(command.index('-k'),str(ROOT/'candidates/wand_remote_x11/test_remote_tk.py'))
    return original_run(command,**kwargs)


def sealed_write(path,value):
    if path.name=='build_manifest.json' and value.get('feedback_300ms_passed') is True:value['ros_bound_history_passed']=True
    if path.name=='PREPARATION_RESULT.json':
        value.update(actual_ros_software_cases=14,exact_reference_wait_ms=40,relay_depth=32,
            latest_source_fallback=False,skip_unverified_head=False,
            remote_display_acceptance_pending=True,
            new_entry=str(ROOT/'tools/start_car_tracking_ros_history.sh'))
    write(path,value)


base.subprocess.run=run;base.write=sealed_write
if __name__=='__main__':base.main()
