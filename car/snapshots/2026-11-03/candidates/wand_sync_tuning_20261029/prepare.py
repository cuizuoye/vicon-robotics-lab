"""Pin SIM-only tuning entry to complete model/ROS/UI evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from model import ROOT,save,protected_hashes
from dashboard import hashes,REPORT


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',required=True,type=Path)
    parser.add_argument('--ros',required=True,type=Path)
    parser.add_argument('--stress',required=True,type=Path)
    parser.add_argument('--readonly',required=True,type=Path)
    args=parser.parse_args()
    model=json.loads((args.sweep/'RESULT.json').read_text())
    ros=json.loads((args.ros/'RESULT.json').read_text())
    assert model['profiles']==80 and model['comparison_count']==880 and model['original_unchanged']
    assert ros['passed'] and ros['inherited_safety_cases']==15 and ros['independent_watchdog_stall_passed']
    assert not ros['physical_car_started'] and not ros['firmware_changed']
    stress=json.loads((args.stress/'RESULT.json').read_text())
    readonly=json.loads((args.readonly/'RESULT.json').read_text())
    assert stress['passed'] and stress['active_s']>=60 and stress['synthetic_motion']
    assert stress['concurrent_real_sign_readonly'] and not stress['hardware_output']
    for path,sha in json.loads((args.stress/'IMPLEMENTATION.json').read_text()).items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha, path
    assert readonly['passed'] and readonly['isolated_reader'] and readonly['never_prepared']
    assert readonly['never_engaged'] and not readonly['hardware_output'] and not readonly['cpu_profile']
    assert readonly['reader']['ipc_rejected']==0 and stress['reader']['ipc_rejected']==0
    process=subprocess.run([sys.executable,'-m','unittest','-v','test_model','test_scope','test_dashboard',
                            'test_isolated_reader','test_live_analysis','test_packet_work'],
        cwd=Path(__file__).parent,capture_output=True,text=True)
    (args.ros/'unit_tests.log').write_text(process.stdout+process.stderr)
    if process.returncode:raise RuntimeError(process.stdout+process.stderr)
    evidence=[args.sweep/'RESULT.json',args.sweep/'SWEEP.json',args.ros/'RESULT.json',
              args.ros/'unit_tests.log',args.ros/'independent_timer_stall/TIMER_STALL_RESULT.json',
              args.stress/'RESULT.json', args.stress/'TRACE_LOG_RESULT.json',
              args.stress/'IMPLEMENTATION.json', args.readonly/'RESULT.json']
    save(REPORT/'PREPARED.json',dict(passed=True,sources=hashes(),
        tests=dict(pure_scope_ui=22,model_comparisons=880,ros_inherited_safety=15,
                   matched_scheduling_runs=4,independent_timer_stall=True,
                   real_source_readonly_with_isolated_reader=True,
                   synthetic_motion_with_real_source_load_s=stress['active_s']),
        evidence={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence},
        protected=protected_hashes(),default_profile=dict(speed=.7,accel=4.,rpm=400.),
        slow_plant_stop_confirmation_not_qualified=True,
        hardware_output=False,firmware_changed=False,real_sign_accepted=False))
    print('SYNC_TUNING_SIM_PREPARED',flush=True)
