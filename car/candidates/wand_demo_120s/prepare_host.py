#!/usr/bin/env python3
"""Independent120s signed-arena preparation. No ROS init, sockets or flashing."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v2/ready'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(d):return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and not any(
    k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(p,a,b):
    s=p.read_text()
    if s.count(a)!=1:raise ValueError('input drift: '+str(p)+' '+a)
    p.write_text(s.replace(a,b))
def replacements(p, pairs):
    for a,b in pairs:edit(p,a,b)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    for name in ('arena_config.py','arena_geometry.py','demo_policy.py','pose_quality.py'):
        shutil.copyfile(HERE/name,c/name)
    shutil.copyfile(HERE/'demo_authorization.py',h/'demo_authorization.py')
    edit(c/'pilot_core.py','PILOT_TIME_LIMIT_20S','PILOT_TIME_LIMIT_120S')
    for name in ('pilot_ui.py','operator_console.py'):
        p=c/name;s=p.read_text().replace('20秒','120秒').replace('PILOT_TIME_LIMIT_20S','PILOT_TIME_LIMIT_120S')
        s=s.replace('3×3m','2.0×0.9m')
        p.write_text(s)
    edit(c/'pilot_ui.py',"    if 'PILOT_TIME_LIMIT_120S' in (reason or ''):",
        "    if 'PILOT_TIME_LIMIT_120S' in (reason or '') or reason == 'SESSION_TIME_LIMIT':")
    edit(h/'stages.py','BOUNDED_3X3_DEMO_20S_4WS8_V4_STAGE_','BOUNDED_SIGNED_ARENA_120S_4WS8_V4_STAGE_')
    edit(h/'authorization.py','BOUNDED_3X3_DEMO_20S_NOT_QUALIFICATION','BOUNDED_OFFSET_ARENA_120S_NOT_QUALIFICATION')
    edit(h/'authorization.py',"profile.get('arena_revision')!='fixed_corner_3x3_buffer050_v1'",
        "profile.get('arena_revision')!='fixed_reference_signed_120s_buffer050_v1'")
    edit(h/'authorization.py','config.arena_forward_max_m!=3 or config.arena_left_max_m!=3 or config.arena_buffer_m!=.5 or',
        'config.arena_forward_min_m!=-1.5 or config.arena_forward_max_m!=.5 or\n        config.arena_left_min_m!=-.4 or config.arena_left_max_m!=.5 or config.arena_buffer_m!=.5 or')
    edit(h/'authorization.py','fixed 1:1 / 3x3 / 50cm safety geometry required',
        'fixed1:1 / confirmed signed arena /50cm safety geometry required')
    replacements(h/'commissioning.py',[
        ('4WS8_DEMO_3X3_20S_CLEAR_POWER_CUT_READY','4WS8_SIGNED_ARENA_120S_CLEAR_POWER_CUT_READY'),
        ("'arena_forward_max_m', 'arena_left_max_m', 'arena_buffer_m')",
         "'arena_forward_max_m', 'arena_left_max_m', 'arena_buffer_m',\n                   'arena_forward_min_m', 'arena_left_min_m')"),
        ('fixed_corner_3x3_buffer050_v1','fixed_reference_signed_120s_buffer050_v1'),
        ('explicit fixed 3x3/0.50m arena profile required','explicit confirmed signed/0.50m arena profile required'),
        ('ARENA_3X3_SITE_CONFIRMATION_REQUIRED','SIGNED_ARENA_SITE_CONFIRMATION_REQUIRED'),
        ('goals still fixed to 3x3m','goals still fixed to confirmed signed subregion')])
    # Derive the display bounds from the same exact signed geometry. These are
    # requested coordinates, not a claim that Vicon coverage was measured.
    sys.path.insert(0,str(runtime/m['control_package']))
    module=importlib.import_module(m['control_package']+'.arena_geometry')
    for p in (runtime/m['hardware_package']/'config').glob('stage*.NOT_APPROVED.json'):
        profile=json.loads(p.read_text());s=profile['safety']
        s.update(arena_forward_min_m=-1.5,arena_forward_max_m=.5,
            arena_left_min_m=-.4,arena_left_max_m=.5)
        g=module.FixedArena(s['arena_origin_x_m'],s['arena_origin_y_m'],s['arena_heading_rad'])
        bounds=g.bbox(g.physical_bounds())
        s.update(dict(zip(('arena_x_min','arena_x_max','arena_y_min','arena_y_max'),bounds)))
        profile['arena_revision']='fixed_reference_signed_120s_buffer050_v1'
        profile['input_workspace']['mapping']='1:1 XY; confirmed forward[-1.5,.5]/left[-.4,.5] goals; independent footprint fence'
        profile['evidence']['nominal_center_workspace'].update(forward_min_m=-1.5,forward_max_m=.5,
            left_min_m=-.4,left_max_m=.5)
        profile['evidence']['operator_request']=str(HERE/'USER_REQUEST.json')
        profile['evidence']['site_scope']='User-confirmed subregion and buffer only; full Vicon coverage still unmeasured'
        profile['arena_buffer_policy'].update(hard_center_range_at_reference_heading_m=[-1.8,.8,-.7,.8],
            physical_clear_extent_m=[3.19,2.11],all_yaw_clear_extent_planning_m=[3.3,2.2])
        p.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    tests=out/'tests';tests.mkdir()
    for name in ('support.py','test_demo_120s.py'):
        shutil.copyfile(HERE/name,tests/name)
    # Reuse the startup/reconnect/real-frame and pose-diagnostic regressions.
    for name in ('test_oneclick.py','test_pose_quality.py'):
        shutil.copyfile(OLD/'tests'/name,tests/name)
        p=tests/name;s=p.read_text().replace('from test_pilot_bounds import ','from support import ').replace('from test_pilot_ui import model','from support import model')
        if name=='test_oneclick.py':
            s=s.replace("'safety_core.py','pilot_core.py','demo_policy.py','staged_config.py','arena_geometry.py'",
                "'safety_core.py','staged_config.py'")
            s=s.replace("('stage0_contract.py','stage0_session.py','authorization.py','pilot_authorization.py')",
                "('stage0_contract.py','stage0_session.py')")
        else:
            s=s.replace('origin_local=(1.5,1.5)','origin_local=(0.,0.)')
            s=s.replace("'safety_core.py','pilot_core.py','demo_policy.py','staged_config.py'",
                "'safety_core.py','staged_config.py'")
            s=s.replace("('nodes.py','stage0_contract.py','stage0_session.py','authorization.py')",
                "('nodes.py','stage0_contract.py','stage0_session.py')")
        p.write_text(s)
    env=dict(os.environ,STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(HERE),
        PYTHONPATH=str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'host_tests.xml')],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    log=r.stdout+r.stderr;(out/'host_tests.log').write_text(log);print(log,flush=True)
    if r.returncode:raise ValueError('120s/signed-arena tests failed; no hardware opened')
    pins=dict(m['protected_input_hashes'])
    for key in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[key])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    sources={**m['source_hashes'],**hashes(HERE)}
    for p in (ROOT/'tools/start_car_demo_120s.sh',Path('/home/PRIMARY_USER/.local/bin/小车120')):sources[str(p)]=sha(p)
    result=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=pins,draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Independent120s from held-edge, exact signed user-confirmed subregion. Stage4, raw-pose/source/output guards retained; old entry and firmware unchanged.')
    (out/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n');check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True)
    if r.returncode:raise ValueError('private120s build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,
        tests=log.strip().splitlines()[-1],max_active_s=120.,forward_bounds_m=[-1.5,.5],
        left_bounds_m=[-.4,.5],buffer_m=.5,mapping_scale=1.,stage=4,
        protected_files_unchanged=len(pins),hardware_started=False,flashed=False,
        full_vicon_coverage_measured=False,stopping_distance_qualified=False,
        electrical_thermal_qualified=False,default_entry_changed=False),indent=2)+'\n')

if __name__=='__main__':main()
