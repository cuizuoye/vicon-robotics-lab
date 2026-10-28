#!/usr/bin/env python3
"""Prepare an independent 3x3/20s host; never initialize ROS or access hardware."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3_20s/ready'
BASE=HERE.parent/'servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}


def edit(path,old,new):
    value=path.read_text()
    if value.count(old)!=1:raise ValueError('input drift: '+str(path)+' '+old)
    path.write_text(value.replace(old,new))


def replace_obsolete_tests(path,names):
    """Only obsolete short-radius tests are replaced, by new fixed-fence tests."""
    source=path.read_text();lines=source.splitlines(keepends=True);found=[];spans=[]
    for node in ast.parse(source).body:
        if isinstance(node,ast.FunctionDef) and node.name in names:
            start=min([node.lineno]+[v.lineno for v in node.decorator_list])-1
            spans.append((start,node.end_lineno));found.append(node.name)
    if set(found)!=set(names):raise ValueError('test selection drift: '+str(path))
    for start,end in sorted(spans,reverse=True):del lines[start:end]
    path.write_text(''.join(lines))
    return found


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    replay_input=OLD/'ground_stage4_single_gb9j_sxc/observation.jsonl'
    replay_hash=sha(replay_input)
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware',out/'firmware',target_is_directory=True)
    control=runtime/m['control_package']/m['control_package'];hw=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'demo_policy.py',control/'demo_policy.py')
    shutil.copyfile(HERE/'demo_authorization.py',hw/'demo_authorization.py')
    core=control/'pilot_core.py'
    value=core.read_text();start=value.index('POLICY = dict(');end=value.index('\n\n\ndef start_issue',start)
    value=value[:start]+'from .demo_policy import POLICY, active_limits, metrics'+value[end:]
    start=value.index('        dx, dy = car.x-self.car.x, car.y-self.car.y')
    end=value.index('\n\n\nclass PilotSupervisor',start)
    value=value[:start]+'        return active_limits(self, car, wand, goal)'+value[end:]
    core.write_text(value)
    edit(core,"prepare_blocked=start_issue(self.c,self.inputs) if not self.pilot.used else 'PILOT_CONSUMED')",
         "prepare_blocked=start_issue(self.c,self.inputs) if not self.pilot.used else 'PILOT_CONSUMED',\n            **metrics(self.pilot, self.inputs))")
    ui=control/'pilot_ui.py'
    edit(ui,"p.get('max_active_s') == POLICY['max_active_s'] and type(p.get('used')) is bool",
         "(p.get('max_active_s') == POLICY['max_active_s']\n                and p.get('spatial_policy') == POLICY['spatial_policy']\n                and type(p.get('used')) is bool)")
    edit(ui,'杆仅前移约5cm，松键立即停车','3×3m区内1:1跟随，松键立即停车')
    edit(ui,'再前移约5cm；最长20秒、仅一次','3×3m区内全向跟随；最长20秒、仅一次')
    edit(ui,"    return original_reason_text(reason)",
         "    for key,text in {'PILOT_GOAL_FENCE':'独立目标门：目标不在固定3×3m区域内',\n        'PILOT_FOOTPRINT_FENCE':'独立车体门：车体已触及保留余量的硬停围栏',\n        'PILOT_HEADING_LIMIT':'车身朝向偏差超限，保持松键检查'}.items():\n        if key in (reason or ''):return text\n    return original_reason_text(reason)")
    console=control/'operator_console.py'
    edit(console,'单次落地小位移 · 最长20秒','3×3m全向跟随演示 · 最长20秒')
    edit(console,'单次≤20秒 / 杆≤15cm / 车≤20cm','单次≤20秒 / 3×3m目标区 / 四边50cm缓冲')
    edit(console,"f'首轮杆仅前移约5cm，水平总偏移≤15cm\\n'",
         "f'固定3×3m全向目标区；不再使用15cm短程停机门槛\\n'")
    edit(console,'按绿色车中心可用区限幅；本轮最多20秒、一次使能；车中心偏移≤20cm',
         '按绿色目标区限幅，杆可继续移动或收回；本轮最多20秒、一次使能')
    edit(hw/'pilot_authorization.py',"KIND = 'FINITE_FIRST_GROUND_OBSERVATION_NOT_QUALIFICATION'",
         'from .demo_authorization import KIND, verify_range_request')
    edit(hw/'pilot_authorization.py','which adds independent fixed 2s/short-path gates',
         'which adds independent fixed 20s/arena gates')
    edit(hw/'pilot_authorization.py',"    evidence = Path(review['suspended_evidence'])",
         "    verify_range_request(review)\n    evidence = Path(review['suspended_evidence'])")
    edit(hw/'authorization.py',"review.get('kind') == 'FINITE_FIRST_GROUND_OBSERVATION_NOT_QUALIFICATION'",
         "review.get('kind') == 'BOUNDED_3X3_DEMO_20S_NOT_QUALIFICATION'")
    edit(hw/'stages.py',"SCOPE='FINITE_GROUND_PILOT_V3_20S_ACTIVE_ORIGIN_4WS8_V4_STAGE_'",
         "SCOPE='BOUNDED_3X3_DEMO_20S_4WS8_V4_STAGE_'")
    edit(hw/'commissioning.py',"CONSENT = '4WS8_V3_ACTIVE_ORIGIN_20S_CLEAR_POWER_CUT_READY'",
         "CONSENT = '4WS8_DEMO_3X3_20S_CLEAR_POWER_CUT_READY'")
    launcher=(OLD/'launch_pilot.py').read_text()
    launcher=launcher.replace('explicit private stage4 small-displacement ground pilot','explicit private stage4 bounded 3x3m / 20s demonstration')
    launcher=launcher.replace("user_authorization=str(user),user_authorization_sha256=sha(user))",
        "user_authorization=str(user),user_authorization_sha256=sha(user),\n        range_request="+repr(str(HERE/'USER_REQUEST.json'))+",\n        range_request_sha256=sha(Path("+repr(str(HERE/'USER_REQUEST.json'))+")))")
    launcher=launcher.replace('hand/goal15cm, car20cm, path25cm','fixed 3x3m clamped goals and independent body fence')
    launcher=launcher.replace('首轮有限落地观察，不是连续高速合格证。','3×3m有界演示入口，不是连续高速或电气/温升合格证。')
    launcher=launcher.replace('随后仅前移约5cm，不做返回或连续跟随。','可在固定3×3m区内全向跟随，边界限幅；松键结束本轮，不自动恢复。')
    launcher=launcher.replace('确认原起点、原朝向、Sign/car01可见、场地空旷、有人可立即断电；其他小车控制程序已关。',
        '确认车在绿色区内且原朝向正确，Sign/car01可见；3×3m和四边50cm缓冲空旷、有人可立即断电；其他小车控制程序已关。停车距离尚未完成全速实测，不要贴边高速试停。')
    launcher=launcher.replace('4WS8_V3_ACTIVE_ORIGIN_20S_CLEAR_POWER_CUT_READY','4WS8_DEMO_3X3_20S_CLEAR_POWER_CUT_READY')
    (out/'launch_demo.py').write_text(launcher)
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    replaced={
        'test_pilot_bounds.py':['test_independent_bounds_even_if_other_controller_compromised'],
        'test_origin_capture.py':['test_post_enable_fifteen_cm_limit_still_relative_to_new_anchor',
            'test_hand_limit_stops_full_chain_after_successful_adjusted_enable',
            'test_original_firmware_arena_mapping_and_thresholds_unchanged'],
        'test_duration_20s.py':['test_duration_change_does_not_relax_geometry_limits_or_watchdogs'],
    }
    for file,names in replaced.items():replace_obsolete_tests(tests/file,names)
    edit(tests/'test_pilot_bounds.py','user_authorization=str(u),user_authorization_sha256=sha(u))',
         'user_authorization=str(u),user_authorization_sha256=sha(u),\n        range_request='+repr(str(HERE/'USER_REQUEST.json'))+',\n        range_request_sha256=sha(Path('+repr(str(HERE/'USER_REQUEST.json'))+')))')
    edit(tests/'test_pilot_ui.py','pilot=dict(used=used,max_active_s=20.0,remaining_s=4.2,prepare_blocked=blocked)',
         "pilot=dict(used=used,max_active_s=20.0,remaining_s=4.2,prepare_blocked=blocked,spatial_policy=POLICY['spatial_policy'])")
    edit(tests/'test_duration_20s.py',"['release','occlusion','feedback','hand_limit']","['release','occlusion','feedback']")
    edit(tests/'test_duration_20s.py',"    else:h.hand=replace(anchor,x=anchor.x+.151)\n",'')
    shutil.copyfile(HERE/'test_demo_range.py',tests/'test_demo_range.py')
    (out/'TEST_SELECTION.json').write_text(json.dumps(dict(replaced_obsolete_short_range_tests=replaced,
        replacement='test_demo_range.py: fixed-fence independence, >15cm movement, all directions, long path, corner reach and boundary clamp',
        preserved='All other duration, origin, source/codec, authority, safety and real Tk widget tests retained'),indent=2)+'\n')
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),
        PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''),
        DISPLAY=env.get('DISPLAY',':1'),XAUTHORITY=env.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'host_tests.xml')],
        cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'host_tests.log').write_text(r.stdout);print(r.stdout,flush=True)
    if r.returncode:raise ValueError('3x3 demonstration regressions failed; no hardware opened')
    summary=r.stdout.strip().splitlines()[-1]
    pins=dict(m['protected_input_hashes'])
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[group])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    if sha(replay_input)!=replay_hash:raise ValueError('recorded Sign replay input drift')
    pins[str(replay_input)]=replay_hash
    manifest=dict(m);manifest.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**hashes(HERE)},protected_input_hashes=pins,
        host_change='3x3/20s bounded demonstration; removes 15cm/20cm/25cm/side/reverse short-pilot limits, preserves active origin, base safety, stage4, independent fixed goal/footprint bounds and heading guard',
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    manifest['source_hashes'][str(out/'launch_demo.py')]=sha(out/'launch_demo.py')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],cwd=ROOT,env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout);print(r.stdout,flush=True)
    if r.returncode:raise ValueError('private build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,tests=summary,
        max_active_s=20.,goal_area_m=[3.,3.],mapping_scale=1.,stage=4,
        flashed=False,hardware_started=False,continuous_ground_qualified=False,
        stopping_distance_qualified=False,electrical_thermal_qualified=False,
        firmware_reused_unchanged=True,prior_versions_and_original_rectangle_preserved=True,
        protected_files_unchanged=len(pins)),indent=2)+'\n')


if __name__=='__main__':main()
