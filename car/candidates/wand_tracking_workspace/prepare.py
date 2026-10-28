#!/usr/bin/env python3
"""Copy the existing 4WS8 host into a private tracking-workspace candidate.

No firmware build, programmer, network access, ROS initialization or hardware.
All transformations below target freshly generated private files only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_rotation_hardware_2026-10-26/prepared'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('Exact parent source changed: '+str(p)+' / '+old)
    p.write_text(s.replace(old,new))
def section(p,start,end,new):
    s=p.read_text();a=s.index(start);b=s.index(end,a)
    p.write_text(s[:a]+new+s[b:])


def generate(out):
    m=check(OLD/'build_manifest.json');out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    (out/'COLCON_IGNORE').touch()
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    protected.update(hashes(ROOT/'docs/wand_sync_4ws9_2026-10-27/prepared/runtime/src'))
    for name in ('build_manifest.json','arm/mecanum_f407_4ws9_staged_NOT_FLASHED.bin'):
        p=ROOT/'docs/wand_sync_4ws9_2026-10-27/prepared'/name;protected[str(p)]=sha(p)
    for p in (ROOT/'tools/start_car_demo_large_type.sh',ROOT/'tools/start_car_xy_yaw_demo.sh',
              Path('/home/PRIMARY_USER/.local/bin/小车演示')):protected[str(p)]=sha(p)
    for p,v in protected.items():
        if sha(p)!=v:raise ValueError('Protected parent drift: '+p)
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'tracking_config.py',c/'tracking_config.py')
    shutil.copyfile(HERE/'tracking_ui.py',c/'tracking_ui.py')
    # A real policy switch, not a large numeric pretend fence or hidden UI.
    edit(c/'core.py','    governed_targets: bool = False',
         '    workspace_radius_enabled: bool = True\n    governed_targets: bool = False')
    edit(c/'core.py',"if name == 'governed_targets':", "if name in ('governed_targets','workspace_radius_enabled'):")
    edit(c/'core.py','if math.hypot(*raw) * self.config.scale > self.config.max_goal_radius_m:',
         'if self.config.workspace_radius_enabled and math.hypot(*raw) * self.config.scale > self.config.max_goal_radius_m:')
    policy=c/'demo_policy.py'
    section(policy,'POLICY = dict(', '\n\ndef active_limits', '''POLICY = dict(operator_input_mode=INPUT_MODE,stage=4,max_active_s=120.,
    engagements_per_process=1,spatial_policy='TRACKING_ONLY_NO_FIXED_FENCE_V1',
    fixed_fence_enabled=False,target_radius_limit_enabled=False,
    yaw_tracking_limit_rad=.65,yaw_rate_limit_radps=.10,yaw_accel_limit_radps2=.40,
    translation_scale=1.,rotation_scale=1.,reference_point='Sign5',
    origin_capture='FIRST_VALID_NEUTRAL_ACTIVE_V3',
    source_binding='SAME_RAW_SIGN_AND_FIVE_MARKERS_DIGEST_AND_CAR_STAMP_V1')
''')
    section(policy,'    if not monitor.config.goal_inside(goal):','    # The old .20rad', '')
    edit(policy,"car_path_m=monitor.path,nominal_goal_forward_m=POLICY['nominal_goal_forward_m'],\n        nominal_goal_left_m=POLICY['nominal_goal_left_m'],rotation_enabled=True,",
         "car_path_m=monitor.path,fixed_fence_enabled=False,target_radius_limit_enabled=False,rotation_enabled=True,")
    policy.write_text(policy.read_text().replace('Fixed-arena','Tracking-only'))
    # Consumers explicitly construct the new class; legacy profiles cannot
    # accidentally opt out of the fence, nor can old approval admit this mode.
    edit(h/'commissioning.py','from mecanum_wand_4ws8_control.rotation_core import RotationSafetyConfig as SafetyConfig',
         'from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig as SafetyConfig')
    edit(h/'commissioning.py',"CONSENT = '4WS8_SPACE_TOGGLE_120S_CLEAR_POWER_CUT_READY'",
         "CONSENT = '4WS8_TRACKING_ONLY_NO_FIXED_FENCE_CLEAR_POWER_CUT_READY'")
    edit(h/'commissioning.py',"('fixed_arena_checked', 'footprint_measured'",
         "('tracking_only_no_obstacle_detection_acknowledged', 'footprint_measured'")
    edit(h/'commissioning.py',"'fixed_reference_signed_120s_buffer050_v1'","'TRACKING_ONLY_NO_FIXED_FENCE_V1'")
    edit(h/'commissioning.py',"required = ('extended_arena_and_buffer_clear_and_tracked',\n                        'saved_start_and_heading_match', 'immediate_power_cut_ready_now')",
         "required = ('all_intended_paths_and_stopping_clearance_checked_now',\n                        'no_fixed_fence_or_obstacle_detection_acknowledged_now', 'immediate_power_cut_ready_now')")
    edit(h/'commissioning.py',"self.profile.get('arena_confirmations', {})","self.profile.get('tracking_confirmations', {})")
    edit(h/'commissioning.py',"        if self.config.max_goal_radius_m > 4.75: raise ValueError('acceptance target radius is at most 4.75m; goals still fixed to confirmed signed subregion')\n",'')
    # The old review remains historical evidence of stage4 suspended behavior;
    # it does not authorize a new space. New request + fresh site checks do.
    oldkind='BOUNDED_XY_YAW_4WS8_EXISTING_ENVELOPE_NOT_QUALIFICATION'
    newkind='TRACKING_ONLY_XY_YAW_4WS8_NOT_FULL_COVERAGE_QUALIFICATION'
    parent=h/'tracking_parent_authorization.py';shutil.copyfile(h/'authorization.py',parent)
    edit(parent,repr(oldkind),repr(newkind))
    shutil.copyfile(HERE/'authorization.py',h/'authorization.py')
    edit(h/'pilot_authorization.py','KIND='+repr(oldkind),'KIND='+repr(newkind))
    with (h/'pilot_authorization.py').open('a') as f:
        f.write(f'''

_verify_rotation_history = verify_pilot_review
TRACKING_REQUEST=Path({str(HERE/'USER_REQUEST.json')!r})
TRACKING_REQUEST_SHA={sha(HERE/'USER_REQUEST.json')!r}
def verify_pilot_review(review,stage,purpose,firmware_sha):
    if (review.get('tracking_request')!=str(TRACKING_REQUEST)
            or review.get('tracking_request_sha256')!=TRACKING_REQUEST_SHA
            or sha(TRACKING_REQUEST)!=TRACKING_REQUEST_SHA
            or review.get('full_vicon_coverage_qualified') is not False):
        raise ValueError('independent no-fence request required; full coverage not qualified')
    return _verify_rotation_history(review,stage,purpose,firmware_sha)
''')
    # No dormant fixed-start requirement in the preflight. Yaw is captured at
    # the genuine enable edge; the rigid-body installation offset is retained.
    pose=out/'pose_check.py';shutil.copyfile(ROOT/'candidates/wand_demo_120s_toggle/pose_check.py',pose)
    edit(pose,'from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig',
         'from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig as StagedSafetyConfig')
    edit(pose,"ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready/runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'",
         repr(str(runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json')))
    edit(pose,'json.loads(('+repr(str(runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'))+").read_text())", 
         'json.loads(Path('+repr(str(runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'))+").read_text())")
    section(pose,'            error=math.atan2(',"            if abs(car.z", 
            "            result['car_world_m']=[car.x,car.y,car.z]\n            result['car_heading_deg']=math.degrees(car.yaw)\n")
    ui=c/'operator_console.py'
    section(ui,"        c = self.map\n        c.delete('all')\n        for i in range",'        status, gate = self.model.status, self.model.gate',
            '        self.draw_tracking_map()\n        p = self.node.car\n')
    section(ui,"        elif self.node.raw_wand is not None and p is not None:\n",'        self.buttons[\'start\'].configure',
            '        else:\n            self.hand_text.set(self.tracking_hand_text())\n')
    edit(ui,'from .rotation_ui import RotationConsoleMixin\nclass RotationConsole(RotationConsoleMixin, LargeTypeConsole):',
         'from .tracking_ui import TrackingConsoleMixin\nclass RotationConsole(TrackingConsoleMixin, LargeTypeConsole):')
    for name in ('oneclick_app.py','auto_open.py','large_type.py'):shutil.copyfile(OLD/name,out/name)
    app=out/'oneclick_app.py'
    edit(app,'READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')')
    edit(app,'from mecanum_wand_4ws8_control.rotation_ui import label_launcher',
         'from mecanum_wand_4ws8_control.tracking_ui import label_launcher')
    edit(app,'return dict(rotation_request=',
         f'return dict(tracking_request={str(HERE/"USER_REQUEST.json")!r},tracking_request_sha256={sha(HERE/"USER_REQUEST.json")!r},full_vicon_coverage_qualified=False,rotation_request=')
    edit(app,"str(HERE/'pose_check.py')","str(READY/'pose_check.py')")
    edit(app,"profile['confirmations']['fixed_arena_checked']=True",
         "profile['confirmations']['tracking_only_no_obstacle_detection_acknowledged']=True")
    edit(app,"profile['arena_confirmations']={k:True for k in profile['arena_confirmations']}",
         "profile['tracking_confirmations']={k:True for k in profile['tracking_confirmations']}")
    edit(app,"if m.get('rotation_migration_passed') is not True:",
         "if m.get('tracking_workspace_passed') is not True:")
    edit(app,'旋转迁移离线验证尚未完成；不得启动','无固定围栏版离线验证尚未完成；不得启动')
    edit(app,'独立旋转候选尚未实机验收；需明确 --launch 打开未使能窗口，现场确认后才能准备',
         '无固定围栏候选需明确 --launch 打开未使能窗口；重新确认本轮全部路径空旷后才能准备')
    edit(app,'4/4 检查Sign、car01及绿色区/朝向','4/4 检查Sign、car01定位及落地姿态；无固定位置/朝向门槛')
    edit(app,"self.append('小车区域坐标：'+str(value.get('car_local_m'))+'；朝向误差：'+str(round(value.get('heading_error_deg',0),2))+'°')",
         "self.append('小车世界坐标：'+str(value.get('car_world_m'))+'；本轮朝向：'+str(round(value.get('car_heading_deg',0),2))+'°（启用时记录）')")
    # Draft approval remains false until a human confirms the new actual site.
    profiles=[]
    for i in range(7):
        p=runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json';v=json.loads(p.read_text())
        v['safety']['goal_boundary_policy']='tracking';v['arena_revision']='TRACKING_ONLY_NO_FIXED_FENCE_V1'
        v['confirmations'].pop('fixed_arena_checked',None)
        v['confirmations']['tracking_only_no_obstacle_detection_acknowledged']=False
        v.pop('arena_confirmations',None);v.pop('arena_buffer_policy',None)
        v['tracking_confirmations']={k:False for k in ('all_intended_paths_and_stopping_clearance_checked_now',
            'no_fixed_fence_or_obstacle_detection_acknowledged_now','immediate_power_cut_ready_now')}
        v['input_workspace']=dict(spatial_policy=v['arena_revision'],fixed_fence_enabled=False,
            target_radius_limit_enabled=False,full_vicon_coverage_measured=False,obstacle_detection_available=False)
        v['evidence'].pop('nominal_center_workspace',None)
        v['evidence']['site_scope']='Historical geometry retained as metadata only; no fixed fence; current path clearance requires human confirmation'
        write(p,v);profiles.append(str(p))
    # The real DDS rehearsal uses these adapters but software sources/Plant,
    # never constructs HardwareBridge or a real Vicon receiver.
    rehearsal=out/'ros_rehearsal.py';shutil.copyfile(ROOT/'candidates/wand_rotation_hardware/ros_rehearsal.py',rehearsal)
    edit(rehearsal,'from mecanum_wand_4ws8_control.rotation_core import RotationSafetyConfig',
         'from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig')
    edit(rehearsal,"('normal_stop','occlusion','source_digest','yaw_intent','actuator_fault','source_conflict')",
         "('normal_stop','occlusion','car_occlusion','source_digest','yaw_intent','actuator_fault','source_conflict','wide_motion')")
    edit(rehearsal,'source_on=True,fault_bits=0','source_on=True,car_source_on=True,fault_bits=0')
    edit(rehearsal,"origin=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad)",
         "origin=Pose(12.,-8.,c.ground_z_m,0.)")
    edit(rehearsal,"posepub['car'].publish(pose_message(p,stamp,world='map'))",
         "if state.car_source_on:posepub['car'].publish(pose_message(p,stamp,world='map'))")
    edit(rehearsal,"gesture=.0 if state.active is None else .07*math.sin(t*1.6)",
         "gesture=.0 if state.active is None else (.45*t if case=='wide_motion' else .07*math.sin(t*1.6))")
    edit(rehearsal,"yaw=.5+(.0 if state.active is None else .10*math.sin(t*1.6))",
         "yaw=.5+(.0 if state.active is None or case=='wide_motion' else .10*math.sin(t*1.6))")
    edit(rehearsal,'point=np.array([1.+gesture,2.,1.2])','point=np.array([30.+gesture,-20.,1.2])')
    edit(rehearsal,'                def fixture():\n                    now=',
         "                def fixture():\n                    # SOFTWARE fixture discovery barrier: do not discard the\n                    # very first selected sample before all sensor subscribers\n                    # exist. No production adapter or safety deadline is changed.\n                    if posepub['car'].get_subscription_count()<3 or pubs['rotation_packet'].get_subscription_count()<2:return\n                    now=")
    edit(rehearsal,'source.create_timer(.01,fixture)','source.create_timer(.05,fixture)')
    # The original 100Hz fixture capped integration at 30ms. At the new 20Hz
    # source cadence that would simulate only 60% of elapsed time, silently
    # slowing the SOFTWARE Plant. Integrate actual fixture time, with a 100ms
    # upper bound; no production controller or deadline is changed.
    edit(rehearsal,'dt=min(.03,max(0.,now-state.last_fixture))',
         'dt=min(.10,max(0.,now-state.last_fixture))')
    edit(rehearsal,"(4.5 if case=='normal_stop' else 1.2)","(16. if case=='wide_motion' else 4.5 if case=='normal_stop' else 1.2)")
    edit(rehearsal,"if case=='normal_stop':model.end_trial", "if case in ('normal_stop','wide_motion'):model.end_trial")
    edit(rehearsal,"elif case=='occlusion':state.source_on=False",
         "elif case=='occlusion':state.source_on=False\n                        elif case=='car_occlusion':state.car_source_on=False")
    edit(rehearsal,'state.source_on=True;state.restored=now','state.source_on=state.car_source_on=True;state.restored=now')
    edit(rehearsal,'while time.monotonic()-started<13.:','while time.monotonic()-started<(25. if case==\'wide_motion\' else 13.):')
    edit(rehearsal,"if state.active is not None and state.injected is None and now-state.active>",
         "if state.active is not None and state.injected is None and now-state.active>.2:\n                        assert sup.core.state=='ACTIVE' and not gate.core.fault,(case,'unexpected pre-injection stop',sup.core.reason,gate.core.fault)\n                    if state.active is not None and state.injected is None and now-state.active>")
    edit(rehearsal,"assert state.max_speed<=.7+1e-6 and .005<state.max_wz<=.1+1e-6",
         "assert state.max_speed<=.7+1e-6 and state.max_wz<=.1+1e-6\n                if case!='wide_motion':assert state.max_wz>.005")
    edit(rehearsal,"if case=='normal_stop':assert state.positive and state.negative",
         "if case=='normal_stop':assert state.positive and state.negative\n                if case=='wide_motion':assert plant.pose.x-origin.x>5.,plant.pose")
    edit(rehearsal,'zero_before_click=True,stopped_after_injection_s=',
         'zero_before_click=True,car_x_m=plant.pose.x,car_start_x_m=origin.x,stopped_after_injection_s=')
    sources={**hashes(HERE),str(ROOT/'tools/start_car_tracking_workspace.sh'):sha(ROOT/'tools/start_car_tracking_workspace.sh')}
    for name in ('oneclick_app.py','auto_open.py','large_type.py','pose_check.py','ros_rehearsal.py'):
        p=out/name;sources[str(p)]=sha(p)
    manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,draft_profiles=profiles,tracking_workspace_passed=False,
        spatial_policy='TRACKING_ONLY_NO_FIXED_FENCE_V1',original_entry_changed=False,
        fixed_fence_enabled=False,target_radius_limit_enabled=False,
        firmware_change=False,full_vicon_coverage_qualified=False)
    write(out/'build_manifest.json',manifest);check(out/'build_manifest.json')
    return manifest


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    generate(a.output.resolve());print('Generated OFFLINE ONLY; validation pending')
