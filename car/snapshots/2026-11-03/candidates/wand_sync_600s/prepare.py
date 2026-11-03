#!/usr/bin/env python3
"""Derive a separate 4WSA long-session candidate; never flash or open hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
PARENT = ROOT/'docs/wand_sync_4ws9_2026-10-27/prepared'
CURRENT = ROOT/'docs/wand_feedback_300ms_startup_2026-10-27/prepared'
CONTROL = 'mecanum_wand_4wsa_control'
HARDWARE = 'mecanum_wand_4wsa_hardware'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def hashes(folder):
    return {str(p): sha(p) for p in Path(folder).rglob('*') if p.is_file()
            and not any(k in ('__pycache__', '.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}


def edit(p, old, new, count=1):
    s = p.read_text()
    if s.count(old) != count:
        raise ValueError(f'Unexpected parent layout {p}: {old[:90]!r}')
    p.write_text(s.replace(old, new))


def renamed(s):
    return (s.replace('mecanum_wand_4ws9', 'mecanum_wand_4wsa')
            .replace('WAND_4WS9', 'WAND_4WSA').replace('4WS9', '4WSA')
            .replace('4ws9', '4wsa').replace('wand_4wsa_sync', 'wand_4wsa_long_sync')
            .replace('0x39535734', '0x41535734').replace('0x7900', '0x7A00')
            .replace('0x7980', '0x7A80').replace('4WSA_V5', '4WSA_V6')
            .replace('v5', 'v6').replace('STAGED_V5', 'STAGED_V6')
            .replace('SPACE_TOGGLE_120S', 'SPACE_TOGGLE_600S')
            .replace('PILOT_TIME_LIMIT_120S', 'PILOT_TIME_LIMIT_600S')
            .replace('120秒', '600秒'))


def generate(out):
    if out.exists():
        raise ValueError('Use a new output folder; accepted candidates are never overwritten')
    parent = json.loads((PARENT/'build_manifest.json').read_text())
    if parent.get('offline_passed') is not True:
        raise ValueError('Qualified offline 4WS9 parent required')
    pins = {str(PARENT/'build_manifest.json'): sha(PARENT/'build_manifest.json')}
    for group in ('runtime_hashes', 'source_hashes', 'firmware_source_hashes',
                  'protected_input_hashes', 'evidence_hashes'):
        pins.update(parent[group])
    current = json.loads((CURRENT/'build_manifest.json').read_text())
    pins[str(CURRENT/'build_manifest.json')] = sha(CURRENT/'build_manifest.json')
    for group in ('runtime_hashes', 'source_hashes', 'generated_firmware_hashes', 'protected_input_hashes'):
        pins.update(current.get(group, {}))
    pins[str(ROOT/'tools/start_car_tracking_300ms_ready.sh')] = sha(ROOT/'tools/start_car_tracking_300ms_ready.sh')
    pins[str(ROOT/'tools/start_car_tracking_auto.sh')] = sha(ROOT/'tools/start_car_tracking_auto.sh')
    pins[str(ROOT/'candidates/wand_sync_4ws9/test_sync.py')] = sha(ROOT/'candidates/wand_sync_4ws9/test_sync.py')
    for p, digest in pins.items():
        if sha(p) != digest:
            raise ValueError('Parent drift: '+p)
    out.mkdir(parents=True)
    (out/'COLCON_IGNORE').touch()
    write(out/'protected_inputs.json', pins)
    fw = out/'firmware'
    shutil.copytree(PARENT/'firmware', fw)
    rt = out/'runtime/src'
    shutil.copytree(PARENT/'runtime/src', rt,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.egg-info'))
    for base in (fw, rt):
        for p in base.rglob('*'):
            if p.is_file():
                p.write_text(renamed(p.read_text()))
        for p in sorted(base.rglob('*'), key=lambda v: len(v.parts), reverse=True):
            n = renamed(p.name)
            if n != p.name:
                p.rename(p.with_name(n))
    profile = fw/'Inc/four_speed_pwm_profile.h'
    edit(profile, '#define SERVO_TARGET_RPM 400.0f',
         '#undef FOUR_DRIVE_RUN_LIMIT_MS\n#define FOUR_DRIVE_RUN_LIMIT_MS 630000U\n'
         '#define SERVO_TARGET_RPM 400.0f')
    edit(profile, 'FOUR_DRIVE_RUN_LIMIT_MS==150000U,"absolute budget unchanged"',
         'FOUR_DRIVE_RUN_LIMIT_MS==630000U,"600s host plus 30s independent margin"')
    header = fw/'Inc/four_link_bench.h'
    edit(header, '    uint32_t coupled_shape_revision;',
         '    uint32_t coupled_shape_revision;\n    uint32_t absolute_run_limit_ms;')
    edit(header, 'sizeof(four_link_bench_status_t)==200', 'sizeof(four_link_bench_status_t)==204')
    edit(header, '200-byte', '204-byte')
    link = fw/'Src/four_link_motion.c'
    edit(link, 's->output_authority_version = 5U;', 's->output_authority_version = 6U;')
    edit(link, 's->coupled_shape_revision=1U;',
         's->coupled_shape_revision=1U; s->absolute_run_limit_ms=FOUR_DRIVE_RUN_LIMIT_MS;')
    for name in ('test_controller.c', 'test_stages.c'):
        p = fw/'tests'/name
        s = p.read_text().replace('output_authority_version==5', 'output_authority_version==6')
        s = s.replace('sizeof(s)==200', 'sizeof(s)==204')
        s = s.replace('now-150000', 'now-FOUR_DRIVE_RUN_LIMIT_MS')
        p.write_text(s)
    shutil.copyfile(HERE/'test_long_run.c', fw/'tests/test_long_run.c')
    with (fw/'CMakeLists.txt').open('a') as stream:
        stream.write('''
if(NOT SERVO_ARM)
  add_executable(test_long_run tests/test_long_run.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_long_run PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_long_run PRIVATE m)
  add_test(NAME long_run_and_independent_deadline COMMAND test_long_run)
endif()
''')
    c, h = rt/CONTROL/CONTROL, rt/HARDWARE/HARDWARE
    edit(c/'core.py', '    governed_targets: bool = False',
         '    workspace_radius_enabled: bool = True\n    governed_targets: bool = False')
    edit(c/'core.py', "if name == 'governed_targets':", "if name in ('governed_targets','workspace_radius_enabled'):")
    edit(c/'core.py', 'if math.hypot(*raw) * self.config.scale > self.config.max_goal_radius_m:',
         'if self.config.workspace_radius_enabled and math.hypot(*raw) * self.config.scale > self.config.max_goal_radius_m:')
    # Validate all historical non-time guards against their unmodified strict
    # classes. Only explicitly enumerated 600s and 300ms values are normalized.
    staged = c/'staged_config.py'
    edit(staged, '        choices=[i for i,s in enumerate(STAGES)',
         "        if self.max_session_s != 600. or self.feedback_timeout_s != .30:\n"
         "            raise ValueError('Exact 600s session and existing 300ms feedback guard required')\n"
         '        choices=[i for i,s in enumerate(STAGES)')
    edit(staged, 'max_yaw_rate_radps=.10,max_yaw_accel_radps2=.40)',
         'max_yaw_rate_radps=.10,max_yaw_accel_radps2=.40,max_session_s=120.,feedback_timeout_s=.20)')
    # Preserve the current explicit no-fixed-fence policy without inventing
    # a larger arena. All pose/tilt/source/tracking guards still apply.
    source = CURRENT/'runtime/src/mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/tracking_config.py'
    s = source.read_text()
    s = s.replace("        if (self.max_speed_mps,self.max_accel_mps2)!=(.70,.80):\n"
                  "            raise ValueError('this host keeps existing stage4 .70/.80; no automatic speed upgrade')\n", '')
    (c/'tracking_config.py').write_text(s)
    policy = c/'demo_policy.py'
    edit(policy, 'max_active_s=120.0', 'max_active_s=600.0')
    s = policy.read_text().replace('SIGNED_XY_YAW_4WSA_V6_TIER_PILOT_V1', 'TRACKING_ONLY_NO_FIXED_FENCE_600S_4WSA_V6')
    s = s.replace("    nominal_goal_forward_m=[-1.5,.5], nominal_goal_left_m=[-.4,.5],",
                  '    fixed_fence_enabled=False, target_radius_limit_enabled=False,')
    s = s.replace("nominal_goal_forward_m=POLICY['nominal_goal_forward_m'],\n        nominal_goal_left_m=POLICY['nominal_goal_left_m'],rotation_enabled=True,",
                  'fixed_fence_enabled=False,target_radius_limit_enabled=False,rotation_enabled=True,')
    policy.write_text(s)
    stages = c/'stages.py'
    edit(stages, "SCOPE='HARDWARE_4WSA_V6_XY_YAW_STAGE_'", "SCOPE='HARDWARE_4WSA_V6_600S_XY_YAW_STAGE_'")
    shutil.copyfile(stages, h/'stages.py')
    contract = h/'stage0_contract.py'
    edit(contract, 'SIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size+SERVO.size+ROTATION.size',
         "DURATION=struct.Struct('<I')\nSIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size+SERVO.size+ROTATION.size+DURATION.size")
    edit(contract, 'exactly 200 bytes', 'exactly 204 bytes')
    edit(contract, '    yr,ya,wr,revision=ROTATION.unpack_from(payload,184)',
         "    budget,=DURATION.unpack_from(payload,200)\n"
         "    if budget != 630000: raise ValueError('Independent 630s firmware deadline required')\n"
         '    yr,ya,wr,revision=ROTATION.unpack_from(payload,184)')
    edit(contract, 'version!=5', 'version!=6')
    s = contract.read_text().replace('output_authority_version=5', 'output_authority_version=6')
    s = s.replace("status.get('output_authority_version')==5", "status.get('output_authority_version')==6")
    s = s.replace('yaw_rate_limit=yr,yaw_accel_limit=ya', 'absolute_run_limit_ms=budget,yaw_rate_limit=yr,yaw_accel_limit=ya')
    contract.write_text(s)
    auth = h/'authorization.py'
    s = auth.read_text().replace("f['authority_version']!=5", "f['authority_version']!=6")
    s = s.replace("f['diagnostic_payload_bytes']!=200", "f['diagnostic_payload_bytes']!=204")
    s = s.replace("hw.get('output_authority_version')!=5", "hw.get('output_authority_version')!=6")
    s = s.replace("hw.get('diagnostic_payload_bytes')!=200", "hw.get('diagnostic_payload_bytes')!=204")
    s = s.replace('from mecanum_wand_4wsa_control.rotation_core import RotationSafetyConfig',
                  'from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig')
    s = s.replace('fixed_reference_signed_120s_buffer050_v1', 'tracking_only_600s_4wsa_v6')
    s = s.replace("KIND='4WSA_V6_CONTROLLED_TRIAL_NOT_QUALIFICATION'", "KIND='4WSA_V6_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION'")
    auth.write_text(s)
    commission = h/'commissioning.py'
    edit(commission, 'from mecanum_wand_4wsa_control.rotation_core import RotationSafetyConfig as SafetyConfig',
         'from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as SafetyConfig')
    s = commission.read_text().replace('121,', '122,')
    s = s.replace("hw.get('diagnostic_payload_bytes') != 200", "hw.get('diagnostic_payload_bytes') != 204")
    s = s.replace("hw.get('output_authority_version') != 5", "hw.get('output_authority_version') != 6")
    s = s.replace('self.motion_status_size = 200', 'self.motion_status_size = 204')
    s = s.replace('200-byte', '204-byte')
    s = s.replace("if self.config.max_session_s > 120: raise ValueError('acceptance session is at most 120s')",
                  "if self.config.max_session_s != 600: raise ValueError('exact 600s candidate session required')")
    s = s.replace('fixed_reference_signed_120s_buffer050_v1', 'tracking_only_600s_4wsa_v6')
    s = s.replace("('fixed_arena_checked', 'footprint_measured'", "('tracking_only_no_obstacle_detection_acknowledged', 'footprint_measured'")
    s = s.replace("required = ('extended_arena_and_buffer_clear_and_tracked',\n                        'saved_start_and_heading_match', 'immediate_power_cut_ready_now')",
                  "required = ('all_intended_paths_and_stopping_clearance_checked_now',\n                        'no_fixed_fence_or_obstacle_detection_acknowledged_now', 'immediate_power_cut_ready_now')")
    s = s.replace("self.profile.get('arena_confirmations', {})", "self.profile.get('tracking_confirmations', {})")
    s = s.replace("'max_accel_mps2', 'max_yaw_rate_radps'", "'max_accel_mps2', 'feedback_timeout_s', 'max_yaw_rate_radps'")
    commission.write_text(s)
    guard = c/'rotation_receiver_guard.py'
    edit(guard, "('0','121')", "('0','122')")
    # The inherited console uses strict legacy config until a runtime is
    # supplied; no unchecked standalone CLI is exported for this candidate.
    ui = c/'rotation_ui.py'
    s = ui.read_text().replace(' · 120秒', ' · 600秒')
    s = s.replace('不是A2高速角参数', '当前所选档位不代表实车已验收')
    ui.write_text(s)
    # Reuse the CURRENT viewport implementation; a display box is not an arena.
    for name in ('tracking_ui.py', 'operator_console.py'):
        source = CURRENT/'runtime/src/mecanum_wand_4ws8_control/mecanum_wand_4ws8_control'/name
        s = renamed(source.read_text().replace('4ws8', '4ws9').replace('4WS8', '4WS9'))
        (c/name).write_text(s)
    tracking_ui = c/'tracking_ui.py'
    edit(tracking_ui,
         "top[1].configure(text='1:1 · 平移≤0.70m/s · 转速≤5.7°/秒 · 最长600秒 · 其他保护保留')",
         "top[1].configure(text=f'档{self.config.stage} · 1:1 · 平移≤{self.config.max_speed_mps:.2f}m/s · ' \n"
         "            f'旋转≤{self.config.max_yaw_rate_radps:.2f}rad/s / {self.config.max_yaw_accel_radps2:.2f}rad/s² · 最长600秒')")
    for stage in range(7):
        p = rt/HARDWARE/f'config/stage{stage}.NOT_APPROVED.json'
        profile = json.loads(p.read_text())
        profile['safety'].update(max_session_s=600., feedback_timeout_s=.30, goal_boundary_policy='tracking')
        profile['arena_revision'] = 'tracking_only_600s_4wsa_v6'
        profile['hardware'].update(required_motion_tag='4WSA', diagnostic_payload_bytes=204, output_authority_version=6)
        profile['trial_review'].update(kind='4WSA_V6_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION')
        profile['operator_input_mode'] = 'SPACE_TOGGLE_600S_V1'
        profile['long_session_qualified'] = False
        profile['confirmations'].pop('fixed_arena_checked', None)
        profile['confirmations']['tracking_only_no_obstacle_detection_acknowledged'] = False
        profile['tracking_confirmations'] = dict(all_intended_paths_and_stopping_clearance_checked_now=False,
            no_fixed_fence_or_obstacle_detection_acknowledged_now=False, immediate_power_cut_ready_now=False)
        profile['input_workspace'].update(mapping='1:1 XY+yaw; no fixed arena or radius; valid tracking and all dynamic guards required')
        profile['evidence'].update(change_scope='Independent 4WSA v6 600s staged offline candidate',
            input_semantics='Explicit tap starts; second Space/Esc/focus/fault/600s stops; release does not stop; not ground-qualified',
            site_scope='Full Vicon coverage unmeasured; no obstacle detection',
            operator_request=str(HERE/'USER_REQUEST.json'))
        write(p, profile)
    # Port existing tests mechanically, keeping parent tests and all accepted
    # runtimes intact. New duration cases live in a separately authored suite.
    oldtests = ROOT/'candidates/wand_sync_4ws9/test_sync.py'
    tests = renamed(oldtests.read_text())
    tests = tests.replace('from mecanum_wand_4wsa_control.rotation_core import RotationSafetyConfig,RotationSupervisor,RotationFinalGate',
                          'from mecanum_wand_4wsa_control.rotation_core import RotationSupervisor,RotationFinalGate\n'
                          'from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig')
    tests = tests.replace('(.2,.2,.1,.15,120.)', '(.2,.3,.1,.15,600.)')
    tests = tests.replace('assert SIZE==200', 'assert SIZE==204').replace("['output_authority_version']==5", "['output_authority_version']==6")
    tests = tests.replace('184,188,192,196)', '184,188,192,196,200)')
    tests = tests.replace('HARDWARE_4WSA_V6_XY_YAW_STAGE_6', 'HARDWARE_4WSA_V6_600S_XY_YAW_STAGE_6')
    tests = tests.replace('h.now-120.', 'h.now-600.')
    # This reused test asserts files are identical after *only* renaming. The
    # new source-preservation tests compare the actual complete pinned tree.
    tests = tests[:tests.index('def test_source_binding_and_console_stop_implementations_were_not_relaxed():')]
    (out/'test_reused_sync.py').write_text(tests)
    rehearsal = renamed((PARENT/'ros_rehearsal.py').read_text())
    rehearsal = rehearsal.replace('from mecanum_wand_4wsa_control.rotation_core import RotationSafetyConfig',
                                  'from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig')
    rehearsal = rehearsal.replace('phases=[];faults=[];calls=[];trace=[]',
                                  'phases=[];faults=[];calls=[];trace=[];unexpected_active_faults=[]')
    rehearsal = rehearsal.replace("                    if gate.core.fault and (not faults or faults[-1]!=gate.core.fault):faults.append(gate.core.fault)",
        "                    if gate.core.fault and (not faults or faults[-1]!=gate.core.fault):faults.append(gate.core.fault)\n"
        "                    if state.active is not None and state.injected is None and sup.core.state=='ACTIVE' and gate.core.fault:\n"
        "                        unexpected_active_faults.append(gate.core.fault)")
    rehearsal = rehearsal.replace("                assert not state.before_click_nonzero and state.output_count>0",
                                  "                assert not state.before_click_nonzero and state.output_count>0\n"
                                  "                assert not unexpected_active_faults,unexpected_active_faults")
    rehearsal = rehearsal.replace('gate_faults=faults,', 'gate_faults=faults,unexpected_active_faults=unexpected_active_faults,')
    (out/'ros_rehearsal.py').write_text(rehearsal)
    manifest = dict(identity='4WSA', authority_version=6, diagnostic_payload_bytes=204,
                    offline_passed=False, runtime_src=str(rt), control_package=CONTROL, hardware_package=HARDWARE,
                    host_session_s=600, independent_firmware_limit_ms=630000,
                    source_hashes=hashes(HERE), runtime_hashes=hashes(rt), firmware_source_hashes=hashes(fw),
                    protected_input_hashes=pins, firmware={}, flashed=False, motion_started=False,
                    original_entry_changed=False, network_changed=False,
                    current_entry=str(ROOT/'tools/start_car_tracking_300ms_ready.sh'))
    write(out/'build_manifest.json', manifest)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.output.resolve())
    print('Generated separate NOT_APPROVED 4WSA 600s candidate; no hardware access')
