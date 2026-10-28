#!/usr/bin/env python3
"""Bounded offline validation only; no real Vicon, Tenda or actuator IO."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from prepare import ROOT, HERE, CONTROL, HARDWARE, generate, sha, hashes, write


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    out = p.parse_args().output.resolve()
    m = generate(out)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')

    def run(name, args, environment=env, timeout=120):
        r = subprocess.run(list(map(str, args)), cwd=ROOT, env=environment,
                           capture_output=True, text=True, timeout=timeout)
        (out/(name+'.log')).write_text(r.stdout+r.stderr)
        print(name+': '+('PASS' if r.returncode == 0 else 'FAIL'), flush=True)
        if r.returncode:
            raise ValueError((r.stdout+r.stderr)[-10000:])
        return r.stdout.strip()

    fw, native, arm = out/'firmware', out/'native', out/'arm'
    run('native_configure', ['cmake', '-S', fw, '-B', native, '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug'])
    run('native_build', ['cmake', '--build', native, '-j2'])
    run('native_actual_controller_and630s', ['ctest', '--test-dir', native, '--output-on-failure'])
    run('native_yaw_ramps', [native/'test_rotation', native])
    run('arm_configure', ['cmake', '-S', fw, '-B', arm, '-G', 'Ninja', '-DSERVO_ARM=ON', '-DCMAKE_BUILD_TYPE=Debug'])
    run('arm_build', ['cmake', '--build', arm, '--target', 'mecanum_f407_4wsa_staged_NOT_FLASHED', '-j2'])
    image = arm/'mecanum_f407_4wsa_staged_NOT_FLASHED.bin'
    m['firmware'] = dict(path=str(image), sha256=sha(image), bytes=image.stat().st_size,
        identity='4WSA', authority_version=6, diagnostic_payload_bytes=204, independent_limit_ms=630000,
        flashed=False, physical_qualified=False, stages=list(range(7)), suspended_limit_ms=5000)
    write(out/'build_manifest.json', m)
    rt = Path(m['runtime_src'])
    testenv = dict(env, SYNC_OUTPUT=str(out), PYTHONPATH=os.pathsep.join([
        str(out), str(rt/CONTROL), str(rt/HARDWARE), str(ROOT/'src/mecanum_vicon_control'),
        os.environ.get('PYTHONPATH', '')]))
    tests = run('host_reused_yaw_and_new600s_tests', [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
        out/'test_reused_sync.py', HERE/'test_long_session.py', '--junitxml='+str(out/'tests.xml')], testenv)
    run('same_time_comparison', [sys.executable, '-c',
        'import json;from test_reused_sync import comparison;print(json.dumps(comparison()))'], testenv)
    comparison = json.loads((out/'same_time_comparison.log').read_text().strip().splitlines()[-1])
    write(out/'SAME_TIME_COMPARISON.json', comparison)
    esp = ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready/esp_project/main'
    source = (ROOT/'candidates/wand_sync_4ws9/test_esp_payload.c').read_text().replace('200', '204')
    (out/'test_esp_payload204.c').write_text(source)
    run('esp_protocol_build', ['cc', '-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-g',
        '-fsanitize=address,undefined', '-fno-pie', '-no-pie', '-I'+str(esp), '-I'+str(ROOT/'protocol'),
        out/'test_esp_payload204.c', esp/'mc_protocol.c', '-o', out/'test_esp_payload204'])
    run('preserved_esp_protocol204', [out/'test_esp_payload204'])
    run('private_ros_packages', ['colcon', '--log-base', out/'colcon_log', 'build', '--base-paths', rt,
        '--build-base', out/'runtime/build', '--install-base', out/'runtime/install', '--symlink-install',
        '--packages-select', CONTROL, HARDWARE])
    logs = out/'private_ros_logs'
    logs.mkdir()
    run('private_ros_yaw_fault_rehearsal', [sys.executable, out/'ros_rehearsal.py', '--runtime', rt,
        '--output', out/'REAL_ROS_REHEARSAL.json'], dict(testenv, ROS_DOMAIN_ID='187',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOG_DIR=str(logs)))
    for name, digest in m['protected_input_hashes'].items():
        if sha(name) != digest:
            raise ValueError('Protected baseline changed: '+name)
    m.update(offline_passed=True, runtime_hashes=hashes(rt), firmware_source_hashes=hashes(fw),
             source_hashes=hashes(HERE), original_entry_changed=False)
    write(out/'build_manifest.json', m)
    for stage in range(7):
        run('file_check_stage'+str(stage), [sys.executable, HERE/'cli.py', '--prepared', out, '--stage', stage, '--check'])
    blocked = subprocess.run([sys.executable, str(HERE/'cli.py'), '--prepared', str(out), '--launch'],
                             cwd=ROOT, env=testenv, capture_output=True, text=True)
    assert blocked.returncode == 2 and 'no ROS or socket opened' in blocked.stderr
    write(out/'PREPARATION_RESULT.json', dict(passed=True, identity='4WSA', host_session_s=600,
        independent_firmware_limit_ms=630000, tests=tests.strip().splitlines()[-1],
        native_actual_c_and_sanitizers=True, arm_built=True,
        real_ros_software_cases=6, same_time_model_comparison=comparison, firmware=m['firmware'],
        inherited_yaw_implementation='reused 4WS9', hardware_launch_entry_blocked=True,
        original_entry_changed=False, protected_files_unchanged=len(m['protected_input_hashes']),
        firmware_flashed=False, physical_motion_started=False, network_settings_changed=False,
        stopping_electrical_thermal_qualified=False, continuous600s_physical_qualified=False,
        hardware_startup_console_migration_pending=True))
    print(json.dumps(dict(passed=True, output=str(out), firmware=m['firmware']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
