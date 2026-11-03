"""Offline-only world-view, client safety and isolated loopback regression."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT, HERE, BASE, OUT, WEB_PATCHES, client_source, sha, save, hashes
from ground_entry import check, profile


def main():
    ready = OUT / 'host'
    manifest = check(ready, release=False)
    if manifest['host_offline_passed']:
        raise ValueError('never overwrite qualified snapshots')
    folder = Path(tempfile.mkdtemp(prefix='qualification_', dir=OUT))
    evidence = {}
    env = dict(os.environ, PYTHONPATH=str(HERE) + os.pathsep + os.environ.get('PYTHONPATH', ''), PYTHONDONTWRITEBYTECODE='1')

    def run(name, args, environ=env):
        result = subprocess.run(args, env=environ, capture_output=True, text=True, timeout=100)
        for suffix, value in (('stdout', result.stdout), ('stderr', result.stderr)):
            target = folder / (name + '.' + suffix + '.log')
            target.write_text(value)
            evidence[str(target)] = sha(target)
        print(name + ': ' + str(result.returncode), flush=True)
        if result.returncode:
            raise ValueError(result.stdout[-2000:] + result.stderr[-6000:])
        return result.stdout

    assert (ready / 'web/app.js').read_text() == client_source()
    # The entire pre-render client (focus, deadline, input, lease, WebSocket,
    # stops, auto-prepare) is byte-identical to the qualified parent.
    parent_prefix = (BASE / 'web/app.js').read_text().split('function render(now){', 1)[0]
    assert (ready / 'web/app.js').read_text().startswith(parent_prefix)
    for relative, changes in WEB_PATCHES.items():
        value = (BASE / relative).read_text()
        for old, new in changes:
            assert value.count(old) == 1
            value = value.replace(old, new)
        assert value == (ready / relative).read_text()
    for relative, digest in manifest['inherited_parent_runtime_hashes'].items():
        assert sha(ready / relative) == digest, relative
    for name in ('receiver.py', 'idle_probe.py', 'server.cjs', 'worker.py', 'ground_runtime.py', 'hardware_runtime.py',
                 'browser_operator.py', 'control_channel.py', 'web/input_guard.js', 'web/focus_evidence.js'):
        assert sha(ready / name) == sha(BASE / name), name
    unit = json.loads(run('admission', [sys.executable, '-B', str(HERE / 'test_admission.py')]).splitlines()[-1])
    assert unit['passed'] and unit['total_tests'] == 65
    world = json.loads(run('world_projection', ['node', str(HERE / 'test_world.cjs'), str(folder / 'world/RESULT.json')]).splitlines()[-1])
    assert world['passed']
    run('focus_client', ['node', str(ROOT / 'candidates/wand_stopfix_20261102/test_focus.cjs'), str(ready), str(folder / 'focus_client')])
    dom = json.loads((folder / 'focus_client/RESULT.json').read_text())
    assert dom['passed'] and dom['checks'] == 24
    visual = json.loads(run('headless_ui', ['node', str(HERE / 'test_ui.cjs'), str(ready), str(folder / 'headless_ui')]).splitlines()[-1])
    assert visual['passed'] and not visual['hardware_socket_opened']
    cases = []
    for case in ('no_start', 'normal', 'esc', 'second_space', 'blur', 'proof_timeout'):
        target = folder / case
        target.mkdir()
        path, consent = profile(ready, target, True)
        e = dict(env, WAND_4WSC_MODE='rehearsal', WAND_4WSC_PROFILE=str(path), WAND_PROFILE_APPROVAL=sha(path),
                 WAND_HARDWARE_CONSENT=consent, WAND_4WSC_PURPOSE='sign', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        run('browser_' + case, ['node', str(ready / 'tests/browser_peer.cjs'), str(ready), str(target / 'RESULT.json'), case, '5'], e)
        value = json.loads((target / 'RESULT.json').read_text())
        assert value['passed'] and not value['physical_output'] and value['stop_confirmed']
        cases.append(value)
        for item in Path(value['session']).rglob('*'):
            if item.is_file() and item.suffix in ('.json', '.jsonl', '.log'):
                evidence[str(item)] = sha(item)
    for item in folder.rglob('*'):
        if item.is_file():
            evidence[str(item)] = sha(item)
    parent = json.loads((BASE.parent / 'QUALIFICATION.json').read_text())
    report = dict(passed=True, presentation_revision='TRACKER_WORLD_UI_V1', change_scope='canvas renderer and static presentation only; frozen path rebound',
                  parent_qualification=str(BASE.parent / 'QUALIFICATION.json'), unit_tests=65, world_projection_checks=world['checks'],
                  DOM_VM_checks=24, headless_browser_checks=visual['checks'], browser_protocol_cases_rerun=len(cases), browser_cases=cases,
                  actual_hardware_io_this_revision=False, physical_motion_started=False, firmware_changed=False,
                  network_settings_changed=False, physical_qualified=False, unchanged_control_and_guard_files=True,
                  pre_render_client_byte_identical=True, world_grid_origin=[0, 0], world_frame='map',
                  receiver_identity_map_transform_preserved=True, coordinates_in_meters=True,
                  orange_is_mapped_target_not_raw_Sign_world_position=True,
                  fake_cameras_added=False, coverage_or_obstacle_boundary_claimed=False,
                  inherited_native_100ms_cases=parent['inherited_native_100ms_cases'],
                  inherited_real_zero_only_check=parent['inherited_real_zero_only_check'],
                  inherited_checks_are_not_new_measurements=True,
                  screenshot_1280=str(folder / 'headless_ui/demo_1280x720.png'),
                  screenshot_1920=str(folder / 'headless_ui/demo_1920x1080.png'))
    manifest.update(host_offline_passed=True, hardware_launch_enabled=True, evidence=evidence, source_hashes=hashes(HERE))
    wrapper = ROOT / 'tools/start_car_tracker.sh'
    manifest['source_hashes'][str(wrapper)] = sha(wrapper)
    save(ready / 'HOST_MANIFEST.json', manifest)
    try:
        post = json.loads(run('post_qualified', [sys.executable, '-B', str(HERE / 'test_admission.py'), '--post-qualified']).splitlines()[-1])
        assert post['passed'] and not post['hardware_opened']
        target = OUT / 'QUALIFICATION.json'
        save(target, report)
        evidence[str(target)] = sha(target)
        manifest['evidence'] = evidence
        manifest['qualification'] = str(target)
        save(ready / 'HOST_MANIFEST.json', manifest)
        check(ready)
        save(OUT / 'LATEST_PREPARED.json', dict(prepared=str(ready), manifest_sha256=sha(ready / 'HOST_MANIFEST.json'), qualification=str(target)))
    except BaseException:
        manifest.update(host_offline_passed=False, hardware_launch_enabled=False)
        save(ready / 'HOST_MANIFEST.json', manifest)
        raise
    print('TRACKER_UI_QUALIFIED ' + str(target), flush=True)


if __name__ == '__main__':
    main()
