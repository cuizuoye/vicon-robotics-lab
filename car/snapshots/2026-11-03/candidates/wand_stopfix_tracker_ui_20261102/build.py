"""Freeze a display-only Vicon world-grid fork; never accesses hardware."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
BASE = ROOT / 'docs/wand_stopfix_demo_ui_2026-11-02/prepared_v1/host'
OUT = ROOT / 'docs/wand_stopfix_tracker_ui_2026-11-02/prepared_v1'
PARENT = ROOT / 'candidates/wand_stopfix_demo_ui_20261102'
RECORD = ROOT / 'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH = ROOT / 'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND = '4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT = 'STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
WEB_PATCHES = {
    'web/index.html': [
        ('<title>Sign 1:1直接跟随 · 4WSC独立实机入口</title>', '<title>Sign遥操作 · Vicon世界坐标</title>'),
        ('蓝：小车反馈　橙：Sign的1:1位置与朝向', '蓝：car01　橙：Sign 1:1映射目标'),
        ('aria-label="小车与Sign目标轨迹"', 'aria-label="Vicon世界XY坐标：小车与Sign映射目标"'),
    ],
    'web/style.css': [
        ('height:360px;background:white;border:1px solid #dce4ec;',
         'height:clamp(360px,calc(100vh - 360px),760px);background:#242b32;border:1px solid #586573;'),
    ],
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def hashes(folder):
    return {str(p): sha(p) for p in sorted(folder.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}


def edit(path, old, new):
    value = path.read_text()
    if value.count(old) != 1:
        raise ValueError('unexpected parent layout ' + str(path) + ': ' + old[:60])
    path.write_text(value.replace(old, new))


def client_source():
    value = (BASE / 'web/app.js').read_text()
    prefix, render = value.split('function render(now){', 1)
    preamble = render.split("  const canvas=$('map')", 1)[0]
    return (prefix + (HERE / 'renderer.js').read_text() + '\nconst trackerCamera={halfY:3};\n'
            + 'function render(now){' + preamble
            + "  const canvas=$('map'),box=canvas.getBoundingClientRect(),w=box.width,h=box.height;\n"
            + '  if(canvas.width!==Math.round(w)||canvas.height!==Math.round(h)){canvas.width=Math.round(w);canvas.height=Math.round(h);}\n'
            + "  const ctx=canvas.getContext('2d');ctx.setTransform(1,0,0,1,0,0);\n"
            + '  TrackerWorldView.draw(ctx,w,h,c,g,trace,trackerCamera);\n}\n'
            + 'requestAnimationFrame(render);connect();\n')


def main():
    if OUT.exists():
        raise ValueError('never overwrite a prepared snapshot')
    manifest = json.loads((BASE / 'HOST_MANIFEST.json').read_text())
    assert manifest['host_offline_passed'] and manifest['hardware_launch_enabled']
    pins = {str(BASE / 'HOST_MANIFEST.json'): sha(BASE / 'HOST_MANIFEST.json')}
    for group in ('protected', 'source_hashes', 'artifact_hashes', 'evidence'):
        pins.update(manifest[group])
    for path, digest in pins.items():
        assert sha(path) == digest, 'parent drift ' + path
    ready = OUT / 'host'
    shutil.copytree(BASE, ready, ignore=shutil.ignore_patterns('sessions', 'HOST_MANIFEST.json', '__pycache__', '*.pyc'))
    for name in ('ground_entry.py', 'deployment_record.py', 'test_admission.py', 'test_ui.cjs'):
        if (HERE / name).exists():
            raise ValueError('do not overwrite source ' + name)
        shutil.copyfile(PARENT / name, HERE / name)
    edit(HERE / 'deployment_record.py',
         "HOST=ROOT/'docs/wand_stopfix_demo_ui_2026-11-02/prepared_v1/host'",
         "HOST=ROOT/'docs/wand_stopfix_tracker_ui_2026-11-02/prepared_v1/host'")
    shutil.copyfile(HERE / 'deployment_record.py', ready / 'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py')
    for relative, changes in WEB_PATCHES.items():
        for old, new in changes:
            edit(ready / relative, old, new)
    (ready / 'web/app.js').write_text(client_source())
    changed = {str(p.relative_to(ready)) for p in ready.rglob('*')
               if p.is_file() and sha(p) != sha(BASE / p.relative_to(ready))}
    expected = set(WEB_PATCHES) | {'web/app.js', 'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py'}
    assert changed == expected, changed
    inherited = {p: digest for p, digest in manifest['inherited_runtime_hashes'].items()
                 if (ready / p).exists() and sha(ready / p) == digest}
    direct = {str(Path(p).relative_to(BASE)): digest for p, digest in manifest['artifact_hashes'].items()
              if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed}
    manifest.update(host_offline_passed=False, hardware_launch_enabled=False, protected=pins,
                    source_hashes=hashes(HERE), artifact_hashes=hashes(ready), evidence={},
                    inherited_runtime_hashes=inherited, inherited_parent_runtime_hashes=direct,
                    changed_runtime_files=sorted(changed), parent_qualification=str(BASE.parent / 'QUALIFICATION.json'),
                    presentation_revision='TRACKER_WORLD_UI_V1')
    manifest.pop('qualification', None)
    save(ready / 'HOST_MANIFEST.json', manifest)
    save(OUT / 'BUILD.json', dict(passed=True, changed_runtime_files=sorted(changed),
         renderer_only=True, hardware_socket_opened=False, motion_started=False,
         world_frame='map', receiver_map_xyz=[0, 0, 0], receiver_map_rpy=[0, 0, 0],
         orange_is_mapped_target_not_raw_Sign_world_position=True))
    print(ready)


if __name__ == '__main__':
    main()
