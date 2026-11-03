"""Presentation-only fork of the qualified STOPFIX host. Never opens hardware."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
BASE = ROOT / 'docs/wand_stopfix_feedback_2026-11-02/prepared_v1/host'
OUT = ROOT / 'docs/wand_stopfix_demo_ui_2026-11-02/prepared_v1'
PARENT = ROOT / 'candidates/wand_stopfix_feedback_fix_20261102'
RECORD = ROOT / 'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH = ROOT / 'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND = '4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT = 'STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'

# Exact substitutions keep the entire transport/input/authority/control logic
# unchanged. Hidden diagnostic rows remain available for fault messages.
UI_PATCHES = {
    'web/index.html': [
        ('STOPFIX_V1 · 档6 · 最长600秒受控试运行 · 停车/电流/温升未量化', 'STOPFIX_V1 · 档6'),
        ('（无中间参考点、无电子围栏）', ''),
        ('<aside><h2>连接与安全状态</h2><p id="connection">', '<aside><p id="connection" hidden>'),
        ('<p id="timing">', '<p id="timing" hidden>'),
        ('仅明确审批的硬件模式可输出；准备不转轮。窗口失焦、卡顿、断连仍终止；不会自动恢复。',
         '自动READY后单击空格开始；再次空格 / Esc停止。'),
    ],
    'web/app.js': [
        ("function text(id,value){if($(id).textContent!==value)$(id).textContent=value;}",
         "function text(id,value){if($(id).textContent!==value)$(id).textContent=value;\n"
         "  if(id==='connection')$(id).hidden=!/缺少令牌|停止|终止|保护/.test(value);\n}"),
        ('4WSC · 档6 · 600秒受控试运行 ≤600秒 · 停车/电流/温升未量化 · 1:1平移＋旋转 · 参数以本轮合同为准',
         'Sign · 1:1平移＋旋转跟随 · 参数以本轮配置为准'),
        ('本轮合同模式：自动READY后单击空格开始；再次空格/Esc停止；松键不停。失焦、遮挡、断连仍停车。',
         '自动READY后单击空格开始；再次空格 / Esc停止。'),
        ('档6 · 600秒受控试运行（停车/电流/温升未量化） · 档${v.stage}', '档${v.stage}'),
        ("  ctx.fillStyle='#a23925';ctx.font='18px Microsoft YaHei,sans-serif';ctx.fillText('视图不是围栏；Vicon可见 ≠ 周围无障碍',12,26);\n", ''),
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


def replace_once(path, old, new):
    value = path.read_text()
    if value.count(old) != 1:
        raise ValueError('unexpected parent layout: ' + str(path) + ' / ' + old[:60])
    path.write_text(value.replace(old, new))


def main():
    if OUT.exists():
        raise ValueError('never overwrite an existing snapshot')
    parent = json.loads((BASE / 'HOST_MANIFEST.json').read_text())
    assert parent['host_offline_passed'] and parent['hardware_launch_enabled']
    pins = {str(BASE / 'HOST_MANIFEST.json'): sha(BASE / 'HOST_MANIFEST.json')}
    for group in ('protected', 'source_hashes', 'artifact_hashes', 'evidence'):
        pins.update(parent[group])
    for path, digest in pins.items():
        assert sha(path) == digest, 'parent drift: ' + path
    ready = OUT / 'host'
    shutil.copytree(BASE, ready, ignore=shutil.ignore_patterns('sessions', 'HOST_MANIFEST.json', '__pycache__', '*.pyc'))
    for name in ('ground_entry.py', 'deployment_record.py', 'test_admission.py'):
        if (HERE / name).exists():
            raise ValueError('candidate source already exists: ' + name)
        shutil.copyfile(PARENT / name, HERE / name)
    replace_once(HERE / 'deployment_record.py',
                 "HOST=ROOT/'docs/wand_stopfix_feedback_2026-11-02/prepared_v1/host'",
                 "HOST=ROOT/'docs/wand_stopfix_demo_ui_2026-11-02/prepared_v1/host'")
    shutil.copyfile(HERE / 'deployment_record.py', ready / 'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py')
    # The changed renderer is audited by exact UI_PATCHES equivalence instead of
    # pretending it is byte-identical to the parent renderer.
    replace_once(HERE / 'test_admission.py',
                 "'control_channel.py','web/app.js','web/input_guard.js'",
                 "'control_channel.py','web/input_guard.js'")
    for relative, changes in UI_PATCHES.items():
        for old, new in changes:
            replace_once(ready / relative, old, new)
    changed = {str(p.relative_to(ready)) for p in ready.rglob('*')
               if p.is_file() and sha(p) != sha(BASE / p.relative_to(ready))}
    expected = set(UI_PATCHES) | {'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py'}
    assert changed == expected, changed
    inherited = {p: digest for p, digest in parent['inherited_runtime_hashes'].items()
                 if (ready / p).exists() and sha(ready / p) == digest}
    direct = {str(Path(p).relative_to(BASE)): digest for p, digest in parent['artifact_hashes'].items()
              if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed}
    manifest = dict(parent)
    manifest.update(host_offline_passed=False, hardware_launch_enabled=False,
                    protected=pins, source_hashes=hashes(HERE), artifact_hashes=hashes(ready), evidence={},
                    inherited_runtime_hashes=inherited, inherited_parent_runtime_hashes=direct,
                    changed_runtime_files=sorted(changed), parent_qualification=str(BASE.parent / 'QUALIFICATION.json'),
                    presentation_revision='DEMO_UI_V1')
    manifest.pop('qualification', None)
    save(ready / 'HOST_MANIFEST.json', manifest)
    save(OUT / 'BUILD.json', dict(passed=True, presentation_only=True,
         changed_runtime_files=sorted(changed), firmware_changed=False,
         network_changed=False, hardware_socket_opened=False, motion_started=False))
    print(ready)


if __name__ == '__main__':
    main()
