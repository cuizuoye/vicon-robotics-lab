"""Independent source-intake repair; preserve every qualified predecessor."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
BASE = ROOT/'docs/wand_background_session_display_2026-11-02/prepared_v1/host'
PARENT = ROOT/'candidates/wand_background_session_display_20261102'
OUT = ROOT/'docs/wand_static_source_2026-11-02/prepared_v1'
RECORD = ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH = ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND = '4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT = 'STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
PACKAGE = 'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')

def hashes(folder):
    return {str(p):sha(p) for p in sorted(folder.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts
            and p.name != 'HOST_MANIFEST.json'}

def edit(value, old, new):
    assert value.count(old) == 1, old[:100]
    return value.replace(old, new)

def main():
    if OUT.exists():
        raise ValueError('never overwrite a prepared snapshot')
    m = json.loads((BASE/'HOST_MANIFEST.json').read_text())
    assert m['host_offline_passed'] and m['hardware_launch_enabled']
    pins = {str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for group in ('protected', 'source_hashes', 'artifact_hashes', 'evidence'):
        pins.update(m[group])
    for path, digest in pins.items():
        assert sha(path) == digest, path
    ready = OUT/'host'
    shutil.copytree(BASE, ready, ignore=shutil.ignore_patterns('sessions', 'HOST_MANIFEST.json', '__pycache__', '*.pyc'))
    for name in ('ground_entry.py', 'startup_entry.py', 'deployment_record.py', 'test_admission.py',
                 'stop_command.py', 'launcher.py', 'launcher.js', 'launcher.html', 'launcher.css'):
        assert not (HERE/name).exists()
        (HERE/name).write_text((PARENT/name).read_text().replace(
            'docs/wand_background_session_display_2026-11-02/prepared_v1/host',
            'docs/wand_static_source_2026-11-02/prepared_v1/host'))
    shutil.copyfile(HERE/'deployment_record.py', ready/PACKAGE/'deployment_record.py')
    shutil.copyfile(HERE/'source_pipeline.py', ready/PACKAGE/'source_pipeline.py')
    value = (ready/PACKAGE/'source_relay.py').read_text()
    begin = value.index('        node = RotationRawSource(context)')
    end = value.index('    except Exception as error:', begin)
    value = value[:begin]+'''        from .source_pipeline import BufferedRawSource, passive_loop
        node = BufferedRawSource(context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        passive_loop(node, executor, channel, stop, ready)
'''+value[end:]
    value = edit(value, '        self.ipc_rejected = 0', '        self.ipc_rejected = 0\n        self.discarded_ipc = 0')
    begin = value.index('    def poll(self):')
    end = value.index('    def close(self):', begin)
    value = value[:begin]+'''    def poll(self):
        from .source_pipeline import poll_latest
        return poll_latest(self)

'''+value[end:]
    value = edit(value, "        errors = {}", "        errors = {}\n        pipeline = {}\n        validated_cached_reuse = False")
    value = edit(value, "                conflicts, errors = value['conflicts'], dict(value['rejected'])",
                 "                conflicts, errors = value['conflicts'], dict(value['rejected'])\n                pipeline = dict(value.get('pipeline', {}))")
    value = edit(value, '                            p = self.cached_packet\n',
                 '                            p = self.cached_packet\n                            validated_cached_reuse = True\n')
    value = edit(value, '        except (ValueError, TypeError, KeyError, AttributeError):\n            self.rejected += 1',
                 '''        except (ValueError, TypeError, KeyError, AttributeError) as error:
            if str(error) != 'SIGN_PACKET_AGE_OR_MIXED_FRAME' or not validated_cached_reuse:
                self.rejected += 1
            else:
                # A once-validated immutable relay cache merely expired.
                # Do not manufacture a geometry rejection; unchanged input
                # watchdogs still stop if no fresh source arrives in 200ms.
                pipeline['cached_packet_expired'] = True''')
    value = edit(value, 'rejected=errors,conflicts=conflicts,passive_process=True)',
                 'rejected=errors,conflicts=conflicts,passive_process=True,pipeline=pipeline)')
    (ready/PACKAGE/'source_relay.py').write_text(value)
    value = (ready/PACKAGE/'optimized_nodes.py').read_text()
    value = edit(value, 'import os\n', 'import os\nfrom .source_pipeline import AsyncGraphMixin\n')
    value = edit(value, 'class Supervisor(RotationSupervisorNode):', 'class Supervisor(AsyncGraphMixin, RotationSupervisorNode):')
    value = edit(value, 'class Gate(BoundGateMixin, RotationGateNode):', 'class Gate(AsyncGraphMixin, BoundGateMixin, RotationGateNode):')
    value = edit(value, "        # Nonzero duplicate only; every ZERO and revocation always passes.\n", """        # Coalesce only a duplicate already-zero ACTIVE permit for the SAME
        # validated frame. Every transition to zero, new frame, revocation,
        # STOP/ESTOP and fault bypasses this optimization. Watchdogs still tick.
""")
    value = edit(value, "        if (output != ZERO and not self.core.fault and not self.core.hard_fault\n",
        """        duplicate_active_zero = bool(output == ZERO and getattr(self, '_last_output', None) == ZERO
            and f and f.state == 'ACTIVE' and f.enabled and f.epoch
            and f.epoch == self.core.primed_epoch)
        if ((output != ZERO or duplicate_active_zero) and not self.core.fault and not self.core.hard_fault
""")
    value = edit(value, '        self._last_cid, self._last_emitted = cid, now',
        '        self._last_cid, self._last_emitted = cid, now\n        self._last_output = output')
    value = edit(value, "and now-getattr(self, '_last_emitted', -float('inf')) < .008):",
        "and now-getattr(self, '_last_emitted', -float('inf')) < (.020 if duplicate_active_zero else .008)):")
    (ready/PACKAGE/'optimized_nodes.py').write_text(value)
    value = (ready/'hardware_runtime.py').read_text()
    value = edit(value, 'self.status={};self.gate={};self.transport={};', 'self.status={};self.gate={};self.transport={};self.source={};')
    value = edit(value, "('transport_status',self.transport)):", "('transport_status',self.transport),('source_status',self.source)):")
    value = edit(value, 'status=self.status,gate=self.gate,transport=self.transport,physical_output=self.physical)',
                 'status=self.status,gate=self.gate,transport=self.transport,source=self.source,physical_output=self.physical)')
    (ready/'hardware_runtime.py').write_text(value)
    changed = {str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file()
               and (not (BASE/p.relative_to(ready)).exists() or sha(p) != sha(BASE/p.relative_to(ready)))}
    m.update(host_offline_passed=False, hardware_launch_enabled=False, protected=pins,
        source_hashes=hashes(HERE), artifact_hashes=hashes(ready), evidence={},
        direct_changed_runtime_files=sorted(changed),
        changed_runtime_files=sorted(set(m['changed_runtime_files']) | changed),
        inherited_runtime_hashes={p:d for p,d in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==d},
        inherited_parent_runtime_hashes={str(Path(p).relative_to(BASE)):d for p,d in m['artifact_hashes'].items()
            if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed},
        presentation_revision='BACKGROUND_STATIC_SOURCE_V3')
    m.pop('qualification', None)
    save(ready/'HOST_MANIFEST.json', m)
    print(ready)

if __name__ == '__main__':
    main()
