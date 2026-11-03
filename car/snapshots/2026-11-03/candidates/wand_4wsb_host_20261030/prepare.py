"""Build a private host overlay; NEVER program, connect or initialize ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
PARENT = ROOT/'docs/wand_sync_600s_2026-10-28/prepared'
FW = ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready'
REPORT = ROOT/'docs/wand_4wsb_host_2026-10-30'
CONTROL = 'mecanum_wand_4wsb_control'
HARDWARE = 'mecanum_wand_4wsb_hardware'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def hashes(folder):
    return {str(p): sha(p) for p in sorted(Path(folder).rglob('*')) if p.is_file()
            and not any(k in ('__pycache__', '.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}


def edit(p, old, new, count=1):
    s = p.read_text()
    if s.count(old) != count:
        raise ValueError('pinned layout drift: '+str(p)+' '+old[:100])
    p.write_text(s.replace(old, new))


def rename(s):
    return (s.replace('4wsa', '4wsb').replace('4WSA', '4WSB')
            .replace('0x7A00', '0x7B00').replace('0x7A80', '0x7B80')
            .replace('_V6', '_V7').replace('_v6', '_v7'))


def generate(out):
    if out.exists():
        raise ValueError('new output directory required; never overwrite a release')
    parent = json.loads((PARENT/'build_manifest.json').read_text())
    firmware = json.loads((FW/'build_manifest.json').read_text())
    if not parent['offline_passed'] or firmware['identity'] != '4WSB' or not firmware['delivery_complete']:
        raise ValueError('verified parent and firmware package required')
    pins = dict(firmware['protected_input_hashes'])
    for group in ('runtime_hashes', 'source_hashes', 'firmware_source_hashes', 'protected_input_hashes'):
        pins.update(parent[group])
    for p in (PARENT/'build_manifest.json', FW/'build_manifest.json', FW/'DELIVERY_RESULT.json'):
        pins[str(p)] = sha(p)
    reused = [ROOT/'candidates/wand_ros_bound_history'/n for n in ('bound_history.py', 'bound_rotation.py')]
    reused += [ROOT/'candidates/wand_latency_fast_follow_20261029/catchup.py',
               ROOT/'candidates/wand_sync_tuning_20261029/packet_work.py']
    pins.update({str(p): sha(p) for p in reused})
    for p, digest in pins.items():
        if sha(p) != digest:
            raise ValueError('protected input drift: '+p)
    out.mkdir(parents=True)
    (out/'COLCON_IGNORE').touch()
    save(out/'PROTECTED_BEFORE.json', pins)
    rt = out/'runtime/src'
    shutil.copytree(PARENT/'runtime/src', rt,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.egg-info'))
    for p in rt.rglob('*'):
        if p.is_file():
            p.write_text(rename(p.read_text()))
    for p in sorted(rt.rglob('*'), key=lambda x: len(x.parts), reverse=True):
        if rename(p.name) != p.name:
            p.rename(p.with_name(rename(p.name)))
    c, h = rt/CONTROL/CONTROL, rt/HARDWARE/HARDWARE
    stages = (ROOT/'candidates/wand_sync_4wsb_20261029/stages.py').read_text()
    stages += "\nSCOPE = 'HARDWARE_4WSB_V7_600S_XY_YAW_STAGE_'\n"
    for p in (c/'stages.py', h/'stages.py'):
        p.write_text(stages)
    contract = (FW/'fourwsb_wire/contract.py').read_text()
    contract = contract.replace('class ProtocolError(ValueError):\n    """Offline decoder only; no live transport is constructed."""\n    pass',
                                'from mecanum_vicon_control.transport_protocol import ProtocolError')
    (h/'stage0_contract.py').write_text(contract)
    for name in ('commissioning.py', 'authorization.py'):
        p = h/name
        s = p.read_text().replace('range(7)', 'range(8)')
        s = s.replace('!=204', '!=208').replace('!= 204', '!= 208').replace('= 204', '= 208')
        s = s.replace('!=6', '!=7').replace('!= 6:', '!= 7:')
        s = s.replace('204-byte v6', '208-byte v7')
        p.write_text(s)
    edit(h/'commissioning.py', "DOMAIN, NAMESPACE, WORLD = 122, '/wand_4wsb_long_sync', 'map'",
         "DOMAIN, NAMESPACE, WORLD = 124, '/wand_4wsb_sync', 'map'")
    edit(c/'rotation_receiver_guard.py', "('0','122')", "('0','124')")
    edit(h/'rotation_nodes.py',
         "or (f.state=='STANDBY' and (v['sign_digest'] is None or v['car_stamp'] is None)))):",
         "or (f.state=='STANDBY' and f.command==ZERO))):")
    edit(h/'commissioning.py', "        states = ('STANDBY', 'ARMED', 'ACTIVE', 'STOPPING', 'STOP_UNCONFIRMED', 'PAUSED', 'FAULT')",
         "        from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm\n"
         "        if max(map(abs, wheel_rpm(command))) > self.limits.target_rpm+1e-6:\n"
         "            raise ValueError('signed output exceeds coupled wheel RPM envelope')\n"
         "        states = ('STANDBY', 'ARMED', 'ACTIVE', 'STOPPING', 'STOP_UNCONFIRMED', 'PAUSED', 'FAULT')")
    edit(h/'commissioning.py','    def decode(self, text, steady, ros):',
         '    def decode(self, text, steady, ros, *, ignore_expired_disabled=False):')
    edit(h/'commissioning.py',
         "        if not fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']), steady, ros, .10):\n            raise ValueError('authority expired')",
         "        fresh = fresh_envelope(finite_number(p['issued_steady']), finite_number(p['issued_ros']), steady, ros, .10)")
    edit(h/'commissioning.py',"        if self.owner is None:\n            if p['permit'] or command != ZERO: raise ValueError('initial disabled baseline required')",
         "        if not fresh:\n"
         "            from .cold_authority import discardable_zero\n"
         "            if ignore_expired_disabled and discardable_zero(self,p,command,steady,ros):return None\n"
         "            raise ValueError('authority expired')\n"
         "        if self.owner is None:\n            if p['permit'] or command != ZERO: raise ValueError('initial disabled baseline required')")
    edit(h/'commissioning.py','            p, command = self.verifier.decode(text, steady, ros)',
         '            from .cold_authority import virgin_transport\n'
         '            decoded = self.verifier.decode(text, steady, ros, ignore_expired_disabled=virgin_transport(self))\n'
         '            if decoded is None:return False\n'
         '            p, command = decoded')
    # Strict temporal policy is reused from the current 300ms feedback host,
    # not silently reset to the older parent's 200ms transport variant.
    timeout = "FEEDBACK_TIMEOUT_S = .300\n"
    (h/'feedback_timeout.py').write_text(timeout)
    edit(h/'startup_clock.py', 'if not -.02 <= now-result <= .20:',
         'if not -.02 <= now-result <= .30:')
    edit(h/'startup_clock.py', 'k in self.accepted_at and 0 <= now - self.accepted_at[k] <= .20',
         'k in self.accepted_at and 0 <= now - self.accepted_at[k] <= .30')
    edit(h/'nodes.py', 'v is not None and 0 <= now-v <= .20 for v in',
         'v is not None and 0 <= now-v <= .30 for v in')
    # Reuse separately tested source retention and FIFO validation verbatim
    # except package names. No latest-source fallback, timestamp renewal or
    # command/lease/watchdog extension is permitted.
    (c/'bound_history.py').write_text((reused[0]).read_text())
    (h/'bound_rotation.py').write_text(reused[1].read_text().replace('4ws8', '4wsb'))
    servo = reused[2].read_text().replace('from common import RotationSession', 'from .rotation_servo import RotationSession')
    servo = servo.replace('from mecanum_wand_4ws8_control.core import limit_vector', 'from .core import limit_vector')
    servo = servo.replace('SIM ONLY catch-up candidate, not a hardware authority or launch interface.',
                          'Private 4WSB catch-up port; all outputs still cross the independent gate.')
    (c/'fast_catchup.py').write_text(servo)
    edit(c/'rotation_core.py', '        self.follower=RotationSession(c.follower_config(),c.project_goal,c.project_reference)',
         '        from .fast_catchup import FastCatchupSession\n'
         '        self.follower=FastCatchupSession(c.follower_config(),c.project_goal,c.project_reference)')
    for sub, destination in (('control', c), ('hardware', h)):
        source = HERE/sub
        if source.exists():
            for p in source.glob('*.py'):
                shutil.copyfile(p, destination/p.name)
    # Real entrypoints exclusively use the new validated optimized adapters.
    setup = rt/HARDWARE/'setup.py'
    setup.write_text(setup.read_text().replace('.rotation_nodes:', '.optimized_nodes:'))
    for launch in (rt/HARDWARE/'launch').glob('*.py'):
        s = launch.read_text().replace("'link_receive_timeout_s': .20", "'link_receive_timeout_s': .30")
        launch.write_text(s)
    for i, stage in enumerate(json.loads((FW/'stages.json').read_text())):
        template = rt/HARDWARE/f'config/stage{min(i,6)}.NOT_APPROVED.json'
        v = json.loads(template.read_text())
        v.update(stage=i, staged_runtime_approval={}, long_session_qualified=False)
        v['safety'].update(max_speed_mps=stage['speed'], max_accel_mps2=stage['accel'],
            max_yaw_rate_radps=stage['yaw_rate'], max_yaw_accel_radps2=stage['yaw_accel'])
        v['hardware'].update(required_motion_tag='4WSB', output_authority_version=7,
            diagnostic_payload_bytes=208, pwm_cap_permille=stage['duty'],
            motor_target_rpm_cap=stage['target_rpm'], wheel_slew_guard_cps_s=stage['wheel_slew_guard'])
        v['trial_review'].update(authorized_stage=i, human_authorized=False, image_sha256=None)
        v['evidence']['change_scope'] = '4WSB v7 PC host offline port; physical admission NOT approved'
        save(rt/HARDWARE/f'config/stage{i}.NOT_APPROVED.json', v)
    manifest = dict(identity='4WSB', authority_version=7, diagnostic_payload_bytes=208,
        firmware=dict(path=firmware['firmware']['bin']['path'], sha256=firmware['firmware']['bin']['sha256'],
                      identity='4WSB', authority_version=7, diagnostic_payload_bytes=208),
        runtime_src=str(rt), control_package=CONTROL, hardware_package=HARDWARE,
        protected=pins, runtime_hashes=hashes(rt), source_hashes=hashes(HERE),
        host_offline_passed=False, hardware_admitted=False, flashed=False,
        physical_qualified=False, network_changed=False, firmware_build_unchanged=True)
    save(out/'HOST_MANIFEST.json', manifest)
    print('PRIVATE_4WSB_HOST_GENERATED '+str(out), flush=True)
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    generate(p.parse_args().output.resolve())
