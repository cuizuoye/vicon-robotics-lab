#!/usr/bin/env python3
"""Private host-only feedback-order/stop fix. Never flash or open devices."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT/'docs/wand_tracking_autostart_2026-10-27/prepared'
spec = importlib.util.spec_from_file_location('autostart_prepare_helpers',
    ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
sha, hashes, write, edit, check = (getattr(helpers, n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m = check(OLD/'build_manifest.json')
    if not m.get('autostart_passed'):
        raise ValueError('accepted automatic frontend required')
    protected = {str(OLD/'build_manifest.json'): sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes', 'source_hashes', 'generated_firmware_hashes', 'protected_input_hashes'):
        protected.update(m[group])
    out.mkdir(parents=True, exist_ok=False)
    out.chmod(0o700)
    (out/'COLCON_IGNORE').touch()
    runtime = out/'runtime/src'
    shutil.copytree(m['runtime_src'], runtime,
        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.egg-info'))
    h = runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'kernel_ingress.py', h/'kernel_ingress.py')
    clock = h/'startup_clock.py'
    edit(clock, "        elif f.message_type != MessageType.ESTOP:\n            if f.session_id != self.clock.session_id or not self.clock.qualified or self.clock.fault:",
        """        elif (f.message_type == MessageType.CONTROL_ENABLE
              and f.flags in (0, int(FrameFlags.ACK_REQUIRED))
              and f.payload == struct.pack('<BBH', 0, 0, 0)):
            # A stop is not motion authority. Freshness/faults must NEVER block
            # this exact same-qualified-session disable. No clear/select/enable.
            if (not self.clock.qualified or f.session_id != self.clock.session_id
                    or f.session_id != self.session_id()):
                raise OSError('disable requires the same qualified session')
        elif f.message_type != MessageType.ESTOP:
            if f.session_id != self.clock.session_id or not self.clock.qualified or self.clock.fault:""")
    edit(clock, '                if len(f.payload) != 4:',
        '                if f.flags not in (0, int(FrameFlags.ACK_REQUIRED)) or len(f.payload) != 4:')
    edit(clock, '                            startup_clock=self.clock.snapshot(now)))',
        "                            kernel_receive=getattr(self.socket, 'last_receive', None),\n"
        '                            startup_clock=self.clock.snapshot(now)))')
    n = h/'nodes.py'
    edit(n, 'from .receive_event import ReceiveEventPump',
        'from .receive_event import ReceiveEventPump\nfrom .kernel_ingress import KernelIngressSocket')
    edit(n, '        self.sock = ProgressSocket(self.sock, self.esp_ip, lambda: self.session.session_id,',
        '        self.ingress = KernelIngressSocket(self.sock)\n'
        '        self._receive_in_progress = False\n'
        '        self._first_feedback_trip_recorded = False\n'
        '        self.sock = ProgressSocket(self.ingress, self.esp_ip, lambda: self.session.session_id,')
    edit(n, '    def safety_tick(self):\n        # A failed worker/guard',
        '    def safety_tick(self):\n'
        '        # Drain a bounded batch BEFORE judging freshness, including timer\n'
        '        # and authenticated-command callbacks. Source stamps stay unchanged.\n'
        '        if not self._receive_in_progress:\n'
        '            self.drain_receive()\n'
        '        if self.ingress.error:\n'
        "            self._block_startup('KERNEL_RECEIVE_METADATA_INVALID')\n"
        '        # A failed worker/guard')
    edit(n, '        return super().safety_tick()\n\n    # Override every unsigned',
        """        result = super().safety_tick()
        if (self.policy.fault or self.local_stop_latched) and not self._first_feedback_trip_recorded:
            self._first_feedback_trip_recorded = True
            # Evidence cannot renew a lease or change the stop decision.
            try:
                evidence = self.ingress.peek()
            except (OSError, ValueError) as error:
                evidence = dict(peek_error=str(error))
            self.get_logger().warn('FEEDBACK_FIRST_TRIP ' + json.dumps(dict(
                fault=self.policy.fault, clock=self.sock.clock.snapshot(time.monotonic()),
                last_receive=self.ingress.last_receive, queue=evidence,
                drain_budget_exhaustions=self.ingress.budget_exhaustions)))
        return result

    # Override every unsigned""")
    edit(n, '    def drain_receive(self):\n        super().drain_receive()\n        samples = self.sock.clock.accepted_at',
        '    def drain_receive(self):\n'
        '        if self._receive_in_progress:\n'
        '            return\n'
        '        self._receive_in_progress = True\n'
        '        self.ingress.begin_drain()\n'
        '        try:\n'
        '            super().drain_receive()\n'
        '        finally:\n'
        '            self.ingress.end_drain()\n'
        '            self._receive_in_progress = False\n'
        '        samples = self.sock.clock.accepted_at')
    edit(n, "        self.force_local_disabled()\n        self.enabled_epoch = ''\n        if hard and",
        """        # Attempt the exact disable NOW, before the periodic sender's link
        # expiry check can disconnect the session. Keep the original bounded
        # repeated-disable queue and MCU lease as independent backstops.
        if self.pending_control_action == 'disable' and self.session.connected and (
                self.motion_gate_requested or self.safety_enabled or self.enabled_epoch):
            self.send_immediate(self.session.control_enable(False, False, self.monotonic_ms()),
                                'CONTROL_DISABLE_IMMEDIATE')
        self.force_local_disabled()
        self.enabled_epoch = ''
        if hard and""")
    # Frontend is byte-identical except its private location/validation gate and
    # a distinct instance protocol; it cannot silently show a different build.
    for name in ('oneclick_app.py', 'auto_open.py', 'launcher_ui.py', 'pose_check.py', 'ros_rehearsal.py'):
        shutil.copyfile(OLD/name, out/name)
    app = out/'oneclick_app.py'
    edit(app, repr(str(OLD)), repr(str(out)))
    edit(app, "m.get('autostart_passed')", "m.get('feedback_guard_passed')")
    text = app.read_text().replace('SHOW_AUTO_DEFAULT_V1', 'SHOW_FEEDBACK_GUARD_V1').replace(
        "b'AUTO_DEFAULT_V1'", "b'FEEDBACK_GUARD_V1'")
    app.write_text(text)
    edit(out/'pose_check.py', str(OLD/'runtime/src'), str(runtime))
    changes = []
    for p in runtime.rglob('*'):
        if p.is_file():
            rel = p.relative_to(runtime)
            old = Path(m['runtime_src'])/rel
            if not old.exists() or sha(old) != sha(p): changes.append(str(rel))
    expected = {f"{m['hardware_package']}/{m['hardware_package']}/{n}"
                for n in ('nodes.py', 'startup_clock.py', 'kernel_ingress.py')}
    if set(changes) != expected: raise ValueError('unexpected changes: '+str(changes))
    write(out/'PRIVATE_CHANGE_INVENTORY.json', changes)
    sources = hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_feedback.sh', *(out/n for n in
            ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'))]:
        sources[str(p)] = sha(p)
    m.update(runtime_src=str(runtime), runtime_hashes=hashes(runtime), source_hashes=sources,
        protected_input_hashes=protected, feedback_guard_passed=False,
        controller_bridge_and_console_bytes_unchanged=False,
        feedback_protection_ms=200, firmware_changed=False, parent_entry_changed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m)
    check(out/'build_manifest.json')
    return m


if __name__ == '__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    generate(ap.parse_args().output.resolve())
