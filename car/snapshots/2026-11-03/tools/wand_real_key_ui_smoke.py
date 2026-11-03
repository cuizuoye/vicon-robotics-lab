#!/usr/bin/env python3
"""Private X11 keyboard + REAL Sign / SOFTWARE car regression. No real output.

Only XTest on a newly created private Xephyr display is permitted here; never
send keyboard input to the user's desktop. This test does not move the Sign.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import tkinter as tk
import traceback

import rclpy
from PIL import ImageGrab
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.operator_console import OperatorNode, Console
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero
from mecanum_wand_hardware.commissioning import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, verification='PRIVATE XTEST / REAL SIGN / SOFTWARE CAR; NO HARDWARE')
    nested = backend = node = root = app = None
    display = ':88'
    try:
        if Path('/tmp/.X11-unix/X88').exists():
            raise RuntimeError('refusing to reuse an existing X display')
        result['domain0_before'] = audit_domain_zero()
        with (args.output/'xserver.log').open('x') as xlog, (args.output/'backend.log').open('x') as log:
            nested = subprocess.Popen(['Xephyr', display, '-screen', '1160x840', '-nolisten', 'tcp',
                '-noreset', '-ac', '-title', 'Private keyboard regression'], stdout=xlog, stderr=subprocess.STDOUT)
            until = time.monotonic()+5.
            while not Path('/tmp/.X11-unix/X88').exists():
                if time.monotonic()>until or nested.poll() is not None: raise RuntimeError('Xephyr failed')
                time.sleep(.03)
            os.environ.update(DISPLAY=display, ROS_DOMAIN_ID='94', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            runtime = Runtime('shadow', str(args.profile.resolve()))
            rclpy.init(domain_id=94)
            model = OperatorModel()
            node = OperatorNode(model, runtime)
            root = tk.Tk()
            app = Console(root, node, model)
            assert node.hand_pub is None and app.key_state is not None
            keyboard = ctypes.CDLL('libXtst.so.6')
            keyboard.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
            keyboard.XTestFakeKeyEvent.restype = ctypes.c_int
            app.key_state.x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
            backend = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=shadow', 'gui:=false',
                'profile:='+str(args.profile.resolve())], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)

            def pump(duration):
                end = time.monotonic()+duration
                while time.monotonic()<end:
                    root.focus_force()  # PRIVATE display only.
                    root.update()
                    time.sleep(.002)

            def wait(predicate, seconds=10.):
                end = time.monotonic()+seconds
                while time.monotonic()<end:
                    pump(.02)
                    if predicate(): return
                raise AssertionError(dict(status=model.status, gate=model.gate, notice=model.notice))

            def physical_key(down):
                assert os.environ['DISPLAY'] == display and app.key_state.display
                assert keyboard.XTestFakeKeyEvent(app.key_state.display, app.key_state.keycode, int(down), 0)
                app.key_state.x11.XSync(app.key_state.display, 0)
                pump(.06)

            def arm():
                wait(lambda: model.can_start(time.monotonic())
                     and not app.buttons['start'].instate(['disabled']))
                app.buttons['start'].invoke()
                wait(lambda: model.status.get('state') == 'ARMED' and model.status.get('gate_ready'))
                physical_key(True)
                wait(lambda: model.status.get('state') == 'ACTIVE')

            wait(lambda: model.fresh(time.monotonic()) and node.raw_wand is not None)
            # Required release/unlock is a real server-side key edge, not
            # direct model mutation or an injected operator lease.
            physical_key(True)
            physical_key(False)
            arm()
            pump(2.0)  # Native X server autorepeat beyond its initial delay.
            assert model.held and model.status['state'] == 'ACTIVE'
            app.draw(time.monotonic())
            assert 'Z=' in app.hand_text.get() and '不映射' in app.hand_text.get()
            assert '0.25m' not in app.hand_text.get()
            # Reproduce a Release/Press pair delayed FAR beyond the old 5ms
            # heuristic while the server still confirms the key is down.
            root.event_generate('<KeyRelease-space>')
            pump(.08)
            assert model.held and model.status['state'] == 'ACTIVE'
            root.event_generate('<KeyPress-space>')
            pump(.08)
            assert model.held and model.status['state'] == 'ACTIVE'
            started = time.monotonic()
            physical_key(False)
            wait(lambda: model.gate.get('output_zero'), .4)
            result['release_to_zero_status_observed_s'] = time.monotonic()-started
            result['release_timing_note'] = 'Sign remained still; this is not moving-car braking latency'
            wait(lambda: model.status.get('state') == 'PAUSED' and model.status.get('stop_confirmed'))
            pump(.5)
            assert not model.held and model.status['state'] == 'PAUSED'
            arm()
            # Deliberately swallow Tk's release callback. The read-only server
            # polling must revoke, never keep a lost release enabled.
            root.bind_class('SafeWandKeys', '<KeyRelease-space>', lambda _: 'break')
            physical_key(False)
            wait(lambda: not model.held and model.status.get('state') == 'PAUSED')
            wait(lambda: model.status.get('stop_confirmed'))
            # Screenshot capture can block >100ms. Do it ONLY after released
            # enable and confirmed standstill, never during the active lease.
            app.draw(time.monotonic())
            root.update_idletasks()
            ImageGrab.grab(xdisplay=display).save(args.output/'real_input_console.png')
            assert not any(n == 'wand_hardware_bridge' for n, _ in node.get_node_names_and_namespaces())
            result.update(passed=True, final_state=model.status['state'], stop_confirmed=True,
                checks=['real server press/release', '2s actual server hold remains enabled',
                        '80ms delayed repeat pair does not release', 'true release stops',
                        'no automatic resume', 'lost Tk release caught by read-only polling',
                        'no synthetic Sign publisher', 'no hardware bridge'],
                operator_events=model.events)
            app.close()
            pump(.2)
        result['domain0_after'] = audit_domain_zero()
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        if node: result.update(supervisor=node.model.status, gate=node.model.gate, events=node.model.events)
    finally:
        if app and app.key_state and app.key_state.display:
            try:
                keyboard.XTestFakeKeyEvent(app.key_state.display, app.key_state.keycode, 0, 0)
                app.key_state.close()
            except Exception: pass
        if node: node.close_node()
        if root:
            try: root.destroy()
            except tk.TclError: pass
        if rclpy.ok(): rclpy.shutdown()
        if backend and backend.poll() is None:
            backend.send_signal(signal.SIGINT)
            try: backend.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(backend.pid, signal.SIGTERM)
                backend.wait(timeout=5)
        if nested and nested.poll() is None:
            nested.terminate()
            nested.wait(timeout=3)
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
