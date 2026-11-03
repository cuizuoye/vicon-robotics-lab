#!/usr/bin/env python3
"""Test our Tk widgets/key events on a private nested X display + ROS domain96.

The nested server is temporary; no desktop keyboard/mouse automation is used.
Only this test's software launch is started/stopped. No Vicon wand is needed.
"""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import traceback

import rclpy
import tkinter as tk
from PIL import ImageGrab

from mecanum_wand_control.operator_console import Console, OperatorNode
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero


class UITest:
    def __init__(self, root, app, node, model, folder):
        self.root, self.app, self.node, self.model, self.folder = root, app, node, model, folder
        self.keep_private_focus = True

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            if self.keep_private_focus:
                widget = self.root.focus_displayof()
                if widget is None or widget.winfo_toplevel() != self.root:
                    self.root.focus_force()
            self.root.update()
            time.sleep(.002)

    def wait(self, predicate, timeout=8.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.pump(.02)
            if predicate(): return
        raise AssertionError(f'UI wait failed: {self.model.status}; gate={self.model.gate}; '
                             f'focus={self.model.focused}, key={self.model.key_down}, '
                             f'blocked={self.model.blocked}; notice={self.model.notice}')

    def press(self):
        self.root.event_generate('<KeyPress-space>')
        self.pump(.03)

    def release(self):
        self.root.event_generate('<KeyRelease-space>')
        self.pump(.04)

    def arm(self):
        self.release()
        self.wait(lambda: self.model.can_start(time.monotonic())
                  and not self.app.buttons['start'].instate(['disabled']), 12.)
        # Exercise ttk's actual mouse bindings/focus, not only .invoke().
        button = self.app.buttons['start']
        x, y = button.winfo_width()//2, button.winfo_height()//2
        button.event_generate('<Enter>', x=x, y=y)
        button.event_generate('<ButtonPress-1>', x=x, y=y)
        self.pump(.02)
        button.event_generate('<ButtonRelease-1>', x=x, y=y, state=256)
        self.wait(lambda: self.model.status.get('state') == 'ARMED' and self.model.status.get('gate_ready'))
        self.press()
        self.wait(lambda: self.model.status.get('state') == 'ACTIVE')

    def move(self, x):
        self.app.pad.event_generate('<Button-1>', x=x, y=155)
        self.app.pad.event_generate('<B1-Motion>', x=x, y=155)
        self.wait(lambda: self.model.gate.get('speed_mps', 0.) > .025, 3.)

    def run(self):
        self.root.update()
        self.root.focus_force()  # Only our private nested X server, not the user's desktop.
        self.wait(lambda: self.model.focused and self.model.fresh(time.monotonic()), 15.)
        # Reproduce a normal launch/focus transition BEFORE sending any key
        # release. The former test pre-cleared this interlock and missed the UX bug.
        other = tk.Toplevel(self.root)
        other.title('Private startup focus transition')
        other.update()
        self.keep_private_focus = False
        other.focus_force()
        self.wait(lambda: not self.model.focused and self.model.blocked)
        other.destroy()
        self.keep_private_focus = True
        self.root.focus_force()
        self.wait(lambda: self.model.focused and '轻按并松开' in self.app.step_text.get())
        assert not self.model.can_start(time.monotonic())
        assert self.app.buttons['start'].instate(['disabled'])
        print('PASS startup focus interlock is visible and prepare cannot falsely succeed', flush=True)
        self.press()
        assert not self.model.held and self.model.status['state'] == 'STANDBY', ('startup', self.model.status)
        self.release()
        self.arm()
        print('UI armed from real button callback', flush=True)
        self.move(211)
        self.wait(lambda: self.node.car is not None and self.node.car.x > .02, 2.)
        # X11 autorepeat can be represented as a release/press pair. It must
        # not become a physical release or activate a focused ttk button.
        button = self.app.buttons['pause']
        button.focus_force()
        button.event_generate('<KeyRelease-space>')
        button.event_generate('<KeyPress-space>')
        self.pump(.06)
        assert self.model.held and self.model.status['state'] == 'ACTIVE', ('autorepeat', self.model.status, self.model.notice)
        self.app.draw(time.monotonic())
        assert '运行中（非停稳）' in self.app.status_text.get()
        self.root.update_idletasks()
        ImageGrab.grab(xdisplay=os.environ['DISPLAY']).save(self.folder/'console_active.png')
        release_at = time.monotonic()
        self.release()
        self.wait(lambda: self.model.gate.get('output_zero'), .4)
        release_latency = time.monotonic()-release_at
        self.wait(lambda: self.model.status.get('state') == 'PAUSED' and self.model.status.get('stop_confirmed'))
        self.press()
        assert not self.model.held and self.model.status['state'] == 'PAUSED'
        print('PASS UI startup, real key edges, autorepeat and release', flush=True)
        self.arm()
        self.move(245)
        other = tk.Toplevel(self.root)
        other.title('Private UI focus-loss test')
        other.geometry('200x100+20+20')
        other.update()
        focus_at = time.monotonic()
        self.keep_private_focus = False
        other.focus_force()
        self.wait(lambda: not self.model.focused and not self.model.held, .5)
        self.wait(lambda: self.model.gate.get('output_zero'), .4)
        focus_latency = time.monotonic()-focus_at
        other.destroy()
        self.keep_private_focus = True
        self.root.focus_force()
        self.wait(lambda: self.model.focused)
        self.wait(lambda: self.model.status.get('state') == 'PAUSED' and self.model.status.get('stop_confirmed'))
        self.press()
        assert not self.model.held
        print('PASS UI focus loss and no automatic resume', flush=True)
        self.arm()
        self.move(190)
        time.sleep(.35)  # Intentional freeze of only our GUI/operator process.
        self.pump(.20)
        assert not self.model.held and self.model.blocked
        self.wait(lambda: self.model.status.get('state') == 'FAULT' and self.model.status.get('stop_confirmed'))
        assert self.model.gate['output_zero']
        self.press()
        assert not self.model.held
        self.release()
        self.app.buttons['acknowledge'].invoke()
        self.wait(lambda: self.model.status.get('state') == 'PAUSED')
        print('PASS UI/operator event-loop freeze fails closed', flush=True)
        self.arm()
        self.move(240)
        self.app.close()
        # Closing destroys Tk after 250 ms. Pump ROS afterwards to observe
        # genuine backend stop confirmation, not the UI's service result.
        deadline = time.monotonic()+.4
        while time.monotonic() < deadline:
            try: self.root.update()
            except tk.TclError: break
            time.sleep(.002)
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            self.node.pump()
            time.sleep(.005)
        assert not self.model.held and self.model.status.get('stop_confirmed')
        assert self.model.gate.get('output_zero') and self.model.status['state'] != 'ACTIVE'
        return dict(passed=True, verification='PRIVATE TK EVENTS + REAL ROS SOFTWARE NODES; NO WAND/HARDWARE',
            checks=['startup held key rejected', 'space press/release', 'autorepeat suppression',
                    'startup focus lock visible and prepare disabled until release',
                    'prepare through ttk mouse bindings rather than direct invoke',
                    'mouse virtual wand drives software plant', 'focus loss stops',
                    'no focus-return auto-resume', 'GUI freeze stops and cannot resume held key',
                    'closing window requests and confirms software stop'],
            release_zero_observed_s=release_latency, focus_loss_zero_observed_s=focus_latency,
            virtual_car_x_m=self.node.car.x, final_state=self.model.status['state'],
            stop_confirmed=self.model.status['stop_confirmed'], events=self.model.events)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    folder = args.output.resolve()
    report = dict(passed=False)
    nested = backend = node = root = app = None
    display = ':89'
    try:
        if not shutil.which('Xephyr') or Path('/tmp/.X11-unix/X89').exists():
            raise RuntimeError('private display unavailable; refusing to reuse any existing X server')
        report['hardware_publishers_before'] = audit_domain_zero()
        with (folder/'nested_x.log').open('w') as xlog, (folder/'backend.log').open('w') as log:
            nested = subprocess.Popen(['Xephyr', display, '-screen', '1160x840', '-nolisten', 'tcp',
                                       '-noreset', '-ac', '-title', 'Offline Wand UI Test'], stdout=xlog, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while not Path('/tmp/.X11-unix/X89').exists() and time.monotonic() < deadline:
                if nested.poll() is not None: raise RuntimeError('nested X server failed')
                time.sleep(.03)
            os.environ['DISPLAY'] = display
            os.environ['ROS_DOMAIN_ID'] = '96'
            os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
            rclpy.init(domain_id=96)
            model = OperatorModel()
            node = OperatorNode(model)
            root = tk.Tk()
            app = Console(root, node, model)
            backend = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_control', 'wand_safe_sim.launch.py'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            report.update(UITest(root, app, node, model, folder).run())
            report['hardware_publishers_after'] = audit_domain_zero()
    except Exception as exc:
        report.update(passed=False, error=repr(exc), traceback=traceback.format_exc())
        if node is not None:
            report.update(operator_events=node.model.events, supervisor=node.model.status, gate=node.model.gate,
                          focused=node.model.focused, held=node.model.held,
                          car_received=node.car_received, car_error=node.car_error)
        if root is not None and nested is not None and nested.poll() is None:
            try: ImageGrab.grab(xdisplay=display).save(folder/'failure.png')
            except Exception: pass
    finally:
        if node is not None:
            node.model.closing = True
            node.model.held = False
            node.close_node()
        if root is not None:
            try: root.destroy()
            except tk.TclError: pass
        if rclpy.ok(): rclpy.shutdown()
        if backend is not None and backend.poll() is None:
            backend.send_signal(signal.SIGINT)
            try: backend.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(backend.pid, signal.SIGTERM)
                backend.wait(timeout=5)
        if nested is not None and nested.poll() is None:
            nested.terminate()
            nested.wait(timeout=3)
    (folder/'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
