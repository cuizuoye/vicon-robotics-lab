"""Native offline console: virtual wand + real key edges, never a hardware UI."""

from collections import deque
from dataclasses import asdict
import json
import math
import os
import signal
import time
import tkinter as tk
from tkinter import ttk
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Pose
from .toggle_ui import ToggleOperatorModel as OperatorModel
from .toggle_keys import ToggleKeysMixin
from .safety_core import Lease, SafetyConfig, fresh_envelope
from .safety_nodes import guard, LATEST, SENSOR, NAMESPACE, decode_pose, pose_message
from .safety_wire import load_json
from .x11_key_state import X11KeyState
from .operator_guidance import center_bounds, wand_preview
from .pilot_ui import reason_text, real_instruction
from .novice_ui import acknowledge_physical_release


class OperatorNode(Node):
    def __init__(self, model, runtime=None):
        super().__init__('wand_real_operator' if runtime else 'safe_sim_operator',
                         namespace=runtime.namespace if runtime else NAMESPACE)
        self.runtime = runtime
        self.world = runtime.world if runtime else 'wand_safe_map'
        self.input_kind = runtime.mode if runtime else 'virtual'
        (runtime.guard if runtime else guard)(self)
        self.model = model
        model.transport_status, model.transport_at = {}, -math.inf
        self.create_subscription(String, "transport_status", self.on_transport, LATEST)
        self.owner, self.sequence = uuid.uuid4().hex, 0
        self.hand_pub = None if runtime else self.create_publisher(PoseStamped, 'wand_pose', LATEST)
        self.raw_wand = self.display_origin = None
        self.wand_stamp = self.car_stamp = -math.inf
        self.source_status = {}
        if runtime: self.create_subscription(PoseStamped, 'wand_pose', self.on_wand, SENSOR)
        if runtime: self.create_subscription(String, 'source_status', self.on_source, LATEST)
        self.lease_pub = self.create_publisher(String, 'operator_lease', LATEST)
        self.create_subscription(String, 'supervisor_status', self.on_status, LATEST)
        self.create_subscription(String, 'gate_status', self.on_gate, LATEST)
        self.create_subscription(PoseStamped, 'car_pose', self.on_car, SENSOR)
        self.car = None
        self.car_received, self.car_error = 0, ''
        self.trace = deque(maxlen=500)
        self.service_clients = {name: self.create_client(Trigger, name)
                               for name in ('start', 'pause', 'stop', 'acknowledge', 'recover', 'estop')}
        self.pending = set()
        self.last_ui_diagnostic = None
        self.executor_for_console = SingleThreadedExecutor()
        self.executor_for_console.add_node(self)

    def clocks(self):
        return time.monotonic(), self.get_clock().now().nanoseconds*1e-9

    def validated_status(self, msg):
        value = load_json(msg.data)
        steady, ros = self.clocks()
        if not fresh_envelope(value['issued_steady'], value['issued_ros'], steady, ros, .25):
            raise ValueError('stale backend status')
        return value, steady

    def on_transport(self, msg):
        try:
            self.model.transport_status, self.model.transport_at = self.validated_status(msg)
        except (ValueError, TypeError, KeyError):
            self.model.transport_at = -math.inf

    def on_status(self, msg):
        try:
            value, now = self.validated_status(msg)
            self.model.receive_status(value, now)
        except (ValueError, TypeError, KeyError):
            self.model.status_at = -math.inf

    def on_gate(self, msg):
        try:
            value, now = self.validated_status(msg)
            self.model.receive_gate(value, now)
        except (ValueError, TypeError, KeyError):
            self.model.gate_at = -math.inf

    def on_car(self, msg):
        self.car_received += 1
        try:
            self.car, self.car_stamp, _, _ = decode_pose(msg, self.world)
            self.trace.append((self.car.x, self.car.y))
            self.car_error = ''
        except ValueError as exc:
            self.car = None
            self.car_stamp = -math.inf
            self.car_error = str(exc)

    def on_wand(self, msg):
        try:
            pose, self.wand_stamp, _, _ = decode_pose(msg, self.world)
            self.raw_wand = pose
            if self.display_origin is None: self.display_origin = pose
            anchors = self.model.status.get('anchors')
            if self.model.status.get('state') in ('STANDBY', 'ARMED') or not anchors:
                # No new origin is captured until the held-enable edge. Do
                # not display stale previous-session displacement as input.
                self.display_origin = pose
                origin = asdict(pose)
            else:
                origin = anchors['wand_origin']
            # Read-only display, never clamp/re-publish genuine Sign input.
            self.model.x, self.model.y = pose.x-origin['x'], pose.y-origin['y']
        except (ValueError, TypeError, KeyError):
            self.raw_wand = None
            self.wand_stamp = -math.inf

    def on_source(self, msg):
        try: self.source_status, _ = self.validated_status(msg)
        except (ValueError, TypeError, KeyError): self.source_status = {}

    def input_hint(self):
        if self.input_kind == 'virtual': return ''
        ros = self.get_clock().now().nanoseconds*1e-9
        if not -.005 <= ros-self.wand_stamp <= .20:
            return '等待真实Sign：没有新鲜跟踪数据，不可使能；活动会话会由安全链路停车'
        if self.input_kind != 'shadow' and not -.005 <= ros-self.car_stamp <= .20:
            return '等待car01：没有新鲜实车定位，不可使能'
        if self.source_status.get('conflicts'):
            return '检测到旧控制器/重复定位源冲突：先关闭旧运动入口，不会自动切换'
        return ''

    def heartbeat(self):
        now, ros = self.clocks()
        self.model.tick(now)
        diagnostic = (self.model.status.get('state'), self.model.status.get('reason'),
                      self.model.status.get('gate_ready'), self.model.gate.get('fault'),
                      self.model.held, self.model.key_down, self.model.focused, self.model.blocked)
        if diagnostic != self.last_ui_diagnostic:
            self.get_logger().info('UI_STATE '+json.dumps(dict(
                state=diagnostic[0], reason=diagnostic[1], gate_ready=diagnostic[2], gate_fault=diagnostic[3],
                held=diagnostic[4], key_down=diagnostic[5], focused=diagnostic[6], blocked=diagnostic[7],
                operator_input_mode=self.model.input_mode,session_requested=self.model.session_requested,
                click_pending=self.model.click_pending,trial_ended=self.model.ended,
                instruction=self.model.instruction(now)), ensure_ascii=False))
            self.last_ui_diagnostic = diagnostic
        self.sequence += 1
        if self.hand_pub is not None:
            self.hand_pub.publish(pose_message(Pose(-.4+self.model.x, -.4+self.model.y, 1.),
                                               self.get_clock().now().to_msg()))
        lease = Lease(self.owner, self.sequence, now, ros, self.model.held)
        self.lease_pub.publish(String(data=json.dumps(asdict(lease))))

    def pump(self):
        # Bounded work: graphics and ROS may never keep renewing a held lease
        # after a blocked UI loop resumes. Model.tick checks the real gap.
        for _ in range(8): self.executor_for_console.spin_once(timeout_sec=0.)
        self.heartbeat()

    def request(self, name):
        now = time.monotonic()
        if name == 'recover' and not self.model.can_recover(now):
            self.model.notice = '重新准备被拒绝：请松键，等待反馈恢复及停稳确认'
            return False
        if name == 'start' and not self.model.can_start(now):
            self.model.notice = '准备被拒绝：'+self.model.instruction(now)
            return False
        if name not in ('start', 'recover'):
            self.model.revoke(now, '已释放使能；等待后端停车确认')
            self.heartbeat()
        client = self.service_clients[name]
        if name in self.pending or not client.service_is_ready():
            self.model.notice = '服务未就绪或请求尚未完成：'+name
            return False
        self.pending.add(name)
        future = client.call_async(Trigger.Request())

        def done(result):
            self.pending.discard(name)
            try:
                response = result.result()
                self.model.notice = ('请求已接收：' if response.success else '请求被拒绝：')+response.message
            except Exception:
                self.model.notice = '服务失败；使能保持释放'
                self.model.revoke(time.monotonic(), self.model.notice)
        future.add_done_callback(done)
        self.model.record(now, 'service_'+name)
        return True

    def close_node(self):
        self.model.closing = True
        self.heartbeat()
        self.executor_for_console.shutdown()
        self.destroy_node()


class Console:
    def __init__(self, root, node, model):
        self.root, self.node, self.model = root, node, model
        self.release_job = None
        self.focus_job = None
        self.loop_job = self.shutdown_job = self.destroy_job = None
        self.closing = False
        self.last_draw = -math.inf
        self.config = node.runtime.config if node.runtime else SafetyConfig()
        self.wand_reference_margin = node.runtime.wand_reference_margin_m if node.runtime else 0.
        self.virtual = node.input_kind == 'virtual'
        if node.input_kind == 'hardware':
            self.model.notice = '实车入口：尚未准备／未使能；先等待定位与真实停稳反馈'
        # Only the real-input console needs this OS-level check. Keep the
        # established virtual event-test entry unchanged. Opening a separate
        # read-only X connection does not alter any user's keyboard settings.
        self.key_state = None if self.virtual else X11KeyState(os.environ.get('DISPLAY', ''))
        titles = {'virtual': '虚拟标定杆 → 安全仿真车', 'shadow': '真实 Sign → 安全仿真车（实车断电）',
                  'emulator': '协议验收：模拟 Sign → 回环 MCU 仿真器', 'hardware': f'单击启停 · 2.0×0.9m跟随 · 最长600秒 → 4WSA档{self.config.stage} · {self.config.max_speed_mps:.2f}m/s · {self.config.max_accel_mps2:.2f}m/s²'}
        root.title(titles[node.input_kind])
        root.geometry('1280x720')
        root.minsize(1000, 640)
        root.configure(bg='#edf2f7')
        style = ttk.Style(root)
        style.configure('TLabel', font=('Noto Sans CJK SC', -14))
        style.configure('TButton', font=('Noto Sans CJK SC', -14), padding=7)
        top = tk.Frame(root, bg='#15334b')
        top.pack(fill='x')
        tk.Label(top, text=titles[node.input_kind], fg='white', bg='#15334b',
                 font=('Noto Sans CJK SC', -24, 'bold')).pack(anchor='w', padx=18, pady=(10, 2))
        subtitle = ('SOFTWARE ONLY  ·  域97  ·  1:1映射  ·  0.05 m/s  ·  不依赖Vicon/实物杆' if self.virtual else
                    f'域120 · 单次≤600秒 / 2.0×0.9m目标区 / 四边50cm缓冲 · 1:1 · ≤{self.config.max_speed_mps:.2f}m/s · '+('REAL HARDWARE：启动不运动；持续使能后可能运动' if node.input_kind == 'hardware'
                     else 'SOFTWARE ONLY：没有实车运动输出'))
        tk.Label(top, text=subtitle,
                 fg='#bfe8f5', bg='#15334b', font=('Noto Sans CJK SC', -14)).pack(anchor='w', padx=18, pady=(0, 10))
        body = tk.Frame(root, bg='#edf2f7')
        body.pack(fill='both', expand=True, padx=14, pady=10)
        left = tk.Frame(body, bg='#edf2f7')
        left.pack(side='left', fill='both', expand=True, padx=(0, 12))
        right = tk.Frame(body, bg='#edf2f7', width=330)
        right.pack(side='right', fill='y')
        tk.Label(left, text=('实车工作区：蓝色实测车体 / 橙色目标 / 红色固定围栏' if node.input_kind == 'hardware'
                           else '仿真工作区：蓝车 / 橙色目标 / 红色固定围栏'), bg='#edf2f7',
                 font=('Noto Sans CJK SC', -16)).pack(anchor='w')
        self.map = tk.Canvas(left, width=690, height=260, bg='white', highlightthickness=0)
        self.map.pack(fill='both', expand=True, pady=6)
        self.status_text = tk.StringVar()
        self.step_text = tk.StringVar(value=model.instruction(time.monotonic()))
        tk.Label(left, textvariable=self.step_text, justify='left', anchor='w', wraplength=650,
                 bg='#fff0cf', fg='#5b4215', font=('Noto Sans CJK SC', -16, 'bold'),
                 padx=10, pady=8).pack(fill='x')
        tk.Label(left, textvariable=self.status_text, justify='left', anchor='w',
                 bg='white', font=('Noto Sans CJK SC', -14), padx=10, pady=10).pack(fill='x')
        tk.Label(right, text='鼠标拖动虚拟杆（+X向右，+Y向上）' if self.virtual else '真实Sign位移显示（鼠标不可下指令）', bg='#edf2f7',
                 font=('Noto Sans CJK SC', -14)).pack(anchor='w')
        self.pad = tk.Canvas(right, width=310, height=310 if self.virtual else 190, bg='#f8fbff', highlightthickness=1,
                             highlightbackground='#bccbda')
        self.pad.pack(pady=6)
        self.pad.bind('<Button-1>', self.drag)
        self.pad.bind('<B1-Motion>', self.drag)
        ttk.Button(right, text='虚拟杆回中（不是停车键）' if self.virtual else '输入来自真实Sign；禁止鼠标替代',
                   state='normal' if self.virtual else 'disabled', command=self.center).pack(fill='x', pady=(0, 8))
        self.hand_text = tk.StringVar()
        ttk.Label(right, textvariable=self.hand_text, justify='left', wraplength=320).pack(anchor='w')
        self.buttons = {}
        for name, text in (('start', '1  准备好后点这里（不会运动）'), ('pause', '暂停 / 结束本轮'),
                           ('stop', '正常停止 / 结束本轮'), ('recover', '确认并重新准备（不运动）'),
                           ('acknowledge', '仅确认故障（不准备）'),
                           ('estop', '仿真急停（锁存，需重启）')):
            if name == 'estop' and node.input_kind == 'hardware': text = '紧急停止（锁存；异常时使用）'
            button = ttk.Button(right, text=text, command=lambda name=name: self.node.request(name))
            if name in ('start','stop','estop'): button.pack(fill='x', pady=3)
            self.buttons[name] = button
        self.notice = tk.StringVar(value=model.notice)
        tk.Label(root, textvariable=self.notice, anchor='w', justify='left', wraplength=1080,
                 bg='#fff0cf', fg='#5b4215', font=('Noto Sans CJK SC', -14), padx=14, pady=8).pack(fill='x')
        tk.Label(root, text=('单击模式：准备就绪后轻按松开空格启动；再次空格/Esc停止；切出窗口也停车。'
                            if self.virtual else '自动检查与准备 → 看到就绪后单击空格开始；再次空格/Esc/停止结束；松键不停车。'),
                 bg='#15334b', fg='white', font=('Noto Sans CJK SC', -14), pady=9).pack(fill='x')
        # Put our key bindings before ttk.Button's space-key activation.
        root.bind_class('SafeWandKeys', '<KeyPress-space>', self.press)
        root.bind_class('SafeWandKeys', '<KeyRelease-space>', self.release)
        root.bind_class('SafeWandKeys', '<Escape>', self.escape)
        self.install_keys(root)
        root.bind('<FocusIn>', lambda _: self.schedule_focus())
        root.bind('<FocusOut>', lambda _: self.schedule_focus())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.loop_job = root.after(20, self.loop)

    def install_keys(self, widget):
        widget.bindtags(('SafeWandKeys', *widget.bindtags()))
        for child in widget.winfo_children(): self.install_keys(child)

    def schedule_focus(self):
        if self.closing: return
        if self.focus_job is not None: self.root.after_cancel(self.focus_job)
        # Child-to-child focus changes briefly have no X focus. Coalesce the
        # pair without misclassifying an in-window click as leaving the app.
        self.focus_job = self.root.after(15, self.check_focus)

    def check_focus(self):
        self.focus_job = None
        widget = self.root.focus_displayof()
        focused = widget is not None and widget.winfo_toplevel() == self.root
        old = self.model.focused
        self.model.focus(focused, time.monotonic())
        if focused and self.key_state is not None:
            acknowledge_physical_release(self.model,self.server_space_down(),time.monotonic())
        if old and not focused:
            self.cancel_release()
            self.node.heartbeat()
            if self.model.status.get('state') in ('ARMED', 'ACTIVE'): self.node.request('pause')

    def cancel_release(self):
        if self.release_job is not None:
            self.root.after_cancel(self.release_job)
            self.release_job = None

    def press(self, _=None):
        self.cancel_release()  # X11 autorepeat release/press pairs are not a release.
        if self.key_state is not None and not self.server_space_down():
            # A queued tap/press received after a confirmed physical release
            # must never establish a held lease.
            if not self.closing: self.finish_release()
            return 'break'
        self.model.press(time.monotonic())
        self.node.heartbeat()
        return 'break'

    def release(self, _=None):
        self.cancel_release()
        if self.key_state is not None:
            if not self.server_space_down() and not self.closing: self.finish_release()
            return 'break'
        self.release_job = self.root.after(5, self.finish_release)
        return 'break'

    def server_space_down(self):
        try:
            return self.key_state.down()
        except (RuntimeError, OSError) as error:
            self.model.revoke(time.monotonic(), '键盘状态读取失败：使能已释放；请重启操作台')
            self.node.get_logger().error(str(error))
            self.close()
            return False

    def finish_release(self):
        self.release_job = None
        self.model.release(time.monotonic())
        self.node.heartbeat()

    def drag(self, event):
        if not self.virtual: return
        self.model.move((event.x-155)/560., (155-event.y)/560.)
        self.pad.focus_set()
        self.draw_pad()

    def center(self):
        if not self.virtual: return
        self.model.move(0., 0.)
        self.draw_pad()

    def loop(self):
        self.loop_job = None
        if self.closing: return
        # Polling only revokes: it can never synthesize a press or establish
        # enable. A lost KeyRelease is still caught within the UI cycle.
        if self.key_state is not None and self.model.key_down:
            if not self.server_space_down() and not self.closing: self.finish_release()
        if self.closing: return
        self.node.pump()
        now = time.monotonic()
        if now-self.last_draw >= .033:
            self.draw(now)
            self.last_draw = now
        if not self.closing: self.loop_job = self.root.after(20, self.loop)

    def draw_pad(self):
        c = self.pad
        c.delete('all')
        cx, cy = (155., 155.) if self.virtual else (155., 95.)
        radius, scale = (137., 560.) if self.virtual else (77., 400.)
        if self.wand_reference_margin:
            display_radius = max(.25, math.ceil(math.hypot(self.model.x, self.model.y)*1.15*4)/4)
            scale = radius/display_radius
            c.create_text(8, 10, anchor='w', text=f'显示半径 {display_radius:.2f}m（自动缩放，非限位）',
                          fill='#54708c', font=('Noto Sans CJK SC', -13))
        c.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, fill='#e7f2ff', outline='#aec5dd')
        c.create_line(cx, cy-radius-8, cx, cy+radius+8, fill='#b5c6d7')
        c.create_line(cx-radius-8, cy, cx+radius+8, cy, fill='#b5c6d7')
        c.create_text(cx+radius-25, cy+15, text='+X', fill='#54708c')
        c.create_text(cx+19, cy-radius+8, text='+Y', fill='#54708c')
        x, y = cx+scale*self.model.x, cy-scale*self.model.y
        c.create_line(cx, cy, x, y, fill='#25a26e', width=2)
        c.create_oval(x-8, y-8, x+8, y+8, fill='#1d9c6a', outline='')

    def map_point(self, x, y):
        width, height = max(2, self.map.winfo_width()), max(2, self.map.winfo_height())
        c = self.config
        extra = 2*self.wand_reference_margin
        scale = min((width-60)/(c.arena_x_max-c.arena_x_min+.4+extra),
                    (height-45)/(c.arena_y_max-c.arena_y_min+.4+extra))
        return (width/2+(x-(c.arena_x_min+c.arena_x_max)/2)*scale,
                height/2-(y-(c.arena_y_min+c.arena_y_max)/2)*scale)

    def draw(self, now):
        self.draw_tracking_map()
        p = self.node.car
        status, gate = self.model.status, self.model.gate
        fresh = self.model.fresh(now)
        state = status.get('state', 'WAITING') if fresh else 'STALE / 等待连接'
        output_text = f'{gate.get("speed_mps", 0.):.3f} m/s' if fresh else '未知（反馈过期）'
        gate_text = (reason_text(gate['fault']) if not self.virtual and gate.get('fault') else gate.get('fault')) or ('允许输出' if state == 'ACTIVE' and self.model.held
                                         else '零输出 / 等待使能')
        if not fresh: gate_text = '反馈过期'
        # The backend retains the pre-arm standstill flag while ACTIVE;
        # it is not evidence that a moving car is currently stopped.
        stop_text = ('运行中（非停稳）' if state == 'ACTIVE' else
                     '已确认' if status.get('stop_confirmed') and fresh else '未确认')
        display_state = '本轮结束 / 等待停稳' if self.model.pilot_done and not status.get('hard_fault') and not gate.get('hard_fault') else state
        self.status_text.set(f'监督器：{display_state}     末级门：{gate_text}\n'
            f'使能：{"按住" if self.model.held else "释放"}     '
            f'停车反馈：{stop_text}     '
            f'{"输出" if self.virtual else "门控指令"}：{output_text}\n'
            f'原因：{(status.get("reason", "等待后端") or "无故障") if self.virtual else reason_text(status.get("reason", "等待后端"))}')
        if self.virtual:
            self.hand_text.set(f'虚拟杆 X={self.model.x:+.3f}m  Y={self.model.y:+.3f}m\n'
                               '输入最大半径0.245m；位移映射1:1\n界面范围不代表真实场地已标定')
        else:
            self.hand_text.set(self.tracking_hand_text())
        self.buttons['start'].configure(state='normal' if self.model.can_start(now) else 'disabled')
        self.buttons['recover'].configure(state='normal' if self.model.can_recover(now) else 'disabled')
        self.step_text.set(self.model.instruction(now) if self.virtual else self.node.input_hint() or
                           real_instruction(self.model, now, self.node.input_kind == 'hardware'))
        can_ack = (fresh and status.get('state') == 'FAULT' and status.get('stop_confirmed')
                   and not status.get('hard_fault') and not self.model.held and not self.model.pilot_done)
        self.buttons['acknowledge'].configure(state='normal' if can_ack else 'disabled')
        self.notice.set(self.model.notice if self.virtual else self.model.notice.replace('虚拟车', '小车'))
        self.draw_pad()

    def close(self):
        if self.closing: return
        self.closing = self.model.closing = True
        self.cancel_release()
        self.model.revoke(time.monotonic(), '窗口关闭：释放使能，已请求停止')
        self.node.request('stop')
        # Keep sending released leases briefly; never wait for a fake stop ACK.
        self.destroy_job = self.root.after(250, self.finish_close)
        self.shutdown_pump()

    def shutdown_pump(self):
        self.shutdown_job = None
        try:
            self.node.pump()
            self.shutdown_job = self.root.after(20, self.shutdown_pump)
        except tk.TclError:
            pass

    def finish_close(self):
        # Tk deletes widget callbacks on destroy, but queued after scripts
        # remain in Tcl. Cancel our jobs first, including the released-lease
        # pump, so a later event-loop update cannot invoke a deleted command.
        for attr in ('release_job', 'focus_job', 'loop_job', 'shutdown_job', 'destroy_job'):
            job = getattr(self, attr)
            if job is not None:
                self.root.after_cancel(job)
                setattr(self, attr, None)
        self.root.destroy()
        if self.key_state is not None: self.key_state.close()


class ToggleConsole(ToggleKeysMixin, Console):
    def __init__(self,root,node,model):
        if not isinstance(model,OperatorModel):raise ValueError("dedicated single-click model required")
        super().__init__(root,node,model)


from .auto_prepare import AutoPrepareConsoleMixin

class AutomaticPrepareConsole(AutoPrepareConsoleMixin, ToggleConsole):
    pass

from .large_type import LargeTypeConsoleMixin

class LargeTypeConsole(LargeTypeConsoleMixin, AutomaticPrepareConsole):
    pass

from .tracking_ui import TrackingConsoleMixin
class RotationConsole(TrackingConsoleMixin, LargeTypeConsole):
    pass
Console = RotationConsole


def run_console(runtime=None):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    model = OperatorModel()
    node = OperatorNode(model, runtime)
    root = tk.Tk()
    app = Console(root, node, model)
    signal.signal(signal.SIGINT, lambda *_: root.after(0, app.close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, app.close))
    try:
        root.mainloop()
    finally:
        node.close_node()
        if rclpy.ok(): rclpy.shutdown()


def main(): run_console()


if __name__ == '__main__':
    main()
