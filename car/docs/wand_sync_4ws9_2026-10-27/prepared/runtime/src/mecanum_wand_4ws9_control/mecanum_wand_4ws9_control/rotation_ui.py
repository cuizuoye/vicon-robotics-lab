"""Large, explicit unaccepted-rotation pilot labels; key/stop logic inherited."""
import math
from .large_type import widgets


class RotationConsoleMixin:
    def __init__(self,*args,**kw):
        super().__init__(*args,**kw)
        self.root.title('平移＋旋转 · 实机候选 · 旋转尚未实测 · 最长120秒')
        top=self.root.winfo_children()[0].winfo_children()
        top[0].configure(text='真实 Sign → 小车平移＋旋转（首验候选）')
        top[1].configure(text=f'档{self.config.stage} · 1:1 · {self.config.max_speed_mps:.2f}m/s / {self.config.max_accel_mps2:.2f}m/s² · {self.config.max_yaw_rate_radps:.2f}rad/s / {self.config.max_yaw_accel_radps2:.2f}rad/s² · 120秒')

    def draw(self,now):
        super().draw(now)
        s=self.model.status;anchors=s.get('anchors');goal=s.get('mapped_goal') or s.get('goal')
        if anchors and goal and self.node.car is not None:
            initial=anchors['car_origin']['yaw'];turn=anchors.get('turn',0.)
            actual=math.atan2(math.sin(self.node.car.yaw-initial),math.cos(self.node.car.yaw-initial))
            lines=self.hand_text.get().splitlines()
            self.hand_text.set('\n'.join(lines[:2])+f'\n杆相对角 {math.degrees(turn):+.1f}°\n'
                f'车相对角 {math.degrees(actual):+.1f}°\n转角1:1；档{self.config.stage}转速≤{math.degrees(self.config.max_yaw_rate_radps):.1f}°/秒\n'
                'Z不映射；松键不停，再次空格/Esc停')
            x,y=self.map_point(goal['x'],goal['y'])
            self.map.create_line(x,y,*self.map_point(goal['x']+.25*math.cos(goal['yaw']),
                goal['y']+.25*math.sin(goal['yaw'])),fill='#e49620',width=3,arrow='last',tags='rotation_target')


def label_launcher(app):
    app.root.title('平移＋旋转 · 实机首验候选 · 不自动运动')
    labels=[w for w in app.detail.master.winfo_children() if w.winfo_class()=='TLabel']
    labels[0].configure(text='小车平移＋旋转 · 待实机首验')
    labels[1].configure(text='位移/转角1:1 · 0.70m/s、0.80m/s² · 转速≤5.7°/秒\n'
        '不是A2高速角参数；检查和准备不运动，空格启停，最长120秒')
    checks=[w for w in app.detail.master.winfo_children() if w.winfo_class()=='TCheckbutton']
    checks[-1].configure(text='有人能立即断电；本轮含旋转且尚未实测；松键不停，空格/Esc停车')
