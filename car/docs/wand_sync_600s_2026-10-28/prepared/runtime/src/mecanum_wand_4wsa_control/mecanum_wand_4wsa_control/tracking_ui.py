"""Auto-fit display; its viewport is NEVER a command boundary."""
from dataclasses import asdict
import math
from tkinter import ttk
from .core import Pose
from .large_type import widgets, variable_label


def label_launcher(app):
    app.root.title('无固定围栏 · 4WSA · 检查和准备不运动')
    labels=[w for w in app.detail.master.winfo_children() if isinstance(w,ttk.Label)]
    labels[0].configure(text='真实Sign跟随 · 无固定电子围栏')
    labels[1].configure(text='位移/转角1:1 · 0.70m/s、0.80m/s² · 最长600秒\n'
        'Vicon可见不代表无障碍；定位丢失/通信故障仍停车')
    captions=(
        '小车已落地放稳；Sign可见拿稳；本轮启动时记录位置和朝向',
        '已人工确认本轮全部活动路径及停车余量空旷；接线固定，无障碍检测',
        '有人能立即断电；无固定围栏，松键不停，再次空格 / Esc停止')
    checks=[w for w in app.detail.master.winfo_children() if isinstance(w,ttk.Checkbutton)]
    for w,text in zip(checks,captions): w.configure(text=text)


class TrackingConsoleMixin:
    def __init__(self,*args,**kw):
        super().__init__(*args,**kw)
        self.root.title('无固定围栏 · 平移＋旋转1:1 · 最长600秒')
        top=self.root.winfo_children()[0].winfo_children()
        top[0].configure(text='真实 Sign → 小车 · 无固定电子围栏')
        top[1].configure(text=f'档{self.config.stage} · 1:1 · 平移≤{self.config.max_speed_mps:.2f}m/s · ' 
            f'旋转≤{self.config.max_yaw_rate_radps:.2f}rad/s / {self.config.max_yaw_accel_radps2:.2f}rad/s² · 最长600秒')
        for w in self.map.master.winfo_children():
            if 'text' in w.keys() and str(w.cget('text')).startswith('蓝：'):
                w.configure(text='蓝：实测车体  橙：目标  青：跟踪点（视图自动缩放，非限位）',
                            font=('Noto Sans CJK SC',-20))

    def map_point(self,x,y):
        xmin,xmax,ymin,ymax=getattr(self,'view_bounds',(-1.,1.,-1.,1.))
        width,height=max(2,self.map.winfo_width()),max(2,self.map.winfo_height())
        scale=max(.01,min(max(1,width-60)/(xmax-xmin),max(1,height-70)/(ymax-ymin)))
        return width/2+(x-(xmin+xmax)/2)*scale, (height+36)/2-(y-(ymin+ymax)/2)*scale

    def draw_tracking_map(self):
        c=self.map;c.delete('all');p=self.node.car
        status=self.model.status
        goal=status.get('mapped_goal') or status.get('goal')
        reference=status.get('goal')
        points=[(x,y) for x,y in self.node.trace if math.isfinite(x) and math.isfinite(y)]
        if p and p.valid(): points.append((p.x,p.y))
        for value in (goal,reference):
            if value and all(math.isfinite(value[k]) for k in ('x','y')):
                points.append((value['x'],value['y']))
        if not points: points=[(0.,0.)]
        xs,ys=zip(*points)
        # Render only: no clamp, source mutation, service or permit operation.
        cx,cy=(max(xs)+min(xs))/2,(max(ys)+min(ys))/2
        hx,hy=max(1.,(max(xs)-min(xs))/2+.4),max(1.,(max(ys)-min(ys))/2+.4)
        self.view_bounds=(cx-hx,cx+hx,cy-hy,cy+hy)
        c.create_text(12,8,anchor='nw',text='无固定围栏；Vicon定位有效 ≠ 周围无障碍',
                      fill='#a23925',font=('Noto Sans CJK SC',-20),tags='tracking_warning')
        trace=[v for x,y in self.node.trace for v in self.map_point(x,y)]
        if len(trace)>=4:c.create_line(*trace,fill='#90b9d9',width=2,tags='car_trace')
        if p and p.valid():
            corners=[]
            for x,y in ((.095,.105),(.095,-.105),(-.095,-.105),(-.095,.105)):
                corners.extend(self.map_point(p.x+math.cos(p.yaw)*x-math.sin(p.yaw)*y,
                                              p.y+math.sin(p.yaw)*x+math.cos(p.yaw)*y))
            c.create_polygon(*corners,fill='#2986c1',outline='#1b5b86',width=2,tags='car_body')
            c.create_line(*self.map_point(p.x,p.y),*self.map_point(p.x+.25*math.cos(p.yaw),p.y+.25*math.sin(p.yaw)),
                          fill='white',width=3,arrow='last')
        for value,color,tag in ((goal,'#e49620','mapped_goal'),(reference,'#008b8b','reference')):
            if value:
                x,y=self.map_point(value['x'],value['y'])
                c.create_oval(x-6,y-6,x+6,y+6,outline=color,width=3,tags=tag)
        if goal:
            c.create_line(*self.map_point(goal['x'],goal['y']),
                *self.map_point(goal['x']+.25*math.cos(goal['yaw']),goal['y']+.25*math.sin(goal['yaw'])),
                fill='#e49620',width=3,arrow='last',tags='rotation_target')

    def tracking_hand_text(self):
        s=self.model.status;a=s.get('anchors');w=self.node.raw_wand;p=self.node.car
        if w is None or p is None:return '等待真实Sign和car01；无有效定位不可使能'
        origin=Pose(**a['wand_origin']) if a else w
        turn=a.get('turn',0.) if a else 0.
        return (f'杆 ΔX={w.x-origin.x:+.3f}m  ΔY={w.y-origin.y:+.3f}m\n'
                f'杆转角 {math.degrees(turn):+.1f}°；Z不映射\n'
                '1:1；无目标限幅 / 无位移半径限制\n'
                '定位/通信超时、异常仍停车\n'
                '松键不停；再次空格 / Esc停止')

    def draw_pad(self):
        self.pad.delete('all') # Hidden real-input pad; never a range gate.
