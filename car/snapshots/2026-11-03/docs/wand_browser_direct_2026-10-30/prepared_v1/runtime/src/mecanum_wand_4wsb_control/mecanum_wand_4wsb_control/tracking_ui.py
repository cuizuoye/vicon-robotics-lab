"""Private persistent Canvas renderer; viewport is NEVER a motion boundary."""
import math
from .core import Pose


class TrackingConsoleMixin:
    def __init__(self,*args,**kw):
        self.tracking_items = None
        super().__init__(*args,**kw)
        self.root.title('4WSB · 平移＋旋转1:1 · 最长600秒')
        top=self.root.winfo_children()[0].winfo_children()
        software=getattr(self.node.runtime,'software_only',False)
        top[0].configure(text='4WSB 软件执行器验收 · 无实车输出' if software else '真实 Sign → 4WSB · 无固定电子围栏')
        top[1].configure(text=f'档{self.config.stage} · 平移≤{self.config.max_speed_mps:.2f}m/s / '
            f'{self.config.max_accel_mps2:.2f}m/s² · 旋转≤{self.config.max_yaw_rate_radps:.2f}rad/s · 最长600秒')
        for w in self.map.master.winfo_children():
            if 'text' in w.keys() and str(w.cget('text')).startswith('蓝：'):
                w.configure(text='蓝：反馈车体  橙：映射目标  青：受监督参考点（视图不是围栏）')

    def map_point(self,x,y):
        xmin,xmax,ymin,ymax=getattr(self,'view_bounds',(-1.,1.,-1.,1.))
        width,height=max(2,self.map.winfo_width()),max(2,self.map.winfo_height())
        scale=max(.01,min(max(1,width-60)/(xmax-xmin),max(1,height-70)/(ymax-ymin)))
        return width/2+(x-(xmin+xmax)/2)*scale,(height+36)/2-(y-(ymin+ymax)/2)*scale

    def draw_tracking_map(self):
        c=self.map;p=self.node.car;s=self.model.status
        if self.tracking_items is None:
            self.tracking_items=dict(
                warning=c.create_text(12,8,anchor='nw',text='Vicon定位有效 ≠ 周围无障碍；当前视图不是围栏',
                    fill='#a23925',font=('Noto Sans CJK SC',-20)),
                trail=c.create_line(0,0,0,0,fill='#90b9d9',width=2),
                car=c.create_polygon(0,0,0,0,0,0,fill='#2986c1',outline='#1b5b86',width=2),
                heading=c.create_line(0,0,0,0,fill='white',width=3,arrow='last'),
                goal=c.create_oval(0,0,0,0,outline='#e49620',width=3),
                reference=c.create_oval(0,0,0,0,outline='#008b8b',width=3),
                target_heading=c.create_line(0,0,0,0,fill='#e49620',width=3,arrow='last'))
        goal=s.get('mapped_goal') or s.get('goal');reference=s.get('goal')
        points=[(x,y) for x,y in self.node.trace if math.isfinite(x) and math.isfinite(y)]
        if p and p.valid():points.append((p.x,p.y))
        for value in (goal,reference):
            if value and all(math.isfinite(value[k]) for k in ('x','y')):points.append((value['x'],value['y']))
        if not points:points=[(0.,0.)]
        xs,ys=zip(*points);cx,cy=(max(xs)+min(xs))/2,(max(ys)+min(ys))/2
        hx,hy=max(1.,(max(xs)-min(xs))/2+.4),max(1.,(max(ys)-min(ys))/2+.4)
        self.view_bounds=(cx-hx,cx+hx,cy-hy,cy+hy)
        def show(name,coords=None):
            c.itemconfigure(self.tracking_items[name],state='normal' if coords is not None else 'hidden')
            if coords is not None:c.coords(self.tracking_items[name],*coords)
        trace=[v for x,y in self.node.trace for v in self.map_point(x,y)]
        show('trail',trace if len(trace)>=4 else None)
        if p and p.valid():
            corners=[]
            for x,y in ((.095,.105),(.095,-.105),(-.095,-.105),(-.095,.105)):
                corners.extend(self.map_point(p.x+math.cos(p.yaw)*x-math.sin(p.yaw)*y,
                    p.y+math.sin(p.yaw)*x+math.cos(p.yaw)*y))
            show('car',corners)
            show('heading',(*self.map_point(p.x,p.y),*self.map_point(p.x+.25*math.cos(p.yaw),p.y+.25*math.sin(p.yaw))))
        else:show('car');show('heading')
        for value,name in ((goal,'goal'),(reference,'reference')):
            if value:
                x,y=self.map_point(value['x'],value['y']);show(name,(x-6,y-6,x+6,y+6))
            else:show(name)
        if goal:
            show('target_heading',(*self.map_point(goal['x'],goal['y']),*self.map_point(
                goal['x']+.25*math.cos(goal['yaw']),goal['y']+.25*math.sin(goal['yaw']))))
        else:show('target_heading')

    def tracking_hand_text(self):
        s=self.model.status;a=s.get('anchors');w=self.node.raw_wand;p=self.node.car
        if w is None or p is None:return '等待Sign和小车新鲜定位；未就绪不可使能'
        origin=Pose(**a['wand_origin']) if a else w;turn=a.get('turn',0.) if a else 0.
        return (f'杆 ΔX={w.x-origin.x:+.3f}m  ΔY={w.y-origin.y:+.3f}m\n'
            f'相对转角 {math.degrees(turn):+.1f}°；Z不映射\n'
            '位移/角度1:1；无固定围栏\n定位/通信超时仍停车\n再次空格 / Esc停止；松键不停')

    def draw_pad(self):
        # The real-input pad is hidden. Do not rebuild invisible items.
        pass


def label_launcher(app):
    raise ValueError('legacy launcher is not a 4WSB admission entry')
