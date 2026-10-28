"""Retained-item remote rendering. No model/input/permit/deadline changes."""
import math
import time
from .async_x11 import remote_display


class ChangedVariable:
    def __init__(self,target):self.target=target;self.last=object()
    def get(self):return self.target.get()
    def set(self,value):
        if value!=self.last:self.target.set(value);self.last=value


class ChangedButton:
    def __init__(self,target):self.target=target;self.last={}
    def configure(self,**values):
        changed={k:v for k,v in values.items() if self.last.get(k)!=v}
        if changed:self.target.configure(**changed);self.last.update(changed)


class RemoteRenderMixin:
    def draw(self,now):
        if not remote_display():return super().draw(now)
        variables=('status_text','step_text','hand_text','notice')
        if not hasattr(self,'_remote_variable_cache'):
            self._remote_variable_cache={k:ChangedVariable(getattr(self,k)) for k in variables}
            self._remote_button_cache={k:ChangedButton(v) for k,v in self.buttons.items()}
        originals={k:getattr(self,k) for k in variables};buttons=self.buttons
        try:
            for k,v in self._remote_variable_cache.items():setattr(self,k,v)
            self.buttons=self._remote_button_cache
            start=time.monotonic();super().draw(now)
            elapsed=time.monotonic()-start
            if elapsed>.06 and not getattr(self,'_remote_slow_draw_reported',False):
                self._remote_slow_draw_reported=True
                self.node.get_logger().warn('REMOTE_DRAW_OVERRUN seconds='+str(elapsed))
        finally:
            for k,v in originals.items():setattr(self,k,v)
            self.buttons=buttons

    def map_point(self,x,y):
        if not remote_display():return super().map_point(x,y)
        xmin,xmax,ymin,ymax=self.view_bounds;w,h=self._remote_map_size
        scale=max(.01,min(max(1,w-60)/(xmax-xmin),max(1,h-70)/(ymax-ymin)))
        return w/2+(x-(xmin+xmax)/2)*scale,(h+36)/2-(y-(ymin+ymax)/2)*scale

    def draw_tracking_map(self):
        if not remote_display():return super().draw_tracking_map()
        c=self.map;p=self.node.car;s=self.model.status
        goal=s.get('mapped_goal') or s.get('goal');reference=s.get('goal')
        trace=list(self.node.trace);stride=max(1,math.ceil(len(trace)/120));trace=trace[::stride]
        if self.node.trace and (not trace or trace[-1]!=self.node.trace[-1]):trace.append(self.node.trace[-1])
        points=[v for v in trace if all(math.isfinite(x) for x in v)]
        if p and p.valid():points.append((p.x,p.y))
        for v in (goal,reference):
            if v and all(math.isfinite(v[k]) for k in ('x','y')):points.append((v['x'],v['y']))
        if not points:points=[(0.,0.)]
        xs,ys=zip(*points);cx,cy=(max(xs)+min(xs))/2,(max(ys)+min(ys))/2
        hx,hy=max(1.,(max(xs)-min(xs))/2+.4),max(1.,(max(ys)-min(ys))/2+.4)
        self.view_bounds=cx-hx,cx+hx,cy-hy,cy+hy
        self._remote_map_size=max(2,c.winfo_width()),max(2,c.winfo_height())
        if not hasattr(self,'_remote_map_items'):
            c.delete('all')
            c.create_text(12,8,anchor='nw',text='无固定围栏；Vicon定位有效 ≠ 周围无障碍',
                fill='#a23925',font=('Noto Sans CJK SC',-20),tags='tracking_warning')
            self._remote_map_items=dict(
                trace=c.create_line(0,0,0,0,fill='#90b9d9',width=2,tags='car_trace'),
                car=c.create_polygon(0,0,0,0,0,0,0,0,fill='#2986c1',outline='#1b5b86',width=2,tags='car_body'),
                heading=c.create_line(0,0,0,0,fill='white',width=3,arrow='last'),
                goal=c.create_oval(0,0,0,0,outline='#e49620',width=3,tags='mapped_goal'),
                reference=c.create_oval(0,0,0,0,outline='#008b8b',width=3,tags='reference'),
                target=c.create_line(0,0,0,0,fill='#e49620',width=3,arrow='last',tags='rotation_target'))
        items=self._remote_map_items
        def coords(name,values):
            c.itemconfigure(items[name],state='normal' if values else 'hidden')
            if values:c.coords(items[name],*values)
        coords('trace',[v for x,y in trace for v in self.map_point(x,y)] if len(trace)>=2 else [])
        corners=[];heading=[]
        if p and p.valid():
            for x,y in ((.095,.105),(.095,-.105),(-.095,-.105),(-.095,.105)):
                corners.extend(self.map_point(p.x+math.cos(p.yaw)*x-math.sin(p.yaw)*y,
                    p.y+math.sin(p.yaw)*x+math.cos(p.yaw)*y))
            heading=[*self.map_point(p.x,p.y),*self.map_point(p.x+.25*math.cos(p.yaw),p.y+.25*math.sin(p.yaw))]
        coords('car',corners);coords('heading',heading)
        for name,value in (('goal',goal),('reference',reference)):
            xy=[]
            if value:
                x,y=self.map_point(value['x'],value['y']);xy=[x-6,y-6,x+6,y+6]
            coords(name,xy)
        coords('target',[] if not goal else [*self.map_point(goal['x'],goal['y']),
            *self.map_point(goal['x']+.25*math.cos(goal['yaw']),goal['y']+.25*math.sin(goal['yaw']))])
