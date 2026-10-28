"""Reuse pinned, tested key/focus/stop UI; replace parameter display only."""
from ui import Window


class AccelerationWindow(Window):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.root.title('真实Sign → 仿真车 · A2加速度对照 · 实车无输出')
        def descendants(w):
            yield w
            for child in w.winfo_children(): yield from descendants(child)
        matches=[]
        for w in descendants(self.root):
            if 'text' in w.keys() and str(w.cget('text')).startswith('XY：0.70 m/s / 0.80 m/s²'):
                matches.append(w)
        if len(matches)!=1: raise ValueError('Pinned simulation parameter label layout changed')
        c=self.e.c
        matches[0].configure(text=f'XY：0.70 m/s / {c.max_accel_mps2:.2f} m/s²\n'
            f'旋转：2.00 rad/s / {c.max_yaw_accel_radps2:.2f} rad/s²\n'
            '120秒；并非实车动力学标定\n活动区：前0.5 / 后1.5 m\n左0.5 / 右0.4 m（沿用当前边界）')
