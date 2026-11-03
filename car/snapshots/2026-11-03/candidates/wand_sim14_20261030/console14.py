"""Human single-click UI for SIM14 only; never synthesizes a start key."""
import argparse
from pathlib import Path
import signal
import time
from profile14 import runtime,save,contract

def run(a):
    rt=runtime(a.namespace)
    import tkinter as tk
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsb_control.operator_console import OperatorNode,Console,OperatorModel
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    model=OperatorModel();node=OperatorNode(model,rt);root=tk.Tk()
    class Measured(Console):
        last_heartbeat=0.
        def loop(self):
            super().loop();now=time.monotonic()
            if not self.closing and now-self.last_heartbeat>.2:
                self.last_heartbeat=now
                save(a.output/'HEARTBEAT.json',dict(t=now,state=model.status.get('state'),reason=model.status.get('reason'),
                    gate_ready=model.status.get('gate_ready'),gate_fault=model.gate.get('fault'),
                    instruction=self.step_text.get(),session_requested=model.session_requested,hardware_output=False))
    app=Measured(root,node,model);root.title('真实Sign → 1.40m/s软件车 · 最长600秒 · 无实车输出')
    top=root.winfo_children()[0].winfo_children()
    top[0].configure(text='真实Sign → 1.40m/s平移＋旋转仿真')
    top[1].configure(text='1:1 · 1.40m/s / 4.00m/s² · 2.00rad/s / 12.00rad/s² · 最长600秒 · 假设驱动800RPM')
    app.notice.set('数学执行器模型，非4WSB固件；实车保持断电。检查准备完成后单击空格，再次空格或Esc停止。')
    root.after(100,root.focus_force)
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:root.after(0,app.close))
    try:root.mainloop()
    finally:
        save(a.output/'RESULT.json',dict(state=model.status.get('state'),reason=model.status.get('reason'),
            events=model.events,pilot_used=model.pilot_used,activation_used=model.activation_used,**contract()))
        node.close_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--namespace',required=True);p.add_argument('--output',required=True,type=Path)
    run(p.parse_args())
