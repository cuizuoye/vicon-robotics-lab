"""Manual UI for private real-Sign/SOFTWARE-C-car only; no auto click."""
import argparse
import os
from pathlib import Path
import signal
import time
from common import config,save
from split_runtime import runtime


def run(a):
    rt=runtime(a.namespace,a.stage)
    import tkinter as tk
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsb_control.operator_console import OperatorNode,Console,OperatorModel
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    model=OperatorModel();node=OperatorNode(model,rt);root=tk.Tk()
    class Measured(Console):
        def loop(self):
            super().loop()
            if not self.closing and time.monotonic()-getattr(self,'last_heartbeat',0.)>.2:
                self.last_heartbeat=time.monotonic()
                save(a.output/'HEARTBEAT.json',dict(t=self.last_heartbeat,state=model.status.get('state'),
                    reason=model.status.get('reason'),gate_ready=model.status.get('gate_ready'),
                    gate_fault=model.gate.get('fault'),instruction=self.step_text.get(),
                    raw_sign_valid=node.raw_wand is not None,session_requested=model.session_requested,
                    stage=a.stage,hardware_output=False))
    app=Measured(root,node,model)
    root.title('真实Sign → 4WSB软件车 · 档%d · 实车无输出'%a.stage)
    root.winfo_children()[0].winfo_children()[0].configure(text='真实Sign → ROS+C软件车 · 实车断电')
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,app.close))
    try:root.mainloop()
    finally:
        save(a.output/'RESULT.json',dict(final_state=model.status.get('state'),reason=model.status.get('reason'),
            pilot_used=model.pilot_used,activation_used=model.activation_used,events=model.events,
            hardware_opened=False,software_only=True))
        node.close_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--namespace',required=True)
    p.add_argument('--stage',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
