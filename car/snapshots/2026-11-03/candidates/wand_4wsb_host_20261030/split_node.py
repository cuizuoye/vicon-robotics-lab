"""Run an ACTUAL private supervisor or gate in its own interpreter; no IO bridge."""
import argparse
import gc
import json
import os
import math
from pathlib import Path
import signal
import time
from split_runtime import runtime
from prepare import save
from trace_spool import TraceSpool


def run(a):
    rt=runtime(a.namespace,a.stage)
    if len(os.environ.get('WAND_4WSB_CHANNEL_KEY',''))!=64:raise ValueError('ephemeral test channel key missing')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsb_hardware.optimized_nodes import Supervisor,Gate
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    Base=Supervisor if a.role=='supervisor' else Gate
    gaps=[];previous=None;done=False;gc_at={};gc_pauses=[]
    def collect(phase,info):
        g=info['generation']
        if phase=='start':gc_at[g]=time.monotonic()
        else:
            dt=time.monotonic()-gc_at.pop(g,time.monotonic())
            if dt>.005:gc_pauses.append(dt*1000)
    gc.callbacks.append(collect)
    class Measured(Base):
        def on_candidate(self,msg):
            value=json.loads(msg.data)['frame']
            spool.emit(dict(kind='gate_callback_enter',at=time.monotonic(),seq=value['sequence'],
                issued=value['issued_steady'],pending=len(self.pending)))
            return super().on_candidate(msg)
        def tick(self):
            nonlocal previous
            now=time.monotonic()
            if previous is not None:gaps.append(now-previous)
            previous=now
            return super().tick()
    node=Measured(rt);executor=SingleThreadedExecutor();executor.add_node(node)
    spool=TraceSpool(a.output.with_suffix('.jsonl'))
    if a.role=='supervisor':
        original_publish=node.candidate.publish
        def publish(msg):
            value=json.loads(msg.data)
            spool.emit(dict(kind='control_publish',at=time.monotonic(),seq=value['sequence'],issued=value['issued_steady'],state=value['state']))
            return original_publish(msg)
        node.candidate.publish=publish
    else:
        original_receive=node.on_candidate
        # Existing subscription holds its constructor callback; wrap the
        # lower core receive as well so every accepted/delayed frame is seen.
        core_receive=node.core.receive
        def receive(frame,now,ros):
            r=core_receive(frame,now,ros)
            spool.emit(dict(kind='gate_core_receive',at=time.monotonic(),seq=frame.sequence,
                issued=frame.issued_steady,accepted=bool(r),fault=node.core.fault))
            return r
        node.core.receive=receive
    def health():
        frame=node.core.frame if a.role=='gate' else None
        inputs=node.core.inputs
        def finite(v):return v if v is not None and math.isfinite(v) else None
        spool.emit(dict(kind='health',at=time.monotonic(),role=a.role,
            state=getattr(node.core,'state',None),fault=getattr(node.core,'fault',None),
            seq=frame.sequence if frame else None,issued=frame.issued_steady if frame else None,
            pending=len(getattr(node,'pending',[])),selection_failure=getattr(node,'selection_failure',None),
            car_speed=finite(inputs.car_speed),car_yaw_rate=finite(inputs.car_yaw_rate),
            stationary_since=inputs.stationary_since,feedback=inputs.feedback,
            stop_started=getattr(node.core,'stop_started',None),
            stop_confirmed=getattr(node.core,'stop_confirmed',None)))
    node.create_timer(.02,health)
    def finish(*_):
        nonlocal done
        done=True
    for number in (signal.SIGTERM,signal.SIGINT):signal.signal(number,finish)
    try:
        while not done:executor.spin_once(timeout_sec=.002)
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown();gc.callbacks.remove(collect)
        values=sorted(gaps)
        logging=spool.close()
        save(a.output,dict(role=a.role,samples=len(values),p95_ms=values[int((len(values)-1)*.95)]*1000 if values else None,
            max_ms=max(values)*1000 if values else None,gc_pauses_over5ms=gc_pauses,
            logging=logging,
            hardware_bridge_constructed=False,software_only=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--namespace',required=True)
    p.add_argument('--stage',type=int,choices=range(8),required=True)
    p.add_argument('--role',choices=('supervisor','gate'),required=True)
    p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
