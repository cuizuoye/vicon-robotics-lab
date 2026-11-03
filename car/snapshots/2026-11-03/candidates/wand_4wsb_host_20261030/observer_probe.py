"""Exercise the unchanged production observer in a private software graph."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
from split_runtime import runtime
from prepare import save


def run(a):
    rt=runtime(a.namespace,a.stage)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsb_hardware import pilot_observer as module
    # Constructor dependency injection ONLY in this software-test interpreter.
    # Observer's callbacks, serialization, flush and QoS remain production code.
    module.runtime_from_environment=lambda:rt
    module.NAMESPACE=a.namespace
    os.environ['WAND_PILOT_TRACE']=str(a.output/'production_observer.jsonl')
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    node=module.Observer();executor=SingleThreadedExecutor();executor.add_node(node)
    done=False;started=time.monotonic()
    def finish(*_):
        nonlocal done
        done=True
    for number in (signal.SIGTERM,signal.SIGINT):signal.signal(number,finish)
    try:
        while not done:executor.spin_once(timeout_sec=.002)
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown()
        save(a.output/'RESULT.json',dict(production_observer=True,elapsed_s=time.monotonic()-started,
            bytes=(a.output/'production_observer.jsonl').stat().st_size,
            hardware_opened=False,production_source_modified=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--namespace',required=True)
    p.add_argument('--stage',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
