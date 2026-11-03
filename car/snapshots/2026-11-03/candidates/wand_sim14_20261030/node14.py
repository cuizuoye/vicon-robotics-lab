"""Independent supervisor/gate process; same strict source and timing checks."""
import argparse
import os
from pathlib import Path
import signal
import time
from profile14 import runtime,install,save

def run(a):
    rt=runtime(a.namespace);install()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from mecanum_wand_4wsb_hardware.optimized_nodes import Supervisor,Gate
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    node=(Supervisor if a.role=='supervisor' else Gate)(rt);executor=SingleThreadedExecutor();executor.add_node(node)
    done=False
    def finish(*_):
        nonlocal done
        done=True
    for s in (signal.SIGTERM,signal.SIGINT):signal.signal(s,finish)
    try:
        while not done:executor.spin_once(timeout_sec=.002)
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--namespace',required=True);p.add_argument('--role',choices=('supervisor','gate'),required=True)
    run(p.parse_args())
