"""One independent 4WSC supervisor, gate or exclusive transport process."""
import argparse
import os
from pathlib import Path
import signal
import time
import bootstrap

def main():
    p=argparse.ArgumentParser();p.add_argument('--role',choices=('supervisor','gate','bridge'),required=True)
    a,ros_args=p.parse_known_args();rt=bootstrap.runtime();bootstrap.install()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from mecanum_wand_4wsc_hardware.optimized_nodes import Supervisor,Gate
    from mecanum_wand_4wsc_hardware.nodes import HardwareBridge
    rclpy.init(args=ros_args,domain_id=rt.domain,signal_handler_options=SignalHandlerOptions.NO)
    node=HardwareBridge() if a.role=='bridge' else (Supervisor if a.role=='supervisor' else Gate)(rt)
    executor=SingleThreadedExecutor();executor.add_node(node);done=False
    def end(*_):
        nonlocal done
        done=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,end)
    try:
        while not done:executor.spin_once(timeout_sec=.002)
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
