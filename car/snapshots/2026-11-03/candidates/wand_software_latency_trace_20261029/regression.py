"""Actual Jazzy + Tk profiling and inherited safety regression, SYNTHETIC input."""
import argparse
import json
import math
from pathlib import Path
import time

from trace_log import TraceLog
from instrument import install
from bootstrap import ZERO, save, protected_hashes, input_hashes
from analyze import run as analyze


def timed_chain(folder):
    log = TraceLog(folder)
    module = install(log)
    import rclpy
    from console import Window
    import tkinter as tk
    import threading
    state = dict(x=0., y=0., yaw=0.)
    def provider(now, ros):
        # Fixture ONLY: known 1ms source->receipt age. Source.tick validates
        # using its earlier steady sample, so receipt must not be in its future.
        # This offset is never applied to real Sign data.
        return module.synthetic_packet(state['x'], state['y'], state['yaw'], now, ros-.001)
    module.init_private()
    chain = module.Chain(provider, None)
    class Adapter:
        def __init__(self):
            self.trace = log
            self.snapshot = {}
            self.ui_stamp = 0.
            self.done = threading.Event()
        def post(self, action):
            from app import Worker
            Worker.dispatch(chain, action, time.monotonic())
    adapter = Adapter()
    root = tk.Tk()
    root.withdraw()
    window = Window(root, adapter)
    # Hidden real Tk draw calls, without pretending pixels were presented.
    def spin(seconds, change=None, predicate=None):
        start = time.monotonic()
        previous_snapshot = 0.
        while time.monotonic()-start < seconds:
            elapsed = time.monotonic()-start
            if change:
                change(elapsed)
            chain.touch()
            chain.step()
            now = time.monotonic()
            if now-previous_snapshot > .04:
                snapshot = chain.snapshot()
                window.draw(snapshot)
                previous_snapshot = now
            if predicate and predicate():
                return True
        return False
    try:
        assert spin(6., predicate=lambda:chain.snapshot()['can_prepare']), chain.snapshot()
        assert chain.prepare()
        assert spin(5., predicate=lambda:chain.ready() and not chain.prepare_pending), chain.snapshot()
        assert chain.plant.command == ZERO
        assert chain.engage()
        assert spin(1., predicate=lambda:chain.sup.core.state == 'ACTIVE'), chain.snapshot()
        def move(t):
            state.update(x=.20*(1-math.cos(2*t)), y=.08*math.sin(t), yaw=.03*math.sin(t))
        spin(4., move)
        assert chain.sup.core.state == 'ACTIVE' and not chain.gate.core.fault, chain.snapshot()
        chain.stop('PROFILE_TEST_ESC')
        assert spin(2., predicate=lambda:chain.plant.command == ZERO and chain.sup.core.stop_confirmed)
        save(folder/'CONTRACT.json', dict(real_sign=False, synthetic_input=True, real_car_started=False))
        save(folder/'SESSION_RESULT.json', chain.snapshot())
    finally:
        window.close()
        chain.close()
        rclpy.shutdown()
        log.close()
    report = analyze(folder)
    assert report['sample_valid'], {k:report[k] for k in ('coverage','negative_intervals','sample_valid')}
    assert report['coverage']['controls_verified'] >= 100
    assert report['coverage']['controls_in_car_draw'] >= 40
    assert not report['negative_intervals']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--safety', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before, inputs = protected_hashes(), input_hashes()
    save(args.output/'PROTECTED_BEFORE.json', before)
    timed_folder = args.output/'paired_trace'
    timed_folder.mkdir()
    timed = timed_chain(timed_folder)
    cases = 0
    if args.safety:
        # Existing 15-case actual ROS suite, now with all profiling hooks on.
        folder = args.output/'inherited_safety'
        folder.mkdir()
        log = TraceLog(folder)
        install(log)
        import test_ros_chain
        try:
            test_ros_chain.run(folder)
            cases = len(test_ros_chain.CASES)
        finally:
            log.close()
    assert protected_hashes() == before and input_hashes() == inputs
    result = dict(passed=True, actual_ros=True, actual_tk=True, timing_trace=True,
                  inherited_safety_cases=cases, original_unchanged=True, inputs_unchanged=True,
                  real_sign=False, real_car_started=False, firmware_changed=False,
                  coverage=timed['coverage'])
    save(args.output/'RESULT.json', result)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
