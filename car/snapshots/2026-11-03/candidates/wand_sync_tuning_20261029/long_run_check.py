"""60s synthetic-motion ROS stress with concurrent REAL Sign passive input.

Only the private signed software car is enabled, by this automated fixture.
Real Sign is read/validated as load, not used to command anything. No robot IO.
"""
import argparse
import math
from pathlib import Path
import time

from isolated_reader import IsolatedReader
from model import Limits, ZERO, save, protected_hashes, digest
from ros_tuning import install, TraceLog


def run(folder, seconds):
    folder.mkdir(parents=True, exist_ok=False)
    before = protected_hashes()
    implementation = {str(Path(__file__).with_name(name)):digest(Path(__file__).with_name(name))
        for name in ('dashboard.py','ros_tuning.py','model.py','packet_work.py',
                     'direct_reader.py','isolated_reader.py','long_run_check.py')}
    save(folder/'IMPLEMENTATION.json', implementation)
    save(folder/'PROTECTED_BEFORE.json', before)
    log = TraceLog(folder)
    module = install(log, Limits(.7, 4., 400.), True)
    reader = IsolatedReader()
    reader.start()
    import rclpy
    module.init_private()
    state = dict(x=0., y=0., yaw=0.)
    def provider(now, ros):
        reader(now, ros)  # Exercise actual read/IPC/geometry load; never motion input.
        return module.synthetic_packet(state['x'], state['y'], state['yaw'], now, ros-.001)
    provider.poll=reader.poll
    chain = module.Chain(provider, None)
    last = 0.
    def spin(duration, action=None, predicate=None):
        nonlocal last
        began = time.monotonic()
        while time.monotonic()-began < duration:
            chain.touch()  # AUTOMATED SYNTHETIC FIXTURE, not a live console.
            if action: action(time.monotonic()-began)
            chain.step()
            now = time.monotonic()
            if now-last >= .025:
                chain.snapshot()
                reader.status()
                last = now
            if predicate and predicate(): return True
        return False
    try:
        assert spin(6., predicate=lambda:chain.snapshot()['can_prepare']), chain.snapshot()
        assert chain.prepare()
        assert spin(5., predicate=lambda:chain.ready() and not chain.prepare_pending), chain.snapshot()
        assert chain.plant.command == ZERO
        assert chain.engage()
        assert spin(1., predicate=lambda:chain.sup.core.state == 'ACTIVE'), chain.snapshot()
        def move(t):
            state.update(x=.25*(1-math.cos(2.*t)), y=.15*math.sin(2.*t), yaw=.03*math.sin(t))
        spin(seconds, move)
        assert chain.sup.core.state == 'ACTIVE' and not chain.gate.core.fault, chain.snapshot()
        status = reader.status()
        assert status['ipc_fresh'] and status['complete_packets'] > 1000, status
        assert status['ipc_rejected']==0, status
        chain.stop('operator automated synthetic fixture stop')
        assert spin(3., predicate=lambda:chain.sup.core.stop_confirmed and chain.plant.command == ZERO), chain.snapshot()
        save(folder/'RESULT.json', dict(passed=True, synthetic_motion=True, active_s=seconds,
            concurrent_real_sign_readonly=True, reader=status, actual_ros=True,
            profile=chain.config.name, snapshot=chain.snapshot(), events=chain.events,
            real_sign_human_acceptance=False, hardware_output=False, firmware_changed=False))
    finally:
        chain.close()
        reader.close()
        rclpy.shutdown()
        log.close()
        save(folder/'PROTECTED_AFTER.json', protected_hashes())
        assert before == protected_hashes()
        assert all(digest(Path(path))==sha for path,sha in implementation.items())


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--seconds', type=float, default=60.)
    a=p.parse_args()
    if not 40 <= a.seconds <= 90: p.error('40..90 seconds only')
    run(a.output, a.seconds)
