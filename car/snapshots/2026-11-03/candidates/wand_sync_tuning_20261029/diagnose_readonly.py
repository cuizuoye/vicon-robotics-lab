"""Bounded real-source profiling; NEVER prepare/engage, private software chain only.

cProfile perturbs timing: its output locates CPU work, not acceptance latency.
"""
import argparse
import cProfile
import io
from pathlib import Path
import pstats
import time

from direct_reader import DirectReader
from isolated_reader import IsolatedReader
from model import Limits, ZERO, save
from ros_tuning import install, TraceLog


def run(folder, seconds, isolate=False, profile_cpu=False):
    folder.mkdir(parents=True, exist_ok=False)
    log = TraceLog(folder)
    module = install(log, Limits(), True)
    import rclpy
    reader = IsolatedReader() if isolate else DirectReader()
    reader.start()
    module.init_private()
    chain = module.Chain(reader, None)
    profile = cProfile.Profile()
    last = 0.
    def loop():
        nonlocal last
        began = time.monotonic()
        while time.monotonic()-began < seconds:
            chain.touch()
            chain.step()
            now = time.monotonic()
            if now-last >= .01:
                chain.snapshot()
                reader.status()
                last = now
            assert not chain.requested and not chain.prepare_pending
            assert chain.sup.core.state != 'ACTIVE' and chain.plant.command == ZERO
    try:
        if profile_cpu: profile.runcall(loop)
        else: loop()
        status=reader.status()
        assert status['complete_packets'] > 100 and all(status['flags'].values()), status
        save(folder/'RESULT.json', dict(passed=True, seconds=seconds, reader=status,
            never_prepared=True, never_engaged=True, hardware_output=False,
            isolated_reader=isolate, cpu_profile=profile_cpu,
            profiling_overhead_not_acceptance_timing=profile_cpu))
    finally:
        chain.close()
        reader.close()
        rclpy.shutdown()
        log.close()
        for name, value in [('worker', profile)] if profile_cpu else []:
            stream = io.StringIO()
            pstats.Stats(value, stream=stream).sort_stats('cumulative').print_stats(55)
            (folder/(name+'_profile.txt')).write_text(stream.getvalue())


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--seconds', type=float, default=15.)
    p.add_argument('--isolate', action='store_true')
    p.add_argument('--profile', action='store_true')
    args = p.parse_args()
    if not 1 <= args.seconds <= 30: p.error('bounded 1..30 seconds only')
    run(args.output, args.seconds, args.isolate, args.profile)
