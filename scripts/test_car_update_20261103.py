#!/usr/bin/env python3
"""Portable unit adapter: source imports only; no ROS node/device is created."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / 'car/snapshots/2026-11-03'
OUT = CAR / 'docs/wand_static_source_2026-11-02/prepared_v1'


def main():
    # The archived builder depends on private admission receipts. Never run it.
    build = ModuleType('build')
    build.OUT = OUT
    sys.modules['build'] = build
    sys.path.insert(0, str(CAR / 'src/mecanum_vicon_control'))
    sys.path.insert(0, str(OUT / 'host'))
    spec = importlib.util.spec_from_file_location('public_pipeline_tests',
        CAR / 'candidates/wand_static_source_20261102/test_pipeline.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    suite = unittest.defaultTestLoader.loadTestsFromModule(module)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() and result.testsRun == 27 else 1


if __name__ == '__main__':
    raise SystemExit(main())
