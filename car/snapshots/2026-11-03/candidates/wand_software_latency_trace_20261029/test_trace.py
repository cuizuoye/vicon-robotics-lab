"""Known-delay identity/coverage regressions; no hardware or Vicon needed."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest

from analyze import attribute
from trace_log import TraceLog


def fixture():
    cid = ['controller', 7, 'epoch']
    oid = ['gate', 13]
    ms = 1_000_000
    def row(kind, **value):
        return dict(kind=kind, **value)
    return [
        row('source_relay', sign='digest', source_local_ns=0, raw_first_ns=ms,
            raw_complete_ns=2*ms, begin_ns=5*ms, end_ns=5*ms+10,
            clock_pair_span_ns=1000),
        row('control_publish', cid=cid, sign='digest', state='ACTIVE', enabled=True,
            issued_ns=10*ms, begin_ns=11*ms, end_ns=11*ms+10),
        row('gate_receive', cid=cid, begin_ns=12*ms),
        row('gate_accept', cid=cid, begin_ns=12*ms, end_ns=13*ms, accepted=True),
        row('sign_encode', oid=oid, cid=cid, begin_ns=15*ms, end_ns=16*ms),
        row('signed_publish', oid=oid, cid=cid, begin_ns=17*ms, end_ns=17*ms+10,
            state='ACTIVE', permit=True),
        row('plant_receive', oid=oid, begin_ns=18*ms),
        row('verify_accept', oid=oid, begin_ns=18*ms, end_ns=19*ms, applied=True),
        row('model_step', model_seq=1, oid=oid, begin_ns=20*ms, end_ns=21*ms),
        row('snapshot', snapshot_seq=1, model_seq=1, oid=oid, cid=cid,
            begin_ns=24*ms, end_ns=25*ms),
        row('draw', draw_seq=1, snapshot_seq=1, model_seq=1, oid=oid, cid=cid,
            begin_ns=29*ms, end_ns=30*ms),
    ]


class TraceTests(unittest.TestCase):
    def test_known_exact_delays(self):
        result = attribute(fixture())
        self.assertEqual(result['metrics']['local_source_to_verified']['median_ms'], 19.)
        self.assertEqual(result['metrics']['local_source_to_car_draw']['median_ms'], 30.)
        self.assertEqual(result['metrics']['control_publish_to_gate_receive']['median_ms'], 1.)
        self.assertEqual(result['metrics']['verified_to_model_step']['median_ms'], 2.)
        self.assertEqual(result['negative_intervals'], [])

    def test_no_nearest_time_matching_when_id_missing(self):
        events = fixture()
        events[7]['oid'] = ['different_gate', 13]
        result = attribute(events)
        self.assertNotIn('local_source_to_car_draw', result['metrics'])
        self.assertEqual(result['coverage']['controls_verified'], 0)

    def test_display_identity_tamper_rejected(self):
        for field, bad in (('oid', ['wrong', 13]), ('model_seq', 2), ('cid', ['wrong', 7, 'epoch'])):
            events = fixture()
            events[-1][field] = bad
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'identity mismatch'):
                attribute(events)

    def test_repeated_draw_is_not_extra_source_sample(self):
        events = fixture()
        duplicate = dict(events[-1], draw_seq=2, begin_ns=39_000_000, end_ns=40_000_000)
        result = attribute(events+[duplicate])
        self.assertEqual(result['metrics']['local_source_to_car_draw']['n'], 1)
        self.assertEqual(result['metrics']['local_source_to_car_draw']['median_ms'], 30.)

    def test_duplicate_controller_identity_rejected(self):
        events = fixture()
        with self.assertRaisesRegex(ValueError, 'duplicate trace identity'):
            attribute(events+[deepcopy(events[1])])

    def test_negative_stage_not_hidden(self):
        events = fixture()
        events[2]['begin_ns'] = 9_000_000
        result = attribute(events)
        self.assertEqual(result['negative_intervals'][0]['metric'], 'control_publish_to_gate_receive')

    def test_rejected_accept_never_produces_completed_path(self):
        events = fixture()
        events[3]['accepted'] = False
        result = attribute(events)
        self.assertNotIn('local_source_to_verified', result['metrics'])
        self.assertEqual(result['paired_draw_paths'], [])

    def test_model_supersession_is_counted_not_fabricated(self):
        events = fixture()
        events = [e for e in events if e['kind'] not in ('model_step', 'snapshot', 'draw')]
        result = attribute(events)
        self.assertEqual(result['coverage']['verified_superseded_before_model'], 1)
        self.assertEqual(result['coverage']['controls_applied_to_model'], 0)

    def test_no_secret_or_control_fields_added_to_wire(self):
        text = Path(__file__).with_name('instrument.py').read_text()
        self.assertNotIn("payload['cid'] =", text)
        for name in ('HardwareBridge(', 'sendto(', 'Serial(', 'set_parameter('):
            self.assertNotIn(name, text)

    def test_bounded_writer_flush_and_producer_cost(self):
        with tempfile.TemporaryDirectory() as folder:
            log = TraceLog(folder)
            for i in range(100):
                log.emit('fixture', sequence=i)
            log.close()
            result = json.loads((Path(folder)/'TRACE_LOG_RESULT.json').read_text())
            self.assertEqual(result['written'], 100)
            self.assertEqual(result['dropped'], 0)
            self.assertEqual(result['error'], '')
            self.assertTrue(result['writer_stopped'])
            self.assertEqual(len(result['enqueue_cost_ns']), 100)

    def test_high_dpi_console_keeps_preparation_controls_visible(self):
        import tkinter as tk
        from console import Window
        root = tk.Tk()
        root.withdraw()
        root.tk.call('tk', 'scaling', 3.)
        worker = SimpleNamespace(snapshot={}, ui_stamp=0., done=threading.Event(),
                                 post=lambda action:None, trace=SimpleNamespace(emit=lambda *a, **k:None))
        window = Window(root, worker)
        try:
            window.state.set('状态：STANDBY\nSign：有效\n剩余：120.0秒\n末级门：正常\n停稳：是')
            root.update_idletasks()
            self.assertLess(root.winfo_reqheight(), 721)
            for button in (window.prepare_button, window.toggle_button):
                self.assertLessEqual(button.winfo_y()+button.winfo_height(), button.master.winfo_height())
        finally:
            window.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
