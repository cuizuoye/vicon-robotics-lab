import importlib.util
from pathlib import Path
import struct
import unittest

spec=importlib.util.spec_from_file_location('probe',Path(__file__).parents[1]/'tools/probe.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)

class ProbeTests(unittest.TestCase):
    def test_request(self):
        self.assertEqual(len(probe.request(1,2,999)),32)
        self.assertEqual(probe.REQUEST.unpack(probe.request(1,2,999)),
                         (b'SAP1',1,1,0,1,2,999,0))
    def test_nonzero(self):
        for seq,sid in [(0,1),(1,0),(-1,1),(2**32,1)]:
            with self.assertRaises(ValueError):probe.request(seq,sid,1)
    def valid(self,**changes):
        values=dict(magic=b'SAR1',ver=1,kind=2,reserved=0,seq=1,sid=2,
                    host=999,rx=100,tx=102,boot=42,count=1,gap=0,tail=0)
        values.update(changes)
        return probe.REPLY.pack(*values.values())
    def test_reply(self):
        self.assertEqual(probe.reply(self.valid(),2,{1:999})['boot'],42)
    def test_reject_schema(self):
        for change in [dict(magic=b'SAP1'),dict(ver=2),dict(kind=1),dict(reserved=1),
                       dict(sid=3),dict(seq=8),dict(host=998),dict(tx=99),dict(tail=1)]:
            with self.assertRaises(ValueError):probe.reply(self.valid(**change),2,{1:999})
    def test_lengths(self):
        for n in [0,32,63,65,128]:
            with self.assertRaises(ValueError):probe.reply(bytes(n),2,{1:999})
    def test_stats(self):
        self.assertIsNone(probe.stats([])['p99_ms'])
        s=probe.stats([20,100,101,150,200])
        self.assertEqual((s['over_100ms'],s['at_least_150ms'],s['p99_ms']),(3,2,200))
    def test_no_motor_paths(self):
        root=Path(__file__).parents[1]
        source=(root/'main/app_main.c').read_text()
        self.assertNotIn('uart_driver_install(',source)
        self.assertNotIn('nvs_flash_erase(',source)
        self.assertIn('WIFI_STORAGE_RAM',source)
        self.assertNotIn('WIFI_MODE_APSTA',source)

if __name__=='__main__':unittest.main()
