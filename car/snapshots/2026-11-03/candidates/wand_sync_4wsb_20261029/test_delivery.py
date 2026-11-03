"""File/protocol regressions; never open ROS, a programmer or a robot link."""
import argparse
import importlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

from verify_bundle import check_hex,verify,sha

OUT=None
CONTRACT=None


class DeliveryTests(unittest.TestCase):
    def payload(self, stage=7): return (OUT/f'native/active{stage}.bin').read_bytes()

    def test_actual_c_snapshots_for_all_eight_stages(self):
        for stage in range(8):
            with self.subTest(stage=stage):
                value=CONTRACT.decode_status(self.payload(stage))
                self.assertEqual(value['stage'],stage)
                self.assertEqual(value['tag'],'4WSB')
                self.assertEqual(value['output_authority_version'],7)
                self.assertEqual(value['absolute_run_limit_ms'],630000)

    def test_experimental_tier_exact_policy(self):
        value=CONTRACT.decode_status(self.payload())
        self.assertEqual(value['wheel_target_slew_guard_cps_s'],33000.)
        self.assertAlmostEqual(value['yaw_rate_limit'],.10,places=6)
        self.assertAlmostEqual(value['yaw_accel_limit'],.40,places=6)
        self.assertAlmostEqual(struct.unpack_from('<f',self.payload(),168)[0],4.)

    def test_stage_guard_and_limits_cannot_be_faked(self):
        for offset,value in [(204,26000.),(204,float('nan')),(168,2.),(184,2.),(188,12.)]:
            raw=bytearray(self.payload());struct.pack_into('<f',raw,offset,value)
            with self.subTest(offset=offset,value=value),self.assertRaises(ValueError):
                CONTRACT.decode_status(raw)
        raw=bytearray(self.payload(0));struct.pack_into('<f',raw,204,33000.)
        with self.assertRaises(ValueError): CONTRACT.decode_status(raw)

    def test_old_and_new_statuses_mutually_reject(self):
        root=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
        parent=root/'docs/wand_sync_600s_2026-10-28/prepared'
        sys.path.insert(0,str(parent/'runtime/src/mecanum_wand_4wsa_hardware'))
        sys.path.insert(0,str(root/'src/mecanum_vicon_control'))
        old=importlib.import_module('mecanum_wand_4wsa_hardware.stage0_contract')
        with self.assertRaises(ValueError):old.decode_status(self.payload())
        with self.assertRaises(ValueError):
            CONTRACT.decode_status((parent/'native/rotation_stage6.bin').read_bytes())
        raw=bytearray(self.payload());raw[:4]=b'4WSA'
        with self.assertRaises(ValueError):CONTRACT.decode_status(raw)

    def test_production_flags_and_crc(self):
        payload=self.payload();header=CONTRACT.HEADER.pack(b'MC',1,0x31,8,10,17,1000,len(payload))
        frame=header+payload;frame+=struct.pack('<I',zlib.crc32(frame))
        self.assertEqual(CONTRACT.decode_status_frame(frame)['stage'],7)
        raw=bytearray(frame);raw[30]^=1
        with self.assertRaises(ValueError):CONTRACT.decode_status_frame(raw)
        header=CONTRACT.HEADER.pack(b'MC',1,0x31,2,10,17,1000,len(payload))
        raw=header+payload;raw+=struct.pack('<I',zlib.crc32(raw))
        with self.assertRaises(ValueError):CONTRACT.decode_status_frame(raw)

    def test_stage_selection_no_bool_or_out_of_range(self):
        for bad in (True,-1,8,7.0,'7'):
            with self.subTest(stage=bad),self.assertRaises(ValueError):CONTRACT.tier_payload(bad)
        self.assertEqual(CONTRACT.tier_payload(7),struct.pack('<BBH',0,0,0x7B07))

    def test_hex_and_bin_exact_match_and_corruption(self):
        binary=(OUT/'arm/mecanum_f407_4wsb_staged_NOT_FLASHED.bin').read_bytes()
        path=OUT/'arm/mecanum_f407_4wsb_staged_NOT_FLASHED.hex'
        self.assertTrue(check_hex(path,binary,0x08000000)['hex_bin_equal'])
        with tempfile.TemporaryDirectory() as tmp:
            bad=Path(tmp)/'bad.hex';data=path.read_text();bad.write_text(data[:10]+'0'+data[11:])
            # Guarantee corruption even if the old nibble was already zero.
            bad.write_text(data[:10]+('1' if data[10]!='1' else '2')+data[11:])
            with self.assertRaises(ValueError):check_hex(bad,binary,0x08000000)

    def test_portable_checker_does_not_open_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            for extension in ('bin','hex'):
                (folder/('app.'+extension)).write_bytes((OUT/'arm'/('mecanum_f407_4wsb_staged_NOT_FLASHED.'+extension)).read_bytes())
            metadata=dict(identity='4WSB',authority_version=7,diagnostic_payload_bytes=208,
                bin='app.bin',hex='app.hex',image_bytes=(folder/'app.bin').stat().st_size,
                flash_address='0x08000000',host_migration_complete=False)
            (folder/'BUNDLE.json').write_text(json.dumps(metadata))
            (folder/'SHA256SUMS').write_text('\n'.join(sha(p)+'  '+p.name for p in folder.iterdir() if p.is_file())+'\n')
            def deny(*args,**kwargs):raise AssertionError('unexpected device/process IO')
            with patch.object(socket,'socket',deny),patch.object(subprocess,'Popen',deny):
                result=verify(folder)
            self.assertTrue(result['passed']);self.assertFalse(result['hardware_opened'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepared',required=True,type=Path)
    a=p.parse_args();OUT=a.prepared.resolve();sys.path.insert(0,str(OUT))
    CONTRACT=importlib.import_module('fourwsb_wire.contract')
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(DeliveryTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
