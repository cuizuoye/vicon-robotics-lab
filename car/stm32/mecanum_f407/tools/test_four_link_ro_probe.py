import struct
import unittest
from four_link_ro_probe import decode_status, decode_telemetry, TAG
from mecanum_vicon_control.transport_protocol import (
    Frame, MessageType, WheelTelemetry, pack_wheel_telemetry,
)

class ReadonlyTests(unittest.TestCase):
    def test_status(self):
        v = [TAG,100,100,10,10,40]+[0]*8+[1,0]
        self.assertEqual(decode_status(struct.pack('<6I8I2H',*v))['tag'],'4RO1')
        for field, value in [(0,0),(3,0),(4,31),(5,1001),(6,1),(13,1),(14,0),(15,1)]:
            bad=v.copy(); bad[field]=value
            with self.assertRaises(ValueError): decode_status(struct.pack('<6I8I2H',*bad))
        with self.assertRaises(ValueError): decode_status(b'')

    def test_telemetry(self):
        t=WheelTelemetry(1,(1,2,3,4),(10,-20,30,-40),0,109,0,2)
        f=Frame(MessageType.WHEEL_TELEMETRY,0,1,1,1,pack_wheel_telemetry(t))
        self.assertEqual(decode_telemetry(f)['encoder_counts'],(10,-20,30,-40))
        for flags, state, fault in [(8,109,0),(0,111,0),(0,1,0),(4,109,2),(4,109,0),(0,109,1)]:
            bad=Frame(f.message_type,flags,1,1,1,pack_wheel_telemetry(
                WheelTelemetry(1,t.wheel_rpm,t.encoder_counts,0,state,fault,2)))
            with self.assertRaises(ValueError): decode_telemetry(bad)
        timed=Frame(f.message_type,4,1,1,1,pack_wheel_telemetry(
            WheelTelemetry(1,t.wheel_rpm,t.encoder_counts,0,125,1,2)))
        self.assertEqual(decode_telemetry(timed,True)['fault_bits'],1)

if __name__=='__main__': unittest.main()
