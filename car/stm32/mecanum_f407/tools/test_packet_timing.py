import ipaddress
import struct
import unittest
from packet_timing import analyze, pcap_records, ipv4_udp
from mecanum_vicon_control.transport_protocol import encode_frame, pack_cmd_vel, MessageType

PC='10.236.223.61';ESP='10.236.223.148'

def packet(seq, *, velocity=0, link=1, src=PC, dst=ESP, sid=9):
    payload=encode_frame(MessageType.CMD_VEL,seq,sid,20*seq,pack_cmd_vel(velocity,0,0))
    udp=struct.pack('!HHHH',46001,46000,8+len(payload),0)+payload
    ip=bytearray(20);ip[0]=0x45;ip[9]=17
    struct.pack_into('!H',ip,2,20+len(udp))
    ip[12:16]=ipaddress.IPv4Address(src).packed;ip[16:20]=ipaddress.IPv4Address(dst).packed
    prefix={1:b'\x00'*12+b'\x08\x00',101:b'',113:b'\x00'*14+b'\x08\x00',
            276:b'\x08\x00'+b'\x00'*18}[link]
    return prefix+ip+udp

def pcap(rows, *, link=1, endian='<', nanos=False):
    magic=0xa1b23c4d if nanos else 0xa1b2c3d4
    out=struct.pack(endian+'IHHIIII',magic,2,4,0,0,256,link)
    scale=10**9 if nanos else 10**6
    for ms,body in rows:
        out+=struct.pack(endian+'IIII',10+ms//1000,(ms%1000)*scale//1000,len(body),len(body))+body
    return out

def probe():
    return dict(sent=[dict(kind=16,sequence=s,session_id=9) for s in (1,2,3)],
                timing=dict(esp_fault=dict(session_id=9),esp_events=[
                    dict(kind=1,sequence=1,at_ms=1000),dict(kind=1,sequence=2,at_ms=1153)]))

class PacketTests(unittest.TestCase):
    def test_sequence_gap_correlation(self):
        r=analyze(pcap([(0,packet(1)),(20,packet(2))]),probe(),PC,ESP)
        self.assertEqual(r['captured_zero_commands'],2)
        self.assertAlmostEqual(r['max_capture_command_gap_ms'],20)
        self.assertEqual(r['sent_but_not_captured_sequences'],[[9,3]])
        self.assertEqual(r['matched_receive_intervals'][0]['esp_gap_ms'],153)
        self.assertAlmostEqual(r['matched_receive_intervals'][0]['capture_gap_ms'],20)

    def test_endian_precision_link_types(self):
        for link in (1,101,113,276):
            for endian in ('<','>'):
                for nanos in (False,True):
                    r=analyze(pcap([(0,packet(1,link=link))],link=link,endian=endian,nanos=nanos),probe(),PC,ESP)
                    self.assertEqual(r['captured_zero_commands'],1)

    def test_nonzero_rejected(self):
        with self.assertRaises(ValueError):
            analyze(pcap([(0,packet(1,velocity=0.1))]),probe(),PC,ESP)

    def test_other_peer_and_session_ignored(self):
        r=analyze(pcap([(0,packet(1,dst='10.236.223.99')),(20,packet(2,sid=8))]),probe(),PC,ESP)
        self.assertFalse(r['capture_available'])

    def test_truncation_rejected(self):
        data=pcap([(0,packet(1))])
        for invalid in (b'',data[:10],data[:30],data[:-1]):
            with self.assertRaises(ValueError):list(pcap_records(invalid))

    def test_fragment_ignored(self):
        raw=bytearray(packet(1));struct.pack_into('!H',raw,20,0x2000)
        self.assertIsNone(ipv4_udp(1,raw))

if __name__=='__main__':unittest.main()
