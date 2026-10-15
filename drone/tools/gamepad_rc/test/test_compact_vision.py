import math
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compact_vision as c
from trs_vicon_receive_bench import allowed


def test_explicit_fc_yaw_alignment_rotation_and_roundtrip():
    source=[math.cos(.3),0,0,math.sin(.3)]
    fc=[math.cos(-.2),0,0,math.sin(-.2)]
    angle=c.yaw_alignment(source,fc)
    assert angle==pytest.approx(-1)
    msg,p,q=c.message(m.MAVLink(None),10000,[1,0,-1],source,angle)
    assert p==pytest.approx([math.cos(-1),math.sin(-1),-1])
    assert c.euler(q)==pytest.approx([0,0,-.4])
    assert c.pose_allowed(msg)
    packet=msg.pack(m.MAVLink(None))
    assert len(packet)==128
    decoded=m.MAVLink(None).parse_char(packet)
    assert decoded.get_type()=='VISION_POSITION_ESTIMATE'
    assert decoded.yaw==pytest.approx(-.4,abs=1e-6)
    assert all(decoded.covariance[i]>0 for i in (0,6,11,15,18,20))


def test_unknown_alignment_tilt_conflict_and_singularity_rejected():
    with pytest.raises(ValueError):
        c.message(m.MAVLink(None),1,[0,0,0],[1,0,0,0],math.nan)
    with pytest.raises(ValueError):
        c.yaw_alignment([math.cos(.1),math.sin(.1),0,0],[1,0,0,0])
    with pytest.raises(ValueError):
        c.euler([math.sqrt(.5),0,math.sqrt(.5),0])


def test_compact_transmission_is_an_explicit_trs_receive_only_exception():
    enc=m.MAVLink(None)
    msg,_,_=c.message(enc,10,[1,2,3],[1,0,0,0],0)
    assert allowed(msg,route='trs',safe=True,bench_confirmed=True,compact_timing=True)
    for key in ('safe','bench_confirmed','compact_timing'):
        gates=dict(safe=True,bench_confirmed=True,compact_timing=True)
        gates[key]=False
        assert not allowed(msg,route='trs',**gates)
    assert not allowed(msg,route='usb',safe=True,bench_confirmed=True,compact_timing=True)
    for command in (400,185,176,246,511):
        assert not allowed(enc.command_long_encode(1,1,command,0,1,0,0,0,0,0,0),
            route='trs',safe=True,bench_confirmed=True,compact_timing=True)
    msg.covariance[0]=0
    assert not allowed(msg,route='trs',safe=True,bench_confirmed=True,compact_timing=True)


def row_and_snapshot():
    row={'pc_sent_us':1_050_000,'wire_time_usec':1_040_000,'source_stamp_ns':123,
         'position':[1,2,-3],'q':[1,0,0,0]}
    snap={'pose_frame':1,'reset_counter':0,'position':[1,2,-3],'q':[1,0,0,0],
          'timestamp':70_000,'timestamp_sample':69_999}
    before={'remote_timestamp':1_000_000,'estimated_offset':-1_000_000,
            'observed_offset':-1_000_000,'round_trip_time':1000}
    after={**before,'remote_timestamp':1_200_000}
    return row,snap,before,after


def test_same_source_arrival_measurement_preserves_clock_scope():
    row,snap,before,after=row_and_snapshot()
    measurement=c.arrival_measurement(row,snap,before,after)
    assert measurement['age_at_send_ms']==10
    assert measurement['send_to_fc_publication_ms']==20
    assert measurement['source_ros_publication_to_fc_ms']==30
    assert not measurement['camera_exposure_latency_measured']
    assert not measurement['wireless_clock_conversion_verified']
    assert measurement['fc_publication_minus_sample_us']==1


@pytest.mark.parametrize('problem',['unbracketed','long_rtt','negative_rtt','bad_residual','wrong_frame','wrong_pose','negative_delay'])
def test_invalid_clock_or_pose_does_not_generate_latency_result(problem):
    row,snap,before,after=row_and_snapshot()
    if problem=='unbracketed': after['remote_timestamp']=1_040_000
    if problem=='long_rtt': before['round_trip_time']=10000
    if problem=='negative_rtt': before['round_trip_time']=-1
    if problem=='bad_residual': before['observed_offset']+=3000
    if problem=='wrong_frame': snap['pose_frame']=2
    if problem=='wrong_pose': snap['position']=[2,2,-3]
    if problem=='negative_delay': snap['timestamp']=40000
    with pytest.raises(ValueError): c.arrival_measurement(row,snap,before,after)


def test_fixed_internal_series_query_is_usb_only_and_bounded():
    enc=m.MAVLink(None)
    msg=enc.serial_control_encode(10,2,0,0,len(c.SERIES_COMMAND),c.SERIES_COMMAND.ljust(70,b'\0'))
    assert allowed(msg,route='usb',safe=True,compact_timing=True)
    assert not allowed(msg,route='trs',safe=True,compact_timing=True)
    assert not allowed(msg,route='usb',safe=True)
    assert not allowed(msg,route='usb',compact_timing=True)


def test_transmit_frequency_must_not_replace_measured_internal_series_rate():
    rows=[{'timestamp':1_000_000+i*31_250} for i in range(12)]
    timing=c.series_timing(rows)
    assert timing['rate_hz']==32
    assert timing['maximum_publication_gap_ms']==31.25
    rows[5]['timestamp']=rows[4]['timestamp']
    with pytest.raises(ValueError): c.series_timing(rows)


def test_complete_series_parser_requires_twelve_fresh_blocks():
    block=('TOPIC: vehicle_visual_odometry\n timestamp: {stamp}\n timestamp_sample: {stamp}\n'
           ' pose_frame: 1\n velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
           ' position: [1, 2, -3]\n q: [1, 0, 0, 0]\n')
    text=''.join(block.format(stamp=1_000_000+i*31_250) for i in range(12))
    rows=c.visual_series(text)
    assert c.series_timing(rows)['rate_hz']==32
    assert c.visual_series(block.format(stamp=1)) is None
