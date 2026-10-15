"""仅用于独立固定F50无线悬空镜像的100字节诊断。"""
import struct
from .transport_protocol import ProtocolError

STATUS=struct.Struct('<8I8I4h4h4h3hBBHH')
TAG=0x31425734


def decode_bench_status(payload):
    if len(payload)!=STATUS.size: raise ProtocolError('not 100-byte wireless bench diagnostics')
    v=STATUS.unpack(payload)
    if v[0]!=TAG or not 1<=v[3]<=v[4]<=30 or v[5]>1000:
        raise ProtocolError('wrong bench identity or sampling deadline')
    cc=list(v[8:16]); duty=list(v[16:20]); target=list(v[20:24]); filtered=list(v[24:28])
    body=list(v[28:31]); mask,wheel,wdg,permit=v[31:35]
    if mask>15 or wheel>4 or wdg or permit not in (0,1) or v[7]>500:
        raise ProtocolError('invalid bench safety state')
    active=body==[50,0,0]
    if body not in ([0,0,0],[50,0,0]): raise ProtocolError('not fixed F50')
    if active:
        if not permit or not all(250<=t<=300 for t in target):
            raise ProtocolError('active output without permit or four targets')
    elif any(duty+target+filtered+cc) or mask:
        raise ProtocolError('nonzero stopped controller/registers')
    for i,d in enumerate(duty):
        sign=(1,-1,1,-1)[i]
        if d*sign<0 or abs(d)>100 or d%10:
            raise ProtocolError('wrong polarity, duty resolution or 10% limit')
        expected=([100,100-d//10] if d>0 else
                  [100-abs(d)//10,100] if d<0 else [0,0])
        if cc[2*i:2*i+2]!=expected: raise ProtocolError('actual CCR mapping mismatch')
        if mask&(1<<i) and d!=80*sign: raise ProtocolError('startup duty mismatch')
        if not 0<=filtered[i]<=1000: raise ProtocolError('filtered speed outside bench bound')
    return dict(tag='4WB1',ticks=v[1],updates=v[2],dt_min=v[3],dt_max=v[4],irq_us_max=v[5],
                started_ms=v[6],stationary_ms=v[7],compares=cc,raw_duty=duty,target_cps=target,
                filtered_cps=filtered,body_milli=body,startup_mask=mask,fault_wheel=wheel,
                watchdog_reset=wdg,permitted=permit,active=active)
