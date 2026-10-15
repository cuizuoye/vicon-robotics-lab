"""Validate the distinct 4WM1 bounded-streaming diagnostic identity, not 4WB1/4RO1."""
from .bench_link_status import STATUS
from .transport_protocol import ProtocolError

TAG=0x314D5734

def decode_motion_status(payload):
    if len(payload)!=STATUS.size:raise ProtocolError('not a 100-byte motion status')
    v=STATUS.unpack(payload)
    if v[0]!=TAG or not 1<=v[3]<=v[4]<=30 or v[5]>1000:
        raise ProtocolError('wrong motion identity or sampling deadline')
    cc=list(v[8:16]);duty=list(v[16:20]);target=list(v[20:24]);filtered=list(v[24:28])
    body=list(v[28:31]);mask,wheel,wdg,permit=v[31:35]
    if mask>15 or wheel>4 or wdg or permit not in (0,1) or v[7]>500:
        raise ProtocolError('invalid motion safety state')
    if abs(body[0])>50 or abs(body[1])>50 or abs(body[2])>200:
        raise ProtocolError('body velocity outside commissioning envelope')
    if any(abs(t)>300 for t in target) or any(abs(f)>10000 for f in filtered):
        raise ProtocolError('wheel value outside commissioning envelope')
    if not permit and any(cc+duty+target+filtered+body+[mask]):
        raise ProtocolError('nonzero output without permit')
    for i,d in enumerate(duty):
        if abs(d)>100 or d%10:raise ProtocolError('invalid 10-percent PWM limit/resolution')
        expected=[100,100-d//10] if d>0 else [100-abs(d)//10,100] if d<0 else [0,0]
        if cc[2*i:2*i+2]!=expected:raise ProtocolError('actual CCR mapping mismatch')
        if target[i] and d*target[i]*(1,-1,1,-1)[i]<0:raise ProtocolError('wrong wheel polarity')
        if mask&(1<<i) and abs(d)!=80:raise ProtocolError('startup duty mismatch')
    return dict(tag='4WM1',ticks=v[1],updates=v[2],dt_min=v[3],dt_max=v[4],irq_us_max=v[5],
        started_ms=v[6],stationary_ms=v[7],compares=cc,raw_duty=duty,target_cps=target,
        filtered_cps=filtered,body_milli=body,startup_mask=mask,fault_wheel=wheel,
        watchdog_reset=wdg,permitted=permit)
