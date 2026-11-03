"""Validate bounded-streaming motion identities, never bench/readonly images."""
from .bench_link_status import STATUS
from .transport_protocol import ProtocolError

TAG=0x314D5734
GROUND_TAG=0x31475734
GROUND2_TAG=0x32475734
GROUND3_TAG=0x33475734
GROUND4_TAG=0x34475734
GROUND5_TAG=0x35475734
GROUND6_TAG=0x36475734
GROUND7_TAG=0x37475734
GROUND8_TAG=0x38475734
GROUND9_TAG=0x39475734
GROUNDA_TAG=0x41475734
GROUNDB_TAG=0x42475734
PROFILES={
    TAG: ('4WM1',100,80),
    GROUND_TAG: ('4WG1',200,150),
    GROUND2_TAG: ('4WG2',350,300),
    GROUND3_TAG: ('4WG3',350,300),
    GROUND4_TAG: ('4WG4',350,300),
    GROUND5_TAG: ('4WG5',350,300),
    GROUND6_TAG: ('4WG6',350,300),
    GROUND7_TAG: ('4WG7',350,None),
    GROUND8_TAG: ('4WG8',350,None),
    GROUND9_TAG: ('4WG9',350,None),
    GROUNDA_TAG: ('4WGA',350,None),
    GROUNDB_TAG: ('4WGB',350,None),
}
MOTION_TAGS=frozenset(profile[0] for profile in PROFILES.values())
MOTION_LINEAR_LIMITS={
    tag: (0.20 if tag in ('4WG8', '4WG9') else
          0.15 if tag == '4WGB' else 0.10 if tag == '4WGA' else 0.05)
    for tag in MOTION_TAGS
}
MOTION_WIRE_TIMEOUTS={
    tag: (600 if tag in ('4WGA', '4WGB') else 150) for tag in MOTION_TAGS}

def decode_motion_status(payload):
    if len(payload)!=STATUS.size:raise ProtocolError('not a 100-byte motion status')
    v=STATUS.unpack(payload)
    profile=PROFILES.get(v[0])
    if profile is None or not 1<=v[3]<=v[4]<=30 or v[5]>1000:
        raise ProtocolError('wrong motion identity or sampling deadline')
    tag,duty_limit,startup_duty=profile
    cc=list(v[8:16]);duty=list(v[16:20]);target=list(v[20:24]);filtered=list(v[24:28])
    body=list(v[28:31]);mask,wheel,wdg,permit=v[31:35]
    if mask>15 or wheel>4 or wdg or permit not in (0,1) or v[7]>500:
        raise ProtocolError('invalid motion safety state')
    linear_mm_s=(200 if tag in ('4WG8', '4WG9') else
                 150 if tag == '4WGB' else 100 if tag == '4WGA' else 50)
    target_limit_cps=1200 if tag in ('4WG8', '4WG9', '4WGA', '4WGB') else 300
    if abs(body[0])>linear_mm_s or abs(body[1])>linear_mm_s or abs(body[2])>200:
        raise ProtocolError('body velocity outside commissioning envelope')
    if any(abs(t)>target_limit_cps for t in target) or any(abs(f)>10000 for f in filtered):
        raise ProtocolError('wheel value outside commissioning envelope')
    if not permit and any(cc+duty+target+filtered+body+[mask]):
        raise ProtocolError('nonzero output without permit')
    for i,d in enumerate(duty):
        if abs(d)>duty_limit or d%10:raise ProtocolError('invalid bounded PWM limit/resolution')
        expected=[100,100-d//10] if d>0 else [100-abs(d)//10,100] if d<0 else [0,0]
        if cc[2*i:2*i+2]!=expected:raise ProtocolError('actual CCR mapping mismatch')
        if target[i] and d*target[i]*(1,-1,1,-1)[i]<0:raise ProtocolError('wrong wheel polarity')
        if mask&(1<<i):
            if startup_duty is None:
                if not d:raise ProtocolError('launch state without bounded duty')
            elif abs(d)!=startup_duty:raise ProtocolError('startup duty mismatch')
    return dict(tag=tag,ticks=v[1],updates=v[2],dt_min=v[3],dt_max=v[4],irq_us_max=v[5],
        started_ms=v[6],stationary_ms=v[7],compares=cc,raw_duty=duty,target_cps=target,
        filtered_cps=filtered,body_milli=body,startup_mask=mask,fault_wheel=wheel,
        watchdog_reset=wdg,permitted=permit)
