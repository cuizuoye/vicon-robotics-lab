"""4WS7 offline protocol/schema tools. NO ROS, socket, serial or live launcher.

Default validation rejects any execution authority in a hardware-locked image.
Synthetic fixtures must be explicitly labelled by the caller. A passing file
is NOT permission to run a tier or evidence of physical performance.
"""
from dataclasses import dataclass
import math
import struct
import zlib

TAG=int.from_bytes(b'4WS7','little')
STATUS=struct.Struct('<8I8I4h4h4h3hBBHH')
SNAPSHOT=struct.Struct('<IIiiiiHHhH')
AUTHORITY=struct.Struct('<4f4bBBH')
SERVO=struct.Struct('<5fH6B')
HEADER=struct.Struct('<2sBBHIIIH')
SIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size+SERVO.size
CPR=(3179/3,1049,3167/3,1061)


@dataclass(frozen=True)
class Stage:
    speed: float
    accel: float
    duty: int
    output_slew: float


STAGES=(Stage(.20,.08,350,1200),Stage(.35,.20,450,1600),Stage(.50,.40,600,2000),
        Stage(.70,.40,700,2000),Stage(.70,.80,800,2500),
        Stage(.70,1.20,800,3000),Stage(.70,2.,800,4000))


def tier_payload(stage, *, enable=False):
    if type(stage) is not int or stage not in range(len(STAGES)) or type(enable) is not bool:
        raise ValueError('explicit stage 0..6 and Boolean enable required')
    return struct.pack('<BBH',int(enable),0,0x7000+stage)


def decode_status(payload, *, synthetic_native_fixture=False):
    if len(payload)!=SIZE:
        raise ValueError('exactly 184 bytes of 4WS7 v2 required')
    v=STATUS.unpack_from(payload)
    f=SNAPSHOT.unpack_from(payload,100)
    a=AUTHORITY.unpack_from(payload,132)
    extra=SERVO.unpack_from(payload,156)
    shape=list(extra[:3]);accel,slew,duty_limit=extra[3:6]
    stage_id,selected,saturated,released,rejoining,reserved=extra[6:]
    if (v[0]!=TAG or not 1<=v[3]<=v[4]<=30 or v[5]>1000 or v[7]>500 or
        stage_id not in range(7) or selected not in (0,1) or released!=0 or
        saturated>15 or rejoining>15 or reserved):
        raise ValueError('identity/timing/stage or unreleased-policy mismatch')
    stage=STAGES[stage_id]
    if (not all(math.isfinite(x) for x in extra[:5]) or
        not math.isclose(accel,stage.accel,rel_tol=1e-6) or slew!=stage.output_slew or
        duty_limit!=stage.duty or math.hypot(*shape[:2])>stage.speed+1e-6 or abs(shape[2])>.100001):
        raise ValueError('stage policy or shaped SI vector mismatch')
    cc=list(v[8:16]);duty=list(v[16:20]);target=list(v[20:24]);filtered=list(v[24:28]);body=list(v[28:31])
    startup,wheel,watchdog,permit=v[31:35]
    effective=list(a[:4]);directions=list(a[4:8]);reverse,version,auth_reserved=a[8:]
    if (startup>15 or wheel>4 or watchdog or permit not in (0,1) or reverse>15 or
        version!=2 or auth_reserved or reverse & rejoining or
        math.hypot(*body[:2])>stage.speed*1000+1.5 or abs(body[2])>100 or
        any(abs(x)>10000 for x in filtered) or
        any(not math.isfinite(x) for x in effective)):
        raise ValueError('invalid authority/velocity fields')
    if (permit and not selected) or (not permit and any(cc+duty+target+filtered+body+effective+directions+[startup,reverse,rejoining])):
        raise ValueError('output or grant without selected permit')
    if not synthetic_native_fixture and (permit or any(cc+duty+effective)):
        raise ValueError('OFFLINE image has no hardware execution authority')
    if any(abs(integer-float_si*1000)>1.01 for integer,float_si in zip(body,shape)):
        raise ValueError('body diagnostics differ from coordinated vector')
    x,y,w=shape;rotation=.146*w
    rim=(x-y-rotation,x+y+rotation,x+y-rotation,x-y+rotation)
    for i,(d,e,g,sign,cpr) in enumerate(zip(duty,effective,directions,(1,-1,1,-1),CPR)):
        if abs(d)>stage.duty or d%10 or g not in (-1,0,1) or abs(e)*60/cpr>400.001 or abs(target[i])*60/cpr>400.1:
            raise ValueError('wheel RPM/PWM outside stage envelope')
        expected_cps=rim[i]*cpr/(math.pi*.060)
        if abs(expected_cps)<20:expected_cps=0
        if abs(target[i]-expected_cps)>1.01:
            raise ValueError('wheel target is not from coordinated SI vector')
        expected=[100,100-d//10] if d>0 else [100-abs(d)//10,100] if d<0 else [0,0]
        if cc[2*i:2*i+2]!=expected:
            raise ValueError('actual CCR mismatch')
        if (e and (1 if e>0 else -1)!=g) or (d and (not e or d*e*sign<=0)):
            raise ValueError('output not from current effective/granted direction')
        if reverse & (1<<i) and (d or e):
            raise ValueError('coast-neutral override has driven output')
        if not target[i] and (d or e):
            raise ValueError('output despite zero requested wheel target')
        if not reverse & (1<<i) and not rejoining & (1<<i) and abs(e-target[i])>1.01:
            raise ValueError('normal output target differs from coordinated SI plan')
        if startup & (1<<i) and (abs(d)>300 or reverse & (1<<i)):
            raise ValueError('startup envelope violated')
    if (f[6] not in range(7) or (not f[6] and any(f)) or
        (f[6] and not 1<=f[7]<=4) or abs(f[8])>800 or f[8]%10):
        raise ValueError('invalid first feedback fault snapshot')
    return dict(tag='4WS7',stage=stage_id,selected=bool(selected),permitted=bool(permit),
                body_si=shape,effective_cps=effective,raw_duty=duty,
                reversing_mask=reverse,rejoining_mask=rejoining,saturated_mask=saturated,
                hardware_released=False,synthetic_only=synthetic_native_fixture)


def decode_status_frame(frame, *, synthetic_native_fixture=False):
    if len(frame)!=HEADER.size+SIZE+4:
        raise ValueError('frame length')
    magic,version,kind,flags,sequence,session,sender_ms,length=HEADER.unpack_from(frame)
    if magic!=b'MC' or version!=1 or kind!=0x31 or flags&~15 or not flags&2 or not session or length!=SIZE:
        raise ValueError('frame header')
    if zlib.crc32(frame[:-4])!=struct.unpack_from('<I',frame,len(frame)-4)[0]:
        raise ValueError('frame CRC')
    return decode_status(frame[HEADER.size:-4],synthetic_native_fixture=synthetic_native_fixture)
