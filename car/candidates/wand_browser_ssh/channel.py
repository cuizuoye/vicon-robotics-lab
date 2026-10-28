"""Clock-local challenge evidence from the browser MAIN thread, never a lease.

No ROS/actuator code. Nonces retain their original server monotonic time;
replays, late replies and resumed windows cannot refresh a stopped session.
"""
from collections import OrderedDict,deque
import math
import secrets

TTL=.100
CONTROL_GAP=.100
PROTOCOL='WAND_BROWSER_V1'


class Channel:
    def __init__(self,readonly=True):
        self.readonly=readonly;self.owner=None;self.opened=None;self.started=None
        self.nonces=OrderedDict();self.last_sequence=-1;self.proof=None;self.generation=0
        self.events=deque();self.fault='';self.stopped=False;self.last_tick=None
        self.last_receive=None;self.max_receive_gap=0.;self.max_proof_age=0.;self.max_control_gap=0.
        self.max_client_gap=0.;self.proofs=0;self.key_events={'press':0,'release':0}

    def stop(self,reason):
        if not self.fault:self.fault=reason
        self.stopped=True;self.events.clear()

    def open(self,connection,now):
        if self.owner is not None:return False
        self.owner=connection;self.opened=now;return True

    def disconnect(self,connection):
        if connection==self.owner:self.stop('BROWSER_DISCONNECTED')

    def challenge(self,now):
        if self.owner is None or self.stopped:return None
        for key,issued in list(self.nonces.items()):
            if now-issued>.30:del self.nonces[key]
        nonce=secrets.token_urlsafe(18);self.nonces[nonce]=now
        while len(self.nonces)>32:self.nonces.popitem(last=False)
        return dict(kind='challenge',connection=self.owner,nonce=nonce,ttl_ms=100,protocol=PROTOCOL)

    def receive(self,connection,data,now):
        if connection!=self.owner or self.owner is None:return False
        if self.stopped:return False
        # Stops need no current challenge; an old stop can never enable motion.
        if isinstance(data,dict) and set(data)=={'type','reason'} and data.get('type')=='stop':
            self.stop('CLIENT_'+str(data.get('reason','STOP'))[:80]);return False
        try:
            if not isinstance(data,dict) or set(data)!={'type','nonce','sequence','focused','visible','space','ui_gap_ms','events'}:
                raise ValueError('MESSAGE_SCHEMA')
            if data['type']!='pulse' or type(data['sequence']) is not int or data['sequence']<=self.last_sequence:
                raise ValueError('SEQUENCE_OR_TYPE')
            if any(type(data[k]) is not bool for k in ('focused','visible','space')):raise ValueError('PRESENCE_SCHEMA')
            gap=data['ui_gap_ms']
            if type(gap) not in (float,int) or not math.isfinite(gap) or not 0<=gap<=100:
                raise ValueError('CLIENT_LOOP_GAP')
            issued=self.nonces.pop(data['nonce'],None)
            if issued is None or not 0<=now-issued<=TTL:raise ValueError('CHALLENGE_EXPIRED_OR_REPLAYED')
            if not data['focused'] or not data['visible']:raise ValueError('WINDOW_NOT_FOREGROUND')
            actions=data['events']
            if not isinstance(actions,list) or len(actions)>8:raise ValueError('EVENT_QUEUE')
            for action in actions:
                if not isinstance(action,dict):raise ValueError('EVENT_SCHEMA')
                kind=action.get('kind')
                if kind in ('press','release','stop','estop'):
                    if set(action)!={'kind'}:raise ValueError('KEY_EVENT_SCHEMA')
                elif kind=='setpoint' and not self.readonly:
                    if set(action)!={'kind','x','y','yaw','visible'} or type(action['visible']) is not bool:
                        raise ValueError('SETPOINT_SCHEMA')
                    if any(type(action[k]) not in (int,float) or not math.isfinite(action[k]) for k in ('x','y','yaw')):
                        raise ValueError('SETPOINT_NONFINITE')
                    if max(abs(action['x']),abs(action['y']))>5. or abs(action['yaw'])>math.pi:
                        raise ValueError('SOFTWARE_INPUT_LIMIT')
                else:raise ValueError('ACTION_NOT_ALLOWED')
            if len(self.events)+len(actions)>64:raise ValueError('EVENT_OVERFLOW')
            if any(v['kind'] in ('stop','estop') for v in actions):
                self.stop('OPERATOR_ESTOP' if any(v['kind']=='estop' for v in actions) else 'OPERATOR_STOP')
                return False
            self.max_client_gap=max(self.max_client_gap,gap/1000)
            self.max_proof_age=max(self.max_proof_age,now-issued)
            if self.last_receive is not None:self.max_receive_gap=max(self.max_receive_gap,now-self.last_receive)
            self.last_receive=now;self.last_sequence=data['sequence'];self.proofs+=1;self.generation+=1
            self.proof=dict(issued=issued,received=now,generation=self.generation,space=data['space'])
            if self.started is None:self.started=now
            for v in actions:
                if v['kind'] in self.key_events:self.key_events[v['kind']]+=1
                if not self.readonly:self.events.append(v)
            return True
        except (ValueError,TypeError,KeyError) as error:
            self.stop(str(error));return False

    def tick(self,now):
        if self.last_tick is not None:
            gap=now-self.last_tick;self.max_control_gap=max(self.max_control_gap,gap)
            if not 0<=gap<=CONTROL_GAP and self.started is not None:self.stop('SERVER_CONTROL_LOOP_GAP')
        self.last_tick=now
        if self.owner is not None and not self.stopped:
            if self.proof is not None and not 0<=now-self.proof['issued']<=TTL:
                self.stop('BROWSER_PROOF_TIMEOUT')
            elif self.proof is None and now-self.opened>2.:self.stop('INITIAL_BROWSER_PROOF_TIMEOUT')

    def evidence(self,now):
        return self.proof if not self.stopped and self.proof and 0<=now-self.proof['issued']<=TTL else None

    def summary(self,now):
        return dict(fault=self.fault,proofs=self.proofs,elapsed_s=0. if self.started is None else now-self.started,
            max_client_gap_s=self.max_client_gap,max_control_gap_s=self.max_control_gap,
            max_receive_gap_s=self.max_receive_gap,max_proof_age_s=self.max_proof_age,
            key_events=self.key_events,readonly=self.readonly)
