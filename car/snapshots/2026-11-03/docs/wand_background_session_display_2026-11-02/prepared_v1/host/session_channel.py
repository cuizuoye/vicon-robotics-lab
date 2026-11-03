"""One explicit, bounded backend session; browser packets are NOT heartbeats.

Fresh single-use START offers prevent delayed/replayed starts. The backend MAIN
loop must still run within100ms. STOP always wins; no stopped epoch can resume.
"""
from collections import OrderedDict, deque
import math
import secrets

CONTROL_GAP=.100
START_TTL=1.
PROTOCOL='WAND_BACKEND_SESSION_V1'


class Channel:
    def __init__(self,readonly=True):
        self.owner=None;self.opened=self.started=self.last_tick=None
        self.stopped=False;self.fault='';self.events=deque();self.nonces=OrderedDict()
        self.last_sequence=-1;self.ready=False;self.start_used=False;self.intent=False
        self.accepted_commands=0;self.max_control_gap=0.;self.last_offer=-math.inf
        self.last_backend_at=None
        self.proof=None  # Compatibility for legacy diagnostic fields only; never populated.

    def open(self,connection,now):
        if self.owner is not None or self.stopped:return False
        self.owner=connection;self.opened=now;return True

    def stop(self,reason):
        if not self.fault:self.fault=reason
        self.stopped=True;self.intent=False;self.ready=False;self.events.clear();self.nonces.clear()

    def disconnect(self,connection):
        if connection==self.owner:self.stop('BROWSER_DISCONNECTED')

    def begin_tick(self,now):
        if self.last_tick is not None:
            gap=now-self.last_tick;self.max_control_gap=max(self.max_control_gap,gap)
            if (not 0<=gap<=CONTROL_GAP) and self.start_used:self.stop('SERVER_CONTROL_LOOP_GAP')
        self.last_tick=now

    def finish_tick(self,now):
        if self.start_used and (self.last_tick is None or not 0<=now-self.last_tick<=CONTROL_GAP):
            self.stop('SERVER_CONTROL_LOOP_GAP')
        if self.start_used and (self.last_backend_at is None or not 0<=now-self.last_backend_at<=CONTROL_GAP):
            self.stop('BACKEND_COMMAND_CHANNEL_TIMEOUT')
        # Deliberately no browser-presence timeout. Source/feedback/lease guards
        # live downstream and remain mandatory on EVERY backend iteration.

    def loop_fresh(self,now):
        return bool(self.last_tick is not None and 0<=now-self.last_tick<=CONTROL_GAP
                    and self.last_backend_at is not None and 0<=now-self.last_backend_at<=CONTROL_GAP and not self.stopped)

    def backend_tick(self,issued,now):
        # Node and Python use the same local CLOCK_MONOTONIC. A transport thread
        # may enqueue this observation, but only this main-loop call validates it.
        if type(issued) not in (int,float) or not math.isfinite(issued) or not 0<=now-issued<=CONTROL_GAP:
            if self.start_used:self.stop('BACKEND_COMMAND_CHANNEL_TIMEOUT')
            return False
        if self.last_backend_at is None or issued>self.last_backend_at:self.last_backend_at=issued
        return True

    def session_authorized(self,now):
        return bool(self.intent and self.start_used and self.loop_fresh(now))

    def challenge(self,now):
        if not self.owner or self.stopped or not self.ready or self.start_used or now-self.last_offer<.25:return None
        self.last_offer=now
        for nonce,at in list(self.nonces.items()):
            if now-at>START_TTL:del self.nonces[nonce]
        nonce=secrets.token_urlsafe(24);self.nonces[nonce]=now
        while len(self.nonces)>8:self.nonces.popitem(last=False)
        return dict(kind='intent_offer',protocol=PROTOCOL,connection=self.owner,nonce=nonce,
                    ttl_ms=1000,issued_steady=now,action='start')

    def receive(self,connection,data,now):
        if connection!=self.owner or self.owner is None or self.stopped:return False
        # Stops need no fresh nonce/sequence and can never enable motion.
        if isinstance(data,dict) and set(data)=={'type','reason'} and data['type']=='stop':
            self.stop('OPERATOR_ESTOP' if data['reason']=='ESTOP' else 'CLIENT_'+str(data['reason'])[:80]);return False
        try:
            if not isinstance(data,dict) or set(data)!={'type','action','nonce','sequence'}:raise ValueError('COMMAND_SCHEMA')
            if data['type']!='command' or data['action']!='start':raise ValueError('ACTION_NOT_ALLOWED')
            if type(data['sequence']) is not int or data['sequence']<=self.last_sequence:raise ValueError('COMMAND_REPLAY')
            issued=self.nonces.pop(data['nonce'],None)
            if issued is None or not 0<=now-issued<=START_TTL:raise ValueError('START_EXPIRED_OR_REPLAYED')
            if self.start_used or not self.ready or not self.loop_fresh(now):raise ValueError('START_NOT_READY')
            self.last_sequence=data['sequence'];self.start_used=self.intent=True;self.ready=False
            self.started=now;self.accepted_commands+=1;self.nonces.clear();self.events.append({'kind':'start'})
            return True
        except (ValueError,TypeError,KeyError) as error:
            self.stop(str(error));return False

    def summary(self,now):
        return dict(fault=self.fault,elapsed_s=0. if self.started is None else now-self.started,
                    authority_mode=PROTOCOL,accepted_commands=self.accepted_commands,
                    explicit_start_received=self.start_used,session_requested=self.intent,
                    max_control_gap_s=self.max_control_gap,browser_presence_is_authority=False,
                    proofs=0,max_proof_age_s=None,max_client_gap_s=None)
