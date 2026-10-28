"""Non-authoritative observation AFTER the original strict channel has failed.

No ROS imports, leases, clear-fault or recovery operations. The ORIGINAL
Channel is latched forever. A separate bounded sample recorder keeps measuring
transport jitter; its samples are never accepted as control evidence.
"""
from collections import OrderedDict,deque
import math
import secrets
from channel import Channel

FIELDS={'type','nonce','sequence','focused','visible','space','ui_gap_ms','events'}

class Probe:
    def __init__(self):
        self.strict=Channel(readonly=True)
        self.pending=OrderedDict();self.echo_pending=OrderedDict()
        self.samples=deque(maxlen=6000);self.echo_samples=deque(maxlen=6000)
        self.first=None;self.first_failure=None;self.sequence=-1
        self.closed=False;self.end_reason='';self.replays=0
        self.max_client_gap=0.;self.last_receipt=None;self.max_receipt_gap=0.
        self.last_tick=None;self.max_tick_gap=0.;self.raw_count=0
        self.failures=deque(maxlen=32)

    def open(self,connection,now):return self.strict.open(connection,now)

    def fail(self,reason,now):
        self.closed=True;self.end_reason=reason;self.strict.stop(reason);self.mark_failure(now)

    def disconnect(self,connection,now):
        if connection==self.strict.owner:self.fail('BROWSER_DISCONNECTED',now)

    def mark_failure(self,now):
        if self.strict.fault and self.first_failure is None:
            proof=self.strict.proof
            self.first_failure=dict(reason=self.strict.fault,steady=now,
                elapsed_s=0. if self.first is None else now-self.first,
                last_proof_age_s=None if proof is None else now-proof['issued'],
                last_receive_gap_s=None if self.last_receipt is None else now-self.last_receipt,
                pending_challenges=len(self.pending))

    def tick(self,now):
        if self.last_tick is not None:self.max_tick_gap=max(self.max_tick_gap,now-self.last_tick)
        self.last_tick=now;self.strict.tick(now);self.mark_failure(now)

    def challenge(self,now):
        if self.strict.owner is None or self.closed:return None
        msg=self.strict.challenge(now)
        if msg is None:msg=dict(kind='challenge',connection=self.strict.owner,
            nonce=secrets.token_urlsafe(18),ttl_ms=100,protocol='WAND_BROWSER_V1')
        self.pending[msg['nonce']]=now
        self.echo_pending[msg['nonce']]=now
        while len(self.pending)>512:self.pending.popitem(last=False)
        while len(self.echo_pending)>512:self.echo_pending.popitem(last=False)
        return msg

    def receive(self,connection,data,now,*,node_received=None,reader_received=None,node_gap=None):
        if connection!=self.strict.owner or self.closed:return
        if isinstance(data,dict) and data.get('type')=='diagnostic_echo':
            # Timing sample ONLY: not passed to Channel.receive, cannot start
            # a session, advance its generation or freshen its original proof.
            if set(data)!={'type','nonce'}:
                self.fail('ECHO_SCHEMA',now);return
            try:issued=self.echo_pending.pop(data['nonce'],None)
            except TypeError:issued=None
            if issued is None:self.fail('ECHO_NONCE_INVALID',now);return
            self.echo_samples.append(dict(echo_roundtrip_s=now-issued,at=now,authority=False))
            return
        if isinstance(data,dict) and set(data)=={'type','reason'} and data['type']=='stop':
            self.fail('CLIENT_'+str(data['reason'])[:80],now);return
        try:
            if not isinstance(data,dict) or set(data)!=FIELDS or data['type']!='pulse':raise ValueError('SCHEMA')
            if type(data['sequence']) is not int or data['sequence']<=self.sequence:raise ValueError('SEQUENCE')
            if any(type(data[k]) is not bool for k in ('focused','visible','space')):raise ValueError('PRESENCE')
            gap=data['ui_gap_ms']
            if type(gap) not in (int,float) or not math.isfinite(gap) or not 0<=gap<=60000:raise ValueError('GAP_SCHEMA')
            actions=data['events']
            if not isinstance(actions,list) or len(actions)>8 or any(
                    not isinstance(v,dict) or set(v)!={'kind'} or v['kind'] not in ('press','release') for v in actions):
                raise ValueError('READONLY_ACTION')
            issued=self.pending.pop(data['nonce'],None)
            if issued is None:self.replays+=1;raise ValueError('NONCE_REPLAY_OR_UNKNOWN')
            if self.first is None:self.first=now
            self.sequence=data['sequence'];self.raw_count+=1
            self.max_client_gap=max(self.max_client_gap,gap/1000)
            if self.last_receipt is not None:self.max_receipt_gap=max(self.max_receipt_gap,now-self.last_receipt)
            self.last_receipt=now
            self.strict.receive(connection,data,now);self.mark_failure(now)
            def delta(later,earlier):
                if type(later) not in (int,float) or type(earlier) not in (int,float):return None
                v=later-earlier
                return v if math.isfinite(v) and 0<=v<60. else None
            sample=dict(at=now,sequence=self.sequence,roundtrip_s=now-issued,
                issue_to_node_receive_s=delta(node_received,issued),
                node_to_reader_s=delta(reader_received,node_received),
                reader_to_main_s=delta(now,reader_received),node_event_gap_max_s=node_gap,
                client_loop_gap_s=gap/1000,focused=data['focused'],visible=data['visible'],
                strict_fault=self.strict.fault,authority=False)
            self.samples.append(sample)
        except (ValueError,TypeError,KeyError) as error:self.fail('OBSERVATION_'+str(error),now)

    def summary(self,now):
        def stats(key):
            values=sorted(v[key] for v in (*self.samples,*self.echo_samples) if type(v.get(key)) in (int,float))
            return None if not values else dict(max=max(values),p50=values[len(values)//2],p95=values[min(len(values)-1,int(len(values)*.95))])
        return dict(strict_passed=not bool(self.strict.fault),strict_fault=self.strict.fault,
            first_failure=self.first_failure,strict_proofs=self.strict.proofs,raw_replies=self.raw_count,
            diagnostic_echoes=len(self.echo_samples),
            post_failure_samples=sum(bool(v['strict_fault']) for v in self.samples),
            max_client_gap_s=self.max_client_gap,max_receive_gap_s=self.max_receipt_gap,
            max_python_loop_gap_s=self.max_tick_gap,
            timings={k:stats(k) for k in ('roundtrip_s','issue_to_node_receive_s','node_to_reader_s',
                'reader_to_main_s','client_loop_gap_s','node_event_gap_max_s','echo_roundtrip_s')},
            elapsed_s=0. if self.first is None else now-self.first,closed=self.closed,
            end_reason=self.end_reason,ros_initialized=False,hardware_socket_opened=False,
            authority_ever_acquired=False,physical_motion_started=False,
            purpose='OBSERVER ONLY; strict failure is never cleared; not motion qualification')
