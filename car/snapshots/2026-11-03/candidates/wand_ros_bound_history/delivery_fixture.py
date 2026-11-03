"""Private DDS fault injection ONLY; not installed into a runtime package."""
import json
import time


class DeliveryFixture:
    def __init__(self,state,case):
        self.state,self.case=state,case;self.pending=[];self.counts={}
        self.observed=dict(delayed=0,reordered=0,dropped=0,batched=0)
        self.car_stamps=[];self.batch_until=None

    def wrap(self,callback,topic):
        def deliver(msg):
            state=self.state;now=time.monotonic()
            if state.injected is None or state.restored is not None:return callback(msg)
            if self.case=='reference_loss' and topic=='car_pose' and now-state.injected<.045:
                self.observed['dropped']+=1;return
            delay=0.
            if self.case=='reference_reorder' and topic in ('car_pose','rotation_packet'):
                self.counts[topic]=self.counts.get(topic,0)+1
                delay=.025 if self.counts[topic]%2 else 0.
            if self.case=='candidate_burst' and topic=='candidate':
                # Do not delay STOP even in the software injection fixture.
                if not json.loads(msg.data)['frame']['enabled']:return callback(msg)
                if self.batch_until is None or now>self.batch_until:self.batch_until=now+.030
                delay=max(0.,self.batch_until-now);self.observed['batched']+=1
            if delay:
                self.pending.append((now+delay,callback,msg,topic));self.observed['delayed']+=1
            else:
                if any(v[3]==topic for v in self.pending):self.observed['reordered']+=1
                callback(msg)
        return deliver

    def flush(self):
        now=time.monotonic();ready=[v for v in self.pending if v[0]<=now]
        self.pending=[v for v in self.pending if v[0]>now]
        for _,callback,msg,_ in sorted(ready,key=lambda v:v[0]):callback(msg)
