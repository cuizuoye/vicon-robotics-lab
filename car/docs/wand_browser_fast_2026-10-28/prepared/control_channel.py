"""Bounded ingress-before-expiry arbitration; original nonce TTL stays100ms."""
from channel import Channel as OriginalChannel,TTL,CONTROL_GAP

class Channel(OriginalChannel):
    def begin_tick(self,now):
        # Must run BEFORE the ingress drain: queued fresh input cannot mask an
        # actual100ms blocked controller/UI task.
        if self.last_tick is not None:
            gap=now-self.last_tick;self.max_control_gap=max(self.max_control_gap,gap)
            if not 0<=gap<=CONTROL_GAP and self.started is not None:self.stop('SERVER_CONTROL_LOOP_GAP')
        self.last_tick=now

    def finish_tick(self,now):
        # Only a fresh ORIGINAL challenge can replace an older proof. Deadline
        # is evaluated before the ROS step/held lease, never by a background IO thread.
        if self.owner is not None and not self.stopped:
            if self.proof is not None and not 0<=now-self.proof['issued']<=TTL:self.stop('BROWSER_PROOF_TIMEOUT')
            elif self.proof is None and now-self.opened>2.:self.stop('INITIAL_BROWSER_PROOF_TIMEOUT')

    def tick(self,now):self.begin_tick(now);self.finish_tick(now)

    def receive(self,connection,data,now):
        if (connection==self.owner and self.owner is not None and not self.stopped
                and isinstance(data,dict) and data=={'type':'stop','reason':'ESTOP'}):
            self.stop('OPERATOR_ESTOP');return False
        return super().receive(connection,data,now)

    def challenge(self,now):
        v=super().challenge(now)
        if v is not None:v['issued_steady']=now
        return v
