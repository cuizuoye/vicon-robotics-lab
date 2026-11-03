"""Explicit session -> MAIN-loop checked ROS leases, independent of rendering."""
import json
import time
from dataclasses import asdict
from std_msgs.msg import String
from mecanum_wand_4wsc_control.operator_console import OperatorNode
from mecanum_wand_4wsc_control.safety_core import Lease
from session_prepare import SessionPrepare


class SessionOperator(OperatorNode):
    def __init__(self,model,runtime,channel):
        if runtime.motion_tag!='4WSC' or runtime.purpose!='sign':raise ValueError('4WSC Sign required')
        self.channel=channel;self.prepare=SessionPrepare();self.stop_sent=False
        self.last_publish=-float('inf');self.last_held=None
        super().__init__(model,runtime);runtime.guard(self)

    def heartbeat(self):
        now,ros=self.clocks();self.model.tick(now)
        if self.model.held and not self.channel.session_authorized(now):
            self.model.end_trial(now,'后台会话已撤销；不会自动恢复')
        held=bool(self.model.held and self.channel.session_authorized(now))
        if held==self.last_held and now-self.last_publish<.020:return
        self.sequence+=1;self.last_publish=now;self.last_held=held
        # This is explicitly backend-session authority, not fabricated browser
        # evidence. A100ms main-loop gap is checked before ANY lease renewal.
        self.lease_pub.publish(String(data=json.dumps(asdict(Lease(self.owner,self.sequence,now,ros,held)))))

    def step(self):
        begin=time.monotonic()
        for _ in range(24):
            self.executor_for_console.spin_once(timeout_sec=0.)
            if time.monotonic()-begin>.003:break
        now=self.clocks()[0]
        self.model.backend_available=bool(self.channel.owner and self.channel.loop_fresh(now))
        self.model.tick(now)
        if self.channel.stopped:
            self.model.end_trial(now,self.channel.fault)
            if not self.stop_sent:
                self.stop_sent=True;self.request('estop' if self.channel.fault=='OPERATOR_ESTOP' else 'stop')
        elif self.channel.owner:
            while self.channel.events:
                action=self.channel.events.popleft()
                if action!={'kind':'start'} or not self.model.request_session(now):
                    self.channel.stop('START_REQUEST_REJECTED');self.model.end_trial(now,self.channel.fault);break
            ready=self.service_clients['start'].service_is_ready() and 'start' not in self.pending
            if self.prepare.poll(self.model,now,space_down=False,service_ready=ready,input_issue=self.input_hint()):
                if not self.request('start'):self.prepare.cancel('准备请求被拒绝；不自动重试')
            if self.prepare.cancelled:self.channel.stop('PREPARE_BLOCKED: '+self.prepare.reason)
        self.channel.ready=bool(not self.channel.stopped and self.prepare.phase=='READY' and self.model.start_ready(now))
        self.heartbeat()

    def snapshot(self,now):
        s=self.model.status
        return dict(state=s.get('state','WAITING'),reason=s.get('first_stop_reason') or s.get('reason',''),
            instruction=self.model.instruction(now) if self.prepare.phase in ('READY','MANUAL_ACTIVE') else self.prepare.instruction(),
            prepare_phase=self.prepare.phase,car=asdict(self.car) if self.car else None,
            wand=asdict(self.raw_wand) if self.raw_wand else None,goal=s.get('goal'),target_policy='DIRECT_SIGN_1TO1',
            trace=list(self.trace)[::max(1,len(self.trace)//100)],gate=self.model.gate,
            session_requested=self.model.session_requested,trial_ended=self.model.ended,
            remaining_s=s.get('pilot',{}).get('remaining_s',600.),backend_status=s,
            input_issue=self.input_hint(),model_events=self.model.events[-20:])
