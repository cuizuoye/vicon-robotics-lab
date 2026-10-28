"""Reuse the reviewed UI model + ROS operator. Hardware construction forbidden."""
import json
from dataclasses import asdict
from mecanum_wand_4ws8_control.operator_console import OperatorNode
from mecanum_wand_4ws8_control.safety_core import Lease
from mecanum_wand_4ws8_control.auto_prepare import AutoPrepareOnce
from mecanum_wand_4ws8_control.novice_ui import acknowledge_physical_release
from std_msgs.msg import String


class BrowserOperatorNode(OperatorNode):
    def __init__(self,model,runtime,channel):
        if runtime.mode!='shadow' or not runtime.namespace.startswith('/browser_software_'):
            raise RuntimeError('This candidate has NO hardware operator entry')
        self.channel=channel;self.last_published_generation=-1;self.stop_sent=False
        self.prepare_policy=AutoPrepareOnce()
        super().__init__(model,runtime)
        if self.context.get_domain_id()!=187:raise RuntimeError('Browser software requires private DDS187')

    def heartbeat(self):
        now,ros=self.clocks();self.model.tick(now)
        proof=self.channel.evidence(now)
        if self.model.held and proof is None:self.model.end_trial(now,'浏览器失联/失焦：已撤能，不会自动恢复')
        # Coalesce unchanged intent at50Hz, including idle/released leases.
        # Revocation edges bypass this rate limit; no proof is retimestamped.
        held=bool(self.model.held)
        if held==getattr(self,'last_published_held',None) and now-getattr(self,'last_publish_steady',-float('inf'))<.020:return
        if self.model.held:
            if proof['generation']==self.last_published_generation:return
            # Pin lease time to the challenge's ORIGINAL time, not this pump's
            # clock: background pumping cannot freshen old browser evidence.
            stamp=proof['issued'];issued_ros=ros-(now-stamp)
            self.last_published_generation=proof['generation']
        else:stamp,issued_ros=now,ros
        self.sequence+=1
        lease=Lease(self.owner,self.sequence,stamp,issued_ros,bool(self.model.held))
        self.last_published_held=bool(self.model.held);self.last_publish_steady=now
        self.lease_pub.publish(String(data=json.dumps(asdict(lease))))

    def step(self,now,setpoint):
        for _ in range(12):self.executor_for_console.spin_once(timeout_sec=0.)
        # Subscription receipt stamps are sampled INSIDE those callbacks.
        # A pre-spin clock would make fresh receipts appear to be in the future.
        now=self.clocks()[0]
        # Check the unchanged100ms local scheduling guard BEFORE queued events.
        self.model.tick(now)
        evidence=self.channel.evidence(now)
        if self.channel.stopped:
            self.model.end_trial(now,self.channel.fault)
            if not self.stop_sent:
                self.stop_sent=True;self.request('estop' if self.channel.fault=='OPERATOR_ESTOP' else 'stop')
        elif evidence:
            self.model.focused=True
            acknowledge_physical_release(self.model,evidence['space'],now)
            while self.channel.events:
                action=self.channel.events.popleft();kind=action['kind']
                if kind=='press':
                    was=self.model.session_requested;self.model.press(now)
                    if was and not self.model.session_requested:self.request('stop')
                elif kind=='release':self.model.release(now,commit=True)
                elif kind=='setpoint':setpoint(action)
            ready=self.service_clients['start'].service_is_ready() and 'start' not in self.pending
            if self.prepare_policy.poll(self.model,now,space_down=evidence['space'],
                    service_ready=ready,input_issue=self.input_hint()):self.request('start')
            if self.prepare_policy.cancelled:
                self.model.end_trial(now,self.prepare_policy.reason)
                if not self.stop_sent:self.stop_sent=True;self.request('stop')
        self.heartbeat()

    def snapshot(self,now):
        s=self.model.status
        return dict(state=s.get('state','WAITING'),reason=self.channel.fault or s.get('first_stop_reason') or s.get('reason',''),
            instruction=self.model.instruction(now),prepare_phase=self.prepare_policy.phase,
            car=asdict(self.car) if self.car else None,wand=asdict(self.raw_wand) if self.raw_wand else None,
            goal=s.get('mapped_goal') or s.get('goal'),reference=s.get('goal'),
            trace=list(self.trace)[::max(1,len(self.trace)//100)],gate=self.model.gate,
            session_requested=self.model.session_requested,trial_ended=self.model.ended,
            input_issue=self.input_hint(),backend_status=s,
            model_notice=self.model.notice,model_events=self.model.events[-24:],
            remaining_s=s.get('pilot',{}).get('remaining_s',120.))
