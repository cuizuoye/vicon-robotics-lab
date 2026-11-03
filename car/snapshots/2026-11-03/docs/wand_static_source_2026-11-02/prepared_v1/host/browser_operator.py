"""Browser MAIN-thread proof -> existing SIM12 ROS lease. No Tk/X11 calls."""
import json
from dataclasses import asdict
import time
from mecanum_wand_4wsc_control.operator_console import OperatorNode
from mecanum_wand_4wsc_control.safety_core import Lease
from mecanum_wand_4wsc_control.auto_prepare import AutoPrepareOnce
from mecanum_wand_4wsc_control.novice_ui import acknowledge_physical_release
from std_msgs.msg import String


class BrowserOperator(OperatorNode):
    def __init__(self,model,runtime,channel):
        if runtime.motion_tag!='4WSC' or runtime.purpose!='sign':
            raise ValueError('4WSC Sign runtime required')
        self.channel=channel;self.last_generation=-1;self.stop_sent=False
        self.last_held=None;self.last_publish=-float('inf');self.prepare=AutoPrepareOnce()
        super().__init__(model,runtime)
        runtime.guard(self)

    def heartbeat(self):
        now,ros=self.clocks();self.model.tick(now);proof=self.channel.evidence(now)
        if self.model.held and proof is None:
            self.model.end_trial(now,'浏览器证据过期：撤权，不自动恢复')
        held=bool(self.model.held)
        if held==self.last_held and now-self.last_publish<.020:return
        if held:
            if proof['generation']==self.last_generation:return
            stamp=proof['issued'];issued_ros=ros-(now-stamp)
            self.last_generation=proof['generation']
        else:stamp,issued_ros=now,ros
        self.sequence+=1;self.last_publish=now;self.last_held=held
        lease=Lease(self.owner,self.sequence,stamp,issued_ros,held)
        self.lease_pub.publish(String(data=json.dumps(asdict(lease))))

    def step(self):
        begin=time.monotonic()
        for _ in range(24):
            self.executor_for_console.spin_once(timeout_sec=0.)
            if time.monotonic()-begin>.003:break
        now=self.clocks()[0];self.model.tick(now);proof=self.channel.evidence(now)
        if self.channel.stopped:
            self.model.end_trial(now,self.channel.fault)
            if not self.stop_sent:
                self.stop_sent=True;self.request('estop' if self.channel.fault=='OPERATOR_ESTOP' else 'stop')
        elif proof:
            self.model.focused=True
            acknowledge_physical_release(self.model,proof['space'],now)
            while self.channel.events:
                action=self.channel.events.popleft()
                if action['kind']=='press':
                    was=self.model.session_requested;self.model.press(now)
                    if was and not self.model.session_requested:
                        self.channel.stop('OPERATOR_SECOND_SPACE');self.request('stop');break
                elif action['kind']=='release':self.model.release(now,commit=True)
                else:
                    self.channel.stop('NON_KEY_INPUT_FORBIDDEN');break
            ready=self.service_clients['start'].service_is_ready() and 'start' not in self.pending
            if self.prepare.poll(self.model,now,space_down=proof['space'],service_ready=ready,input_issue=self.input_hint()):
                if not self.request('start'):self.prepare.cancel('准备请求被拒绝，不自动重试')
            if self.prepare.cancelled:
                self.channel.stop('PREPARE_BLOCKED: '+self.prepare.reason)
        self.heartbeat()

    def snapshot(self,now):
        s=self.model.status
        instruction=self.model.instruction(now)
        if self.prepare.phase not in ('READY','MANUAL_ACTIVE'):
            instruction=self.prepare.instruction() or instruction
        return dict(state=s.get('state','WAITING'),reason=s.get('first_stop_reason') or s.get('reason',''),
            instruction=instruction,prepare_phase=self.prepare.phase,
            car=asdict(self.car) if self.car else None,wand=asdict(self.raw_wand) if self.raw_wand else None,
            goal=s.get('goal'),target_policy='DIRECT_SIGN_1TO1',
            trace=list(self.trace)[::max(1,len(self.trace)//100)],gate=self.model.gate,
            session_requested=self.model.session_requested,trial_ended=self.model.ended,
            remaining_s=s.get('pilot',{}).get('remaining_s',600.),
            backend_status=s,input_issue=self.input_hint(),model_events=self.model.events[-20:])
