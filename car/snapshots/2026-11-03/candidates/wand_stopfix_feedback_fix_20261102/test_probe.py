"""Exact-C simulated100ms telemetry, <=260ms batching, loss and stalled progress."""
import heapq
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
from build import ROOT,OUT,RECORD,save

READY=OUT/'host';sys.path.insert(0,str(READY));import bootstrap
import idle_probe
from native_peer import Oracle
from mecanum_vicon_control.transport_protocol import decode_frame,MessageType as M

spec=importlib.util.spec_from_file_location('unchanged_old_probe',ROOT/'docs/wand_stopfix_deploy_2026-11-02/prepared_v1/host/idle_probe.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)

class Clock:
    def __init__(self):self.t=1.
    def monotonic(self):return self.t
    def sleep(self,value):self.t+=value

class Socket:
    def __init__(self,clock,case):
        self.clock,self.case=clock,case;self.c=Oracle();self.next_pub=1.1;self.pending=[];self.serial=0;self.closed=False;self.sent=[]
    def setsockopt(self,*args):pass
    def bind(self,*args):pass
    def setblocking(self,*args):pass
    def close(self):self.closed=True
    def queue(self,data,at):
        self.serial+=1;heapq.heappush(self.pending,(at,self.serial,data))
    def advance(self):
        ms=int(self.clock.t*1000);self.c.step(ms)
        while self.next_pub<=self.clock.t:
            self.next_pub+=.1
            if self.case=='feedback_loss' and self.clock.t-1>=3:continue
            tick=self.c.now;bucket=(tick+199)//200
            release=bucket*.2+(.060 if bucket%2 else 0.)
            for kind in (48,49):self.queue(self.c.packet(kind),max(release,self.clock.t))
    def sendto(self,data,destination):
        idle_probe.zero_only(data);self.sent.append(decode_frame(data));self.advance()
        f=decode_frame(data)
        if self.case=='stalled_progress' and self.clock.t-1>=3 and f.message_type==M.CMD_VEL:return len(data)
        reply=self.c.request(data,now=int(self.clock.t*1000))
        if reply:self.queue(reply,self.clock.t+.006)
        return len(data)
    def recvfrom(self,size):
        self.advance()
        if not self.pending or self.pending[0][0]>self.clock.t:raise BlockingIOError()
        return heapq.heappop(self.pending)[2],('127.0.0.1',14000)

def run(module,case,folder):
    c=Clock();sock=Socket(c,case);p=json.loads((READY/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=14000,telemetry_port=14001)
    with patch.object(module.socket,'socket',return_value=sock),patch.object(module.time,'monotonic',c.monotonic),patch.object(module.time,'sleep',c.sleep):
        try:r=module.run(p,RECORD,folder,module.CONSENT,8.,rehearsal=True)
        except ValueError:r=json.loads((folder/'RESULT.json').read_text())
    assert sock.closed and not sock.c.status['permitted'] and not sock.c.status['selected']
    assert all(not x.flags and x.message_type in (M.HELLO,M.CMD_VEL) for x in sock.sent)
    return r

def main():
    folder=Path(tempfile.mkdtemp(prefix='probe_tests_',dir=OUT));results={}
    for name,module,case in (('old_false_block',old,'batched'),('new_accepts_fresh100ms_source',idle_probe,'batched'),
                             ('new_feedback_loss_stops',idle_probe,'feedback_loss'),('new_stalled_progress_stops',idle_probe,'stalled_progress')):
        results[name]=run(module,case,folder/name)
    assert not results['old_false_block']['passed']
    assert results['new_accepts_fresh100ms_source']['passed']
    for name,reason in (('new_feedback_loss_stops','300ms'),('new_stalled_progress_stops','1s')):
        r=results[name];assert not r['passed'] and reason in r['reason'] and r['elapsed_s']<5.,r
    report=dict(passed=True,physical_output=False,actual_new_C=True,source_period_s=.1,batching_max_gap_s=.26,
        current_300ms_guard_preserved=True,existing_bridge_1s_progress_guard_enforced=True,select_or_enable_sent=False,
        cases=results,folder=str(folder))
    save(folder/'RESULT.json',report);print(json.dumps(report))
if __name__=='__main__':main()
