"""Optional transparent socket timing for no-power ROS observation only."""
import time
from mecanum_vicon_control.transport_protocol import decode_frame,unpack_cmd_vel,MessageType,FrameFlags

class TimedSocket:
    def __init__(self,socket,origin,clock=time.monotonic):
        self.socket=socket;self.origin=origin;self.clock=clock;self.records=[]
    def __getattr__(self,name):return getattr(self.socket,name)
    def sendto(self,data,*args,**kwargs):
        row={'start_s':self.clock()-self.origin}
        try:
            result=self.socket.sendto(data,*args,**kwargs)
        except OSError as error:
            row.update(end_s=self.clock()-self.origin,error_errno=error.errno)
            self.records.append(row);raise
        row['end_s']=self.clock()-self.origin
        # Decode AFTER recording return time. Never modify or retry a frame.
        try:
            frame=decode_frame(data)
            row.update(kind=int(frame.message_type),sequence=frame.sequence,
                       session_id=frame.session_id,flags=int(frame.flags))
            if frame.message_type==MessageType.CMD_VEL:
                c=unpack_cmd_vel(frame.payload)
                row['disabled_zero']=not(frame.flags&FrameFlags.ENABLED) and c.vx_mps==c.vy_mps==c.wz_radps==0
                row['timeout_ms']=c.timeout_ms
        except ValueError:
            row['decode_error']=True
        self.records.append(row)
        return result

def summarize_sends(records,before_s):
    commands=[r for r in records if r.get('kind')==int(MessageType.CMD_VEL)
              and r['end_s']<=before_s and 'error_errno' not in r]
    gaps=[{'ms':(b['start_s']-a['start_s'])*1000,'from_sequence':a['sequence'],
           'to_sequence':b['sequence'],'session_id':a['session_id']}
          for a,b in zip(commands,commands[1:]) if a['session_id']==b['session_id']]
    return dict(command_count=len(commands),before_host_s=before_s,
        max_same_session_send_gap=max(gaps,key=lambda g:g['ms']) if gaps else None,
        max_send_call_ms=max(((r['end_s']-r['start_s'])*1000 for r in commands),default=None),
        all_commands_disabled_zero=bool(commands) and all(r['disabled_zero'] for r in commands),
        timeout_ms_observed=sorted({r['timeout_ms'] for r in commands}),
        error_count=sum('error_errno' in r for r in records if r['end_s']<=before_s),
        note='Userspace sendto start/return only, not proof of radio transmission; feedback clocks are host monotonic')
