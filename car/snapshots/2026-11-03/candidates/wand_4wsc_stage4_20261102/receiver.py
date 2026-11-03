"""Own only the passive Vicon process we start; never alter another receiver."""
import json
import os
from pathlib import Path
import subprocess
import sys

PROVENANCE=Path('/home/PRIMARY_USER/vicon_receiver_ws/drone_sim/tools/gamepad_rc')


class Receiver:
    def __init__(self):self.child=None

    def start(self,folder):
        sys.path.insert(0,str(PROVENANCE))
        from vicon_receiver_provenance import verify_current,child_environment,recheck_before_spawn
        from mecanum_wand_4wsc_control.rotation_receiver_guard import verify_existing
        proof=verify_current();existing=[]
        for p in Path('/proc').iterdir():
            if not p.name.isdecimal():continue
            try:
                if (p/'comm').read_text().strip()=='vicon_client':existing.append(p)
            except OSError:pass
        if len(existing)>1:raise ValueError('多个Vicon接收器；不停止或替换外部进程')
        if not existing:
            params=dict(hostname='10.10.10.2',buffer_size='200',namespace='vicon',world_frame='map',
                vicon_frame='vicon',map_xyz='[0.0,0.0,0.0]',map_rpy='[0.0,0.0,0.0]',
                map_rpy_in_degrees='false',stream_mode='ServerPush',enable_marker_ray_data='false',
                enable_debug_data='false',enable_device_data='false',publish_latency='true',capture_frame_samples='false')
            command=[proof['installed'],'--ros-args']
            for k,v in params.items():command+=['-p',k+':='+v]
            env=child_environment(os.environ,proof['policy']);env.update(ROS_DOMAIN_ID='0',
                ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(folder/'receiver_ros_logs'))
            recheck_before_spawn(proof)
            with (folder/'receiver.log').open('xb') as log:
                self.child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        old=os.environ.get('ROS_DOMAIN_ID')
        try:
            os.environ['ROS_DOMAIN_ID']='0';current=verify_existing()
        finally:
            if old is None:os.environ.pop('ROS_DOMAIN_ID',None)
            else:os.environ['ROS_DOMAIN_ID']=old
        (folder/'RECEIVER.json').write_text(json.dumps(dict(current,owned=self.child is not None,
            device_control=False,network_configuration_changed=False),ensure_ascii=False,indent=2)+'\n')

    def close(self):
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:self.child.kill();self.child.wait(timeout=2)
