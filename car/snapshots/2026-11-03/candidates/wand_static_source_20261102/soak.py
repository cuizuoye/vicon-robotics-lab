"""Actual ROS raw-source -> control/gate -> native-C LOOPBACK, no real robot.

Test-only raw publisher/context197 and receiver. Production reader remains
domain0; this test cannot construct a hardware runtime or use motor ports.
"""
import argparse
from dataclasses import asdict
import json
import math
import multiprocessing as mp
import os
import queue
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch
from build import OUT, sha, save
from ground_entry import profile

READY = OUT/'host'
sys.path.insert(0, str(READY))
import bootstrap


def raw_child(channel, stop, ready):
    bootstrap.install()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsc_hardware.source_pipeline import BufferedRawSource,passive_loop
    context = Context();node = executor = None
    try:
        rclpy.init(context=context,domain_id=197,signal_handler_options=SignalHandlerOptions.NO)
        # Only the fixture substitutes binary provenance, never the runtime.
        with patch('mecanum_wand_4wsc_control.rotation_receiver_guard.verify_existing',return_value={'test_only_domain':197}):
            node = BufferedRawSource(context)
        original = node.audit
        calls = [0]
        def slow_graph():
            calls[0]+=1
            if calls[0]%50==0:time.sleep(.12)
            return original()
        node.audit = slow_graph
        stall_ms = float(os.environ.get('WAND_SOAK_INPUT_STALL_MS','0'))
        if stall_ms:
            import mecanum_wand_4wsc_hardware.source_pipeline as pipeline
            original_drain = pipeline.drain_callbacks
            stall_after = time.monotonic()+8.
            stalled = [False]
            def delayed_drain(*args,**kw):
                if not stalled[0] and time.monotonic()>stall_after:
                    stalled[0]=True;time.sleep(stall_ms/1000.)
                return original_drain(*args,**kw)
            pipeline.drain_callbacks=delayed_drain
        executor = SingleThreadedExecutor(context=context);executor.add_node(node)
        passive_loop(node,executor,channel,stop,ready)
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()
        channel.cancel_join_thread()


class ThreadRawPublisher:
    def __init__(self, motion_s=0., fault=None):
        bootstrap.install()
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import PointStamped,PoseStamped
        from mecanum_wand_4wsc_control.safety_nodes import SENSOR
        self.rclpy=rclpy;self.context=Context()
        rclpy.init(context=self.context,domain_id=197,signal_handler_options=SignalHandlerOptions.NO)
        self.node=Node('vicon_client',context=self.context,use_global_arguments=False,enable_rosout=False)
        self.sign=self.node.create_publisher(PoseStamped,'/vicon/Sign/Sign',SENSOR)
        self.car=self.node.create_publisher(PoseStamped,'/vicon/car01/car01',SENSOR)
        self.markers={name:self.node.create_publisher(PointStamped,'/vicon/Sign/markers/'+name,SENSOR)
                      for name in ('Sign1','Sign2','Sign3','Sign4','Sign5')}
        self.stop=threading.Event();self.rig=None;self.started=None;self.motion_s=motion_s
        self.frames=0;self.max_gap_s=0.;self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.fault=fault
        self.thread.start()

    def run(self):
        from geometry import synthetic_packet
        from geometry_msgs.msg import PointStamped
        from mecanum_wand_4wsc_control.safety_nodes import pose_message
        from mecanum_wand_4wsc_control.core import Pose
        previous=time.monotonic()
        try:
            while not self.stop.is_set():
                now=time.monotonic();self.max_gap_s=max(self.max_gap_s,now-previous);previous=now
                stamp=self.node.get_clock().now().to_msg();ros=stamp.sec+stamp.nanosec*1e-9
                t=0. if self.started is None else now-self.started
                x=.06*math.sin(2*math.pi*t/self.motion_s) if self.motion_s and t<self.motion_s else 0.
                packet=synthetic_packet(1.+x,1.,0.,now,ros);s=packet.samples[0]
                msg=pose_message(Pose(*s.position,0.),stamp,world='map')
                msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=s.quaternion
                dropped = self.fault=='source_drop' and self.started is not None and t>=3.
                if not dropped:self.sign.publish(msg)
                for s in packet.samples[1:]:
                    msg=PointStamped();msg.header.stamp=stamp;msg.header.frame_id='map'
                    msg.point.x,msg.point.y,msg.point.z=map(float,s.position)
                    if self.fault=='geometry' and self.started is not None and t>=3. and s.name=='Sign1':msg.point.x+=.05
                    if not dropped:self.markers[s.name].publish(msg)
                pose=(0.,0.,0.) if self.rig is None else self.rig.peer.sample()['pose']
                self.car.publish(pose_message(Pose(pose[0],pose[1],.1,pose[2]),stamp,world='map'))
                self.frames+=1
                self.stop.wait(max(0.,.005-(time.monotonic()-now)))
        except BaseException as error:self.error=repr(error)

    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        self.node.destroy_node();self.rclpy.shutdown(context=self.context)


def publisher_child(inputs, stats, stop, motion_s, fault):
    # Real Tracker is an independent C++ process/machine. Do not put its
    # surrogate in the ROS control driver's Python GIL/context.
    bootstrap.install()
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PointStamped,PoseStamped
    from mecanum_wand_4wsc_control.safety_nodes import SENSOR,pose_message
    from mecanum_wand_4wsc_control.core import Pose
    from mecanum_wand_4wsc_hardware.source_relay import latest
    from geometry import synthetic_packet
    context=Context();node=None;value={};frames=0;max_gap=0.;error=None
    try:
        rclpy.init(context=context,domain_id=197,signal_handler_options=SignalHandlerOptions.NO)
        node=Node('vicon_client',context=context,use_global_arguments=False,enable_rosout=False)
        sign=node.create_publisher(PoseStamped,'/vicon/Sign/Sign',SENSOR)
        car=node.create_publisher(PoseStamped,'/vicon/car01/car01',SENSOR)
        markers={name:node.create_publisher(PointStamped,'/vicon/Sign/markers/'+name,SENSOR)
                 for name in ('Sign1','Sign2','Sign3','Sign4','Sign5')}
        previous=last_stats=time.monotonic()
        while not stop.is_set():
            now=time.monotonic();max_gap=max(max_gap,now-previous);previous=now
            for _ in range(4):
                try:value=inputs.get_nowait()
                except queue.Empty:break
            stamp=node.get_clock().now().to_msg();ros=stamp.sec+stamp.nanosec*1e-9
            started=value.get('started');t=0. if started is None else now-started
            x=.06*math.sin(2*math.pi*t/motion_s) if motion_s and t<motion_s else 0.
            packet=synthetic_packet(1.+x,1.,0.,now,ros);sample=packet.samples[0]
            msg=pose_message(Pose(*sample.position,0.),stamp,world='map')
            msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=sample.quaternion
            dropped=fault=='source_drop' and started is not None and t>=3.
            if not dropped:sign.publish(msg)
            for sample in packet.samples[1:]:
                msg=PointStamped();msg.header.stamp=stamp;msg.header.frame_id='map'
                msg.point.x,msg.point.y,msg.point.z=map(float,sample.position)
                if fault=='geometry' and started is not None and t>=3. and sample.name=='Sign1':msg.point.x+=.05
                if not dropped:markers[sample.name].publish(msg)
            pose=value.get('pose',(0.,0.,0.))
            car.publish(pose_message(Pose(pose[0],pose[1],.1,pose[2]),stamp,world='map'))
            frames+=1
            if now-last_stats>=.2:
                latest(stats,dict(frames=frames,max_gap_s=max_gap,error=None));last_stats=now
            stop.wait(max(0.,.01-(time.monotonic()-now)))
    except BaseException as exc:error=repr(exc)
    finally:
        latest(stats,dict(frames=frames,max_gap_s=max_gap,error=error))
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()
        stats.cancel_join_thread();inputs.cancel_join_thread()


class RawPublisher:
    def __init__(self,motion_s=0.,fault=None):
        ctx=mp.get_context('spawn')
        self.inputs=ctx.Queue(maxsize=1);self.stats=ctx.Queue(maxsize=1);self.stop=ctx.Event()
        self.value=dict(frames=0,max_gap_s=0.,error=None);self.last_update=0.
        self.rig=None;self.started=None
        self.process=ctx.Process(target=publisher_child,args=(self.inputs,self.stats,self.stop,motion_s,fault),daemon=True)
        self.process.start()
    def update(self):
        from mecanum_wand_4wsc_hardware.source_relay import latest
        now=time.monotonic()
        if now-self.last_update>=.02:
            pose=(0.,0.,0.) if self.rig is None else self.rig.peer.sample()['pose']
            latest(self.inputs,dict(pose=pose,started=self.started));self.last_update=now
        for _ in range(4):
            try:self.value=self.stats.get_nowait()
            except queue.Empty:break
    @property
    def frames(self):return self.value['frames']
    @property
    def max_gap_s(self):return self.value['max_gap_s']
    @property
    def error(self):return self.value['error'] or (None if self.process.is_alive() else 'RAW_PUBLISHER_EXITED')
    def close(self):
        self.stop.set();self.process.join(timeout=2.);self.update()
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=1.)
        for channel in (self.inputs,self.stats):channel.close();channel.cancel_join_thread()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=600.)
    ap.add_argument('--motion-seconds',type=float,default=0.)
    ap.add_argument('--fault',choices=('source_drop','geometry'))
    ap.add_argument('--stall-input-ms',type=float,default=0.)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();assert 1<=a.seconds<=600
    a.output.mkdir(parents=True,exist_ok=False)
    path,consent=profile(READY,a.output,True)
    os.environ.update(WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),
        WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ['WAND_SOAK_INPUT_STALL_MS']=str(a.stall_input_ms)
    bootstrap.install()
    from mecanum_wand_4wsc_hardware.source_relay import PassiveReader,SourceRelay
    from session_channel import Channel
    from ground_runtime import GroundRig
    rt=bootstrap.runtime()
    assert rt.software_only and rt.domain==196 and rt.ip=='127.0.0.1' and rt.interface=='lo'
    assert rt.command_port not in (46000,46001) and rt.telemetry_port not in (46000,46001)
    publisher=RawPublisher(a.motion_seconds,a.fault)
    reader=PassiveReader()
    reader.process=mp.get_context('spawn').Process(target=raw_child,args=(reader.channel,reader.stop,reader.ready),daemon=True)
    original_init=SourceRelay.__init__
    def fixture_init(node,runtime=None,reader_ignored=None,**kw):
        assert runtime.software_only
        return original_init(node,runtime,reader)
    channel=Channel(readonly=True);session=a.output/'session';session.mkdir()
    rig=None;closed=None;result=None;error=None;active_start=None;minimum_remaining=600.;samples=0
    try:
        with patch.object(SourceRelay,'__init__',fixture_init):rig=GroundRig(rt,channel,session)
        publisher.rig=rig
        now=time.monotonic();assert channel.open('offline-soak',now)
        deadline=now+a.seconds+35.;sent=False;last_progress=now
        while time.monotonic()<deadline:
            begin=time.monotonic();channel.begin_tick(begin);channel.backend_tick(begin,begin);channel.finish_tick(begin)
            rig.step(time.monotonic());samples+=1
            publisher.update()
            if channel.ready and not sent:
                offer=channel.challenge(time.monotonic());assert offer
                accepted=channel.receive('offline-soak',dict(type='command',action='start',nonce=offer['nonce'],sequence=1),time.monotonic())
                assert accepted;sent=True
            if rig.active_since is not None and active_start is None:
                active_start=rig.active_since;publisher.started=active_start
                print('SOAK_ACTIVE '+json.dumps(dict(session=str(session),seconds=a.seconds)),flush=True)
            if active_start is not None:
                elapsed=time.monotonic()-active_start
                minimum_remaining=min(minimum_remaining,rig.status.get('pilot',{}).get('remaining_s',600.))
                if elapsed>=a.seconds:
                    channel.stop('SOAK_TIMER_COMPLETE');break
            if channel.stopped:break
            if publisher.error:raise RuntimeError(publisher.error)
            if time.monotonic()-last_progress>=20.:
                print('SOAK_PROGRESS '+json.dumps(dict(active_s=None if active_start is None else time.monotonic()-active_start,
                    state=rig.status.get('state'),source=rig.source.get('pipeline'),frames=publisher.frames)),flush=True)
                last_progress=time.monotonic()
            time.sleep(max(0.,.005-(time.monotonic()-begin)))
        elapsed=0. if active_start is None else time.monotonic()-active_start
        permitted_reason=channel.fault in ('SOAK_TIMER_COMPLETE','STAGE6_LONG600_TIME_COMPLETE')
        if a.fault:
            assert active_start is not None and 3.<=elapsed<4.,(elapsed,channel.fault)
            assert not permitted_reason and channel.fault,(elapsed,channel.fault)
        else:
            assert active_start is not None and elapsed>=a.seconds-.1 and permitted_reason,(elapsed,channel.fault)
    except BaseException as exc:
        error=repr(exc)
        raise
    finally:
        if rig is not None:closed=rig.close()
        publisher.close()
        elapsed=0. if active_start is None else (closed['transitions'][-2]['t']-active_start if len(closed['transitions'])>=2 else 0.)
        final=None if closed is None else closed['exact_C_peer']['final_sample']['status']
        zero=bool(final and not final['permitted'] and all(v==0 for v in final['raw_duty']) and all(v==0 for v in final['target_cps']))
        result=dict(passed=error is None and bool(closed and closed['stop_confirmed'] and zero),error=error,
            reason=channel.fault,requested_active_s=a.seconds,observed_active_s=elapsed,
            motion_first_s=a.motion_seconds,static_pose_republished_with_original_new_frame_stamps=True,
            injected_fault=a.fault,injected_input_stall_ms=a.stall_input_ms,
            actual_raw_ROS_domain=197,actual_control_ROS_domain=196,actual_native_C_loopback=True,
            accepted_commands=channel.accepted_commands,max_control_gap_s=channel.max_control_gap,
            frames=publisher.frames,raw_publisher_max_gap_s=publisher.max_gap_s,
            source_diagnostics={} if rig is None else rig.source.get('pipeline'),
            native_output_zero=zero,stop_confirmed=bool(closed and closed['stop_confirmed']),
            physical_output=False,firmware_changed=False,network_settings_changed=False)
        save(a.output/'RESULT.json',result)
        print('SOAK_RESULT '+json.dumps(result),flush=True)

if __name__=='__main__':main()
