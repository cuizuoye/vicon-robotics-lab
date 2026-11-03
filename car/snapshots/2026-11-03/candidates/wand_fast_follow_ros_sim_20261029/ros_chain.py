"""Actual DDS source binding -> independent final gate -> signed SOFTWARE car.

Fixed private domain189, local discovery and a per-run namespace/key. No
HardwareBridge/Runtime, UDP motor port, serial port, or unsigned plant input.
The retained .20/.10/.15/.30 deadlines and .70/.80 limits are not relaxed.
"""
from dataclasses import asdict
import json
import math
import os
import secrets
import time
from types import SimpleNamespace as NS

from bootstrap import (config, FastCatchupSession, ZERO, Pose, Velocity,
                       history_imports, save)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from mecanum_wand_4ws8_control.rotation_source import Packet, Raw, definition, canonical
from mecanum_wand_4ws8_control.rotation_geometry import REQUIRED, rotation
from mecanum_wand_4ws8_control.rotation_core import RotationSupervisor
from mecanum_wand_4ws8_control.pose_quality import CarPoseQuality
from mecanum_wand_4ws8_control.safety_core import Lease
from mecanum_wand_4ws8_control.safety_nodes import pose_message, LATEST, SENSOR
from mecanum_wand_4ws8_control.offline_suite import Plant
from mecanum_wand_4ws8_hardware.rotation_nodes import RotationSupervisorNode, RotationGateNode
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner, AuthorityVerifier

HistoryQosMixin, BoundGateMixin, SourceFaultGate = history_imports()
DOMAIN = 189


class Supervisor(HistoryQosMixin, RotationSupervisorNode):
    def __init__(self, runtime):
        super().__init__(runtime)
        self.use_candidate()

    def use_candidate(self):
        c = self.core.c
        self.core.follower = FastCatchupSession(c.follower_config(), c.project_goal, c.project_reference)

    def create_publisher(self,msg_type,topic,*args,**kwargs):
        if topic!='candidate':return super().create_publisher(msg_type,topic,*args,**kwargs)
        # Same selected-source envelope, with an isolated test hook AFTER
        # binding. The private test may alter a digest; the gate must reject it.
        pub=Node.create_publisher(self,msg_type,topic,HISTORY)
        node=self
        class BoundPublisher:
            def publish(self,msg):
                p=node.selected;car=node.core.inputs.poses.get('car')
                value=dict(frame=json.loads(msg.data),sign_digest=p.digest if p else None,
                    sign_stamp=p.stamp if p else None,car_stamp=car['source_ros'] if car else None)
                hook=getattr(node,'modify_bound',lambda value:value)
                pub.publish(String(data=canonical(hook(value))))
        return BoundPublisher()


class Gate(BoundGateMixin, RotationGateNode):
    def __init__(self, runtime):
        super().__init__(runtime)
        self.authority = AuthoritySigner(os.environ['WAND_4WS8_CHANNEL_KEY'], 4)
        self.modify_wire = lambda text: text  # Isolated test seam, never hardware.

    def publish_output(self, output):
        f = self.core.frame
        permit = bool(f and f.epoch and f.epoch == self.core.primed_epoch
            and not self.core.fault and not self.core.hard_fault and f.state in ('ARMED','ACTIVE'))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = output.vx, output.vy, output.wz
        self.commands.publish(msg)  # Observation only; plant does not subscribe.
        text = self.authority.encode(output, *self.clocks(), epoch=f.epoch if f else '',
            state=f.state if f else 'STANDBY', permit=permit, estop=self.core.hard_fault)
        self.actuator.publish(String(data=self.modify_wire(text)))


class SoftwarePlant(Node):
    def __init__(self, runtime):
        super().__init__('safe_sim_plant', namespace=runtime.namespace, enable_rosout=False)
        runtime.guard(self)
        self.plant = Plant(Pose(0., 0., runtime.config.ground_z_m, 0.))
        self.verifier = AuthorityVerifier(os.environ['WAND_4WS8_CHANNEL_KEY'], 4)
        self.command = ZERO
        self.last_command = -math.inf
        self.last_tick = time.monotonic()
        self.sequence = self.accepted = 0
        self.rejection = ''
        self.fault_bits = 0
        self.feedback_on = self.pose_on = True
        self.pose_pub = self.create_publisher(PoseStamped, 'car_pose', HISTORY)
        self.feedback_pub = self.create_publisher(String, 'plant_feedback', LATEST)
        self.create_subscription(String, 'actuator_command', self.on_command, LATEST)
        self.create_timer(.01, self.tick)

    def on_command(self, msg):
        try:
            _, command = self.verifier.decode(msg.data, time.monotonic(), self.get_clock().now().nanoseconds*1e-9)
            if not self.fault_bits:
                self.command = command
            self.last_command = time.monotonic()
            self.accepted += 1
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.rejection = self.rejection or str(error)
            self.fault_bits |= 1 << 30
            self.command = ZERO

    def tick(self):
        now = time.monotonic()
        dt = min(.10, max(0., now-self.last_tick)); self.last_tick = now
        command = self.command if 0 <= now-self.last_command <= .10 and not self.fault_bits else ZERO
        self.plant.step(command, dt)
        self.sequence += 1
        stamp = self.get_clock().now()
        if self.pose_on:
            self.pose_pub.publish(pose_message(self.plant.pose, stamp.to_msg(), world='map'))
        if self.feedback_on:
            self.feedback_pub.publish(String(data=canonical(dict(sequence=self.sequence,
                issued_steady=now, issued_ros=stamp.nanoseconds*1e-9,
                output_zero=command == ZERO, enabled=command != ZERO,
                speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz), fault_bits=self.fault_bits))))


from mecanum_wand_4ws8_control.bound_history import HISTORY_QOS as HISTORY


def synthetic_packet(x, y, turn, now, ros, z=1.):
    import numpy as np
    d = definition(); q = (0.,0.,math.sin(turn/2),math.cos(turn/2))
    r = rotation(q); raw = np.array([10.+x,10.+y,z])-r@np.array(d['local_points']['Sign5'])
    return Packet((Raw('sign',tuple(raw),q,ros,now), *(
        Raw(k,tuple(raw+r@np.array(d['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))


class Source(HistoryQosMixin, Node):
    def __init__(self, runtime, provider):
        super().__init__('sim_sign_source', namespace=runtime.namespace, enable_rosout=False)
        runtime.guard(self)
        self.provider = provider
        self.packet = self.last_packet = None
        self.sent = 0
        self.on = True
        self.modify_packet = lambda p: p
        self.packet_pub = self.create_publisher(String, 'rotation_packet', HISTORY)
        self.pose_pub = self.create_publisher(PoseStamped, 'wand_pose', SENSOR)
        self.mode_pub = self.create_publisher(String, 'mode_claim', LATEST)
        self.info_pub = self.create_publisher(String, 'source_status', LATEST)
        self.create_timer(.01, self.tick)

    def tick(self):
        now = time.monotonic(); ros = self.get_clock().now().nanoseconds*1e-9
        self.mode_pub.publish(String(data='wand'))
        rejected=getattr(self.provider,'rejected',0)
        self.info_pub.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,rejected={'sign':rejected}))))
        packet = self.provider(now, ros) if self.on else None
        self.packet = packet
        if packet is None or (self.last_packet and packet.stamp <= self.last_packet.stamp):return
        # No synthetic restamping or refreshing of a real received packet.
        packet.pose(now, ros)
        if self.packet_pub.get_subscription_count() < 2:return
        self.last_packet = packet
        self.packet_pub.publish(String(data=canonical(self.modify_packet(packet).payload())))
        stamp = self.get_clock().now().to_msg(); stamp.sec = int(packet.stamp)
        stamp.nanosec = round((packet.stamp-stamp.sec)*1e9)
        if stamp.nanosec == 1_000_000_000:stamp.sec += 1; stamp.nanosec = 0
        self.pose_pub.publish(pose_message(packet.pose(now,ros),stamp,world='map'))
        self.sent += 1


class Operator(Node):
    def __init__(self, chain):
        super().__init__('wand_real_operator', namespace=chain.runtime.namespace, enable_rosout=False)
        chain.runtime.guard(self); self.chain = chain
        self.sequence = 0
        self.lease_pub = self.create_publisher(String,'operator_lease',LATEST)
        self.create_timer(.01,self.tick)

    def tick(self):
        c = self.chain; now = time.monotonic(); ros = self.get_clock().now().nanoseconds*1e-9
        if now-c.ui_seen > .10 and (c.requested or c.prepare_pending or c.sup.core.state=='ARMED'):
            c.stop('UI_HEARTBEAT_TIMEOUT')
        if c.sup.core.state not in ('ARMED','ACTIVE'):c.requested = False
        c.progress_preparation(now,ros)
        self.sequence += 1
        self.lease_pub.publish(String(data=canonical(asdict(Lease('private-sim-ui',self.sequence,now,ros,c.requested)))))


class Chain:
    def __init__(self, provider, output=None):
        if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
            raise ValueError('Private simulation requires process-local LOCALHOST discovery')
        self.config = config(); self.namespace = '/fast_follow_sim_'+secrets.token_hex(5)
        self.runtime = NS(namespace=self.namespace,world='map',config=self.config,mode='shadow')
        def guard(node):
            if node.get_namespace()!=self.namespace or node.context.get_domain_id()!=DOMAIN:
                raise ValueError('simulation ROS domain/namespace escaped')
            if node.get_parameter('use_sim_time').value:raise ValueError('real steady clock required')
        self.runtime.guard = guard
        self.runtime.expected = dict(car_pose='safe_sim_plant',wand_pose='sim_sign_source',
            rotation_packet='sim_sign_source',plant_feedback='safe_sim_plant',
            operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
            cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate')
        self.requested = self.prepare_pending = False
        self.ui_seen = time.monotonic(); self.prepare_requested = 0.
        self.message = '未使能：等Sign有效，点击准备'
        self.sup = Supervisor(self.runtime); self.gate = Gate(self.runtime)
        self.plant = SoftwarePlant(self.runtime); self.source = Source(self.runtime,provider)
        self.operator = Operator(self)
        self.nodes = [self.sup,self.gate,self.plant,self.source,self.operator]
        self.executor = SingleThreadedExecutor()
        self.events = []; self.previous = None; self.output = output
        self.trace_file = (output/'events.jsonl').open('w') if output else None
        self.latest = {}; self.max_pending = 0; self.sessions = 0
        for topic in ('candidate','rotation_packet','plant_feedback','supervisor_status','gate_status'):
            self.operator.create_subscription(String,topic,lambda msg,t=topic:self.observe(t,msg),HISTORY if topic in ('candidate','rotation_packet') else LATEST)
        for node in self.nodes:self.executor.add_node(node)
        if output:save(output/'CONTRACT.json',dict(kind='ACTUAL_ROS_SOFTWARE_ONLY',domain=DOMAIN,
            namespace=self.namespace,hardware_bridge_constructed=False,car_feedback='signed software Plant',
            source_binding='exact original six-topic packet digest/time and simulated car stamp',
            speed=.7,accel=.8,yaw=.1,yaw_accel=.4,session_s=120.,fixed_fence=False,
            xy_scale=1.,real_car_started=False,firmware_changed=False))

    def observe(self,topic,msg):
        value=json.loads(msg.data); self.latest[topic]=value
        if self.trace_file:
            self.trace_file.write(canonical(dict(received=time.monotonic(),topic=topic,value=value))+'\n')

    def touch(self):self.ui_seen=time.monotonic()

    def step(self):
        self.executor.spin_once(timeout_sec=.002)
        self.max_pending=max(self.max_pending,len(self.gate.pending))
        state=(self.sup.core.state,self.sup.core.reason,self.gate.core.fault)
        if state!=self.previous:
            self.events.append(dict(at=time.monotonic(),state=state));self.previous=state

    def prepare(self):
        s,g=self.sup.core,self.gate.core
        if self.ready():
            self.message='READY：准备已完成，单击空格开始，仅软件车';return True
        if self.prepare_pending:return True  # Repeated clicks never renew timeout.
        issue=self.prepare_blocker()
        if issue:
            self.message='暂不能准备：'+issue
            self.events.append(dict(at=time.monotonic(),event='PREPARE_REJECTED',reason=issue))
            return False
        self.requested=False
        if s.pilot.used or s.state in ('FAULT','PAUSED'):
            # Explicit human NEW SIM SESSION only. Preserve software pose and
            # residual velocity; never silently reset or reuse an arm epoch.
            self.sup.core=RotationSupervisor(self.config);self.sup.use_candidate()
            self.sup.core.owner=s.owner;self.sup.core.sequence=s.sequence
            self.gate.core=SourceFaultGate(self.config)
            self.gate.core.owner=s.owner;self.gate.core.last_sequence=s.sequence
            self.sup.core.inputs.graph_ok=self.gate.core.inputs.graph_ok=False
            self.sup.packet_cache=type(self.sup.packet_cache)();self.sup.selected=None
            self.gate.packet_cache=type(self.gate.packet_cache)()
            self.gate.pending.clear();self.gate.cars.clear()
            for node in (self.sup,self.gate):
                node.car_pose_quality=CarPoseQuality(self.config);node.first_stop_reason=''
            self.events.append(dict(at=time.monotonic(),event='EXPLICIT_NEW_SIM_SESSION_KEEP_PLANT'))
        self.sessions+=1;self.prepare_pending=True;self.prepare_requested=time.monotonic()
        self.message='准备中，不运动；等待独立健康与停稳窗口';return True

    def source_issue(self):
        p=self.source.packet
        if p is None:return 'Sign刚体或五点没有完整新帧，请恢复无遮挡跟踪'
        try:p.pose(*self.sup.clocks())
        except (ValueError,TypeError):return 'Sign同帧数据已过期或几何无效，请恢复跟踪'
        return ''

    @staticmethod
    def health_text(issue):
        return {'WAND_POSE_TIMEOUT':'Sign位姿已超时，请恢复无遮挡跟踪',
            'CAR_POSE_TIMEOUT':'软件车位姿已超时',
            'COMMAND_SOURCE_CONFLICT':'等待私有ROS来源身份检查通过',
            'FEEDBACK_TIMEOUT':'软件车反馈已超时',
            'OPERATOR_LEASE_TIMEOUT':'操作台心跳已超时',
            'ACTUATOR_FAULT':'软件执行器故障锁存'}.get(issue,'安全输入未就绪：'+issue if issue else '')

    def prepare_blocker(self):
        s,g=self.sup.core,self.gate.core
        if s.hard_fault or g.hard_fault or self.plant.fault_bits:return '硬故障锁存，不能在本窗口清除'
        if self.requested or s.state=='ACTIVE':return '软件车正在运行，请先停止'
        if self.prepare_pending:return '正在准备，请等待末级门握手'
        if s.state=='ARMED':return '监督器已预备，等待末级门握手；不要重复准备'
        issue=self.source_issue()
        if issue:return issue
        issue=s.inputs.health(*self.sup.clocks())
        if issue:return self.health_text(issue)
        if not s.stop_confirmed or s.state in ('STOPPING','STOP_UNCONFIRMED'):
            return '等待软件车停稳并确认零输出'
        return ''

    def progress_preparation(self,now,ros):
        if not self.prepare_pending:return
        s,g=self.sup.core,self.gate.core
        if now-self.prepare_requested>5.:
            issue=self.source_issue() or self.health_text(s.inputs.health(now,ros))
            self.stop('准备超时：'+(issue or '末级门握手未完成，请保留窗口检查'))
            return
        issue=self.source_issue()
        if issue:
            self.stop('准备已取消：'+issue);return
        if s.hard_fault or g.hard_fault or self.plant.fault_bits:
            self.stop('准备已取消：硬故障锁存');return
        if s.state=='ARMED':
            if self.ready():
                self.prepare_pending=False
                self.message='READY：末级门已确认，单击空格开始，仅软件车'
                self.events.append(dict(at=now,event='PREPARE_READY_AFTER_GATE'))
            else:self.message='准备中（2/2）：监督器已预备，等待末级门确认；尚不可开始'
            return
        if s.state!='STANDBY':
            self.stop('准备已取消：安全状态改变，请停稳后重新准备');return
        ok,_=s.request_start(now,ros)
        self.message=('准备中（2/2）：等待末级门确认；尚不可开始' if ok else
            '准备中（1/2）：'+(self.health_text(s.inputs.health(now,ros)) or '等待连续稳定和停稳时间窗'))

    def ready(self):
        s=self.sup.core; now,ros=self.sup.clocks()
        return bool(s.state=='ARMED' and not self.gate.core.fault and not s.inputs.health(now,ros)
            and not s.hard_fault and not self.gate.core.hard_fault and not self.plant.fault_bits
            and s.gate_fresh(now,ros) and self.gate.core.primed_epoch==s.epoch)

    def engage(self):
        if not self.ready():
            self.message='尚不可开始：'+(self.prepare_blocker() or '请先准备并等待READY');return False
        self.prepare_pending=False
        self.requested=True;self.events.append(dict(at=time.monotonic(),event='FRESH_SPACE_START'));return True

    def stop(self,reason='操作员停止'):
        self.requested=self.prepare_pending=False
        s=self.sup.core
        if s.state in ('ARMED','ACTIVE'):
            s.request_stop(time.monotonic(),reason)
        elif s.state not in ('STANDBY','PAUSED','FAULT','STOPPING','STOP_UNCONFIRMED'):
            s.request_stop(time.monotonic(),reason)
        self.message=reason+'；不会自动恢复'
        self.events.append(dict(at=time.monotonic(),event='STOP_REQUEST',reason=reason))

    def focus_lost(self):
        if self.requested or self.prepare_pending or self.sup.core.state in ('ARMED','ACTIVE'):
            self.stop('操作台失去焦点，准备/跟随已取消')
        else:
            self.events.append(dict(at=time.monotonic(),event='IDLE_FOCUS_LOSS_NO_MOTION_AUTHORITY'))

    def snapshot(self):
        s=self.sup.core; p=self.plant.plant.pose; f=s.follower
        goal=f.mapped_goal
        packet=self.source.packet
        now,ros=self.sup.clocks()
        valid=False
        if packet:
            try:packet.pose(now,ros);valid=True
            except ValueError:pass
        return dict(state=s.state,reason=s.reason,gate_fault=self.gate.core.fault,
            hard_fault=s.hard_fault or self.gate.core.hard_fault,message=self.message,
            ready=self.ready(),sign_valid=valid,source_stamp=packet.stamp if packet else None,
            car=asdict(p),goal=asdict(goal) if goal else None,output=asdict(self.gate.core.output),
            remaining=max(0.,120.-(now-s.active_started)) if s.active_started else 120.,
            stop_confirmed=s.stop_confirmed,accepted_signed=self.plant.accepted,
            signature_fault=self.plant.rejection,catchup_samples=f.catchup_ticks,
            feedforward_samples=f.moving_feedforward_ticks,sessions=self.sessions,
            input_health=s.inputs.health(now,ros),pending=len(self.gate.pending),
            prepare_pending=self.prepare_pending,prepare_blocker=self.prepare_blocker(),
            can_prepare=not self.prepare_blocker() and not self.ready())

    def close(self):
        self.stop('窗口关闭')
        until=time.monotonic()+2.
        while time.monotonic()<until:
            self.touch();self.step()
            if self.sup.core.stop_confirmed and self.plant.command==ZERO:break
        result=self.snapshot();result.update(events=self.events,max_pending=self.max_pending,
            hardware_bridge_constructed=False,physical_car_started=False)
        if self.output:save(self.output/'SESSION_RESULT.json',result)
        self.executor.shutdown()
        for node in reversed(self.nodes):node.destroy_node()
        if self.trace_file:self.trace_file.close()


def init_private():
    os.environ['WAND_4WS8_CHANNEL_KEY']=secrets.token_hex(32)
    rclpy.init(args=[],domain_id=DOMAIN)
