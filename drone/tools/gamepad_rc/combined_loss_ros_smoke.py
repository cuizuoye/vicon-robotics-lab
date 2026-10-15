#!/usr/bin/env python3
"""域91的合成ROS输入中断测试；零串口、零飞控发送/写参。"""

import argparse
import datetime
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import PoseGate
from combined_input_loss import CombinedInputLoss
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='91':
        ap.error('Run only with process-local ROS_DOMAIN_ID=91; no live Vicon topic injection')
    if args.output.exists(): ap.error('Use a new evidence output')
    rclpy.init(args=[])
    node=Node('combined_loss_synthetic_smoke')
    gate=PoseGate(max_age_ms=150)
    loss=CombinedInputLoss()
    topic='/test/combined_input_loss_pose'
    publisher=node.create_publisher(PoseStamped,topic,10)
    times=[]; suppressed=0
    accept=True
    def receive(message):
        nonlocal suppressed
        if not accept:
            suppressed+=1
            return
        p,q=message.pose.position,message.pose.orientation
        gate.ingest(message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                    node.get_clock().now().nanoseconds,time.monotonic_ns(),message.header.frame_id,
                    [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
    subscription=node.create_subscription(PoseStamped,topic,receive,10)
    start=time.monotonic();next_pose=.5;next_take=.5
    published=0
    try:
        while time.monotonic()-start<4:
            elapsed=time.monotonic()-start
            if elapsed>=next_pose:
                message=PoseStamped();message.header.frame_id='map'
                message.header.stamp=node.get_clock().now().to_msg()
                message.pose.position.z=.1;message.pose.orientation.w=1.0
                publisher.publish(message);published+=1
                next_pose=elapsed+1/30
            rclpy.spin_once(node,timeout_sec=.002)
            now_ns=time.monotonic_ns(); elapsed=time.monotonic()-start
            if elapsed>=2 and loss.masked_at_s is None:
                # 合成融合前置条件只作为测试桩，绝不声称连接/验证了实机融合。
                if len(times)<20: raise RuntimeError('Synthetic input discovery failed before planned mask')
                loss.mask(elapsed,fusion_verified=True); accept=False
            if elapsed>=2.5: accept=True
            fresh=gate.fresh(node.get_clock().now().nanoseconds,now_ns)
            permitted=(loss.permits(fresh,elapsed,len(times)) if loss.masked_at_s is not None else fresh)
            if permitted and elapsed>=next_take:
                sample=gate.take_new(node.get_clock().now().nanoseconds,now_ns)
                if sample: times.append(elapsed)
                next_take=elapsed+1/30
        result=loss.result(last_tx_s=times[-1],tx_count_after_recovery=len(times),suppressed=suppressed,
                           fresh_restored=gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()),
                           rollback_verified=True)
        result.update({'scope':'Synthetic ROS runtime only, NOT live Vicon/FC fusion or rollback',
                       'fusion_and_zero_restore_arguments_are_test_stubs':True,
                       'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
                       'ros_domain_id':91,'topic':topic,'published_samples':published,
                       'fresh_samples_consumed_before_latch':len(times),
                       'actual_sent_to_fc':0,'parameter_writes':0,'flight_ready':False})
        with args.output.open('x') as stream:
            json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result['verified'] else 2
    finally:
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()


if __name__=='__main__': raise SystemExit(main())
