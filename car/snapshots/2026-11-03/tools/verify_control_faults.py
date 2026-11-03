#!/usr/bin/env python3
"""Isolated ROS fault injection. Synthetic poses only, no hardware bridge or sockets."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from mecanum_vicon_control.car_state_bridge import CarStateBridge
from mecanum_vicon_control.pose_controller import PoseController
from mecanum_vicon_control.safety_watchdog import SafetyWatchdog

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('refusing to overwrite evidence')
    parameters=['input_pose_topic:=/synthetic/car01','max_vx_mps:=0.05','max_vy_mps:=0.05',
        'max_wz_radps:=0.2','max_linear_accel_mps2:=0.08','max_angular_accel_radps2:=0.4',
        'start_enabled:=false']
    args=['--ros-args']
    for param in parameters:args+=['-p',param]
    rclpy.init(args=args,domain_id=98)
    ex=SingleThreadedExecutor()
    bridge=CarStateBridge();controller=PoseController();watchdog=SafetyWatchdog()
    observer=Node('control_fault_injection_observer')
    nodes=[bridge,controller,watchdog,observer]
    for n in nodes:ex.add_node(n)
    source=observer.create_publisher(PoseStamped,'/synthetic/car01',10)
    goal=observer.create_publisher(PoseStamped,'/car/goal_pose',10)
    enable=observer.create_publisher(Bool,'/car/enable',10)
    state={'kind':'normal','goal':[1.0,0.0]}
    raw=[];out=[];started=time.monotonic()
    def record(rows,m):rows.append([time.monotonic()-started,m.linear.x,m.linear.y,m.angular.z])
    observer.create_subscription(Twist,'/car/cmd_vel_raw',lambda m:record(raw,m),20)
    observer.create_subscription(Twist,'/car/cmd_vel',lambda m:record(out,m),20)
    def publish():
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=observer.get_clock().now().to_msg()
        m.pose.orientation.w=1.0
        if state['kind']=='stale':m.header.stamp.sec-=1
        if state['kind']!='outage':source.publish(m)
        g=PoseStamped();g.header.frame_id='map';g.header.stamp=observer.get_clock().now().to_msg()
        g.pose.position.x,g.pose.position.y=state['goal'];g.pose.orientation.w=1.0
        goal.publish(g)
    timer=observer.create_timer(.02,publish)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
    def zero():return out and all(abs(v)<1e-9 for v in out[-1][1:])
    def activate():
        assert watchdog.is_fresh(watchdog.get_clock().now().nanoseconds)
        enable.publish(Bool(data=True));spin(.8)
        assert watchdog.enabled and any(abs(v)>.005 for v in out[-1][1:])
    results=dict(ros_domain_id=98,synthetic_only=True,hardware_bridge_loaded=False,
                 physical_commands_sent=False,tests={},success=False)
    tests=results['tests']
    try:
        spin(2.2)
        assert not any('udp_bridge' in n or 'vicon_client' in n for n in observer.get_node_names())
        tests['startup_disabled_zero']=not watchdog.enabled and zero()
        activate();tests['explicit_enable_only']=watchdog.enabled
        state['goal']=[0.0,1.0];spin(.9)
        tests['corner_slew_limit']=all(abs(b[j]-a[j])<=.08*.02+1e-8
            for a,b in zip(raw,raw[1:]) for j in (1,2))
        tests['velocity_bounds']=all(abs(r[1])<=.05+1e-9 and abs(r[2])<=.05+1e-9 and
                                     abs(r[3])<=.2+1e-9 for r in raw)
        tests['new_segment_direction']=out[-1][2]>.04 and abs(out[-1][1])<1e-8
        for kind in ['stale','outage','command_outage']:
            if not watchdog.enabled:activate()
            if kind=='command_outage':ex.remove_node(controller)
            else:state['kind']=kind
            begin=time.monotonic();deadline=begin+.6
            while watchdog.enabled and time.monotonic()<deadline:ex.spin_once(timeout_sec=.005)
            elapsed=time.monotonic()-begin;spin(.08)
            tests[kind+'_latched_zero']=not watchdog.enabled and zero()
            tests[kind+'_stops_under_300ms']=elapsed<.30
            results[kind+'_host_stop_s']=elapsed
            if kind=='command_outage':ex.add_node(controller)
            state['kind']='normal';spin(.5)
            tests[kind+'_restore_stays_disabled']=not watchdog.enabled and zero()
        activate();enable.publish(Bool(data=False));spin(.1)
        tests['operator_disable_zero']=not watchdog.enabled and zero()
        results['success']=all(tests.values())
        assert results['success'],tests
    except Exception as error:
        results['error']=str(error)
        raise
    finally:
        enable.publish(Bool(data=False));spin(.06)
        timer.cancel();results['raw_commands']=raw;results['safe_commands']=out
        for n in nodes:ex.remove_node(n);n.destroy_node()
        ex.shutdown();rclpy.shutdown()
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x') as f:json.dump(results,f,indent=2)
        print(json.dumps({k:v for k,v in results.items() if k not in ('raw_commands','safe_commands')}))
    return 0 if results['success'] else 1

if __name__=='__main__':raise SystemExit(main())
