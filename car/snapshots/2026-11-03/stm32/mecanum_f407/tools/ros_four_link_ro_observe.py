#!/usr/bin/env python3
"""真实无线只读桥 + ROS话题订阅验收；无使能和非零速度入口。"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/mecanum_vicon_control'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String, UInt16, UInt32
from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge
from ros_observation_stats import valid_duration,feedback_gaps_ms
from readonly_send_timing import TimedSocket,summarize_sends


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--esp-ip',required=True); p.add_argument('--interface',required=True)
    p.add_argument('--seconds',type=float,default=10); p.add_argument('--output',required=True)
    confirms=p.add_mutually_exclusive_group(required=True)
    confirms.add_argument('--confirm-battery-disconnected',action='store_true',
                          help='battery physically disconnected')
    confirms.add_argument('--confirm-battery-powered-motors-connected-wheels-suspended',
                          action='store_true',
                          help='battery powered, motors connected, chassis secured with all wheels suspended')
    p.add_argument('--record-send-timing',action='store_true')
    p.add_argument('--stop-on-first-fault',action='store_true')
    p.add_argument('--low-latency-qos',action='store_true',
                   help='Bench only: mark this UDP socket EF DSCP and local priority 6; no global network changes')
    a=p.parse_args(argv)
    if not valid_duration(a.seconds): p.error('--seconds must be 5..120 (observation only)')
    if Path(a.output).exists():p.error('Refuse to overwrite prior evidence')
    return a


def main():
    a=parse_args()
    # ROS2与UDP都只对本次进程设置参数，不更改系统代理、路由或网卡。
    args=['--ros-args','-p',f'esp_ip:={a.esp_ip}','-p',f'network_interface:={a.interface}',
          '-p','readonly_mode:=true','-p',f'udp_low_latency_qos:={str(a.low_latency_qos).lower()}']
    for key in ('cmd_topic','enable_topic','estop_topic','control_enable_topic','clear_faults_topic'):
        args += ['-p',f'{key}:=/comm_ro_private/{key}']
    rclpy.init(args=args)
    bridge=CmdVelUdpBridge()
    qos=dict(bridge.socket_qos)
    observer=Node('four_link_readonly_observer')
    executor=SingleThreadedExecutor(); executor.add_node(bridge); executor.add_node(observer)
    rows={key:[] for key in ('connected','wheel_rpm','encoder_counts','fault','state','sequence','diagnostics','link_status')}
    subscriptions=[]
    bindings=[('connected','/car/mc_link_connected',Bool),
              ('wheel_rpm','/car/wheel_rpm',Float32MultiArray),
              ('encoder_counts','/car/encoder_counts',Int32MultiArray),
              ('fault','/car/motor_fault_bits',UInt32),('state','/car/motor_state_flags',UInt16),
              ('sequence','/car/last_motor_command_sequence',UInt32),
              ('diagnostics','/car/mc_link_diagnostics',String),
              ('link_status','/car/mc_link_status',String)]
    start=time.monotonic()
    send_recorder=TimedSocket(bridge.sock,start) if a.record_send_timing else None
    if send_recorder:bridge.sock=send_recorder
    for key,topic,kind in bindings:
        def callback(message,key=key):
            value=list(message.data) if key in ('wheel_rpm','encoder_counts') else message.data
            if key=='diagnostics': value=json.loads(value)
            rows[key].append(dict(time_s=time.monotonic()-start,value=value))
        subscriptions.append(observer.create_subscription(kind,topic,callback,10))
    result=dict(battery_disconnected_confirmed=bool(a.confirm_battery_disconnected),
                battery_powered_motors_connected_wheels_suspended_confirmed=bool(
                    a.confirm_battery_powered_motors_connected_wheels_suspended),
                motors_connected=bool(
                    a.confirm_battery_powered_motors_connected_wheels_suspended),
                motion_enabled=False,readonly_mode=True,topics=rows,
                requested_observation_s=a.seconds,wire_command_timeout_ms=150,socket_qos=qos)
    try:
        next_progress=30.0
        while time.monotonic()-start < a.seconds:
            executor.spin_once(timeout_sec=0.005)
            if a.stop_on_first_fault and rows['fault'] and rows['fault'][-1]['value']:
                raise RuntimeError('MCU fault observed; bounded observation stopped early')
            if time.monotonic()-start>=next_progress:
                print(json.dumps(dict(event='readonly_observation_progress',
                    elapsed_s=round(time.monotonic()-start,2),
                    wheel_frames=len(rows['wheel_rpm']),diagnostic_frames=len(rows['diagnostics']),
                    observed_fault_bits=sorted({r['value'] for r in rows['fault']}),
                    host_stop_latched=bridge.local_stop_latched)),flush=True)
                next_progress+=30.0
        if not rows['connected'] or not rows['connected'][-1]['value']:
            raise RuntimeError('ROS bridge not connected')
        if bridge.local_stop_latched:
            raise RuntimeError('Host stop latch set during readonly observation')
        for key in ('wheel_rpm','encoder_counts','fault','state','sequence','diagnostics'):
            if len(rows[key])<20: raise RuntimeError(f'{key}: insufficient live ROS messages')
        if any(r['value'] for r in rows['fault']): raise RuntimeError('MCU fault reported over ROS')
        if any(r['value']!=109 for r in rows['state']): raise RuntimeError('not healthy four-wheel readonly state')
        if any(len(r['value'])!=4 for key in ('wheel_rpm','encoder_counts') for r in rows[key]):
            raise RuntimeError('not four-wheel feedback')
        if len({r['value'] for r in rows['sequence']})<10: raise RuntimeError('command sequence stalled')
        first,last=rows['diagnostics'][0],rows['diagnostics'][-1]
        elapsed=last['time_s']-first['time_s']
        result['diagnostics_hz']=(len(rows['diagnostics'])-1)/elapsed
        result['sample_hz']=(last['value']['ticks']-first['value']['ticks'])/elapsed
        if not 8 <= result['diagnostics_hz'] <= 12 or not 90 <= result['sample_hz'] <= 110:
            raise RuntimeError('live ROS feedback/sampling rates failed')
        result['success']=True
        print(json.dumps(dict(success=True,sample_hz=result['sample_hz'],
                diagnostics_hz=result['diagnostics_hz'],messages={k:len(v) for k,v in rows.items()},
                last_diagnostics=last['value']),ensure_ascii=False))
    except Exception as error:
        result['success']=False; result['error']=str(error)
        raise
    finally:
        result['observation_end_s']=time.monotonic()-start
        result['feedback_max_host_gap_ms']=feedback_gaps_ms(rows)
        result['feedback_gap_is_not_inbound_command_gap']=True
        result['host_stop_latched']=bridge.local_stop_latched
        result['host_safety_enabled']=bridge.safety_enabled
        result['host_pending_control_action']=bridge.pending_control_action
        bridge.destroy_node(); observer.destroy_node(); executor.shutdown()
        if rclpy.ok(): rclpy.shutdown()
        if send_recorder:
            first_fault=next((r['time_s'] for r in rows['fault'] if r['value']),None)
            result['host_send_timing']=send_recorder.records
            result['prefault_host_send_summary']=summarize_sends(send_recorder.records,
                first_fault if first_fault is not None else result['observation_end_s'])
        Path(a.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__': main()
