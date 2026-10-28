"""Read-only domain0 pose preflight. Never publishes commands or opens MC-Link."""
import json,math,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig as StagedSafetyConfig
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control.safety_nodes import decode_pose
from mecanum_wand_4ws8_control.pose_quality import CarPoseQuality
ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
def main():
    c=StagedSafetyConfig(**json.loads(Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_feedback_300ms_startup_2026-10-27/prepared/runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])
    rclpy.init();node=rclpy.create_node('oneclick_readonly_preflight',enable_rosout=False)
    quality=CarPoseQuality(c)
    def receive(key,msg):
        latest[key]=(msg,time.monotonic())
        if key=='car01':
            try:
                pose,stamp,roll,pitch=decode_pose(msg,'map')
                quality.observe(pose,stamp,time.monotonic(),node.get_clock().now().nanoseconds*1e-9,roll,pitch)
            except (ValueError,TypeError):quality.reject('POSE_DECODE_INVALID',time.monotonic())
    latest={};q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    for key in ('Sign','car01'):
        node.create_subscription(PoseStamped,f'/vicon/{key}/{key}',lambda m,k=key:receive(k,m),q)
    started=time.monotonic()
    while time.monotonic()-started<3.:rclpy.spin_once(node,timeout_sec=.05)
    result=dict(ready=False,issues=[],bodies={})
    try:
        now_ros=node.get_clock().now().nanoseconds*1e-9
        for key in ('Sign','car01'):
            if key not in latest:result['issues'].append(f'看不到{key}：请检查Vicon刚体及遮挡');continue
            m,at=latest[key];p=m.pose.position;q=m.pose.orientation
            age=now_ros-m.header.stamp.sec-m.header.stamp.nanosec*1e-9
            publishers=node.get_publishers_info_by_topic(f'/vicon/{key}/{key}')
            valid=(m.header.frame_id=='map' and -.005<=age<=.20 and time.monotonic()-at<=.20
                and len(publishers)==1 and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)))
            result['bodies'][key]=dict(fresh=valid,age_s=age)
            if not valid:result['issues'].append(f'{key}定位过期、坐标异常或存在重复发布源')
        if 'car01' in latest:
            m=latest['car01'][0];p=m.pose.position;q=m.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+2.356194490192345
            car=Pose(p.x,p.y,p.z,math.atan2(math.sin(yaw),math.cos(yaw)))
            result['car_world_m']=[car.x,car.y,car.z]
            result['car_heading_deg']=math.degrees(car.yaw)
            if abs(car.z-c.ground_z_m)>c.lift_tolerance_m:result['issues'].append('小车高度异常：请确认已经落地放稳')
        result['car_pose_quality']=quality.snapshot()
        issue=quality.preflight_issue()
        if issue:result['issues'].append(issue)
        result['ready']=not result['issues'];print(json.dumps(result,ensure_ascii=False))
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
