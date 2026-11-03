import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge
from mecanum_vicon_control.transport_protocol import decode_frame, unpack_cmd_vel


def test_readonly_bridge_never_enables_or_sends_velocity():
    rclpy.init(args=['--ros-args','-p','readonly_mode:=true'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self): self.frames=[]
        def sendto(self,data,_destination): self.frames.append(data); return len(data)
        def close(self): pass
    try:
        node.on_enable(Bool(data=True))
        assert not node.safety_enabled
        node.on_control_enable(Bool(data=True))
        node.on_clear_faults(Bool(data=True))
        assert node.pending_control_action is None
        cmd=Twist(); cmd.linear.x=1; cmd.linear.y=1; cmd.angular.z=1
        node.on_cmd(cmd)
        node.sock=Sink(); node.session.connected=True; node.session.session_id=10
        node.safety_enabled=True  # 独立防线：即使上层变量被误置，也不能下发运动。
        node.send_command()
        frame=decode_frame(node.sock.frames[-1]); command=unpack_cmd_vel(frame.payload)
        assert frame.flags==0
        assert command.vx_mps==command.vy_mps==command.wz_radps==0
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
