#!/usr/bin/env python3
"""Mechanically derive an isolated real-Sign/SIM ONLY console; preserve inputs."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'candidates/wand_low_latency_servo/live_sign_sim.py'
BASE_SHA = '6d061ed61d83be53570f8ef37277f4ffc9c1a9e6a4bc4dcb75cdf34e9a113646'


def build(output):
    if output.exists():
        raise FileExistsError('new live simulation directory required')
    if hashlib.sha256(BASE.read_bytes()).hexdigest() != BASE_SHA:
        raise ValueError('accepted live adapter drift')
    source = BASE.read_text()
    def once(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError('ambiguous adaptation: '+old[:80])
        source = source.replace(old, new)
    once("DOMAIN=114", "DOMAIN=115")
    once("NAMESPACE='/wand_servo_real_sign_sim'", "NAMESPACE='/wand_servo_matching_sim'")
    once("from mecanum_wand_servo_hardware.commissioning import Runtime as OfflineRuntime",
         "from matching_sim import SimOnlyLimits\nfrom matching_wire import MatchingOutputSigner,MatchingOutputVerifier\n"
         "from mecanum_wand_servo_control.offline_validation import Plant as MatchedPlant\n"
         "from mecanum_wand_servo_hardware.commissioning import Runtime as OfflineRuntime")
    once("super().__init__('shadow',str(PROFILE))", "super().__init__('shadow',str(PROFILE))\n"
         "        self.config=SimOnlyLimits(.70,2.00)")
    once("class TraceGate(GateNode):", "class MatchingPlantNode(PlantNode):\n"
         "    def __init__(self,runtime):\n        super().__init__(runtime)\n"
         "        self.plant=MatchedPlant(runtime.initial_pose,lag_s=.12,gains=(.98,.88,.95))\n"
         "        self.verifier=MatchingOutputVerifier(os.environ.get('WAND_SERVO_CHANNEL_KEY',''))\n\n\n"
         "class TraceGate(GateNode):")
    once("        self.timing_pub=self.create_publisher(String,'gate_timing',LATEST)",
         "        self.signer=MatchingOutputSigner(os.environ.get('WAND_SERVO_CHANNEL_KEY',''))\n"
         "        self.timing_pub=self.create_publisher(String,'gate_timing',LATEST)")
    once("plant=lambda:PlantNode(runtime)", "plant=lambda:MatchingPlantNode(runtime)")
    once("title='真实 Sign → 低延迟连续伺服仿真（实车断电）'", "title='真实 Sign → 0.70m/s · 2.00m/s² 仿真（实车断电）'")
    once("域114 · 1:1 XY · 固定3×3m · ≤0.20m/s / 0.08m/s²", "域115 · 1:1 XY · 固定3×3m · ≤0.70m/s / 2.00m/s²")
    once("低延迟连续跟随：", "0.70/2.00仿真跟随：")
    once("class ServoConsole(Console):", "class ServoConsole(Console):\n"
         "    def loop(self):\n        self.loop_job=None\n        if self.closing:return\n"
         "        if self.key_state is not None and self.model.key_down:\n"
         "            if not self.server_space_down() and not self.closing:self.finish_release()\n"
         "        if self.closing:return\n        self.node.pump()\n        now=time.monotonic()\n"
         "        if now-self.last_draw>=.033:\n            self.draw(now)\n            self.last_draw=now\n"
         "        if not self.closing:self.loop_job=self.root.after(20,self.loop)\n")
    once("lock=open('/tmp/wand_servo_live_sim.lock','a')", "lock=open('/tmp/wand_servo_matching_live_sim.lock','a')")
    once("[w]and_acceptance_transport|[c]md_vel_udp_bridge|[r]ectangle_run_guard",
         "[w]and_acceptance_transport|[c]md_vel_udp_bridge|[r]ectangle_run_guard|[l]ive_sign_sim.py")
    once("mapping_scale=1.,goal_area_m=[3,3],hardware_transport=False,auto_enable=False,",
         "mapping_scale=1.,goal_area_m=[3,3],hardware_transport=False,auto_enable=False,\n"
         "        speed_limit_mps=.70,acceleration_limit_mps2=2.,signed_channel_scope='SIM_ONLY_SIGN_MATCHING_V070_A200_V1',\n"
         "        plant_model='first-order .12s; gains .98/.88/.95, not identified hardware',ui_draw_interval_s=.033,")
    source = source.replace('domain114', 'domain115')
    # Only the UI/adapter copy differs. Controller and independent gate are
    # imported directly from the unchanged hash-checked verified runtime.
    output.mkdir(parents=True)
    (output/'COLCON_IGNORE').write_text('')
    (output/'live_matching_sim.py').write_text(source)
    inputs = {str(BASE): BASE_SHA}
    for name in ('matching_sim.py','matching_wire.py', 'build_live_candidate.py'):
        path=Path(__file__).with_name(name)
        inputs[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output/'generation.json').write_text(json.dumps(dict(simulation_only=True,
        no_hardware_mode=True,domain=115,namespace='/wand_servo_matching_sim',
        inputs_sha256=inputs,output_sha256=hashlib.sha256(source.encode()).hexdigest()),indent=2)+'\n')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    build(parser.parse_args().output)
