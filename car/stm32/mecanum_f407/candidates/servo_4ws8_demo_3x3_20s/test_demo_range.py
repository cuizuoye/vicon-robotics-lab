"""Offline fixed-arena range/authority regressions; no live ROS or hardware."""
from dataclasses import replace
import json
import math
from pathlib import Path
import pytest
from test_pilot_bounds import config,harness,ROOT,GENERATED,actual_review
from test_origin_capture import prepared,adjust,enable
from test_pilot_ui import model
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control.pilot_core import POLICY,PilotMonitor
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.pilot_ui import reason_text
from mecanum_wand_4ws8_hardware.authorization import verify_trial_review


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('offline regression: hardware/ROS initialization prohibited')
    monkeypatch.setattr(socket,'socket',forbidden)
    monkeypatch.setattr(rclpy,'init',forbidden)


def pose_at(c,f,l,yaw=None):
    return Pose(*c.geometry().world(f,l),c.ground_z_m,c.arena_heading_rad if yaw is None else yaw)


def ramp(h,start,end,seconds):
    count=round(seconds/.02)
    for i in range(1,count+1):
        t=i/count;h.set_hand(*(a+(b-a)*t for a,b in zip(start,end)))
        h.step()
        assert h.sup.state=='ACTIVE',(h.sup.reason,h.gate.fault)
        assert h.c.inside(h.plant.pose)
        assert h.c.goal_inside(h.sup.follower.mapped_goal)


@pytest.mark.parametrize('forward,left',[(.5,0.),(-.5,0.),(0.,.5),(0.,-.5),(.8,.6)])
def test_normal_demonstration_all_directions_exceeds_old_hand_car_side_path_limits(forward,left):
    h=harness.Harness(config(),origin_local=(1.5,1.5));h.start()
    ramp(h,(0.,0.),(forward,left),3.)
    h.run(2.)
    assert h.sup.state=='ACTIVE' and h.gate.fault==''
    target=h.sup.follower.mapped_goal
    displacement=math.hypot(target.x-h.origin.x,target.y-h.origin.y)
    assert displacement==pytest.approx(math.hypot(forward,left),abs=1e-7)
    assert h.sup.pilot.path>.25 and h.sup.pilot_status(h.now)['hand_xy_m']>.15
    assert h.sup.pilot_status(h.now)['car_xy_m']>.20


def test_round_trip_and_sideways_motion_does_not_consume_a_25cm_path_budget():
    h=harness.Harness(config(),origin_local=(1.5,1.5));h.start()
    previous=(0.,0.)
    for target in ((.6,0.),(-.6,0.),(0.,0.),(0.,.6),(0.,-.6),(0.,0.)):
        ramp(h,previous,target,2.);previous=target
    assert h.sup.pilot.path>2. and h.gate.pilot.path>2.
    assert h.sup.state=='ACTIVE' and h.sup.pilot.started==h.gate.pilot.started
    h.held=False;assert h.step()==ZERO
    h.run(2.);assert h.sup.stop_confirmed and not h.sup.request_start(h.now,h.ros)[0]


@pytest.mark.parametrize('which',['supervisor','gate'])
def test_old_thresholds_do_not_stop_after_adjusting_sign_before_engagement(which):
    h=prepared();adjust(h,.7,.4);anchor=enable(h)
    core=h.sup if which=='supervisor' else h.gate;m=core.pilot
    old=(m.car,m.wand,m.started)
    # Raw XY beyond 15cm is not itself a failure; the projected goal is bounded.
    core.inputs.poses['wand']['pose']=replace(anchor,x=anchor.x+.75,y=anchor.y+.5)
    core.inputs.poses['car']['pose']=pose_at(h.c,.6,.7)
    m.path=2.
    assert m.active(m.epoch,core.inputs,pose_at(h.c,1.,1.),h.now+.02)==''
    assert (m.car,m.wand,m.started)==old


@pytest.mark.parametrize('which',['supervisor','gate'])
@pytest.mark.parametrize('fault',['goal_f0','goal_f3','goal_l0','goal_l3',
    'car_f0','car_f3','car_l0','car_l3','heading','time','nan','epoch'])
def test_each_independent_guard_enforces_fixed_fences_and_lifecycle(which,fault):
    h=prepared();enable(h);core=h.sup if which=='supervisor' else h.gate;m=core.pilot
    g=h.c.geometry();goal=m.car;stamp=m.started+.1;epoch=m.epoch
    expected=None
    if fault.startswith('goal_'):
        positions={'goal_f0':(-.001,1.5),'goal_f3':(3.001,1.5),
                   'goal_l0':(1.5,-.001),'goal_l3':(1.5,3.001)}
        goal=pose_at(h.c,*positions[fault]);expected='PILOT_GOAL_FENCE'
    elif fault.startswith('car_'):
        f0,f1,l0,l1=g.center_bounds(m.car.yaw)
        positions={'car_f0':(f0-.001,1.5),'car_f3':(f1+.001,1.5),
                   'car_l0':(1.5,l0-.001),'car_l3':(1.5,l1+.001)}
        core.inputs.poses['car']['pose']=pose_at(h.c,*positions[fault]);expected='PILOT_FOOTPRINT_FENCE'
    elif fault=='heading':
        core.inputs.poses['car']['pose']=replace(m.car,yaw=m.car.yaw+.201)
        expected='PILOT_HEADING_LIMIT'
    elif fault=='time':stamp=m.started+20.;expected='PILOT_TIME_LIMIT_20S'
    elif fault=='nan':goal=replace(m.car,x=math.nan);expected='PILOT_POSE_OR_GOAL_INVALID'
    else:epoch='foreign';expected='PILOT_EPOCH_OR_ANCHOR'
    assert m.active(epoch,core.inputs,goal,stamp)==expected


@pytest.mark.parametrize('start',[(.05,.05),(1.5,1.5),(2.95,2.95)])
@pytest.mark.parametrize('target',[(0.,0.),(3.,0.),(0.,3.),(3.,3.)])
def test_every_fixed_corner_is_reachable_by_one_to_one_target_mapping(start,target):
    c=config();origin=pose_at(c,*start)
    from mecanum_wand_4ws8_control.core import RelativeMapping
    wand=Pose(10.,10.,1.)
    mapping=RelativeMapping(origin,wand,1.)
    wx,wy=c.geometry().world(*target)
    sign=replace(wand,x=wand.x+wx-origin.x,y=wand.y+wy-origin.y)
    raw=mapping.goal(sign);goal=c.project_goal(raw,origin,c.max_goal_radius_m)
    assert math.hypot(goal.x-wx,goal.y-wy)<1e-6 and c.goal_inside(goal)


def test_raw_sign_beyond_goal_edge_clamps_then_returns_without_locking():
    h=harness.Harness(config(),origin_local=(2.9,1.5));h.start()
    ramp(h,(0.,0.),(.7,0.),2.);h.run(.5)
    assert h.sup.state=='ACTIVE' and h.sup.follower.goal_limited
    f,l=h.c.geometry().local(h.sup.follower.mapped_goal.x,h.sup.follower.mapped_goal.y)
    # Preserve the existing 2mm accepted-position deadband at the soft edge.
    assert 3.-h.c.follower_config().wand_deadband_m-1e-7<=f<=3.+1e-7
    assert l==pytest.approx(1.5,abs=1e-7)
    ramp(h,(.7,0.),(-.1,0.),3.);h.run(1.)
    assert h.sup.state=='ACTIVE' and not h.sup.follower.goal_limited
    assert h.c.geometry().local(h.sup.follower.mapped_goal.x,h.sup.follower.mapped_goal.y)[0]==pytest.approx(
        2.8,abs=h.c.follower_config().wand_deadband_m+1e-7)


def test_recorded_sign_motion_that_triggered_15cm_limit_no_longer_stops_the_chain():
    path=ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3_20s/ready/ground_stage4_single_gb9j_sxc/observation.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    capture=next(r['data'] for r in rows if r['topic']=='supervisor_status' and r['data']['state']=='ACTIVE')
    origin=Pose(**capture['anchors']['wand_origin']);t0=capture['pilot']['capture_started_steady']
    samples=[r for r in rows if r['topic']=='wand_pose' and t0<=r['received_steady']<=t0+8.]
    h=harness.Harness(config(),origin_local=(.1,.1));h.start();index=0;maximum=0.
    for i in range(1,401):
        while index+1<len(samples) and samples[index+1]['received_steady']<=t0+i*.02:index+=1
        p=samples[index]['data'];dx,dy=p['x']-origin.x,p['y']-origin.y
        h.hand=replace(h.hand_origin,x=h.hand_origin.x+dx,y=h.hand_origin.y+dy,z=p['z'])
        maximum=max(maximum,math.hypot(dx,dy));h.step()
        assert h.sup.state=='ACTIVE',(i*.02,h.sup.reason,h.gate.fault)
    assert maximum>.15 and h.sup.pilot.path>.25
    assert any(r['command_speed_mps']>.01 for r in h.rows)


def test_policy_and_console_report_fixed_arena_not_obsolete_short_move_limits():
    assert POLICY['max_active_s']==20. and POLICY['engagements_per_process']==1
    assert POLICY['spatial_policy']=='FIXED_3X3_GOAL_AND_FOOTPRINT_FENCE_V1'
    assert not any(k in POLICY for k in ('max_hand_xy_m','max_goal_xy_m','max_car_xy_m',
        'max_car_path_m','max_lateral_m','min_forward_m'))
    m=model('ACTIVE',True);m.held=m.key_down=True
    assert '3×3m' in m.instruction(10.) and '5cm' not in m.instruction(10.)
    for key in ('PILOT_GOAL_FENCE','PILOT_FOOTPRINT_FENCE','PILOT_HEADING_LIMIT'):
        assert reason_text(key)!=key
    text=(GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py').read_text()
    assert '杆≤15cm' not in text and '车≤20cm' not in text and '首轮杆仅前移约5cm' not in text
    m=model();m.status['pilot'].pop('spatial_policy')
    assert not m.can_start(10.)


def test_all_native_limits_mapping_arena_watchdogs_and_baseline_algorithms_are_preserved():
    c=config()
    assert (c.max_speed_mps,c.max_accel_mps2)==(.70,.80)
    assert (c.arena_forward_max_m,c.arena_left_max_m,c.arena_buffer_m)==(3.,3.,.50)
    assert (c.command_timeout_s,c.lease_timeout_s,c.feedback_timeout_s)==(.10,.15,.20)
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3_20s/ready/runtime/src'
    for package,files in [('mecanum_wand_4ws8_control',('safety_core.py','core.py','continuous_servo.py',
            'safety_wire.py','safety_nodes.py','arena_config.py','arena_geometry.py','staged_config.py')),
        ('mecanum_wand_4ws8_hardware',('nodes.py','stage0_contract.py','stage0_session.py'))]:
        for name in files:assert (GENERATED/package/package/name).read_bytes()==(old/package/package/name).read_bytes()
    name=Path('mecanum_wand_4ws8_hardware/mecanum_wand_4ws8_hardware/authorization.py')
    expected=(old/name).read_text().replace("review.get('kind') == 'FINITE_FIRST_GROUND_OBSERVATION_NOT_QUALIFICATION'",
        "review.get('kind') == 'BOUNDED_3X3_DEMO_20S_NOT_QUALIFICATION'")
    assert (GENERATED/name).read_text()==expected
    for stage in range(7):
        p=Path(f'mecanum_wand_4ws8_hardware/config/stage{stage}.NOT_APPROVED.json')
        assert (GENERATED/p).read_bytes()==(old/p).read_bytes()


def test_actual_runtime_accepts_only_the_exact_new_profile_and_consent_without_io(tmp_path):
    import hashlib,time
    from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT
    p=json.loads((GENERATED/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())
    p['trial_review']=actual_review()
    p['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,
        flash_record=str(ROOT/'stm32/mecanum_f407/backups/pre_4ws8_staged_20261023_1q2wgy/POST_FLASH_RECORD.json'),
        build_manifest=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3_20s/ready/build_manifest.json'))
    # Synthetic site confirmations confined to the pytest temporary fixture.
    # Constructing this configuration object does not create nodes or sockets.
    p['confirmations']['fixed_arena_checked']=True
    p['arena_confirmations']={k:True for k in p['arena_confirmations']}
    profile=tmp_path/'synthetic_test_profile.json';profile.write_text(json.dumps(p))
    digest=hashlib.sha256(profile.read_bytes()).hexdigest()
    runtime=Runtime('hardware',str(profile),digest,CONSENT)
    assert runtime.config.stage==4 and runtime.wand_reference_margin_m==.50
    with pytest.raises(ValueError):Runtime('hardware',str(profile),digest,'4WS8_V3_ACTIVE_ORIGIN_20S_CLEAR_POWER_CUT_READY')
    with pytest.raises(ValueError):Runtime('hardware',str(profile),'bad',CONSENT)


@pytest.mark.parametrize('change',['missing','path','hash','old_kind','unearned_qualification'])
def test_expanded_scope_requires_the_new_request_and_cannot_claim_qualification(change):
    review=actual_review();assert verify_trial_review(review,4,'sign',review['image_sha256'])
    if change=='missing':review.pop('range_request')
    elif change=='path':review['range_request']=review['user_authorization']
    elif change=='hash':review['range_request_sha256']='bad'
    elif change=='old_kind':review['kind']='FINITE_FIRST_GROUND_OBSERVATION_NOT_QUALIFICATION'
    else:review['continuous_ground_qualified']=True
    with pytest.raises(ValueError):verify_trial_review(review,4,'sign',review['image_sha256'])
