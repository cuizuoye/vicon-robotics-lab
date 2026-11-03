"""Actual ROS matched scheduling A/B and simulation profile/safety validation."""
import argparse
import json
import math
from pathlib import Path
import time

from model import ROOT,Limits,ZERO,save,protected_hashes
from ros_tuning import install,TraceLog


def one(folder,profile,optimized,seconds=8.,stall_timer=False):
    folder.mkdir()
    logger=TraceLog(folder)
    module=install(logger,profile,optimized)
    import rclpy
    from analyze import run as analyze
    state=dict(x=0.,y=0.,yaw=0.)
    def provider(now,ros):
        return module.synthetic_packet(state['x'],state['y'],state['yaw'],now,ros-.001)
    module.init_private()
    c=module.Chain(provider,None)
    def spin(seconds,action=None,predicate=None):
        began=time.monotonic();last=0.
        while time.monotonic()-began<seconds:
            c.touch()
            if action:action(time.monotonic()-began)
            c.step()
            now=time.monotonic()
            if now-last>.025:c.snapshot();last=now
            if predicate and predicate():return True
        return False
    try:
        assert spin(6.,predicate=lambda:c.snapshot()['can_prepare']),c.snapshot()
        assert c.prepare()
        assert spin(5.,predicate=lambda:c.ready() and not c.prepare_pending),c.snapshot()
        assert c.plant.command==ZERO
        assert c.engage()
        assert spin(1.,predicate=lambda:c.sup.core.state=='ACTIVE'),c.snapshot()
        def move(t):state.update(x=.25*(1-math.cos(2*t)),y=.1*math.sin(t),yaw=.03*math.sin(t))
        spin(seconds,move)
        assert c.sup.core.state=='ACTIVE' and not c.gate.core.fault,c.snapshot()
        if stall_timer:
            timers=[timer for timer in c.gate.timers if timer.timer_period_ns==10_000_000]
            assert len(timers)==1
            c.gate.destroy_timer(timers[0])
            began=time.monotonic()
            def isolated_gate_events(t):
                move(seconds+t)
                # SYNTHETIC injection only: keep the supervisor's independent
                # gate-report layer fresh, otherwise it legitimately stops
                # first and masks the gate timer's own ClockCheck. This tests
                # that event commits alone cannot renew THAT timer baseline.
                steady,ros=c.sup.clocks()
                c.sup.core.observe_gate(steady=steady,ros=ros,epoch=c.sup.core.epoch,
                                        fault='',hard_fault=False)
            # Continue the SAME gesture phase; restarting move(t) at zero
            # would inject a pose jump instead of isolating timer starvation.
            assert spin(.18,isolated_gate_events,
                        predicate=lambda:c.gate.core.hard_fault and c.plant.command==ZERO),c.snapshot()
            save(folder/'TIMER_STALL_RESULT.json',dict(passed=True,
                to_zero_s=time.monotonic()-began,event_callbacks_could_not_renew_watchdog=True,
                injected_fresh_supervisor_report_to_isolate_gate_timer=True,
                hard_fault=c.gate.core.fault,physical_car_started=False))
        c.stop('ROS_SCHEDULER_TEST_ESC')
        assert spin(3.,predicate=lambda:c.sup.core.stop_confirmed and c.plant.command==ZERO),c.snapshot()
        save(folder/'CONTRACT.json',dict(real_sign=False,synthetic_fixture=True,optimized=optimized,
            profile=profile.name,domain=190,physical_car_started=False,firmware_changed=False))
    finally:
        c.close();rclpy.shutdown();logger.close()
    result=analyze(folder)
    # No draw required for this ROS-only test; use source->verified/model IDs.
    assert result['trace_complete'] and result['coverage']['controls_verified']>100,result['coverage']
    assert result['coverage']['controls_applied_to_model']>100,result['coverage']
    return {name:result['metrics'].get(name) for name in ('relay_to_control_tick',
        'control_publish_to_gate_receive','gate_accept_to_sign','local_source_to_verified',
        'control_publish_to_verified','local_source_to_model_step')}


def run(output,safety=False,reuse_comparisons=None,reuse_functional=None):
    output.mkdir(parents=True,exist_ok=False)
    before=protected_hashes();save(output/'PROTECTED_BEFORE.json',before)
    results=[]
    # Alternating order limits a one-off machine-load advantage; same config,
    # source curve, software model and signatures in all scheduling comparisons.
    for index,optimized in enumerate((False,True,True,False)):
        if reuse_comparisons:
            from model import digest
            path=reuse_comparisons/('schedule_%d_%s'%(index,'fast' if optimized else 'legacy'))/'LATENCY_RESULT.json'
            saved=json.loads(path.read_text())
            assert saved['trace_complete'] and saved['coverage']['controls_verified']>100
            r={name:saved['metrics'].get(name) for name in ('relay_to_control_tick',
                'control_publish_to_gate_receive','gate_accept_to_sign','local_source_to_verified',
                'control_publish_to_verified','local_source_to_model_step')}
        else:
            r=one(output/('schedule_%d_%s'%(index,'fast' if optimized else 'legacy')),Limits(),optimized)
        results.append(dict(optimized=optimized,metrics=r))
        if reuse_comparisons:results[-1].update(reused_source=str(path),sha256=digest(path))
        print('schedule',index,optimized,r['control_publish_to_verified'],flush=True)
    if reuse_functional:
        def reuse(name):
            path=reuse_functional/name/'LATENCY_RESULT.json'
            saved=json.loads(path.read_text())
            assert saved['trace_complete'] and saved['coverage']['controls_verified']>100
            return {k:saved['metrics'].get(k) for k in ('relay_to_control_tick',
                'control_publish_to_gate_receive','gate_accept_to_sign','local_source_to_verified',
                'control_publish_to_verified','local_source_to_model_step')}
        high=reuse('higher_profile');recommended=reuse('recommended_profile')
    else:
        high=one(output/'higher_profile',Limits(1.2,4.,800.),True)
        recommended=one(output/'recommended_profile',Limits(.7,4.,400.),True)
    watchdog=one(output/'independent_timer_stall',Limits(.7,4.,400.),True,seconds=2.,stall_timer=True)
    cases=0
    if safety:
        logfolder=output/'inherited_safety';logfolder.mkdir()
        logger=TraceLog(logfolder)
        module=install(logger,Limits(),True)
        import test_ros_chain
        # The inherited UI fixture creates Tk synchronously in its ROS test
        # thread, unlike the actual separate GUI/ROS worker. Prewarm Tk before
        # its nodes exist so font/X11 startup cannot manufacture >100ms stale
        # signed frames. No deadline/verification rule is relaxed.
        import tkinter as tk
        import threading
        from types import SimpleNamespace
        from app import Window
        real_tk=tk.Tk
        warm=real_tk();warm.withdraw()
        adapter=SimpleNamespace(snapshot={},ui_stamp=0.,done=threading.Event(),post=lambda action:None)
        initial=Window(warm,adapter);initial.closed=True
        for child in warm.winfo_children():child.destroy()
        tk.Tk=lambda *args,**kwargs:warm
        # Namespace/domain check derives from the private runtime, not a hard
        # motor port. The inherited tests never construct hardware transports.
        try:
            test_ros_chain.run(logfolder)
            cases=len(test_ros_chain.CASES)
        finally:
            tk.Tk=real_tk
            try:
                if warm.winfo_exists():warm.destroy()
            except tk.TclError:pass
            logger.close()
    assert before==protected_hashes()
    save(output/'PROTECTED_AFTER.json',protected_hashes())
    result=dict(passed=True,actual_ros=True,source='synthetic for controlled scheduling comparison',
        comparisons=results,higher_profile=high,recommended_profile=recommended,
        independent_watchdog_stall_passed=True,inherited_safety_cases=cases,
        safety_deadlines_unchanged=True,independent_gate_timer_s=.01,
        original_unchanged=True,physical_car_started=False,firmware_changed=False,
        domain=190,real_sign_user_acceptance=False)
    save(output/'RESULT.json',result)
    print(json.dumps(dict(passed=True,inherited_safety_cases=cases)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--safety',action='store_true')
    parser.add_argument('--reuse-comparisons',type=Path)
    parser.add_argument('--reuse-functional',type=Path)
    args=parser.parse_args();run(args.output,args.safety,args.reuse_comparisons,args.reuse_functional)
