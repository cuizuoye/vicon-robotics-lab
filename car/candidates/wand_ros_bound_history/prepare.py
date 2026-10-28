#!/usr/bin/env python3
"""Independent combined ROS-history + verified remote UI candidate."""
import importlib.util
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_remote_x11_2026-10-27/prepared'
spec=importlib.util.spec_from_file_location('bound_prepare_helpers',ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
sha,hashes,write,edit,check=(getattr(helpers,n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m=check(OLD/'build_manifest.json')
    if not m.get('remote_x11_passed'):raise ValueError('verified remote parent required')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):protected.update(m[group])
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    # Once input expires, close deterministically: do not re-read a cache that
    # could become fresh between two calls and leave the GUI loop unscheduled.
    edit(c/'remote_keys.py',
        "        except (OSError,RuntimeError):\n            self.server_space_down()  # Existing fail-closed revoke/close path.\n            return",
        "        except (OSError,RuntimeError) as error:\n"
        "            self.model.revoke(time.monotonic(),'键盘状态读取失败：使能已释放；请重启操作台')\n"
        "            self.node.get_logger().error(str(error))\n"
        "            self.close()\n"
        "            return")
    shutil.copyfile(HERE/'bound_history.py',c/'bound_history.py');shutil.copyfile(HERE/'bound_rotation.py',h/'bound_rotation.py')
    rotation=h/'rotation_nodes.py'
    edit(rotation,'def source_main():run_node(RotationSignSource)',
        'from .bound_rotation import HistoryQosMixin,BoundGateMixin\n'
        'class RetainedSignSource(HistoryQosMixin,RotationSignSource):pass\n'
        'class RetainedSupervisor(HistoryQosMixin,RotationSupervisorNode):pass\n'
        'class RetainedGate(BoundGateMixin,RotationGateNode):pass\n'
        'RotationSignSource,RotationSupervisorNode,RotationGateNode=RetainedSignSource,RetainedSupervisor,RetainedGate\n\n'
        'def source_main():run_node(RotationSignSource)')
    for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'):
        shutil.copyfile(OLD/name,out/name)
    app=out/'oneclick_app.py';edit(app,repr(str(OLD)),repr(str(out)))
    edit(app,"m.get('remote_x11_passed')","m.get('ros_bound_history_passed')")
    app.write_text(app.read_text().replace('SHOW_REMOTE_X11_V1','SHOW_BOUND_HISTORY_V1').replace("b'REMOTE_X11_V1'","b'BOUND_HISTORY_V1'"))
    edit(out/'pose_check.py',str(OLD/'runtime/src'),str(runtime))
    edit(out/'launcher_ui.py','远程X11候选 · MCU反馈300ms','ROS参考帧修复候选 · MCU反馈300ms')
    # Software fixture publishers must match the new internal relay QoS.
    rehearsal=out/'ros_rehearsal.py'
    edit(rehearsal,'from mecanum_wand_4ws8_control.safety_nodes import pose_message,LATEST,SENSOR',
        'from mecanum_wand_4ws8_control.safety_nodes import pose_message,LATEST,SENSOR\n'
        '    from mecanum_wand_4ws8_control.bound_history import HISTORY_QOS')
    edit(rehearsal,"source.create_publisher(PoseStamped,k+'_pose',SENSOR)",
        "source.create_publisher(PoseStamped,k+'_pose',HISTORY_QOS if k=='car' else SENSOR)")
    edit(rehearsal,"source.create_publisher(String,k,LATEST)",
        "source.create_publisher(String,k,HISTORY_QOS if k=='rotation_packet' else LATEST)")
    shutil.copyfile(HERE/'delivery_fixture.py',out/'delivery_fixture.py')
    edit(rehearsal,"'feedback_310ms','used_budget'):",
        "'feedback_310ms','used_budget','reference_reorder','candidate_burst','reference_loss'):")
    edit(rehearsal,"    ap.add_argument('--runtime',type=Path,required=True);args=ap.parse_args()",
        "    ap.add_argument('--runtime',type=Path,required=True)\n"
        "    ap.add_argument('--cases',nargs='+');args=ap.parse_args()")
    edit(rehearsal,"        for case in (", "        for case in (args.cases or (")
    edit(rehearsal,"'candidate_burst','reference_loss'):","'candidate_burst','reference_loss')):")
    edit(rehearsal,'                gate=RotationGateNode(runtime)',
        "                from delivery_fixture import DeliveryFixture\n"
        "                delivery=DeliveryFixture(state,case)\n"
        "                original_subscription=Node.create_subscription\n"
        "                def subscribed(node,msg_type,topic,callback,*a,**kw):\n"
        "                    if node.get_name()=='safe_wand_gate' and topic in ('car_pose','rotation_packet','candidate'):\n"
        "                        callback=delivery.wrap(callback,topic)\n"
        "                    return original_subscription(node,msg_type,topic,callback,*a,**kw)\n"
        "                Node.create_subscription=subscribed\n"
        "                try:gate=RotationGateNode(runtime)\n"
        "                finally:Node.create_subscription=original_subscription\n"
        "                source.create_timer(.003,delivery.flush)")
    edit(rehearsal,"source.create_timer(.02 if case=='feedback_250ms' else .05,fixture)",
        "source.create_timer(.01 if case.startswith('reference_') else .02 if case=='feedback_250ms' else .05,fixture)")
    edit(rehearsal,"elif case.startswith('feedback_'):pass",
        "elif case.startswith('feedback_') or case in ('reference_reorder','candidate_burst','reference_loss'):pass")
    edit(rehearsal,"                    if case=='feedback_250ms' and state.injected is not None:",
        "                    if case in ('reference_reorder','candidate_burst') and state.active is not None and state.restored is None:\n"
        "                        assert not gate.core.fault,(case,'fault before operator stop',gate.core.fault)\n"
        "                    if case in ('feedback_250ms','reference_reorder','candidate_burst') and state.injected is not None:")
    edit(rehearsal,"                    if case!='feedback_250ms' and state.injected is not None and state.restored is None and now-state.injected>.45:",
        "                    if case not in ('feedback_250ms','reference_reorder','candidate_burst') and state.injected is not None and state.restored is None and now-state.injected>.45:")
    edit(rehearsal,"                result=dict(case=case,passed=True,phases=phases,service_and_click_calls=calls,gate_faults=faults,",
        "                if case in ('reference_reorder','candidate_burst'):\n"
        "                    # Startup/explicit stop can legitimately trip the gate;\n"
        "                    # operation() checks EVERY motion tick before operator stop.\n"
        "                    assert gate.selection_failure is None,(case,faults,gate.selection_failure)\n"
        "                    assert delivery.observed['delayed']>0,delivery.observed\n"
        "                if case=='reference_reorder':assert delivery.observed['reordered']>0,delivery.observed\n"
        "                if case=='reference_loss':\n"
        "                    assert delivery.observed['dropped']>0,delivery.observed\n"
        "                    assert 'ROTATION_SELECTED_SOURCE_INVALID' in faults,(faults,gate.selection_failure)\n"
        "                    assert gate.selection_failure['missing_car'] and not gate.selection_failure['missing_sign']\n"
        "                result=dict(case=case,passed=True,phases=phases,service_and_click_calls=calls,gate_faults=faults,\n"
        "                    delivery_fixture=delivery.observed,source_failure=gate.selection_failure,")
    sources=hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_ros_history.sh',*(out/n for n in
        ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py','delivery_fixture.py'))]:sources[str(p)]=sha(p)
    changes=[]
    for p in runtime.rglob('*'):
        if p.is_file():
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.exists() or sha(p)!=sha(old):changes.append(str(rel))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',changes)
    m.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,ros_bound_history_passed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json');return m
