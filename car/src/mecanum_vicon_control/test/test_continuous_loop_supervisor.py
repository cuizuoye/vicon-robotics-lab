from types import SimpleNamespace

from mecanum_vicon_control.continuous_loop_supervisor import (
    ContinuousLoopSupervisor,
    HOLD,
    PREFLIGHT,
    STANDBY,
    STOPPING,
    VICON_LOST,
    lap_incremented,
    recovery_state_allows_motion,
)


def test_lap_increment_ignores_initial_and_new_run_reset():
    assert not lap_incremented(None, 0)
    assert lap_incremented(0, 1)
    assert not lap_incremented(1, 1)
    assert not lap_incremented(3, 0)


def test_recovery_fault_states_cannot_continue_loop():
    assert recovery_state_allows_motion('TRACKING')
    assert recovery_state_allows_motion('RECOVERY')
    assert not recovery_state_allows_motion(HOLD)
    assert not recovery_state_allows_motion(VICON_LOST)


def test_start_service_is_inert_and_enters_preflight():
    calls = []
    node = SimpleNamespace(
        desired_run=False,
        state=STANDBY,
        resume_after_rearm=True,
        last_lap_seen=4,
        last_stop_reason='old',
        publish_stop=lambda: calls.append('stop'),
        set_state=lambda state: calls.append(state),
    )
    response = SimpleNamespace(success=False, message='')
    returned = ContinuousLoopSupervisor.on_start(node, None, response)
    assert returned is response and response.success
    assert node.desired_run and not node.resume_after_rearm
    assert node.last_lap_seen is None
    assert calls == ['stop', PREFLIGHT]


def test_begin_stop_logs_normal_and_fault_stops_from_distinct_callsites():
    calls = []

    class FakeLogger:
        def info(self, message):
            calls.append(('info', message))

        def error(self, message):
            calls.append(('error', message))

    def make_node():
        return SimpleNamespace(
            desired_run=True,
            resume_after_rearm=True,
            last_stop_reason='',
            state=STANDBY,
            publish_stop=lambda: calls.append(('publish_stop', None)),
            publish=lambda publisher, value: calls.append(('lease', value)),
            lease_pub=object(),
            set_state=lambda state: calls.append(('state', state)),
            get_logger=lambda: FakeLogger(),
        )

    normal = make_node()
    ContinuousLoopSupervisor.begin_stop(normal, 'manual stop', fault=False)
    assert not normal.desired_run and not normal.resume_after_rearm
    assert normal.last_stop_reason == 'manual stop'
    assert ('state', STOPPING) in calls
    assert ('info', '停止连续矩形并关闭全部运动门: manual stop') in calls

    faulted = make_node()
    ContinuousLoopSupervisor.begin_stop(faulted, 'preflight timeout', fault=True)
    assert faulted.last_stop_reason == 'preflight timeout'
    assert ('error', '停止连续矩形并关闭全部运动门: preflight timeout') in calls
