from types import SimpleNamespace

from std_msgs.msg import Bool

from mecanum_vicon_control.safety_watchdog import SafetyWatchdog


def watchdog(require_lease=True, lease_ns=1_000_000_000):
    return SimpleNamespace(
        last_pose_ns=1_000_000_000,
        last_cmd_ns=1_000_000_000,
        pose_timeout_s=0.20,
        command_timeout_s=0.20,
        require_run_lease=require_lease,
        last_run_lease_ns=lease_ns,
        run_lease_timeout_s=0.30,
    )


def test_required_run_lease_is_part_of_freshness():
    node = watchdog()
    assert SafetyWatchdog.is_fresh(node, 1_200_000_000)
    assert not SafetyWatchdog.is_fresh(node, 1_301_000_000)
    node.last_run_lease_ns = None
    assert not SafetyWatchdog.is_fresh(node, 1_100_000_000)


def test_legacy_watchdog_does_not_require_loop_supervisor():
    node = watchdog(require_lease=False, lease_ns=None)
    assert SafetyWatchdog.is_fresh(node, 1_100_000_000)


def test_false_lease_immediately_closes_enabled_gate():
    logs = []
    node = SimpleNamespace(
        enabled=True,
        require_run_lease=True,
        last_run_lease_ns=123,
        last_fault='',
        get_logger=lambda: SimpleNamespace(
            warn=lambda text: logs.append(text)),
    )
    SafetyWatchdog.on_run_lease(node, Bool(data=False))
    assert not node.enabled
    assert node.last_run_lease_ns is None
    assert node.last_fault == 'operator_run_lease_stopped'
    assert logs
