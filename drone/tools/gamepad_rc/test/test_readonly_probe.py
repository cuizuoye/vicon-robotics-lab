"""诊断发送白名单、来源辨认、PING匹配及手柄锁存的离线测试。"""

import io
from pathlib import Path
import struct
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gamepad_estop_monitor import HOME, StopLatch, find_pad  # noqa: E402
from mavlink_readonly_probe import (  # noqa: E402
    Evidence, PARAMETERS, READABLE_PARAMETERS, SAFETY_PARAMETERS,
    decode_parameter, readonly_message, send_readonly,
    PassivePowerEvidence,
    battery_observation_ok,
)


def received(message, sysid=1, compid=1):
    encoder = m.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    return m.MAVLink(None).parse_char(message.pack(encoder))


def heartbeat(armed=False, sysid=1, compid=1, autopilot=m.MAV_AUTOPILOT_PX4):
    return received(m.MAVLink_heartbeat_message(
        m.MAV_TYPE_QUADROTOR, autopilot, 128 if armed else 0, 0, 3, 3), sysid, compid)


def test_all_planned_requests_are_readonly_and_v2():
    enc = m.MAVLink(None, srcSystem=250, srcComponent=190)
    messages = [enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0),
                enc.ping_encode(99999, 19, 0, 0),
                enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)]
    messages += [enc.param_request_read_encode(1, 1, name.encode(), -1) for name in PARAMETERS]
    for message in messages:
        port = io.BytesIO()
        assert readonly_message(message)
        assert send_readonly(enc, port, message) > 0
        packet = port.getvalue()
        assert packet[0] == 0xfd
        decoded = m.MAVLink(None).parse_char(packet)
        assert decoded.get_type() == message.get_type()


@pytest.mark.parametrize('command', [400, 176, 21, 22, 185, 246, 241])
def test_flight_and_maintenance_mutations_cannot_pass_send_gate(command):
    message = m.MAVLink_command_long_message(1, 1, command, 0, 0, 0, 0, 0, 0, 0, 0)
    with pytest.raises(ValueError):
        send_readonly(m.MAVLink(None), io.BytesIO(), message)


def test_rc_parameter_write_and_unsupported_information_requests_are_rejected():
    enc = m.MAVLink(None)
    messages = [enc.rc_channels_override_encode(1, 1, *([1500] * 8)),
                enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', 11, 6),
                enc.param_request_read_encode(1, 1, b'SOMETHING_ELSE', -1),
                enc.command_long_encode(1, 1, 512, 0, 0, 0, 0, 0, 0, 0, 0)]
    assert all(not readonly_message(message) for message in messages)


def test_other_components_and_ground_station_do_not_prove_fc_link():
    evidence = Evidence()
    evidence.ingest(heartbeat(sysid=1, compid=88), 1)
    evidence.ingest(heartbeat(sysid=255), 2)
    assert not evidence.result()['downlink_verified']


def test_armed_heartbeat_stops_maintenance():
    evidence = Evidence()
    evidence.ingest(heartbeat(), 1)
    assert evidence.result()['downlink_verified']
    evidence.ingest(heartbeat(armed=True), 2)
    assert evidence.fatal
    assert not evidence.result()['downlink_verified']


def test_non_px4_heartbeat_is_rejected():
    evidence = Evidence()
    evidence.ingest(heartbeat(autopilot=m.MAV_AUTOPILOT_ARDUPILOTMEGA), 1)
    assert evidence.fatal


def test_ping_requires_nonce_target_and_exact_source():
    evidence = Evidence()
    evidence.pings[(100, 42)] = 1
    for msg in [received(m.MAVLink_ping_message(101, 42, 250, 190)),
                received(m.MAVLink_ping_message(100, 42, 255, 190)),
                received(m.MAVLink_ping_message(100, 42, 250, 190), compid=88)]:
        evidence.ingest(msg, 1.1)
    assert not evidence.ping_rtt_ms
    evidence.ingest(received(m.MAVLink_ping_message(100, 42, 250, 190)), 1.2)
    assert evidence.ping_rtt_ms == [200]
    evidence.ingest(received(m.MAVLink_ping_message(100, 42, 250, 190)), 1.3)
    assert len(evidence.ping_rtt_ms) == 1


def test_parameter_reply_is_correlated_with_this_round_request():
    evidence = Evidence()
    raw = struct.unpack('<f', struct.pack('<i', 7))[0]
    message = received(m.MAVLink_param_value_message(b'RC_MAP_KILL_SW', raw, 6, 926, 10))
    evidence.ingest(message, 1)
    assert not evidence.parameters
    evidence.requests['RC_MAP_KILL_SW'] = 2
    evidence.ingest(message, 2.1)
    assert evidence.parameters['RC_MAP_KILL_SW'] == 7


def test_safety_profile_only_adds_named_parameter_reads():
    enc = m.MAVLink(None)
    for name in SAFETY_PARAMETERS:
        assert readonly_message(enc.param_request_read_encode(1, 1, name.encode(), -1))
        assert not readonly_message(enc.param_set_encode(1, 1, name.encode(), 0, 6))
    assert set(PARAMETERS).issubset(READABLE_PARAMETERS)


def test_partial_parameter_reply_is_not_a_complete_safety_audit():
    evidence = Evidence(expected_parameters=READABLE_PARAMETERS)
    evidence.parameters = {name: 0 for name in PARAMETERS}
    assert not evidence.result()['all_parameters_received']
    assert set(evidence.result()['missing_parameters']) == set(SAFETY_PARAMETERS)
    evidence.parameters.update({name: 0 for name in SAFETY_PARAMETERS})
    assert evidence.result()['all_parameters_received']
    evidence.fatal = 'Armed'
    assert not evidence.result()['all_parameters_received']


@pytest.mark.parametrize('value,ptype,fmt', [(7, 6, '<i'), (-2, 6, '<i'), (44000, 5, '<I')])
def test_integer_parameter_union(value, ptype, fmt):
    raw = struct.unpack('<f', struct.pack(fmt, value))[0]
    message = received(m.MAVLink_param_value_message(b'TEST', raw, ptype, 1, 0))
    assert decode_parameter(message) == value


def test_home_release_keeps_stop_latched():
    latch = StopLatch()
    latch.event(1, HOME, 1)
    latch.event(1, HOME, 0)
    assert latch.latched and not latch.home_pressed and latch.reason == 'home_pressed'


@pytest.mark.parametrize('code', [0x130, 0x131, 0x13b, 0x13d])
def test_other_buttons_and_stick_axes_cannot_arm_or_clear_latch(code):
    latch = StopLatch()
    latch.event(1, code, 1)
    latch.event(3, 0, 32000)
    assert not latch.latched
    latch.event(1, HOME, 1)
    latch.event(1, code, 1)
    assert latch.latched


def test_input_loss_fails_closed_and_home_has_priority():
    latch = StopLatch()
    latch.event(0, 3, 0)
    assert latch.latched and latch.reason == 'input_queue_overflow'
    latch.event(1, HOME, 1)
    assert latch.reason == 'home_pressed'


def test_passive_power_rejects_other_components_and_unknown_voltage():
    power = PassivePowerEvidence()
    battery = m.MAVLink_battery_status_message(0, 0, 3, 32767, [65535]*10, -1, -1, -1, -1)
    power.ingest(received(battery, compid=88), 1)
    assert not power.result()['battery_samples']
    power.ingest(received(battery), 2)
    row = power.result()['battery_samples'][0]
    assert row['reported_voltage_v'] is None and row['current_a'] is None and row['remaining_percent'] is None


def test_passive_power_records_pack_voltage_not_fake_cell_count():
    power = PassivePowerEvidence()
    battery = m.MAVLink_battery_status_message(0, 0, 3, 32767, [11700]+[65535]*9, 130, -1, -1, 85)
    power.ingest(received(battery), 1)
    row = power.result()['battery_samples'][0]
    assert row['reported_voltage_v'] == 11.7 and row['current_a'] == 1.3


def test_passive_clock_detects_reboot_on_one_consistent_topic():
    power = PassivePowerEvidence()
    enc = m.MAVLink(None)
    for stamp in (1000, 1010, 20):
        power.ingest(received(enc.attitude_quaternion_encode(stamp, 1, 0, 0, 0, 0, 0, 0)), stamp/1000)
    assert power.result()['reboot_observed']
    assert power.result()['first_last_boot_ms'] == [1000, 20]


def battery_result():
    return {'battery_only_physical_state_confirmed': True, 'fc_usb_absence_monitored': True,
            'fc_usb_present_at_end': False, 'error': None, 'identity': {'uid2': 'expected'},
            'round_trip_verified': True, 'all_parameters_received': True,
            'all_fc_heartbeats_disarmed': True,
            'heartbeats': [{'elapsed_s': i+.2} for i in range(30)],
            'passive_power': {'battery_samples': [{'reported_voltage_v': 11.7}],
                              'system_power_samples': [], 'boot_sample_count': 300,
                              'reboot_observed': False, 'first_last_boot_ms': [1000, 31000]}}


def test_battery_observation_requires_complete_independent_power_evidence():
    assert battery_observation_ok(battery_result(), 'expected', 30)
    for key in ('battery_only_physical_state_confirmed', 'fc_usb_absence_monitored',
                'round_trip_verified', 'all_parameters_received', 'all_fc_heartbeats_disarmed'):
        d = battery_result()
        d[key] = False
        assert not battery_observation_ok(d, 'expected', 30)
    d = battery_result()
    d['fc_usb_present_at_end'] = True
    assert not battery_observation_ok(d, 'expected', 30)
    assert not battery_observation_ok(battery_result(), 'different', 30)


def test_guarded_props_installed_is_explicit_readonly_not_props_off():
    d = battery_result()
    d.update(battery_only_physical_state_confirmed=False,
             guarded_props_installed_readonly_confirmed=True,
             propeller_state='installed', communication_only_no_motion_authorized=True)
    assert battery_observation_ok(d, 'expected', 30)
    d['propeller_state'] = 'removed'
    assert not battery_observation_ok(d, 'expected', 30)
    d['propeller_state'] = 'installed'
    d['communication_only_no_motion_authorized'] = False
    assert not battery_observation_ok(d, 'expected', 30)


def test_conflicting_power_physical_modes_rejected():
    d = battery_result()
    d.update(guarded_props_installed_readonly_confirmed=True,
             propeller_state='installed', communication_only_no_motion_authorized=True)
    assert not battery_observation_ok(d, 'expected', 30)


def test_power_confirmation_cli_modes_are_mutually_exclusive(tmp_path):
    import subprocess
    tool = Path(__file__).resolve().parents[1] / 'mavlink_readonly_probe.py'
    result = subprocess.run([sys.executable, str(tool), '--port', '/does-not-exist',
        '--output', str(tmp_path/'result.json'), '--battery-only-confirmed',
        '--guarded-props-installed-readonly-confirmed'], capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and 'not allowed with argument' in result.stderr
    assert not (tmp_path/'result.json').exists()


@pytest.mark.parametrize('case', ['unknown_voltage', 'reboot', 'cached_backlog', 'heartbeat_gap', 'too_few_heartbeats'])
def test_battery_observation_rejects_missing_stale_or_unstable_records(case):
    d = battery_result()
    if case == 'unknown_voltage': d['passive_power']['battery_samples'][0]['reported_voltage_v'] = None
    if case == 'reboot': d['passive_power']['reboot_observed'] = True
    if case == 'cached_backlog': d['passive_power']['first_last_boot_ms'] = [1000, 340000]
    if case == 'heartbeat_gap': d['heartbeats'][10]['elapsed_s'] = 14
    if case == 'too_few_heartbeats': d['heartbeats'] = d['heartbeats'][:2]
    assert not battery_observation_ok(d, 'expected', 30)
