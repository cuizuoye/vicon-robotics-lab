import builtins
import json
import socket
import subprocess
import pytest
import native_replay as replay


def test_replay_extracts_only_display_fields_and_does_not_copy_authority(tmp_path,monkeypatch):
    source=tmp_path/'source.jsonl';out=tmp_path/'out';out.mkdir()
    pose=dict(x=1.,y=2.,z=.1,qx=0.,qy=0.,qz=0.,qw=1.,secret='not-copied')
    rows=[dict(received_steady=.9,topic='car_pose',data=pose),
          dict(received_steady=1.,topic='supervisor_status',data=dict(state='ACTIVE',command={'vx':9},source='secret-owner')),
          dict(received_steady=1.1,topic='operator_lease',data=dict(secret='secret-lease')),
          dict(received_steady=2.,topic='wand_pose',data=pose)]
    source.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    monkeypatch.setattr(replay,'SOURCE',source)
    meta=replay.extract(out,1.)
    data=(out/'display_events.jsonl').read_text()
    assert meta['events']==3 and meta['last_event_s']==1.
    assert 'secret' not in data and 'command' not in data and 'operator_lease' not in data


def test_short_or_never_active_recording_cannot_claim_complete_replay(tmp_path,monkeypatch):
    source=tmp_path/'source';source.write_text(json.dumps(dict(received_steady=1.,
        topic='supervisor_status',data=dict(state='ACTIVE')))+'\n')
    out=tmp_path/'out';out.mkdir();monkeypatch.setattr(replay,'SOURCE',source)
    with pytest.raises(ValueError):replay.extract(out,600.)


def test_gui_guard_rejects_ros_transport_executor_and_subprocess():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    originals=(rclpy.init,Node.__init__,SingleThreadedExecutor.__init__,socket.socket,subprocess.Popen,builtins.open)
    try:
        attempts=replay.d.deny_live_io()
        for action in (lambda:rclpy.init(),lambda:Node.__init__(None),
                       lambda:SingleThreadedExecutor.__init__(None),lambda:socket.socket(),
                       lambda:subprocess.Popen(['never-run'])):
            with pytest.raises(RuntimeError):action()
        assert len(attempts)==5
    finally:
        rclpy.init,Node.__init__,SingleThreadedExecutor.__init__,socket.socket,subprocess.Popen,builtins.open=originals
