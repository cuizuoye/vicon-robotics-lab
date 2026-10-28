"""Reuse the reviewed existing receiver; no spawn, network changes or controls."""
from pathlib import Path
import os
import sys

PROVENANCE=Path('/home/PRIMARY_USER/vicon_receiver_ws/drone_sim/tools/gamepad_rc')


def verify_existing():
    sys.path.insert(0,str(PROVENANCE))
    from vicon_receiver_provenance import verify_current
    provenance=verify_current()
    matches=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():continue
        try:
            if (entry/'comm').read_text().strip()=='vicon_client':matches.append(entry)
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if len(matches)!=1:raise ValueError('需要唯一、已经运行的Vicon接收器；不会自动启动或停止它')
    process=matches[0]
    if str((process/'exe').resolve())!=provenance['installed']:
        raise ValueError('现有Vicon程序来源与已核验版本不符；停止只读预览')
    env=dict(v.split('=',1) for v in (process/'environ').read_bytes().decode().split('\0') if '=' in v)
    args=(process/'cmdline').read_bytes().decode().split('\0')
    expected=('hostname:=10.10.10.2','world_frame:=map','vicon_frame:=vicon',
        'map_xyz:=[0.0,0.0,0.0]','map_rpy:=[0.0,0.0,0.0]')
    if env.get('ROS_DOMAIN_ID','0')!='0' or any(x not in args for x in expected):
        raise ValueError('现有Vicon域、坐标系或服务器与基线不同；不更改配置')
    if os.environ.get('ROS_DOMAIN_ID','0') not in ('0','120'):raise ValueError('只读入口要求ROS域0')
    return dict(pid=int(process.name),installed=provenance['installed'],
        installed_sha256=provenance['installed_sha256'],manifest=provenance['manifest'],
        manifest_sha256=provenance['manifest_sha256'],receiver_started=False,
        receiver_changed=False,actuator_outputs=False)


def source_ok(infos):
    # This is a graph identity check, not cryptographic DDS authentication.
    # Executable provenance is checked separately before subscribing.
    return len(infos)==1 and infos[0].node_name=='vicon_client' and infos[0].node_namespace=='/'
