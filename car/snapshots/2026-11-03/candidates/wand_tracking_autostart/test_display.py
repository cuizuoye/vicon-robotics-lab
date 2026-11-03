"""Shell environment selection only. No X connection, SSH, ROS or device."""
import os
from pathlib import Path
import subprocess
import pytest

HELPER=Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_tracking_autostart/display_environment.sh')


def env_test(values,args='--launch'):
    env={k:v for k,v in os.environ.items() if k not in ('SSH_CONNECTION','SSH_CLIENT','SSH_TTY','DISPLAY','XAUTHORITY')}
    env.update(values)
    command='source "$1"; configure_tracking_display '+args+' || exit $?; printf "%s\\n" "${DISPLAY-UNSET}" "${XAUTHORITY-UNSET}"'
    return subprocess.run(['bash','-c',command,'display_fixture',str(HELPER)],env=env,capture_output=True,text=True)


@pytest.mark.parametrize('display',['localhost:10.0','localhost:11.0','127.0.0.1:12.0'])
def test_remote_display_and_default_ssh_authority_are_not_overwritten(display):
    p=env_test(dict(SSH_CONNECTION='TEST ONLY',DISPLAY=display))
    assert p.returncode==0 and p.stdout.splitlines()==[display,'UNSET']


def test_explicit_remote_authority_is_preserved():
    p=env_test(dict(SSH_CONNECTION='TEST ONLY',DISPLAY='localhost:10.0',XAUTHORITY='/tmp/custom-ssh-auth'))
    assert p.returncode==0 and p.stdout.splitlines()==['localhost:10.0','/tmp/custom-ssh-auth']


def test_remote_without_display_does_not_fall_back_to_local_desktop():
    p=env_test(dict(SSH_CONNECTION='TEST ONLY'))
    assert p.returncode==2 and '没有DISPLAY' in p.stderr


def test_file_check_still_works_in_remote_headless_session():
    p=env_test(dict(SSH_CONNECTION='TEST ONLY'),'--check')
    assert p.returncode==0 and p.stdout.splitlines()==['UNSET','UNSET']


def test_explicit_local_display_authority_still_work():
    p=env_test(dict(DISPLAY=':1',XAUTHORITY='/tmp/local-auth'))
    assert p.returncode==0 and p.stdout.splitlines()==[':1','/tmp/local-auth']
