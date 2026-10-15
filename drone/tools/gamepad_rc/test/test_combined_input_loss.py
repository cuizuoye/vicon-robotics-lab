from pathlib import Path
import subprocess
import sys

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from combined_input_loss import CombinedInputLoss


def test_one_mask_requires_prior_fusion_and_latch_never_recovers():
    loss=CombinedInputLoss()
    with pytest.raises(ValueError): loss.mask(10,fusion_verified=False)
    loss.mask(10,fusion_verified=True)
    with pytest.raises(ValueError): loss.mask(11,fusion_verified=True)
    assert loss.permits(True,10.1,50)
    assert not loss.permits(False,10.151,50)
    assert not loss.permits(True,10.5,50)
    result=loss.result(last_tx_s=10.02,tx_count_after_recovery=50,suppressed=12,
                       fresh_restored=True,rollback_verified=True)
    assert result['verified'] and not result['automatic_restart_allowed']
    assert not result['airborne_stop_or_land_verified']


@pytest.mark.parametrize('change',['late_write','late_detect','restarted','missing_input','stale_recovery','restore_failed'])
def test_loss_criteria_not_faked_by_source_returning(change):
    loss=CombinedInputLoss();loss.mask(10,fusion_verified=True)
    loss.permits(False,10.19 if change=='late_detect' else 10.151,50)
    args=dict(last_tx_s=10.02,tx_count_after_recovery=50,suppressed=12,fresh_restored=True,rollback_verified=True)
    if change=='late_write': args['last_tx_s']=10.16
    if change=='restarted': args['tx_count_after_recovery']=51
    if change=='missing_input': args['suppressed']=0
    if change=='stale_recovery': args['fresh_restored']=False
    if change=='restore_failed': args['rollback_verified']=False
    assert not loss.result(**args)['verified']


def test_no_mask_or_no_loss_does_not_pass():
    loss=CombinedInputLoss()
    with pytest.raises(ValueError): loss.permits(True,10,0)
    assert not loss.result(last_tx_s=10,tx_count_after_recovery=0,suppressed=0,
                           fresh_restored=True,rollback_verified=True)['verified']


def test_wireless_source_loss_rejected_without_scoped_permit_before_devices(tmp_path):
    tool = Path(__file__).resolve().parents[1] / 'trs_static_fusion_bench.py'
    result = subprocess.run([sys.executable, str(tool), '--source-loss-test',
        '--backup', '/missing', '--heading-candidate', '/missing',
        '--board-center-candidate', '/missing', '--output', str(tmp_path / 'result.json'),
        '--physical-confirmed'], capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'Source-loss test requires its own scoped static permit' in result.stderr
    assert not (tmp_path / 'result.json').exists()


@pytest.mark.parametrize('mode',[[],['--height-fusion-bench'],['--restore-combined-only']])
def test_source_loss_requires_explicit_combined_only_before_any_serial(tmp_path,mode):
    tool=Path(__file__).resolve().parents[1]/'usb_vicon_receive_bench.py'
    command=[sys.executable,str(tool),'--port','/dev/doesnotexist','--uid2','test',
             '--heading-candidate','/missing','--board-center-candidate','/missing','--output',str(tmp_path/'new.json'),
             '--combined-source-loss-bench','--fusion-physical-confirmed',*mode]
    if '--height-fusion-bench' in mode: command.append('--bench-send')
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==2 and 'requires explicit combined fusion bench' in result.stderr
    assert not (tmp_path/'new.json').exists()
