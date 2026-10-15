import json
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rigid_body_revision as p


def example():
    old = np.array([[-44, 30, 0], [-33, -42, 1], [48, -35, -1], [30, 0, 30], [39, 36, 0]], float)
    theta = .2
    r = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    t = np.array([4., -5., 6.])
    altered = old.copy(); altered[3] += [30, 0, -30]
    new = (altered @ r.T + t)[[3, 4, 2, 0, 1]]
    return old, new, r, t


def test_registration_handles_renaming_and_rebases_offset_once():
    old, new, r, t = example()
    result = p.solve_registration(old, new, [0, 0, 0, 1], [0, 0, 0, 1],
                                  only_old_marker_four_moved_confirmed=True)
    assert result['new_moved_marker_name_candidate'] == 'MircoAir1'
    assert result['fixed_marker_fit_rms_mm'] < 1e-10
    assert np.allclose(result['rotation_old_rigid_to_new_rigid'], r)
    assert np.allclose(result['translation_old_rigid_to_new_rigid_mm'], t)
    assert np.allclose(result['moved_marker_forward_left_up_mm'], [30, 0, -30])
    h, b = p.rebase(result, {'body_orientation_in_rigid_xyzw_candidate':[0,0,0,1]},
                    {'target_reference':'board', 'board_center_in_rigid_m_candidate':[.001,.002,.003]}, 'new')
    assert np.allclose(b['board_center_in_rigid_m_candidate'], r @ [.001,.002,.003] + t/1000)
    assert h['physical_forward_definition_inherited'] and not h['ready_to_send_to_fc']
    assert b['marker_name'] is None and not b['old_elevated_marker_height_assumption_reused']
    assert not b['imu_offset_verified']
    assert h['forward_heading_in_map_deg'] is None
    assert abs(h['forward_heading_in_rigid_xy_deg'] - np.degrees(.2)) < 1e-10
    assert 'start_stamp_ns' not in h


def test_confirmation_and_finite_geometry_required():
    old, new, _, _ = example()
    with pytest.raises(ValueError, match='Operator confirmation'):
        p.solve_registration(old, new, [0,0,0,1], [0,0,0,1])
    with pytest.raises(ValueError, match='finite'):
        p.solve_registration(old * np.nan, new, [0,0,0,1], [0,0,0,1],
                             only_old_marker_four_moved_confirmed=True)
    with pytest.raises(ValueError, match='Non-collinear'):
        p.solve_registration(np.zeros((5,3)), new, [0,0,0,1], [0,0,0,1],
                             only_old_marker_four_moved_confirmed=True)


def test_gate_rejects_old_revision_provisional_and_modified_candidates(tmp_path):
    h, b, registry = [tmp_path/n for n in ('h.json','b.json','r.json')]
    for path in (h, b):path.write_text(json.dumps({'rigid_body_revision_id':'current'}))
    record = {'active_revision_id':'current','heading_candidate_sha256':p.digest(h),
              'board_candidate_sha256':p.digest(b),'static_validation_completed':False}
    registry.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='provisional'):
        p.require_current_calibration(h,b,registry_path=registry)
    registry.write_text(json.dumps({**record,'static_validation_completed':True}))
    assert p.require_current_calibration(h,b,registry_path=registry)['active_revision_id']=='current'
    h.write_text(json.dumps({'rigid_body_revision_id':'old'}))
    with pytest.raises(ValueError, match='old rigid-body'):
        p.require_current_calibration(h,b,registry_path=registry)
    h.write_text(json.dumps({'rigid_body_revision_id':'current','unexpected':'change'}))
    with pytest.raises(ValueError, match='hash mismatch'):
        p.require_current_calibration(h,b,registry_path=registry)


@pytest.mark.parametrize('runner', ['usb_vicon_receive_bench.py', 'trs_static_fusion_bench.py'])
def test_live_cli_rejects_old_same_named_model_before_devices_or_permit_consumption(tmp_path, runner):
    import subprocess
    h, b = tmp_path/'old_heading.json', tmp_path/'old_board.json'
    h.write_text('{}'); b.write_text('{}')
    tool = Path(__file__).resolve().parents[1]/runner
    arguments = ['--heading-candidate',str(h),'--board-center-candidate',str(b),
                 '--output',str(tmp_path/'must_not_exist.json')]
    if runner.startswith('usb_'):
        arguments += ['--port','/not-a-device','--uid2','test','--bench-send']
    else:
        arguments += ['--backup','/missing','--vision-reference-permit','/missing-permit',
                      '--usb-only-props-installed-vision-reference-confirmed']
    result = subprocess.run([sys.executable,str(tool),*arguments],capture_output=True,text=True,timeout=5)
    assert result.returncode == 2 and 'old rigid-body extrinsics' in result.stderr
    assert not (tmp_path/'must_not_exist.json').exists()
