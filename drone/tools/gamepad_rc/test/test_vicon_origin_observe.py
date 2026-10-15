import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vicon_origin_observe import relative_geometry, square_board_center_candidate


def test_world_translation_and_rotation_removed():
    # 世界姿态90度：刚体x对应世界y。
    rigid, flu = relative_geometry([10, 20, 30], [0, 0, math.sqrt(.5), math.sqrt(.5)],
                                  [[10, 21, 30]] * 5, [0, 0, 0, 1])
    assert np.allclose(rigid, [[1, 0, 0]] * 5)
    assert np.allclose(flu, rigid)


def test_candidate_mounting_axes_do_not_change_origin():
    rigid, flu = relative_geometry([0, 0, 0], [0, 0, 0, 1], [[1, 0, 2]] * 5,
                                  [0, 0, 1, 0])
    assert np.allclose(rigid, [[1, 0, 2]] * 5)
    assert np.allclose(flu, [[-1, 0, 2]] * 5)


@pytest.mark.parametrize('markers', [[[0, 0, 0]] * 4, [[float('nan'), 0, 0]] * 5])
def test_missing_or_nonfinite_markers_rejected(markers):
    with pytest.raises(ValueError):
        relative_geometry([0, 0, 0], [0, 0, 0, 1], markers, [0, 0, 0, 1])


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        relative_geometry([0, 0, 0], [0, 0, 0, 0], [[0, 0, 0]] * 5, [0, 0, 0, 1])


def test_square_corner_uses_half_diagonal_not_half_side():
    result = square_board_center_candidate([40 / math.sqrt(2), 0, 30], [0, 0, 0, 1], 40, 30)
    assert np.allclose(result['board_center_in_rigid_m_candidate'], [0, 0, 0])
    assert result['body_origin_in_rigid'] is None
    assert result['imu_offset_verified'] is result['extrinsics_verified'] is False
    assert result['actual_sent_to_fc'] == 0


def test_board_offset_rotation_and_units():
    result = square_board_center_candidate([0, 0, 0], [0, 0, 1, 0], 40, 30)
    assert np.allclose(result['board_center_in_rigid_m_candidate'], [.04 / math.sqrt(2), 0, -.03])


@pytest.mark.parametrize('side,height', [(0, 30), (40, -1), (float('nan'), 30)])
def test_bad_physical_dimensions_rejected(side, height):
    with pytest.raises(ValueError):
        square_board_center_candidate([0, 0, 0], [0, 0, 0, 1], side, height)
