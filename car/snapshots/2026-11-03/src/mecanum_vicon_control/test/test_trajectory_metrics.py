import math
import pytest
from mecanum_vicon_control.trajectory_metrics import (
    point_segment_distance, percentile, tracking_metrics,
)


def test_distance_projection_endpoints_and_degenerate():
    assert point_segment_distance((.5, .2),(0,0),(1,0)) == pytest.approx(.2)
    assert point_segment_distance((2,0),(0,0),(1,0)) == 1
    assert point_segment_distance((3,4),(0,0),(0,0)) == 5


def test_metrics_straight_path():
    m, errors = tracking_metrics([(0,0,.1),(1,.5,.1),(2,1,.1)], [(0,0),(1,0)], [.1]*3)
    assert errors == pytest.approx([.1]*3)
    assert m['nearest_polyline']['rmse_m'] == pytest.approx(.1)
    assert m['active_segment']['max_m'] == pytest.approx(.1)
    assert m['travelled_length_m'] == 1
    assert m['final_goal_error_m'] == pytest.approx(.1)
    assert m['active_segment']['rmse_time_weighted_m'] == pytest.approx(.1)


def test_percentile_interpolates_and_rejects_invalid():
    assert percentile([0,1,2,3], .5) == 1.5
    assert percentile([4], .95) == 4
    with pytest.raises(ValueError): percentile([], .5)
    with pytest.raises(ValueError): percentile([1], 2)


@pytest.mark.parametrize('poses', [[], [(0,0,0)], [(1,0,0),(0,1,0)],
                                 [(0,0,0),(0,1,0)], [(0,0,0),(1,math.nan,0)]])
def test_invalid_samples_rejected(poses):
    with pytest.raises(ValueError): tracking_metrics(poses, [(0,0),(1,0)])


def test_active_error_sample_count_and_nonfinite_geometry():
    with pytest.raises(ValueError): tracking_metrics([(0,0,0),(1,1,0)], [(0,0),(1,0)], [0])
    with pytest.raises(ValueError): point_segment_distance((math.inf,0),(0,0),(1,0))


def test_crossing_nearest_path_does_not_replace_progress_error():
    m, _ = tracking_metrics([(0,0,0),(1,.1,.1)], [(-1,-1),(1,1),(-1,1),(1,-1)], [1,1])
    assert m['nearest_polyline']['max_m'] < 1e-12
    assert m['active_segment']['max_m'] == 1


def test_irregular_samples_use_time_weighted_error():
    m,_ = tracking_metrics([(0,0,0),(.1,.1,0),(1,1,0)], [(0,0),(1,0)], [0,0,1])
    assert m['active_segment']['rmse_m'] == pytest.approx(math.sqrt(1/3))
    assert m['active_segment']['rmse_time_weighted_m'] == pytest.approx(math.sqrt(.45))
