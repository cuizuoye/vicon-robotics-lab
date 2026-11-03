from suspended_rectangle_cutoff import startup_deadline_expired


def test_startup_deadline_stops_a_run_that_never_started():
    assert startup_deadline_expired(None, 20.0, 20.0)


def test_startup_deadline_never_truncates_started_motion():
    assert not startup_deadline_expired(19.5, 25.0, 20.0)
