import math

import pytest

from src.rebot_b601.trajectory import MoveOptions, plan, profile, segment_durations


def opts(**kw):
    base = dict(vmax_deg_s=[60.0] * 2, amax_deg_s2=[120.0] * 2, hz=100.0)
    base.update(kw)
    return MoveOptions(**base)


def test_empty_when_already_there():
    assert plan([1.0, 2.0], [[1.0, 2.0]], opts()) == []
    assert plan([1.0, 2.0], [], opts()) == []


def test_ends_exactly_on_last_waypoint():
    steps = plan([0.0, 0.0], [[10.0, -5.0], [30.0, 0.0]], opts())
    assert steps[-1] == [30.0, 0.0]


def test_velocity_and_acceleration_bounded():
    o = opts(vmax_deg_s=[60.0, 30.0], amax_deg_s2=[120.0, 60.0], hz=200.0)
    steps = plan([0.0, 0.0], [[90.0, -45.0]], o)
    dt = 1.0 / o.hz
    prev = [0.0, 0.0]
    prev_v = [0.0, 0.0]
    for q in steps:
        v = [(b - a) / dt for a, b in zip(prev, q)]
        for j in range(2):
            assert abs(v[j]) <= o.vmax_deg_s[j] * 1.01 + 1e-6
            a = (v[j] - prev_v[j]) / dt
            assert abs(a) <= o.amax_deg_s2[j] * 1.5 + 1e-6  # discretisation slack
        prev, prev_v = q, v


def test_duration_matches_trapezoid():
    # 90 deg at 60 deg/s with ramps of 0.5 s -> path 1.5 s + ramp 0.5 s = 2.0 s
    o = opts(vmax_deg_s=[60.0], amax_deg_s2=[120.0], hz=100.0)
    steps = plan([0.0], [[90.0]], o)
    assert math.isclose(len(steps) / o.hz, 2.0, abs_tol=0.02)


def test_short_move_is_triangular():
    total, ramp, peak = profile(0.1, 0.5)
    assert peak < 1.0 and math.isclose(total, 2 * ramp)


def test_passes_through_waypoints_in_order():
    steps = plan([0.0, 0.0], [[10.0, 0.0], [10.0, 10.0]], opts())
    # first joint reaches 10 before second joint starts moving
    idx = next(i for i, q in enumerate(steps) if q[1] > 0.5)
    assert math.isclose(steps[idx][0], 10.0, abs_tol=0.6)


def test_segment_durations_use_slowest_joint():
    d = segment_durations([[0.0, 0.0], [60.0, 10.0]], [60.0, 5.0])
    assert d == [2.0]


def test_mismatched_dof_rejected():
    with pytest.raises(ValueError):
        plan([0.0, 0.0], [[1.0]], opts())
