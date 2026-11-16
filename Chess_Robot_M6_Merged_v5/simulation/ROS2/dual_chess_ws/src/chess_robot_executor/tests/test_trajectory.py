import random

import pytest

from chess_robot_executor.trajectory import axis_min_time, plan_segment, sample


def test_axis_min_time_triangular_and_trapezoid():
    assert axis_min_time(0.0, 1.0, 1.0) == 0.0
    assert axis_min_time(1.0, 10.0, 2.0) == pytest.approx(2 * (1 / 2) ** 0.5)       # triangular
    assert axis_min_time(10.0, 1.0, 1.0) == pytest.approx(10.0 + 1.0)               # trapezoid
    with pytest.raises(ValueError):
        axis_min_time(1.0, 0.0, 1.0)


def test_segment_endpoints_sync_and_limits():
    rng = random.Random(3)
    vmax, amax = [1.0, 1.5, 0.1], [3.0, 4.0, 0.5]
    for _ in range(60):
        q0 = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0, .15)]
        q1 = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0, .15)]
        seg = plan_segment(q0, q1, vmax, amax)
        assert seg.position(0.0) == pytest.approx(q0) and seg.position(seg.duration) == pytest.approx(q1, abs=1e-9)
        dt = 0.002
        n = int(seg.duration / dt)
        prev_v = [0.0] * 3
        for k in range(1, n):
            v = seg.velocity(k * dt)
            for i in range(3):
                assert abs(v[i]) <= vmax[i] * 1.001 + 1e-9
                assert abs(v[i] - prev_v[i]) / dt <= amax[i] * 1.02 + 1e-6
            prev_v = v
        # time-optimal for the slowest axis: some axis needs (almost) the whole duration
        assert max(axis_min_time(q1[i] - q0[i], vmax[i], amax[i]) for i in range(3)) == pytest.approx(seg.duration)


def test_monotonic_positions_and_zero_end_velocity():
    seg = plan_segment([0, 0], [1.0, -0.5], [1, 1], [2, 2])
    last = seg.position(0)
    for k in range(1, 200):
        p = seg.position(k * seg.duration / 199)
        assert p[0] >= last[0] - 1e-12 and p[1] <= last[1] + 1e-12
        last = p
    assert seg.velocity(0) == [0.0, 0.0] and seg.velocity(seg.duration) == [0.0, 0.0]


def test_zero_move_and_min_duration():
    seg = plan_segment([1, 2], [1, 2], [1, 1], [1, 1])
    assert seg.duration == 0.0
    seg2 = plan_segment([1, 2], [1, 2], [1, 1], [1, 1], min_duration=0.3)
    assert seg2.duration == 0.3 and seg2.position(0.15) == [1, 2]


def test_sample_includes_exact_endpoint():
    seg = plan_segment([0], [0.7], [1], [2])
    pts = sample(seg, 0.02)
    assert pts[0][0] == 0.0 and pts[-1][0] == pytest.approx(seg.duration) and pts[-1][1] == [0.7]
    with pytest.raises(ValueError):
        plan_segment([0], [1, 2], [1], [1])
